// Implementation of the GCPR 2016 Paper "Joint Object Pose Estimation and
// Shape Reconstruction in Urban Street Scenes Using 3D Shape Priors" by Engelmann et al.
// Copyright (C) 2016  Francis Engelmann - Visual Computing Institute RWTH Aachen University
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

// C/C++ includes
#include <iostream>
#include <string>
#include <iostream>
#include <fstream>
#include <time.h>
#include <cmath>
#include <memory>
#include <unordered_map>
#include <tuple>
#include <algorithm>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>

// Ceres includes
#include <ceres/ceres.h>

// VIZ includes
#include <viz/viz.h>

// Own includes
#include "io.h"
#include "utils.h"
#include "tracking.h"
#include "optimization/shape.h"
#include "optimization/pose.h"

struct parameters {
  std::string image_2_prefix;
  int first;
  int last;
  int length;
  std::string seq_string;
  std::string poses_path;
  std::string detections_prefix;
  std::string disparity_prefix;
  std::string calibration_path;
  std::string groundplane_prefix;
  std::string results_prefix;
  bool interactive;

  parameters(int argc, char** argv) {
    if (argc < 12+1) { std::cerr << "Not enough parameters!" << std::endl; }
    image_2_prefix = std::string{argv[1]};
    first = std::stoi(std::string{argv[2]});
    last = std::stoi(std::string{argv[3]});
    length = std::stoi(std::string{argv[4]});
    seq_string = std::string{argv[5]};
    poses_path = std::string{argv[6]};
    detections_prefix = std::string{argv[7]};
    disparity_prefix = std::string{argv[8]};
    calibration_path = std::string{argv[9]};
    groundplane_prefix = std::string{argv[10]};
    results_prefix = std::string{argv[11]};
    interactive = std::string{argv[12]}.compare("1")==0;
}

void print(void) {
  std::cout << image_2_prefix << " ";
  std::cout << first << " ";
  std::cout << last << " ";
  std::cout << length << " ";
  std::cout << seq_string << " ";
  std::cout << poses_path << " ";
  std::cout << detections_prefix << " ";
  std::cout << disparity_prefix << " ";
  std::cout << calibration_path << " ";
  std::cout << groundplane_prefix << " ";
  std::cout << results_prefix << " ";
  std::cout << interactive << std::endl;
}
};

int main(int argc, char** argv) {

  // Read parameters
  parameters params(argc, argv);
  bool print_params = true;
  if (print_params) params.print();

  // Read calibration, poses
  std::vector<Eigen::Matrix4d> projection_matrices = gvl::readCalibrationFile(params.calibration_path);
  std::vector<Eigen::Matrix4d> poses = gvl::readPosesFromFile(params.poses_path);
  std::vector<Eigen::Matrix4d> posesInv; posesInv.reserve(poses.size());
  for(auto& p : poses) posesInv.push_back(p.inverse());

  // Build K matrix
  Eigen::Matrix3d K = projection_matrices.at(0).block<3,3>(0,0);
  double f = projection_matrices.at(0)(0,0);
  double baseline = projection_matrices.at(1)(0,3)/-f;

  // Read SVD decompositions
  Eigen::MatrixXd& S = gvl::ShapeSpacePCA::instance().S;
  Eigen::MatrixXd& V = gvl::ShapeSpacePCA::instance().V;
  Eigen::MatrixXd& mean = gvl::ShapeSpacePCA::instance().mean;
  gvl::readMatrixFromBinaryFile(path_S, S);
  gvl::readMatrixFromBinaryFile(path_V, V);
  gvl::readMatrixFromBinaryFile(path_mean_shape, mean);

  // All detections
  std::vector<std::vector<std::shared_ptr<gvl::Detection>>> all_detections; // all 3dop detections over all the frames
  all_detections.resize(params.last - params.first + 1);

  //**********************************************************************************************
  // Iterate over frames in window to accumulate data
  //**********************************************************************************************

  viz::Visualization w(800,800);
  cv::Mat image;
  double t = params.first;

  // Set up pathes for current frame
  std::string detections_path = params.detections_prefix+gvl::zfill(t,params.length)+".txt";
  std::string disparity_path = params.disparity_prefix+gvl::zfill(t,params.length)+".png";
  std::string image_2_path = params.image_2_prefix+gvl::zfill(t, params.length) +".png";
  std::string groundplane_path = params.groundplane_prefix+gvl::zfill(t, params.length) +".txt";

  // Read detections (3DOP)
  std::vector<gvl::BoundingBox> detected_boundingboxes = gvl::readDetectionsFromFile(detections_path); // detections inside this frame
  cv::Mat disparity = cv::imread(disparity_path, CV_LOAD_IMAGE_ANYDEPTH);
  if (disparity.data==NULL) { std::cout << "Error: " << disparity_path << std::endl; return 0; }
  cv::Mat image_2 = cv::imread(image_2_path);
  if (image_2.data==NULL) { std::cout << "Error: " << image_2_path << std::endl; return 0; }
  Eigen::Vector4d groundplane = gvl::readGroundplaneFromFile(groundplane_path);
  image = image_2;

  // Generate scene-pointcloud
  cv::Mat vertices;
  auto pointcloud = std::make_shared<gvl::Pointcloud>();
  gvl::computeVerticesFromDisparity(disparity, K, baseline, vertices);
  gvl::computePointcloudFromVerticesAndColor(vertices, image_2, *pointcloud);

  // Filter and transform pointcloud
  auto pointcloud_filtered(std::make_shared<gvl::Pointcloud>() );
  gvl::remove_road_plane_from_pointcloud(*pointcloud, groundplane, *pointcloud_filtered);

  // **********************************************************************************************
  // Initialize detections from all bounding boxes in this frame
  // **********************************************************************************************

  all_detections.at(t-params.first).resize(detected_boundingboxes.size());
  for (unsigned int d=0; d<detected_boundingboxes.size(); d++) {
    all_detections.at(t-params.first).at(d) = std::make_shared<gvl::Detection>();
    auto& bb = detected_boundingboxes.at(d);
    auto& det = all_detections.at(t-params.first).at(d);
    det->boundingbox = std::make_shared<gvl::BoundingBox>(bb);
    det->frame_id = t;
    det->translation = Eigen::Vector3d(bb.x, bb.y, bb.z);
    det->rotation_y = bb.rotation_y;
    det->pose = gvl::computePoseFromRotTransScale(det->rotation_y, det->translation, 1.0);
    det->z = Eigen::VectorXd::Zero(r,1);
    det->shape = mean;
    gvl::extract_points_to_detection(image_2,*pointcloud_filtered,*det,det->pose,2.5);
  }

  // **********************************************************************************************
  // Opimtization
  // **********************************************************************************************

  int j=0;
  for (auto& det : all_detections.at(t-params.first) ) {
    if (det->pointcloud->points.size()<200 || det->translation[2] > 30) continue;
    if(false){ // record movie of optimization
      if (j++!=2) continue;
      viz::Visualization v(1440,768);
      //v.lookFromAt(Eigen::Vector3d(0,-9,-20),Eigen::Vector3d(0,0,40));
      //v.addBoundingBox(*det->boundingbox);
      v.addPointcloud(det->pointcloud->getVertices(), det->pointcloud->getColors());
      Eigen::Vector3d from(-1.73963, -2.62134, -0.708406);  //P:
      Eigen::Vector3d at = det->translation;
      v.lookFromAt(from,at);
      if (params.interactive) v.show();
    }

    // Optimization - iterate alternativly until convergence
    double total_cost_curr=DBL_MAX/2;
    double total_cost_prev=DBL_MAX;
    double delta = 0.1;
    int it = 0;
    while (total_cost_prev - total_cost_curr > delta && it<1) {
      total_cost_prev = total_cost_curr;
      //std::cout << "Iteration: " << it++ << std::endl;
      double pose_cost = gvl::optimize_pose(*det, false, &it);
      double shape_cost = gvl::optimize_shape(*det, false, &it);
      total_cost_curr = pose_cost + shape_cost;
    }
  }

  // Prepare visualization
  std::string filename = params.seq_string+"_"+gvl::zfill(t,2);
  std::string path;

  // Visualization used for depth rendering
  viz::Visualization v_depth(image_2.cols, image_2.rows);
  v_depth.setCameraPosition(Eigen::Vector3d(0,0,0));
  v_depth.setK(K, image_2.cols, image_2.rows);
  for (auto& det : all_detections.at(t-params.first) ) {
    if (det->pointcloud->points.size()<200 || det->translation[2] > 30) continue;
    v_depth.addZeroSurfaceFromGrid(det->shape, det->pose, gvl::generateColor(0), false);
  }
  cv::Mat vertices_ren(disparity.rows, disparity.cols, CV_64FC3, cv::Scalar(0,0,0));
  v_depth.renderDepth(vertices_ren);

  // Create color overlap for visualisation
  cv::Mat disparity_res(disparity.rows, disparity.cols, CV_16UC1, cv::Scalar(0));
  gvl::computeDisparityFromVertices(vertices_ren, K, baseline, disparity_res);
  cv::Mat vert;
  gvl::computeVerticesFromDisparity(disparity_res,K,baseline,vert);
  gvl::Pointcloud pointcloud_result;
  gvl::computePointcloudFromVerticesAndColor(vert, image_2, pointcloud_result);

  // Pointcloud without cars
  gvl::Pointcloud pointcloud_without_cars(*pointcloud);
  gvl::remove_cars_from_pointcloud(vertices_ren,pointcloud_without_cars);

  if (false) { // show deteced pointlcoud
    viz::Visualization v(400*1.5,300*1.5);
    for (auto& det : all_detections.at(t-params.first) ) {
      v.addPointcloud(det->pointcloud->getVertices(), det->pointcloud->getColors());
    }
    if (params.interactive) v.show();
    return 0;
  }

  if (true) { // show 3d of result
    cv::Mat channels[3];
    cv::split(vertices_ren, channels);
    cv::imshow("disparity_res_c_path", image_2*0.1 + gvl::generateColorMap(channels[2])*0.9);

    viz::Visualization v(1600,800, 2);
    v.connectCameras(0,1);
    v.lookFromAt(Eigen::Vector3d(0,-7,-20),Eigen::Vector3d(0,-2,40));

    Eigen::Vector3f text_color(255,255,255);

    // Left viewer
    v.setActiveRenderer(0);
    v.setBackgroundColor(0,0,0);
    v.addText2d("Input", 30, 30, text_color);
    v.addPointcloud(pointcloud->getVertices(),pointcloud->getColors(), 3);
    for (auto& det : all_detections.at(t-params.first) ) {
      auto& bb = *det->boundingbox;
      bb.r=0; bb.g=0; bb.b=1;
      v.addBoundingBox(bb.getSize(),bb.getTranslation(),bb.rotation_y,bb.getColor());
    }

    // Right viewer
    v.setActiveRenderer(1);
    v.setBackgroundColor(0,0,0);
    v.addText2d("Our result", 30, 30, text_color);
    v.addPointcloud(pointcloud_result.getVertices(),pointcloud_result.getColors(),5);
    v.addPointcloud(pointcloud_without_cars.getVertices(),pointcloud_without_cars.getColors(),3);
    int i=0;
    for (auto& det : all_detections.at(t-params.first) ) {
      if (det->pointcloud->points.size()<200 || det->translation[2] > 30) continue;
      v.setActiveRenderer(0);
      v.addPointcloud(det->pointcloud->getVertices(),det->pointcloud->getColors(),3);
      v.setActiveRenderer(1);
      v.addZeroSurfaceFromGrid(det->shape, det->pose, Eigen::Vector3f(255,255,255), false, 0,0.1,1);
      v.addZeroSurfaceFromGrid(det->shape, det->pose, Eigen::Vector3f(0,0,0), true, 0,0.1,2);
      //v.addZeroSurfaceFromGrid(det->shape, det->pose, gvl::generateColor(i), true,0,0.1,3);
      //v.addZeroSurfaceFromGrid(det->shape, det->pose, gvl::generateColor(i), false);
      i++;
    }
    v.show();
  }
  return 0;
}
