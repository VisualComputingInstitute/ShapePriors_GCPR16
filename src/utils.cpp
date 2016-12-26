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
#include <stdio.h>
#include <math.h>
#include <float.h>
#include <string>
#include <iostream>
#include <fstream>
#include <string>

// Boost includes
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

// OpenCV incudes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>

// Own includes
#include "utils.h"

namespace gvl {

double jet_interpolate( double val, double y0, double x0, double y1, double x1 ) {
    return (val-x0)*(y1-y0)/(x1-x0) + y0;
}

double jet_base( double val ) {
    if ( val <= -0.75 ) return 0;
    else if ( val <= -0.25 ) return jet_interpolate( val, 0.0, -0.75, 1.0, -0.25 );
    else if ( val <= 0.25 ) return 1.0;
    else if ( val <= 0.75 ) return jet_interpolate( val, 1.0, 0.25, 0.0, 0.75 );
    else return 0.0;
}

double jet_red( double gray ) {
    return jet_base( gray - 0.5 );
}
double jet_green( double gray ) {
    return jet_base( gray );
}
double jet_blue( double gray ) {
    return jet_base( gray + 0.5 );
}

void imshow(const std::string &windowName, const cv::Mat &image)
{
  double mmin, mmax;
  cv::Mat temp;
  cv::minMaxIdx(image, &mmin, &mmax);
  image.convertTo(temp,CV_8UC1, 255 / (mmax-mmin), -mmin*255 / (mmax-mmin));
  cv::applyColorMap(temp, temp, cv::COLORMAP_JET);
  cv::imshow(windowName, temp);
}

Eigen::Vector3d applyJetColorMap(double v) {
  return Eigen::Vector3d(jet_red(v), jet_green(v), jet_blue(v));
}

cv::Mat generateColorMap(const cv::Mat &image)
{
  double mmin, mmax;
  cv::Mat temp;
  cv::minMaxIdx(image, &mmin, &mmax);
  image.convertTo(temp,CV_8UC1, 255.0 / (mmax-mmin), -mmin*255.0 / (mmax-mmin));
  cv::applyColorMap(temp, temp, cv::COLORMAP_JET);
  return temp;
}

void generateColor(unsigned int id, float* color)
{
  if (id%10 == 0) { color[0] = 240; color[1] = 62; color[2] = 36;}
  if (id%10 == 1) { color[0] = 245; color[1] = 116; color[2] = 32;}
  if (id%10 == 2) { color[0] = 251; color[1] = 174; color[2] = 24;}
  if (id%10 == 3) { color[0] = 213; color[1] = 223; color[2] = 38;}
  if (id%10 == 4) { color[0] = 153; color[1] = 204; color[2] = 112;}
  if (id%10 == 5) { color[0] = 136; color[1] = 201; color[2] = 141;}
  if (id%10 == 6) { color[0] = 124; color[1] = 201; color[2] = 169;}
  if (id%10 == 7) { color[0] = 100; color[1] = 199; color[2] = 230;}
  if (id%10 == 8) { color[0] = 64; color[1] = 120; color[2] = 188;}
  if (id%10 == 9) { color[0] = 61; color[1] = 88; color[2] = 167;}
}

void generateColor(const unsigned int id, unsigned char* r, unsigned char* g, unsigned char *b)
{
  float color[3];
  generateColor(id,color);
  *r = color[0];
  *g = color[1];
  *b = color[2];
}

Eigen::Vector3f generateColor(unsigned int id) {
  float c[3];
  generateColor(id,c);
  Eigen::Vector3f color;
  color[0] = c[0];
  color[1] = c[1];
  color[2] = c[2];
  return color;
}

std::string extractFilename(const std::string &path)
{
  std::string filename = path;
  size_t slashPosition = filename.rfind('/');
  if (slashPosition != std::string::npos) filename.erase(0, slashPosition+1);
  size_t dotPosition = filename.rfind('.');
  if (dotPosition != std::string::npos) filename.erase(dotPosition);
  return filename;
}

std::string getTimestamp()
{
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer,80,"%d-%m-%Y_%I:%M:%S",timeinfo);
  std::string timestamp(buffer);

  return timestamp;
}

std::string zfill(int input, int length) {
  std::string result = std::to_string(input);
  size_t n = length-result.length();
  result = std::string (n, '0') + result;
  return result;
}

void plot(std::vector<double> values) {
  cv::Mat img(500,500,CV_8UC3,cv::Scalar(255,255,255));
  cv::imshow("plot", img);
  cv::waitKey(0);
}

void populate_neighborhood(std::vector<Eigen::Vector3i>& n_ids) {
  n_ids.clear();
  n_ids.reserve(8);
  n_ids.push_back(Eigen::Vector3i(0,0,0));
  n_ids.push_back(Eigen::Vector3i(0,0,1));
  n_ids.push_back(Eigen::Vector3i(0,1,0));
  n_ids.push_back(Eigen::Vector3i(1,0,0));
  n_ids.push_back(Eigen::Vector3i(1,0,1));
  n_ids.push_back(Eigen::Vector3i(0,1,1));
  n_ids.push_back(Eigen::Vector3i(1,1,0));
  n_ids.push_back(Eigen::Vector3i(1,1,1));
}

gvl::BoundingBox computeBoundingboxFromDetection(const gvl::Detection& detection, const Eigen::Vector3f& color) {
  gvl::BoundingBox bb;
  bb.height = 1.7;
  bb.width = 2.0;
  bb.length = 4.5;
  bb.x = detection.translation[0];
  bb.y = detection.translation[1];
  bb.z = detection.translation[2];
  bb.rotation_y = detection.rotation_y;
  bb.r = color[0]/255;
  bb.g = color[1]/255;
  bb.b = color[2]/255;
  bb.score = 1.0;
  return bb;
}

void show_poses(std::vector<Eigen::Matrix4d>& poses) {
  std::vector<Eigen::Vector2d> proj(poses.size());
  int min_x=INT_MAX, min_y=INT_MAX;
  int max_x=INT_MIN, max_y=INT_MIN;
  for (auto P : poses) {
    Eigen::Vector4d o(0,0,0,1);
    Eigen::Vector4d o_t = P*o; o_t/=o_t[3];
    Eigen::Vector2d pr(o_t[0], o_t[2]);
    if (pr[0] < min_x) min_x = pr[0];
    if (pr[0] > max_x) max_x = pr[0];
    if (pr[1] < min_y) min_y = pr[1];
    if (pr[1] > max_y) max_y = pr[1];
    proj.push_back(pr);
  }
  int width = std::max(max_x-min_x, 200);
  int height = std::max(max_y-min_y, 200);
  cv::Mat pose_fig(height, width, CV_8UC1, cv::Scalar(255));
  for (auto p : proj) {
    int x = std::round(p[0])+100;
    int y = std::round(p[1])+100;
    pose_fig.at<uint8_t>(y,x)=0.0;
  }
  cv::imshow("Poses",pose_fig);
  cv::waitKey(0);
}

void remove_road_plane_from_pointcloud(gvl::Pointcloud& pointcloud,
                                       Eigen::Vector4d road_plane,
                                       gvl::Pointcloud& pointcloud_filtered) {
  pointcloud_filtered.points.reserve(pointcloud.points.size());
  double &a = road_plane[0];
  double &b = road_plane[1];
  double &c = road_plane[2];
  double &d = road_plane[3];
  gvl::Point p_zero;
  for (unsigned int i=0; i<pointcloud.points.size(); i++) {
    auto& p = pointcloud.points.at(i);
    double dist2groundplane = a*p.x + b*p.y + c*p.z + d;
    double distance2camera = p.x*p.x + p.z*p.z;
    if (dist2groundplane < 3 &&
        dist2groundplane > 0.1 &&
        distance2camera  < lambda_dist2cam*lambda_dist2cam) {
      pointcloud_filtered.points.push_back(p);
    } else{
      p_zero.u=p.u;
      p_zero.v=p.v;
      pointcloud_filtered.points.push_back(p_zero);
    }
  }
}

void keep_road_plane_from_pointcloud(gvl::Pointcloud& pointcloud,
                                       Eigen::Vector4d road_plane,
                                       gvl::Pointcloud& pointcloud_filtered) {
  pointcloud_filtered.points.reserve(pointcloud.points.size());
  double &a = road_plane[0];
  double &b = road_plane[1];
  double &c = road_plane[2];
  double &d = road_plane[3];
  gvl::Point p_zero;
  for (unsigned int i=0; i<pointcloud.points.size(); i++) {
    auto& p = pointcloud.points.at(i);
    double dist2groundplane = a*p.x + b*p.y + c*p.z + d;
    double distance2camera = p.x*p.x + p.z*p.z;
    if (dist2groundplane < 0.1 &&
        distance2camera  < 20*20) {
      pointcloud_filtered.points.push_back(p);
    } else{
      p_zero.u=p.u;
      p_zero.v=p.v;
      pointcloud_filtered.points.push_back(p_zero);
    }
  }
}

void extract_points_to_detection(cv::Mat& color_image,
                                 gvl::Pointcloud& pointcloud_scene,
                                 gvl::Detection& detection,
                                 Eigen::Matrix4d& pose,
                                 double distance_threshold) {

  // Pose is the current position of the camere where color_image was recorded
  // Pointcloud_scene is already in world coordinates

  detection.pointcloud->points.clear();
  // Iterate over 2D rectangle of detection boundingbox
  auto bb = detection.boundingbox;
  auto t = detection.translation;
  for (int v=bb->top; v<bb->bottom; v++) {
    for (int u=bb->left; u<bb->right; u++) {
      int index = v*color_image.cols + u; // compute pointcloud index of current pixel
      gvl::Point p = pointcloud_scene.points.at(index); // get 3d point
      //std::cout << "ndex " << index << " p=" << p.x << std::endl;

      if (std::abs(p.x-pose(0,3)) < 1 &&
          std::abs(p.y-pose(1,3)) < 1 &&
          std::abs(p.z-pose(2,3)) < 1) continue; // ignore of point is close to position of recording camera, TODO: what does zero mean?
      // in the following only consider points within a radius of 5m to orginal detection translation

      Eigen::Vector3d poi(p.x, p.y, p.z);
      double squaredDistance = (poi-t).transpose() * (poi-t);
      //if ( gvl::computeSquaredDistance(p, gvl::Point(t[0], t[1], t[2])) > bb->z + 5 ) continue;
      if (squaredDistance > distance_threshold*distance_threshold) continue;
      detection.pointcloud->points.push_back(p);
      //pointcloud_scene.points.at(index).x = 0;
      //pointcloud_scene.points.at(index).y = 0;
      //pointcloud_scene.points.at(index).z = 0;
      // Color segmented points in original image
      //color_image.at<cv::Vec3b>(v,u) = color_image.at<cv::Vec3b>(v,u)/2.0 + cv::Vec3b(0,0,255);
    }
  }
  //cv::imshow("color image", color_image);
  //cv::waitKey(0);
}

void remove_cars_from_pointcloud(cv::Mat& cars_depth,
                                 gvl::Pointcloud& pointcloud_scene) {

  // Iterate over 2D rectangle of detection boundingbox
  for (int v=0; v<cars_depth.rows; v++) {
    for (int u=0; u<cars_depth.cols; u++) {
      int index = v*cars_depth.cols + u; // compute pointcloud index of current pixel
      if ( cars_depth.at<cv::Vec3d>(v,u)[0] != 0 ||
            cars_depth.at<cv::Vec3d>(v,u)[1] != 0 ||
            cars_depth.at<cv::Vec3d>(v,u)[2] != 0) {
        pointcloud_scene.points.at(index).x = 0;
        pointcloud_scene.points.at(index).y = 0;
        pointcloud_scene.points.at(index).z = 0;
      }
    }
  }
}

void draw_2d_detections(cv::Mat &image_2, std::vector<gvl::BoundingBox> &detections, bool tracking) {
  for (auto & d : detections) {
    if (tracking) {
      int& trackId = d.trackId;
      Eigen::Vector3f color = (trackId>-1) ? gvl::generateColor(trackId) : Eigen::Vector3f(255,255,255);
      int lineWith = (trackId>-1) ? 4:1;
      cv::rectangle(image_2,cv::Point(d.left, d.top),
                            cv::Point(d.right, d.bottom),
                            cv::Scalar(color[2],color[1],color[0]), lineWith);
      if (trackId==-1) continue; // do not draw track id if detection is not assigned to a track
      cv::putText(image_2, std::to_string(trackId),
                  cv::Point(d.left+2, d.bottom-5), // position
                  cv::FONT_HERSHEY_SCRIPT_SIMPLEX, // font face
                  1, // font scale
                  cv::Scalar(color[2],color[1],color[0]), // color
                  3, // thickness
                  8);
    } else {
      int lineWidth = 4;
      cv::rectangle(image_2,cv::Point(d.left, d.top),
                            cv::Point(d.right, d.bottom),
                            cv::Scalar(255*d.score,0*d.score,0*d.score), lineWidth);
    }
  }
}

void show_2d_detections(cv::Mat &image_2, std::vector<gvl::BoundingBox> &detections) {
  cv::Mat det = image_2.clone();
  draw_2d_detections(det, detections);
  cv::imshow("2D Detections", det);
  cv::waitKey(0);
}

}

/*std::cout << "Loading V, S, mean matrices..." << std::endl;
gvl::readMatrixFromFile("/Users/francis/S_new.matrix", S); // "good" originates from the subset of valid cars
gvl::readMatrixFromFile("/Users/francis/mean_shape_new.matrix", mean);
gvl::readMatrixFromFile("/Users/francis/V_new.matrix", V);
//gvl::readMatrixFromSharedMemory("MySharedMemory", "V_new", V);

std::cout << "V: " << V.rows() << "x" << V.cols() << std::endl;
std::cout << "S: " << S.rows() << "x" << S.cols() << std::endl;
std::cout << "mean: " << mean.rows() << "x" << mean.cols() << std::endl;

Eigen::MatrixXd data(p,1);
gvl::readMatrixFromFile("/Users/francis/Downloads/models/tsdfs/AlfaRomeo_Brera_100000_values.tsdf",data);

//Eigen::MatrixXd Y = V_test*V_test.transpose()*data_matrix.block<1,p>(1,0) + mean;
//Eigen::MatrixXd Vr = V;//.block<p,9>(0,0);
Eigen::MatrixXd x = data - mean;// mean is already substracted
Eigen::MatrixXd z = V.transpose() * x;
Eigen::MatrixXd y = V * z + mean;
//std::cout << z << std::endl;
gvl::Visualization vis2(800,800);
vis2.addAxes();
vis2.addZeroSurfaceFromGrid(y, Eigen::Matrix4d::Identity(), Eigen::Vector3f(0,200 ,0));
vis2.show();
return 0;*/


// Project proposals bounding box to ground -----------------------------------------------------
/*int proj_width =  1000;
int proj_height = 1000;
double scaling = 20;
cv::Mat projection_value(proj_height, proj_width, CV_64F, cv::Scalar(0) );
cv::Mat projection_count(proj_height, proj_width, CV_64F, cv::Scalar(0) );
for (auto bb : bounding_boxes) {

  int bb_x_start = std::round((bb->x - bb->width)*scaling);
  int bb_x_end = std::round((bb->x + bb->width)*scaling);
  int bb_z_start = std::round((bb->z - bb->length)*scaling);
  int bb_z_end = std::round((bb->z + bb->length)*scaling);

  // Iterate over pixels that are inside projection of bb
  for (int x=bb_x_start; x<bb_x_end; x++){
    for (int z=bb_z_start; z<bb_z_end; z++){
      int u = x + proj_width/2.0;
      int v = proj_height - z;
      // Only draw if inside image bounds
      if (u < proj_width && u > 0 && v < proj_height && v > 0) {
        projection_value.at<double>(v,u) += (bb->score);
        projection_count.at<double>(v,u) += 1.0;
      }
    }
  }
}*/

// Create ground plane projection of proposals pointcloud ---------------------------------------
/*auto prop_cloud(std::make_shared<gvl::Pointcloud>());
{
  double mmin, mmax;
  cv::Mat temp;
  cv::Mat proj_norm = projection_value;///projection_count;

  cv::minMaxIdx(proj_norm, &mmin, &mmax);
  proj_norm.convertTo(temp,CV_8UC1, 255 / (mmax-mmin), -mmin*255 / (mmax-mmin));
  cv::applyColorMap(temp, temp, cv::COLORMAP_JET);

  for (int v=0; v<projection_value.rows; v++) {
    for (int u=0; u<projection_value.cols; u++) {
       gvl::Point p;
       p.x = (u - proj_width/2.0)/scaling;
       p.y = (-d-a*p.x-c*p.z)/b;
       p.z = (proj_height - v)/scaling;
       p.r = 255-temp.at<cv::Vec3b>(v,u)[0];
       p.g = 255-temp.at<cv::Vec3b>(v,u)[1];
       p.b = 255-temp.at<cv::Vec3b>(v,u)[2];
       prop_cloud->points.push_back(p);
    }
  }
}*/

// Read groundtruth annotations

//cv::Mat proj_count = gvl::generateColorMap(projection_count);
//cv::Mat proj_value = gvl::generateColorMap(projection_value);
//cv::Mat proj_norm = gvl::generateColorMap(projection_value/projection_count);

//cv::Mat proj_gt(proj_height,proj_width,CV_8UC3, cv::Scalar(255, 255, 100));
//proj_gt.at<cv::Vec3b>(50, 50) = cv::Vec3b(0, 0, 0);

// Iterate over the object image, add 3d points to car-pointcloud
/*  for (int v=0; v<obj_map.rows; v++) {
  for (int u=0; u<obj_map.cols; u++) {
    int id = obj_map.at<uchar>(v,u);
    if (id==0) continue; // not a valid label
    std::shared_ptr<gvl::Car> car;
    try {
      car = cars.at(id);
    } catch (const std::out_of_range& oor) {
      car = std::make_shared<gvl::Car>();
      cars[id] = car;
    }

    { // Groundtruth disparity
      double disp = (uint16_t)disparity_gt.at<unsigned short>(v,u)/256.0;
      if (disp == 0) continue;
      double depth = (baseline*f)/disp;

      gvl::Point point;
      point.x = (u - c_x)/f*depth;
      point.y = (v - c_y)/f*depth;
      point.z = depth;
      gvl::generateColor(id, &point.r, &point.g, &point.b);
      car->pointcloud_gt->points.push_back(point);

    }

    { // Estimated disparity
      double disp = (uint16_t)disparity.at<unsigned short>(v,u)/256.0;
      if (disp == 0) continue;

      gvl::Point point;
      point.x = vertices.at<cv::Vec3d>(v,u)[0];
      point.y = vertices.at<cv::Vec3d>(v,u)[1];
      point.z = vertices.at<cv::Vec3d>(v,u)[2];
      gvl::generateColor(id, &point.r, &point.g, &point.b);
      car->pointcloud->points.push_back(point);
    }
  }
}*/
