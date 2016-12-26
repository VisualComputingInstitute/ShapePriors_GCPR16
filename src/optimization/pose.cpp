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

#include "pose.h"

// Own includes
#include "viz/viz.h"
#include "utils.h"

namespace gvl {

double optimize_pose(gvl::Detection& detection, bool visualize, int* it) {
  // Create ceres optimization problem
  ceres::Problem pose_problem;

  // Iterate over the points and add residual block
  for (unsigned int i = 0; i<detection.pointcloud->points.size(); i+=1) {
    gvl::Point p = detection.pointcloud->points.at(i); // points in world coorindates
    Eigen::Vector4d P(p.x, p.y, p.z, 1.0);
    double d_std_dev = 1; // simple assumption
    double N = detection.pointcloud->points.size();
    ceres::CostFunction* pose_cost_function = new PoseCostFunction(P, &detection.shape, d_std_dev);
    pose_problem.AddResidualBlock(pose_cost_function,
                                  new ceres::ScaledLoss(new ceres::HuberLoss(0.1),1.0/(N*0.03),ceres::TAKE_OWNERSHIP),
                                  detection.translation.data(), &(detection.rotation_y));
  }

  double mean = detection.translation[1];
  double variance = std::sqrt(3);
  ceres::CostFunction* trans_y_prior =
      new ceres::AutoDiffCostFunction<gvl::NormalPrior,1,3>
      (new gvl::NormalPrior(mean, variance));
  pose_problem.AddResidualBlock(trans_y_prior,
                                       NULL,
                                       detection.translation.data() );

  viz::Visualization visualizer(1024,768);
  //visualizer.addAxes();

  Eigen::Vector3d centroid = detection.pointcloud->centroid().eigen();
  Eigen::Vector3d norm = centroid; norm.normalize();
  Eigen::Vector3d from = centroid - 10*norm + Eigen::Vector3d(5,-5,0);
  Eigen::Vector3d to = centroid;
  visualizer.lookFromAt(from,to);

  // Set solver options
  ceres::Solver::Options pose_options;
  pose_options.max_num_iterations = 100;
  bool bfgs = false;
  if( bfgs ) {
    pose_options.minimizer_type = ceres::LINE_SEARCH;
    pose_options.line_search_direction_type = ceres::LBFGS;
  }
  else {
    pose_options.minimizer_type = ceres::TRUST_REGION; // Trust region options
    pose_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT; // Trust region options
    pose_options.linear_solver_type = ceres::DENSE_QR;
  }
  pose_options.update_state_every_iteration = true;

  PoseVisualizerCallback callback(detection, visualizer, it);
  if (visualize) {
    pose_options.callbacks.push_back(&callback);
    pose_options.minimizer_progress_to_stdout = true;
  }

  // Solve
  ceres::Solver::Summary summary;
  Solve(pose_options, &pose_problem, &summary);
  if (visualize) std::cout << summary.FullReport() << std::endl;
  else std::cout << summary.BriefReport() << std::endl;

  // Applying new pose
  detection.pose = gvl::computePoseFromRotTransScale(detection.rotation_y, detection.translation);
  detection.opt_successful = summary.IsSolutionUsable();

  return summary.final_cost;
}

void visualizeGradients(const gvl::Detection& detection,
                        std::vector<Eigen::Vector3d>& vertices_poseOpt,
                        std::vector<Eigen::Vector3d>& normals_poseOpt,
                        gvl::Pointcloud& pc_poseOpt) {

  Eigen::MatrixXd& S = gvl::ShapeSpacePCA::instance().S;
  Eigen::MatrixXd& V = gvl::ShapeSpacePCA::instance().V;
  Eigen::MatrixXd& mean = gvl::ShapeSpacePCA::instance().mean;
  std::vector<Eigen::Vector3i>& n_ids = gvl::ShapeSpacePCA::instance().n_ids;

  Eigen::Matrix4d pose = detection.pose;
  Eigen::Matrix4d pose_inv = pose.inverse();

  Eigen::Vector4d gradient(0,0,0,0);
  double gradient_norm = 0;
  double cost = 0;

  for (unsigned int i=0; i<detection.pointcloud->points.size(); i++){
      
     
      
    gvl::Point& p = detection.pointcloud->points.at(i); // points in world coorindates
    Eigen::Vector4d P(p.x, p.y, p.z, 1.0);
    Eigen::Vector4d local = pose_inv*P; //transform to origin of current pose
    local/=local[3];

    // Convert 3D-local-position into 3D-grid-index.
    int px = std::floor((local[0] - limits[0]) / bin_size);
    int py = std::floor((local[1] - limits[2]) / bin_size);
    int pz = std::floor((local[2] - limits[4]) / bin_size);

    // Position relative to voxel_origin
    double u = (local[0] - limits[0]) - px*bin_size;
    double v = (local[1] - limits[2]) - py*bin_size;
    double w = (local[2] - limits[4]) - pz*bin_size;

    // Normalizing to interval [0,1]
    u/=bin_size;
    v/=bin_size;
    w/=bin_size;

    // Check if we are inside TSDF grid
    bool inside_grid = true; // wether we are inside the grid for which TSDF are defined
    if (px<1 || px>dx-2 ||
        py<1 || py>dy-2 ||
        pz<1 || pz>dz-2) {
      inside_grid = false;
    }

    double tsdfs[8] = {0,0,0,0,0,0,0,0};
    for (int i=0; i<8 && inside_grid; i++) {
      // Convert 3D-grid-index into 1D-vector-index.
      int index = dz*dy* (px + n_ids.at(i)[0]) +
          dz*    (py + n_ids.at(i)[1]) +
          (pz + n_ids.at(i)[2]);
      tsdfs[i] = detection.shape(index,0);
    }

    // TSDF value for specified 3D point is interpolated from corner values
    double tri_lin_interpolation =
        ((1-u)*(1-v)*(1-w)) * tsdfs[0] +
        ((1-u)*(1-v)*(0+w)) * tsdfs[1] +
        ((1-u)*(0+v)*(1-w)) * tsdfs[2] +
        ((0+u)*(1-v)*(1-w)) * tsdfs[3] +
        ((0+u)*(1-v)*(0+w)) * tsdfs[4] +
        ((1-u)*(0+v)*(0+w)) * tsdfs[5] +
        ((0+u)*(0+v)*(1-w)) * tsdfs[6] +
        ((0+u)*(0+v)*(0+w)) * tsdfs[7];
    if (!inside_grid) {
      Eigen::Vector3d vec(local[0],local[1],local[2]);
      tri_lin_interpolation = vec.norm() + 3.0;
    }

    cost += tri_lin_interpolation*tri_lin_interpolation;

    Eigen::Vector3d jetColor = gvl::applyJetColorMap(tri_lin_interpolation);
    gvl::Point local_point;
    local_point.x = P[0];
    local_point.y = P[1];
    local_point.z = P[2];
    local_point.r = jetColor[0]*255;
    local_point.g = jetColor[1]*255;
    local_point.b = jetColor[2]*255;
    if (!inside_grid) {
//         std::cout << "outside: " << local.transpose() << std::endl;
        local_point.r = 255;
        local_point.g = 0;
        local_point.b = 255;
    }

    Eigen::Vector3d dist_grad(0,0,0);
    double dist_grad_x = gvl::bilinearInterpolation(Eigen::Vector4d(
        tsdfs[3]-tsdfs[0],
        tsdfs[4]-tsdfs[1],
        tsdfs[6]-tsdfs[2],
        tsdfs[7]-tsdfs[5]),
        Eigen::Vector2d(w,v));

    double dist_grad_y = gvl::bilinearInterpolation(Eigen::Vector4d(
        tsdfs[2]-tsdfs[0],
        tsdfs[6]-tsdfs[3],
        tsdfs[5]-tsdfs[1],
        tsdfs[7]-tsdfs[4]),
        Eigen::Vector2d(u,w));

    double dist_grad_z = gvl::bilinearInterpolation(Eigen::Vector4d(
        tsdfs[1]-tsdfs[0],
        tsdfs[4]-tsdfs[3],
        tsdfs[5]-tsdfs[2],
        tsdfs[7]-tsdfs[6]),
        Eigen::Vector2d(u,v));

    dist_grad[0] = dist_grad_x / bin_size;
    dist_grad[1] = dist_grad_y / bin_size;
    dist_grad[2] = dist_grad_z / bin_size;

    if (!inside_grid) {
      Eigen::Vector3d vec(local[0],local[1],local[2]);
      double norm = vec.norm();
      dist_grad[0] = vec[0]/norm;
      dist_grad[1] = vec[1]/norm;
      dist_grad[2] = vec[2]/norm;
    }

    dist_grad = dist_grad*-1;

    Eigen::Vector4d zero = pose*Eigen::Vector4d(0,0,0,1);
    Eigen::Vector4d asd = pose*Eigen::Vector4d(dist_grad[0],dist_grad[1],dist_grad[2],1);
    asd/=asd[3]; zero/=zero[3];
    dist_grad = Eigen::Vector3d(asd[0]-zero[0], asd[1]-zero[1], asd[2]-zero[2]);

    // Computation of jacobian, actually we do not visualize the jacobian (how would one do that anyhow??) anly the gradient of the TSDF
    Eigen::Vector3d deriv_rot(0,0,0);
    double f = 1.0;
    double rot_y = detection.rotation_y;
    deriv_rot[0] = -local[0]*sin(rot_y*f) + local[2]*cos(rot_y*f);
    deriv_rot[1] = 0.0f;
    deriv_rot[2] = -local[0]*cos(rot_y*f) - local[2]*sin(rot_y*f);

    Eigen::Matrix3d deriv_trans = Eigen::Matrix3d::Identity();

    Eigen::MatrixXd deriv_all(3,4);
    deriv_all.block<3,3>(0,0) = deriv_trans;
    deriv_all.block<3,1>(0,3) = deriv_rot;
    Eigen::MatrixXd jacob = dist_grad.transpose()*deriv_all; // in R^{1x4}
    gradient += jacob.transpose();
    gradient_norm += jacob.transpose().norm();

    // _poseOpt stuff is for visualization
    pc_poseOpt.points.push_back(local_point);
    vertices_poseOpt.push_back( Eigen::Vector3d(P[0], P[1], P[2]) );
    normals_poseOpt.push_back( dist_grad );
  }
  //std::cout << "Final cost = "<< 0.5*cost << std::endl;
  int n = detection.pointcloud->points.size();
  //std::cout << "Gradient sum = " << gradient.transpose() << std::endl;
  //std::cout << "Gradient = " << gradient_norm << std::endl;
  return;
}


}
