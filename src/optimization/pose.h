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

#ifndef GVL_OPTIMIZATION_POSE_H
#define GVL_OPTIMIZATION_POSE_H

// Eigen includes
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

// Ceres includes
#include <ceres/ceres.h>
#include <glog/logging.h>

// VIZ includes
#include <viz/viz.h>

// Own includes
#include "optimization/defines.h"
#include "geometry.h"
#include "tracking.h"
#include "shapespace.h"
#include "utils.h"

namespace gvl {

double optimize_pose(gvl::Detection& detection, bool visualize = false, int* it = 0);

void visualizeGradients(const gvl::Detection& detection,
                        std::vector<Eigen::Vector3d>& vertices_poseOpt,
                        std::vector<Eigen::Vector3d>& normals_poseOpt,
                        gvl::Pointcloud& pc_poseOpt);

class PoseCostFunction : public ceres::SizedCostFunction<1, 3, 1> {
public:                                              //  ^  ^  ^  ^  ^
                                                     //  |  |_ dimension of parameter 1, etc...
  /**                                                //  |_ dimension of residual
   *
   * @brief PoseCostFunction - constructor
   * @param[in] world - point in coordinate system of current detection
   * @param[in] reconstruction
   */
  PoseCostFunction(Eigen::Vector4d& world,
                   Eigen::MatrixXd* reconstruction,
                   double& depth_std_dev):
                   world_(world),
                   reconstruction_(reconstruction),
                   depth_std_dev_(depth_std_dev){}

  virtual ~PoseCostFunction() {}

  virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const {

    const double& trans_x = parameters[0][0];
    const double& trans_y = parameters[0][1];
    const double& trans_z = parameters[0][2];
    const double& rot_y   = parameters[1][0];
    
    std::vector<Eigen::Vector3i>& n_ids = gvl::ShapeSpacePCA::instance().n_ids;
    
    // Convert world point to local point
    Eigen::Matrix4d pose = gvl::computePoseFromRotTransScale(rot_y, Eigen::Vector3d(trans_x, trans_y, trans_z), 1.0);
    Eigen::Matrix4d pose_inv = pose.inverse();
    Eigen::Vector4d local = pose_inv*world_; //transform to origin of current pose
    local/=local[3];
    
//     double xrescale = std::max( local[0] / _limits[0], local[0] / _limits[1] );
//     double yrescale = std::max( local[1] / _limits[2], local[1] / _limits[3] );
//     double zrescale = std::max( local[2] / _limits[4], local[2] / _limits[5] );
// 
//     double rescale = std::max( xrescale, std::max( yrescale, zrescale ) );
//     if( rescale > 1.0 )
//         local /= rescale;

    // Convert 3D-local-position into 3D-grid-index.
    int px = std::floor((local[0] - limits[0]) / bin_size);
    int py = std::floor((local[1] - limits[2]) / bin_size);
    int pz = std::floor((local[2] - limits[4]) / bin_size);

//      int px = std::max( 1.0, std::min( _dx-2.0, std::floor((local[0] - _limits[0]) / _bin_size) ) );
//      int py = std::max( 1.0, std::min( _dy-2.0, std::floor((local[1] - _limits[2]) / _bin_size) ) );
//      int pz = std::max( 1.0, std::min( _dz-2.0, std::floor((local[2] - _limits[4]) / _bin_size) ) );

    // Position relative to voxel_origin
    double u = (local[0] - limits[0]) - px*bin_size;
    double v = (local[1] - limits[2]) - py*bin_size;
    double w = (local[2] - limits[4]) - pz*bin_size;

    // Normalizing to interval [0,1]
    u/=bin_size;
    v/=bin_size;
    w/=bin_size;

    //std::cout << std::endl << "Voxel Index: " << px << " " << py << " " << pz << std::endl;

    // Check if we are inside TSDF grid
    bool inside_grid = true; // wether we are inside the grid for which TSDF are defined
    if (px<1 || px>dx-2 ||
        py<1 || py>dy-2 ||
        pz<1 || pz>dz-2) {
        inside_grid = false;
    }

    //std::cout << "Inside grid = " << (inside_grid) << std::endl;

    // Compute TSDF values of 8 corners of voxel to which specified 3D point belongs to
    double tsdfs[8] = {0,0,0,0,0,0,0,0};
    for (int i=0; i<8 && inside_grid; i++) {
      // Convert 3D-grid-index into 1D-vector-index.
      int index = dz*dy* (px + n_ids.at(i)[0]) +
          dz*    (py + n_ids.at(i)[1]) +
          (pz + n_ids.at(i)[2]);
      tsdfs[i] = (*reconstruction_)(index,0);
    }

    // TSDF value for specified 3D point is interpolated from corner values
    double tri_lin_interpolation = 0;
    
    if( inside_grid ) {
        tri_lin_interpolation =
            ((1-u)*(1-v)*(1-w)) * tsdfs[0] +
            ((1-u)*(1-v)*(0+w)) * tsdfs[1] +
            ((1-u)*(0+v)*(1-w)) * tsdfs[2] +
            ((0+u)*(1-v)*(1-w)) * tsdfs[3] +
            ((0+u)*(1-v)*(0+w)) * tsdfs[4] +
            ((1-u)*(0+v)*(0+w)) * tsdfs[5] +
            ((0+u)*(0+v)*(1-w)) * tsdfs[6] +
            ((0+u)*(0+v)*(0+w)) * tsdfs[7];
    } else {
      Eigen::Vector3d vec(local[0],local[1],local[2]);
      tri_lin_interpolation = vec.norm() + 3.0;
    }

    Eigen::Vector3d dist_grad(0,0,0);
    if (jacobians != NULL && jacobians[0] != NULL) {

      double dist_grad_x = 0;
      double dist_grad_y = 0;
      double dist_grad_z = 0;
      if( inside_grid ) {

        dist_grad_x = gvl::bilinearInterpolation(Eigen::Vector4d(
            tsdfs[3]-tsdfs[0],
            tsdfs[4]-tsdfs[1],
            tsdfs[6]-tsdfs[2],
            tsdfs[7]-tsdfs[5]),
            Eigen::Vector2d(w,v)) / bin_size;

        dist_grad_y = gvl::bilinearInterpolation(Eigen::Vector4d(
            tsdfs[2]-tsdfs[0],
            tsdfs[6]-tsdfs[3],
            tsdfs[5]-tsdfs[1],
            tsdfs[7]-tsdfs[4]),
            Eigen::Vector2d(u,w)) / bin_size;

        dist_grad_z = gvl::bilinearInterpolation(Eigen::Vector4d(
            tsdfs[1]-tsdfs[0],
            tsdfs[4]-tsdfs[3],
            tsdfs[5]-tsdfs[2],
            tsdfs[7]-tsdfs[6]),
            Eigen::Vector2d(u,v)) / bin_size;
            
      } else {
        Eigen::Vector3d vec(local[0],local[1],local[2]);
        double norm = vec.norm();
        dist_grad_x = vec[0]/norm;
        dist_grad_y = vec[1]/norm;
        dist_grad_z = vec[2]/norm;
      }

      dist_grad[0] = dist_grad_x;
      dist_grad[1] = dist_grad_y;
      dist_grad[2] = dist_grad_z;

      Eigen::Matrix3d deriv_rot = Eigen::Matrix3d::Identity();
      deriv_rot(0,0) = -sin(rot_y); deriv_rot(0,2) = +cos(rot_y);
      deriv_rot(2,0) = -cos(rot_y); deriv_rot(2,2) = -sin(rot_y);

      //Eigen::Matrix3d deriv_trans = Eigen::Matrix3d::Identity();
      Eigen::MatrixXd deriv_all(3,4); // in R^{3,4}
      deriv_all.block<3,3>(0,0) = -pose.block<3,3>(0,0).transpose();// * deriv_trans;
      deriv_all.block<3,1>(0,3) = -pose.block<3,3>(0,0).transpose() * (deriv_rot * local.block<3,1>(0,0));

      Eigen::MatrixXd jacob = dist_grad.transpose() * deriv_all; // in R^{1x4}
      jacobians[0][0] = jacob(0,0)/depth_std_dev_;
      jacobians[0][1] = jacob(0,1)/depth_std_dev_;
      jacobians[0][2] = jacob(0,2)/depth_std_dev_;
      jacobians[0][3] = jacob(0,3)/depth_std_dev_;
    }

    residuals[0] = tri_lin_interpolation/depth_std_dev_;
    //std::cout << "depth_std_dev_" << depth_std_dev_ <<std::endl;
    //std::cout << "Final residual = " << residuals[0] << std::endl;

    return true;
  }
private:
  Eigen::Vector4d world_;
  Eigen::MatrixXd* reconstruction_;
  double depth_std_dev_;
};

struct Regularizer_transY {
  Regularizer_transY(double number_of_points,
                     double distance_to_ground,
                     double lambda_pose_reg_trans_y) :
    _number_of_points(number_of_points),
    _distance_to_ground(distance_to_ground),
    _lambda_pose_reg_trans_y(lambda_pose_reg_trans_y){}
  template <typename T> bool operator()(const T* const trans_y, T* residual) const {
    //residual[0] = T(std::sqrt(lambda_pose_reg_trans_y)) * T(trans_y[0]-T(10));
    //residual[0] = T(std::sqrt(lambda_pose_reg_trans_y)) * T(trans_y[0]-_distance_to_ground);
    residual[0] = T(std::sqrt(_number_of_points*_lambda_pose_reg_trans_y)) * T(trans_y[0]-_distance_to_ground);
    return true;
  }
private:
  const double _number_of_points;
  const double _distance_to_ground;
  const double _lambda_pose_reg_trans_y;
};

struct NormalPrior {
  NormalPrior(double mean,
              double variance) :
    mean_(mean),
    variance_(variance){}

  template <typename T> bool operator()(const T* const t, T* residual) const {
    residual[0] = T(t[1]-mean_) / T(variance_);
    return true;
  }
private:
  const double mean_;
  const double variance_;
};

class PoseVisualizerCallback: public ceres::IterationCallback {
public:
  gvl::Detection& detection;
  viz::Visualization& visualization;
  int* iteration;

  PoseVisualizerCallback(gvl::Detection& det, viz::Visualization& vis, int* it):
    detection(det), visualization(vis), iteration(it){}

  virtual ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
    detection.pose = gvl::computePoseFromRotTransScale(detection.rotation_y, detection.translation);
    std::vector<Eigen::Vector3d> vertices, normals;
    viz::Pointcloud tsdf_pointcloud;
    //visualizeGradients(detection, vertices, normals, tsdf_pointcloud);
    visualization.hideAllActors();
    //visualization.addPointcloud(tsdf_pointcloud);
    //visualization.addNormals(vertices, normals);
    visualization.addZeroSurfaceFromGrid(detection.shape, detection.pose, Eigen::Vector3f(0,0,0));
    //visualization.show();
    (*iteration)++;
    std::string path = "/Volumes/STICKY/GCPR_presentation/opt_steps/"+gvl::zfill(*iteration,8)+".png";
    visualization.takeScreenshot2(path);

    return ceres::SOLVER_CONTINUE;
  }
};

}

#endif
