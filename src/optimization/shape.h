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

#ifndef GVL_OPTIMIZATION_SHAPE_H
#define GVL_OPTIMIZATION_SHAPE_H

// Eigen includes
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

// Ceres includes
#include "ceres/ceres.h"
#include "glog/logging.h"

// VIZ includes
#include <viz/viz.h>

// Own includes
#include "optimization/defines.h"
#include "tracking.h"
#include "shapespace.h"
#include "pose.h"
#include "utils.h"

namespace gvl {

double optimize_shape(gvl::Detection& detection, bool visualize = false, int* it = 0);

struct SDF_Residual {
  SDF_Residual(const double& worldX,
               const double& worldY,
               const double& worldZ,
               const double& d_std_dev) :
               worldX_(worldX),
               worldY_(worldY),
               worldZ_(worldZ),
               d_std_dev_(d_std_dev){}

  template <typename T> bool operator()(const T* const z,
                                        T* residual) const {
                                            
    Eigen::MatrixXd& S = gvl::ShapeSpacePCA::instance().S;
    Eigen::MatrixXd& V = gvl::ShapeSpacePCA::instance().V;
    Eigen::MatrixXd& mean = gvl::ShapeSpacePCA::instance().mean;
    std::vector<Eigen::Vector3i>& n_ids = gvl::ShapeSpacePCA::instance().n_ids;

    // Convert 3D-world-position into 3D-grid-index.
    int px = std::floor((worldX_ - limits[0])/bin_size);
    int py = std::floor((worldY_ - limits[2])/bin_size);
    int pz = std::floor((worldZ_ - limits[4])/bin_size);
    
    if( px < 0 || px >= dx-1 || py < 0 || py >= dy-1 || pz < 0 || pz >= dz-1 ) {
        residual[0] = T(3.0);
        return true;
    }
    
    // Position relative to voxel_origin
    double u = (worldX_ - limits[0]) - px*bin_size;
    double v = (worldY_ - limits[2]) - py*bin_size;
    double w = (worldZ_ - limits[4]) - pz*bin_size;

    // Compute TSDF values of 8 corners of voxel to which specified 3D point belongs to
    T tsdfs[8];
    for (int i=0; i<8; i++) {

      // Convert 3D-grid-index into 1D-vector-index
      int index = dz*dy* (px + n_ids.at(i)[0]) +
          dz*    (py + n_ids.at(i)[1]) +
          (pz + n_ids.at(i)[2]);

      // Compute backprojection for current corner
      tsdfs[i] = T(0.0);

      for (int j=0; j<r; j++) {
        tsdfs[i] += T(V(index,j)) * z[j];
      }
      // Add mean
      tsdfs[i] += T(mean(index,0));
    }

    // Normalizing to interval [0,1]
    u /= bin_size;
    v /= bin_size;
    w /= bin_size;

    // TSDF value for specified 3D point is interpolated from corner values
    T tri_lin_interpolation =
        T((1-u)*(1-v)*(1-w)) * tsdfs[0] +
        T((1-u)*(1-v)*(0+w)) * tsdfs[1] +
        T((1-u)*(0+v)*(1-w)) * tsdfs[2] +
        T((0+u)*(1-v)*(1-w)) * tsdfs[3] +
        T((0+u)*(1-v)*(0+w)) * tsdfs[4] +
        T((1-u)*(0+v)*(0+w)) * tsdfs[5] +
        T((0+u)*(0+v)*(1-w)) * tsdfs[6] +
        T((0+u)*(0+v)*(0+w)) * tsdfs[7];
    residual[0] = T(tri_lin_interpolation/d_std_dev_);

    return true;
  }

private:
  const double worldX_; // world point x
  const double worldY_; // world point y
  const double worldZ_; // world point z
  const double d_std_dev_;
};

struct Regularizer_z {
  Regularizer_z(double sqrt_number_of_points_, double lambda_shape_reg_) :
    sqrt_number_of_points(sqrt_number_of_points_),
    lambda_shape_reg(lambda_shape_reg_){}

  template <typename T> bool operator()(const T* const z,
                                        T* residual) const {
                                            
    Eigen::MatrixXd& S = gvl::ShapeSpacePCA::instance().S;
                                            
    // Iterate over manifold coordinates
    for (int i=0; i<r; i++) {
      double sigma = S(i,0);
      residual[i] = z[i] / T(sigma) * T(sqrt_number_of_points) * T(lambda_shape_reg);
    }
    return true;
  }
private:
  const double sqrt_number_of_points;
  double lambda_shape_reg;
};

class ShapeVisualizerCallback: public ceres::IterationCallback {
public:
  gvl::Detection& detection;
  viz::Visualization& visualization;
  int* iteration;

  ShapeVisualizerCallback(gvl::Detection& det, viz::Visualization& vis, int* it) :
  detection(det), visualization(vis), iteration(it) {}

  virtual ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
      
    Eigen::MatrixXd& S = gvl::ShapeSpacePCA::instance().S;
    Eigen::MatrixXd& V = gvl::ShapeSpacePCA::instance().V;
    Eigen::MatrixXd& mean = gvl::ShapeSpacePCA::instance().mean;
    std::vector<Eigen::Vector3i>& n_ids = gvl::ShapeSpacePCA::instance().n_ids;
      
    detection.shape = V.block<p, r>(0,0)*detection.z + mean;

    Eigen::Vector3d centroid = detection.pointcloud->centroid().eigen();
    Eigen::Vector3d norm = centroid; norm.normalize();
    Eigen::Vector3d from = centroid - 10*norm + Eigen::Vector3d(5,-5,0);
    Eigen::Vector3d to = centroid;
    visualization.lookFromAt(from,to);
    visualization.hideAllActors();

    std::vector<Eigen::Vector3d> vertices, normals;
    gvl::Pointcloud tsdf_pointcloud;
    //gvl::visualizeGradients(detection,vertices,normals,tsdf_pointcloud);
    visualization.addPointcloud(tsdf_pointcloud.getVertices(), tsdf_pointcloud.getColors());

    //visualizer->addNormals(vertices, normals);
    visualization.addZeroSurfaceFromGrid(detection.shape, detection.pose, Eigen::Vector3f(0, 0, 0), true);
    //visualization.show();

    (*iteration)++;
    std::string path = "/Volumes/STICKY/GCPR_presentation/opt_steps/"+gvl::zfill(*iteration,8)+".png";
    visualization.takeScreenshot2(path);
    return ceres::SOLVER_CONTINUE;
  }
};

}

#endif
