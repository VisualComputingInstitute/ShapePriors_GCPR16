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

#ifndef GVL_TRACKING_H
#define GVL_TRACKING_H

// Eigen includes
#include <Eigen/Core>

// C/C++ includes
#include <string>
#include <unordered_map>

// Own includes
#include "geometry.h"

namespace gvl {

struct Detection {
  unsigned int frame_id;  // In which frame detected
  int track_id;
  Eigen::Matrix4d pose;   // Rotation + translation
  Eigen::Matrix4d poseInit;   // Rotation + translation init
  Eigen::Vector3d translation; // relative to camera [m]
  double rotation_y; // Rotation around Y-axe, zero is (0,0,-1)
                     // e.g. car faceing down neg. z-axe [-pi..pi]
                     // when seen form top, clockwise rotation
  //double vel_trans;  // translational velicty [scalar]
  //double vel_angle;  // angular velocity [scalar]
  Eigen::Vector2d velocities; // translation and angular velocity
  Eigen::MatrixXd z;      // Reconstructed shape of this detection
  Eigen::MatrixXd shape;  // The reconstruced shape from z

  std::shared_ptr<gvl::Pointcloud> pointcloud; // Depth measurements corresponding to his pointqq
  gvl::Pointcloud pointcloudInit; // Depth measurements corresponding to his pointqq
  std::shared_ptr<gvl::BoundingBox> boundingbox; // the original 3D DOP detection
  bool opt_successful;    // wether ceres optimization was successfull for only this detection, currenly tried for pose optimization
  bool associated_to_track; // wether this detection is associated to a track 0 usefull to start a new track
  bool interpolated; // wether this detection was made up by tracked by bridging and not from 3dop detector

  Detection(void) {
    boundingbox = std::make_shared<gvl::BoundingBox>();
    pointcloud = std::make_shared<gvl::Pointcloud>();
    opt_successful = false;
    associated_to_track = false;
    interpolated = false;
  }


};

struct Track {
  std::map<int, std::shared_ptr<gvl::Detection>> detections;
  Eigen::MatrixXd z;
  Eigen::MatrixXd shape;
  double vel_trans;
  double vel_rot;
  bool dynamic = true; // wether object is moving or not
  bool pose_optimization_successful = false;
  bool shape_optimization_successful = false;

  std::vector<Eigen::Vector3d> getTrajectory(int t=INT_MAX)
  {
    std::vector<Eigen::Vector3d> traj;
    for (auto& det_entry : this->detections) {
      auto& detection = det_entry.second;
      if (detection->frame_id > t) break;
      traj.push_back(detection->translation+Eigen::Vector3d(0,-0.5,0));
    }
    return traj;
  }

  std::vector<Eigen::Vector3d> getTrajectoryInit(int t=INT_MAX)
  {
    std::vector<Eigen::Vector3d> traj;
    for (auto& det_entry : this->detections) {
      auto& detection = det_entry.second;
      if (detection->frame_id > t) break;
      traj.push_back(detection->poseInit.block<3,1>(0,3)+Eigen::Vector3d(0,-1.5f,0));
    }
    return traj;
  }
};

/**
 * @brief computes intersection over union in 2d for two given bounding boxes
 * @param bb1
 * @param bb2
 * @return
 */
double compute_IoU_2D(gvl::BoundingBox& bb1, gvl::BoundingBox& bb2);

/**
 * @brief greedy solution for minimal cost associations
 * @param association_costs
 * @param associations
 */
void solve_associations_greedy(Eigen::MatrixXd& association_costs,
                               Eigen::MatrixXi& associations);

}

Eigen::Vector3d getTrajectory(void);

/*void motion_model_velocity(gvl::Detection& curr_detection, gvl::Detection& pred_detection) {

  //double my = 1.0/2.0*(x-x)
}*/
/*struct Detection {

  Detection(){
    rotation_y = 0;
    r=g=b=0;
    score=1.0;
  }

  double left, top, right, bottom; // 2d bounding box
  double height;  // dimension in y-direction [meters]
  double width;   // dimension in x-direction [meters]
  double length;  // dimension in z-direction [meters]
  double x,y,z;       // position of center of bottom [meters]
  double rotation_y;  // rotation around y [-pi..pi]
  double score;       // score of bounding box
  float r,g,b;        // color of bounding box [0-1]
};

struct Track {
  Eigen::MatrixXd z;
  std::unordered_map<int, Detection> detections;
};*/

#endif // GVL_TRACKING_H
