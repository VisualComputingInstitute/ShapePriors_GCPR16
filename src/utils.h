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

#ifndef GVL_UTILS_H
#define GVL_UTILS_H
#pragma once

#include <time.h>

// Eigen includes
#include <Eigen/Core>

// C/C++ includes
#include <string>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// VIZ includes
#include <viz/viz.h>

// Own includes
#include "geometry.h"
#include "tracking.h"
#include "optimization/defines.h"

namespace gvl {

/**
   * @brief applyJetColorMap
   * @param v
   * @return
   */
Eigen::Vector3d applyJetColorMap(double v);

// Display image after applying a colormap to it
void imshow(const std::string& windowName, const cv::Mat& image);

// Generate image after applying a colormap to it
cv::Mat generateColorMap(const cv::Mat& intput);

// Returns for specifed ID an rgb-color in [0,255]^3
void generateColor(unsigned int id, float* color);
void generateColor(const unsigned int id, unsigned char* r, unsigned char* g, unsigned char *b);
Eigen::Vector3f generateColor(unsigned int id);

// Returns name of file from specified path without extension
std::string extractFilename(const std::string& path);
std::string getTimestamp(void);

/**
   * @brief zfill
   * @param input
   * @param zeros
   * @return
   */
std::string zfill(int input, int zeros);

/**
   * @brief plot
   * @param values
   */
void plot(std::vector<double> values);

/**
   * @brief populate_neighborhood
   * @param n_ids
   */
void populate_neighborhood(std::vector<Eigen::Vector3i>& n_ids);

gvl::BoundingBox computeBoundingboxFromDetection(const gvl::Detection& detection, const Eigen::Vector3f& color);

void show_poses(std::vector<Eigen::Matrix4d>& poses);

void remove_road_plane_from_pointcloud(gvl::Pointcloud& pointcloud,
                                       Eigen::Vector4d road_plane,
                                       gvl::Pointcloud& pointcloud_filtered);

void keep_road_plane_from_pointcloud(gvl::Pointcloud& pointcloud,
                                       Eigen::Vector4d road_plane,
                                       gvl::Pointcloud& pointcloud_filtered);

/**
 * @brief extract_points_to_detection takes 3D points from pointcloud to detection
 * @param color_image - color image
 * @param pointcloud_scene - scene pointcloud in world coordinates
 * @param detection - detection
 * @param pose - pose of camera in world coordinates
 * @param distance_threshold - distance between points
 */
void extract_points_to_detection(cv::Mat& color_image,
                                 gvl::Pointcloud& pointcloud_scene,
                                 gvl::Detection& detection,
                                 Eigen::Matrix4d& pose,
                                 double distance_threshold);

void remove_cars_from_pointcloud(cv::Mat& cars_depth,
                                 gvl::Pointcloud& pointcloud_scene);

void draw_2d_detections(cv::Mat &image_2, std::vector<gvl::BoundingBox> &detections, bool tracking=true);

void show_2d_detections(cv::Mat &image_2, std::vector<gvl::BoundingBox> &detections);

}

#endif // GVL_UTILS_H
