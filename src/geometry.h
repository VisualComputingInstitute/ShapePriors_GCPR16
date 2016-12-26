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

#ifndef GVL_GEOMETRY_H_
#define GVL_GEOMETRY_H_

// Eigen includes
#include <Eigen/Core>

// C/C++ includes
#include <vector>
#include <memory>

// OpenCV includes
#include <opencv2/core/core.hpp>

// VTK includes
#include <vtkActor.h>
#include <vtkTransform.h>
#include <vtkSmartPointer.h>

// Own includes
#include "geometry.h"

namespace gvl {

struct Annotation {
  int id;
  std::string model_name; // name of the ply file
  double rotation_y; // rotation along y-axe, right hand turning (with clock seen from top) [-pi..pi]
  Eigen::Vector3d translation; // position relative to camera [m]
  double scale; // scale, should always be 1.0

  vtkActor* actor;
  vtkTransform* transform;

  Annotation () : actor(vtkActor::New()), transform(vtkTransform::New()) {
    id = 0;
  }
};

struct BoundingBox {

  BoundingBox(){
    rotation_y = 0;
    r=g=b=0;
    score=1.0;
    trackId = -1;
    type = "";
  }

  void transform(Eigen::Matrix4d pose) {
    Eigen::Vector4d p(x,y,z,1);
    p = pose*p;
    p /= p(3,0);
    x = p[0];
    y = p[1];
    z = p[2];
    double delta_rot_y = acos(pose(0,0)); // This seems to work
    rotation_y = rotation_y + delta_rot_y;

    return;
    /*det.pose = poses.at(frameId).inverse()*detection->pose;
    det.translation = det.pose.block<3,1>(0,3);

    Eigen::Vector3d one(0,0,-1);
    Eigen::Vector3d two = det.pose.block<3,3>(0,0)*one;
    two.normalize();

    double dot_prod = one.dot(two);
    auto cross_prod = one.cross(two);
    double angle = std::acos( dot_prod );
    det.rotation_y = angle;
    if (cross_prod[1]<0) det.rotation_y*=-1;*/
  }

  int frameId;
  int trackId;
  std::string type;
  double truncated;
  double occluded;
  double left, top, right, bottom; // 2d bounding box
  double height;  // dimension in y-direction [meters]
  double width;   // dimension in x-direction [meters]
  double length;  // dimension in z-direction [meters]
  double x,y,z;       // position of center of bottom [meters]
  double rotation_y;  // rotation around y [-pi..pi]
  double score;       // score of bounding box
  float r,g,b;        // color of bounding box [0-1]

  Eigen::Vector3d getTranslation() {
    return Eigen::Vector3d(x,y,z);
  }

  Eigen::Vector3d getSize() {
    return Eigen::Vector3d(width,height,length);
  }

  Eigen::Vector3f getColor() {
    return Eigen::Vector3f(r,g,b);
  }
};

typedef struct _Point {
  _Point(double xx, double yy, double zz){
    x = xx; y=yy; z=zz;
  }
  _Point(Eigen::Vector3d p) {
    x = p[0]; y = p[1]; z = p[2];
  }
  _Point(void) {
    x=0; y=0; z=0;
    r=0; y=0; z=0;
  }
  unsigned int u, v; // 2D coordinates from originating image
  double x, y, z; // 3D coordinates
  unsigned char r, g, b;  // RGB color-components [0 - 255]
  Eigen::Vector3d eigen(void){return Eigen::Vector3d(x,y,z);}
} Point;

typedef struct _Pointcloud {

  std::vector<gvl::Point> points;

  bool organized{false}; // wether index of points can be used to find 2d projection

  // Apply 4x4 tranformation to pointcloud
  void transform(const Eigen::Matrix4d& trafo) {
    for (auto& p : points) {
      double x = trafo(0,0)*p.x + trafo(0,1)*p.y + trafo(0,2)*p.z + trafo(0,3);
      double y = trafo(1,0)*p.x + trafo(1,1)*p.y + trafo(1,2)*p.z + trafo(1,3);
      double z = trafo(2,0)*p.x + trafo(2,1)*p.y + trafo(2,2)*p.z + trafo(2,3);
      p.x = x; p.y = y; p.z = z;
      //double w = trafo(3,0)*p.x + trafo(3,1)*p.y + trafo(3,2)*p.z + trafo(3,3);
      //p.x/=w; p.y/=w; p.z/w;
    }
  }

  // Compute average point
  gvl::Point average(void) {
    Eigen::Vector3d avg{0,0,0};
    for (auto p : this->points) avg += p.eigen();
    avg /= this->points.size();
    return gvl::Point{avg};
  }

  // Compute centroid
  gvl::Point centroid(void) {
    double min_x=900, max_x=-900;
    double min_y=900, max_y=-900;
    double min_z=900, max_z=-900;
    for (auto p : points) {
      if (p.x > max_x) max_x=p.x;
      if (p.y > max_y) max_y=p.y;
      if (p.z > max_z) max_z=p.z;
      if (p.x < min_x) min_x=p.x;
      if (p.y < min_y) min_y=p.y;
      if (p.z < min_z) min_z=p.z;
    }

    gvl::Point c;
    c.x = (max_x + min_x)/2.0;
    c.y = (max_y + min_y)/2.0;
    c.z = (max_z + min_z)/2.0;
    return c;
  }

  std::vector<Eigen::Vector3d> getVertices(void) {
    std::vector<Eigen::Vector3d> vertices;
    for (auto&p : points) {
      vertices.push_back(Eigen::Vector3d(p.x, p.y, p.z));
    }
    return vertices;
  }

  std::vector<Eigen::Vector3f> getColors(void) {
    std::vector<Eigen::Vector3f> colors;
    for (auto&p : points) {
      colors.push_back(Eigen::Vector3f(p.r, p.g, p.b));
    }
    return colors;
  }

} Pointcloud;

/**
 * @brief Compute Pose From Rotation and translation
 * @param rotation_y - angle around Y [-pi..pi]
 * @param translation - translation
 * @return
 */
Eigen::Matrix4d computePoseFromRotTransScale(const double rotation_y,
                                             const Eigen::Vector3d& translation,
                                             const double scale=1.0);

/**
 * @brief computeLookAtMatrix
 * @param[in] I - Look at point
 * @param[in] E - Eye point
 * @param[in] U - Up-Vector
 * @return Look-At-Matrix
 */
Eigen::Matrix4d computeLookAtMatrix(const Eigen::Vector3d& I,
                                    const Eigen::Vector3d& E,
                                    const Eigen::Vector3d& U);

/**
 * @brief computeIntrinsicMatrix
 * @param[in] f_x - focal length x
 * @param[in] f_y - focal length y
 * @param[in] p_x - principal point x
 * @param[in] p_y - principal point y
 * @param[in] s   - skew
 * @return Intrinsic Matrix K
 */
Eigen::Matrix3d computeIntrinsicMatrix(const double f_x,
                                       const double f_y,
                                       const double p_x,
                                       const double p_y,
                                       const double s=0);

/**
 * @brief computeTransformationToPlane. Given a groundplane,
 * compute transformation that aligns pointclouds groundplane
 * to the XZ-plane
 * @param[in] a - first normal component of plane
 * @param[in] b - second normal component of plane
 * @param[in] c - third normal component of plane
 * @param[in] d - distance from origin to plane
 * @return Transformation matrix
 */
Eigen::Matrix4d computeTransformationFromPlane(const double a,
                                               const double b,
                                               const double c,
                                               const double d);


/**
 * @brief computeVerticalDistanceToPlane
 * @param a
 * @param b
 * @param c
 * @param d
 * @param x - query position
 * @param z - query position
 * @return
 */
double computeVerticalDistanceToPlane(const double a,
                             const double b,
                             const double c,
                             const double d,
                                      double x,
                                      double z);

/**
 * @brief computeVerticesFromDisparity
 * @param[in] disparity
 * @param[in] K - intrinsics matrix
 * @param[in] b - baseline
 * @param[out] vertices: matric of same size as disaprity containing vertices
 */
void computeVerticesFromDisparity(const cv::Mat& disparity,
                                  const Eigen::Matrix3d& K,
                                  const double b,
                                  cv::Mat& vertices);

/**
 * @brief computeVerticesFromDisparity
 * @param vertices
 * @param K
 * @param b
 * @param disparity
 */
void computeDisparityFromVertices(const cv::Mat& vertices,
                                  const Eigen::Matrix3d& K,
                                  const double b,
                                  cv::Mat& disparity);

/**
 * @brief computeNormalsFromVertices
 * Use 8-neighborhood to compute normals.
 * Only use neighbors that are within a distance threshold.
 * Normal vector corresponds to eigenvector with smallest eigenvector.
 * All normals are reoriented to point towards camera/origin.
 * @param[in]  vertices
 * @param[out] normals
 */
void computeNormalsFromVertices(const cv::Mat& vertices, cv::Mat& normals);

/**
 * @brief computeNormalsFromVerticesSimple
 * use two neighbors (left, top?) and use cross product to compute normal.
 * @param vertices
 * @param normals
 */
void computeNormalsFromVerticesSimple(const cv::Mat& vertices, cv::Mat& normals);

/**
 * @brief computePointcloudFromVerticesAndColor
 * @param[in] vertices - CV_64FC3
 * @param[in] colors -  CV_8UC3
 * @param[out] pointcloud
 */
void computePointcloudFromVerticesAndColor(const cv::Mat &vertices,
                                           const cv::Mat &colors,
                                           gvl::Pointcloud& pointcloud);

/**
 * @brief compute axes aligned BoundingBoxFromPointcloud
 * @param pointcloud
 * @param bb
 */
void computeBoundingBoxFromPointcloud(const gvl::Pointcloud& pointcloud, gvl::BoundingBox& bb);

/**
 * @brief computeSquaredDistance
 * @param p1
 * @param p2
 * @return
 */
double computeSquaredDistance(const gvl::Point& p1, const gvl::Point& p2);

/**
 * @brief bilinearInterpolation
 * @param[in] values - corner values
 * @param[in] position - inside unit square
 * @return interpolated corner values
 */
double bilinearInterpolation(const Eigen::Vector4d& values,
                             const Eigen::Vector2d& position);

/**
 * @brief computeBoundingBoxFromAnnotation, default dimesional, Annotation contains only pose
 * @param[in] annotation
 * @param[out] bb
 */
void computeBoundingBoxFromAnnotation(const gvl::Annotation& annotation,
                                      const Eigen::Vector3f& color,
                                      gvl::BoundingBox& bb);

double computeMedian(std::vector<double>& values);

// This does NOT seem to work!!!
void transform_plane(const Eigen::Matrix4d& trafo, Eigen::Vector4d& p);

}

#endif // GVL_GEOMETRY_H_
