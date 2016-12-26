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

// Eigen includes
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <Eigen/SVD>

// C/C++ includes
#include <iostream>
#include <tuple>

// Own includes
#include "geometry.h"

namespace gvl {

// See http://www.cs.virginia.edu/~gfx/Courses/1999/intro.fall99.html/lookat.html
// for implementation details.
Eigen::Matrix4d computeLookAtMatrix(const Eigen::Vector3d& I,
                                    const Eigen::Vector3d& E,
                                    const Eigen::Vector3d& U)
{
  Eigen::Vector3d F = I-E;
  Eigen::Vector3d f = F/F.norm();
  Eigen::Vector3d u = U/U.norm();
  Eigen::Vector3d s = f.cross(u);
  Eigen::Vector3d w = s.cross(f);

  Eigen::Matrix<double,4,4> M = Eigen::Matrix<double,4,4>::Identity();
  M(0,0) = s(0); M(0,1) = s(1); M(0,2) = s(2);
  M(1,0) = w(0); M(1,1) = w(1); M(1,2) = w(2);
  M(2,0) = -f(0); M(2,1) = -f(1); M(2,2) = -f(2);

  Eigen::Matrix<double,4,4> T = Eigen::Matrix<double,4,4>::Identity();
  T(0,3) = -E(0); T(1,3) = -E(1); T(2,3) = -E(2);

  return M*T;
}

Eigen::Matrix3d computeIntrinsicMatrix(const double f_x,
                                       const double f_y,
                                       const double p_x,
                                       const double p_y,
                                       const double s)
{
  Eigen::Matrix<double,3,3> K = Eigen::Matrix<double,3,3>::Identity();
  K(0,0) = f_x;
  K(1,1) = f_y;
  K(0,2) = p_x;
  K(1,2) = p_y;
  K(0,1) = s;
  return K;
}

// See http://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
Eigen::Matrix4d computeTransformationFromPlane(const double a,
                                               const double b,
                                               const double c,
                                               const double d)
{
  Eigen::Vector3d n; n << a,b,c;
  Eigen::Vector3d m; m << 0,-1,0;
  Eigen::Vector3d v; v = n.cross(m);
  double sin = v.norm(); // sine of angle
  double ccos = n.dot(m); // cosine of angle
  Eigen::Matrix3d v_x; v_x << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
  Eigen::Matrix3d R; R = Eigen::Matrix3d::Identity() + v_x + v_x*v_x*(1-ccos)/(sin*sin);
  Eigen::Vector3d t; t << 0, -d, 0;
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3,3>(0,0) = R;
  T.block<3,1>(0,3) = t;
  return T;
}

void computeVerticesFromDisparity(const cv::Mat& disparity,
                                  const Eigen::Matrix3d& K,
                                  const double b,
                                  cv::Mat& vertices)
{
  // Init variables
  const double max_depth = 1000;
  const double min_depth = 1;
  const double& baseline = b;
  const double& f = K(0,0);
  Eigen::Matrix3d Kinv = K.inverse();
  vertices = cv::Mat(disparity.rows, disparity.cols, CV_64FC3, cv::Scalar(0,0,0));

  // Iterate over disparity pixels, compute corresponding vertex
  for (int v=0; v<disparity.rows; v++) {
    for (int u=0; u<disparity.cols; u++) {
      // Compute depth
      double disp = (double)((uint16_t)(disparity.at<unsigned short>(v,u)))/256.0;
      double depth = (baseline*f)/(disp); //256 according to spspstereo
      if (depth < min_depth || depth > max_depth) continue;

      // Compute vertice
      //Eigen::Vector3d v3d = depth*Kinv*Eigen::Vector3d(u,v,1.0);
      vertices.at<cv::Vec3d>(v,u)[0] = depth*(Kinv(0,0)*u + Kinv(0,1)*v + Kinv(0,2));
      vertices.at<cv::Vec3d>(v,u)[1] = depth*(Kinv(1,0)*u + Kinv(1,1)*v + Kinv(1,2));
      vertices.at<cv::Vec3d>(v,u)[2] = depth*(Kinv(2,0)*u + Kinv(2,1)*v + Kinv(2,2));
    }
  }
}

void computeDisparityFromVertices(const cv::Mat& vertices,
                                  const Eigen::Matrix3d& K,
                                  const double b,
                                  cv::Mat& disparity)
{

  // Init variables
  double max_depth = 1000;
  double min_depth = 1;
  const double& baseline = b;
  const double& f = K(0,0);

  // Iterate over disparity pixels, compute corresponding vertex
  for (int v=0; v<vertices.rows; v++) {
    for (int u=0; u<vertices.cols; u++) {
      cv::Vec3d vertex = vertices.at<cv::Vec3d>(v,u);
      const double& depth = vertex[2];
      if (depth < min_depth || depth > max_depth) continue;
      double disp = (baseline*f)/depth;
      //unsigned short disp_old = disparity.at<unsigned short>(v,u);
      unsigned short disp_new = disp*256;
      disparity.at<unsigned short>(v,u) = disp_new;
    }
  }
}

void computeNormalsFromVertices(const cv::Mat& vertices, cv::Mat& normals)
{
  double neighbor_dist_threshold = 0.5;
  normals = cv::Mat(vertices.rows, vertices.cols, CV_64FC3, cv::Scalar(0,0,1) );

  // Iterate over vertices in image
  for (int v=1; v<normals.rows-1; v++) {
    for (int u=1; u<normals.cols-1; u++) {

      // Go over 8-neighborhood to compute #neighbors
      cv::Vec3d b = vertices.at<cv::Vec3d>(v,u); // center
      //unsigned char neighbor_mask = 0;  // binary mask to mark neighbors within distance threshold
      unsigned char neighbor_count = 0; // number of neighbors within distance threshold
      char neighbor_curr = -1;  // index of current neighor that is checked, used for mask
      cv::Vec3d centroid(0,0,0); // Accumulator to compute centroid
      Eigen::MatrixXd data_matrix(9,3);

      // Iterate over 8-neighborhood
      for (int x=u-1; x<=u+1; x++) {
        for (int y=v-1; y<=v+1; y++) {
          neighbor_curr++;
          cv::Vec3d a = vertices.at<cv::Vec3d>(y,x);
          cv::Vec3d d = a-b;
          double squared_dist = (d).dot(d);

          // Only accept points within distance threshold
          if (squared_dist < neighbor_dist_threshold*neighbor_dist_threshold) {
            data_matrix(neighbor_count, 0) = a[0];
            data_matrix(neighbor_count, 1) = a[1];
            data_matrix(neighbor_count, 2) = a[2];
            neighbor_count++;
            //neighbor_mask = neighbor_mask | (1<<neighbor_curr);
            centroid += a;
          }
        }
      }

      // if hte number of neighbors is too small, we need at least 3 vertices
      if (neighbor_count < 3) continue; // default normal (0,0,1) is assigned, see init

      // Compute centroid and resize data_matrix
      centroid /= (double)neighbor_count;
      data_matrix.resize(neighbor_count, 3);

      // Subtract mean from data_matrix
      for (int i=0; i<neighbor_count; i++) {
        data_matrix(i,0) -= centroid[0];
        data_matrix(i,1) -= centroid[1];
        data_matrix(i,2) -= centroid[2];
      }

      // Last eigenvector corresponds to normal
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(data_matrix, Eigen::ComputeThinU | Eigen::ComputeFullV);

      /*double ev0 = svd.singularValues()[0];
      double ev1 = svd.singularValues()[1];
      double ratio = ev0/ev1;
      if (ratio>16) continue; // default normal (0,0,1) is assigned, see init*/

      Eigen::Vector3d n = svd.matrixV().col(2);
      n /= n.norm();

      // Get the correct orientation, assuming all points are visible from origin
      Eigen::Vector3d vector; vector << b[0],b[1],b[2];
      vector/=vector.norm();
      double dot = vector.dot(n);
      if (dot>0) n*=-1;


      cv::Vec3d normal;
      normal[0] = n[0];
      normal[1] = n[1];
      normal[2] = n[2];
      normals.at<cv::Vec3d>(v,u) = normal;

    }
  }
}

void computeNormalsFromVerticesSimple(const cv::Mat& vertices, cv::Mat& normals)
{
  normals = cv::Mat(vertices.rows, vertices.cols, CV_64FC3, cv::Scalar(0,0,0) );
  for (int v=1; v<normals.rows-1; v++) {
    for (int u=1; u<normals.cols-1; u++) {

      cv::Vec3d v1 = vertices.at<cv::Vec3d>(v+1,u);
      cv::Vec3d v2 = vertices.at<cv::Vec3d>(v,u);
      cv::Vec3d v3 = vertices.at<cv::Vec3d>(v,u+1);

      Eigen::Vector3d v1_; v1_ << v1[0],v1[1],v1[2];
      Eigen::Vector3d v2_; v2_ << v2[0],v2[1],v2[2];
      Eigen::Vector3d v3_; v3_ << v3[0],v3[1],v3[2];
      Eigen::Vector3d n = (v1_-v2_).cross(v3_-v2_);
      n /= n.norm();
      cv::Vec3d normal; normal[0] = n[0]; normal[1] = n[1]; normal[2] = n[2];
      normals.at<cv::Vec3d>(v,u) = normal;
    }
  }
}

void computePointcloudFromVerticesAndColor(const cv::Mat &vertices,
                                           const cv::Mat &colors,
                                           gvl::Pointcloud& pointcloud)
{
  // Check that vertices and colors have the same size
  assert(vertices.cols == colors.cols && vertices.rows == colors.rows);

  // Allocate as many points as pixels in the image,
  // this allows to address the 3d-points by 2d-coordinates
  pointcloud.points.resize(vertices.cols*vertices.rows);

  for (int v=0; v<vertices.rows; v++) {
    for (int u=0; u<vertices.cols; u++) {
      gvl::Point point;
      point.x = (double)vertices.at<cv::Vec3d>(v,u)[0];
      point.y = (double)vertices.at<cv::Vec3d>(v,u)[1];
      point.z = (double)vertices.at<cv::Vec3d>(v,u)[2];
      point.r = (unsigned char)colors.at<cv::Vec3b>(v,u)[2];
      point.g = (unsigned char)colors.at<cv::Vec3b>(v,u)[1];
      point.b = (unsigned char)colors.at<cv::Vec3b>(v,u)[0];
      point.u = u;
      point.v = v;
      int index = v*vertices.cols + u;
      pointcloud.points.at(index) = point;
    }
  }
}

void computeBoundingBoxFromPointcloud(const Pointcloud &pointcloud, BoundingBox& bb)
{
  double min_x=900, max_x=-900;
  double min_y=900, max_y=-900;
  double min_z=900, max_z=-900;
  for (auto p : pointcloud.points) {
    if (p.x > max_x) max_x=p.x;
    if (p.y > max_y) max_y=p.y;
    if (p.z > max_z) max_z=p.z;
    if (p.x < min_x) min_x=p.x;
    if (p.y < min_y) min_y=p.y;
    if (p.z < min_z) min_z=p.z;
  }
  bb.height = max_y - min_y;
  bb.width = max_x - min_x;
  bb.length = max_z - min_z;
  bb.x = (max_x + min_x)/2.0;
  bb.y = max_y;
  bb.z = (max_z + min_z)/2.0;
  bb.rotation_y=0;
}

double computeSquaredDistance(const Point& p1, const Point& p2)
{
  double d1 = (p1.x-p2.x);
  double d2 = (p1.y-p2.y);
  double d3 = (p1.z-p2.z);
  return d1*d1+d2*d2+d3*d3;
}

double bilinearInterpolation(const Eigen::Vector4d& values,
                             const Eigen::Vector2d& position) {
  const double& u = position[0];
  const double& v = position[1];
  return (1-u)*(1-v)*values[0] +
         (0+u)*(1-v)*values[1] +
         (1-u)*(0+v)*values[2] +
         (0+u)*(0+v)*values[3];
}

Eigen::Matrix4d computePoseFromRotTransScale(const double rotation_y,
                                             const Eigen::Vector3d& translation,
                                             const double scale)
{
  //Eigen::AngleAxisd aa(rotation_y, Eigen::Vector3d(0,1,0)); aa.matrix();
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  rotation(0,0) = std::cos(rotation_y);   rotation(0,2) = std::sin(rotation_y);
  rotation(2,0) = -std::sin(rotation_y);  rotation(2,2) = std::cos(rotation_y);
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose.block<3,3>(0, 0) = rotation*scale;
  pose.block<3,1>(0, 3) = translation;
  return pose;
}

void computeBoundingBoxFromAnnotation(const Annotation &annotation,
                                      const Eigen::Vector3f &color,
                                      BoundingBox &bb)
{
  bb.height = 1.7;
  bb.width = 2.0;
  bb.length = 4.5;
  bb.x = annotation.translation[0];
  bb.y = annotation.translation[1];
  bb.z = annotation.translation[2];
  bb.rotation_y = annotation.rotation_y;
  bb.r = color[0];
  bb.g = color[1];
  bb.b = color[2];
  bb.score = 1.0;
}

double computeMedian(std::vector<double>& values) {
  std::sort(values.begin(), values.end());
  std::cout << "Median >> Values: "; for (auto t: values) std::cout << t << " "; std::cout << std::endl;
  double median;
  if (values.size()%2 == 1) {
    std::cout << "Median >> Size: " << values.size() << std::endl;
    median = values.at(values.size()/2);
  } else {
    median = 0.5*values.at(values.size()/2 - 1)+0.5*values.at(values.size()/2);;
  }
  std::cout << "Median: " << median << std::endl;
  return median;
}

double computeVerticalDistanceToPlane(const double a,
                                      const double b,
                                      const double c,
                                      const double d,
                                      const double x,
                                      const double z)
{
  double y = (-d-c*z-a*x)/b;
  return y;
}

// From: http://stackoverflow.com/questions/7685495/transforming-a-3d-plane-by-4x4-matrix
// This does NOT seem to work!!!
void transform_plane(const Eigen::Matrix4d &trafo, Eigen::Vector4d &p)
{
  std::cerr << "Transform_plane does NOT seem to work!!!" << std::endl;
  Eigen::Vector4d O = Eigen::Vector4d(p[0]*p[3], p[1]*p[3], p[2]*p[3], 1);
  Eigen::Vector4d N = Eigen::Vector4d(p[0], p[1], p[2], 0.0);
  O = trafo*O;
  N = (trafo.inverse()).transpose() * N;

  Eigen::Vector3d n = Eigen::Vector3d(N[0],N[1],N[2]);
  Eigen::Vector3d o = Eigen::Vector3d(O[0],O[1],O[2]);
  p[0] = N[0];
  p[1] = N[1];
  p[2] = N[2];
  p[3] = o.dot(n);

  //vector4 O = (xyz * d, 1)
  //vector4 N = (xyz, 0)
  //O = M * O
  //N = transpose(invert(M)) * N
  //xyz = N.xyz
  //d = dot(O.xyz, N.xyz)
}


}
