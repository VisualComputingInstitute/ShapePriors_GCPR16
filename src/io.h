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

#ifndef GVL_IO_H
#define GVL_IO_H

// Eigen includes
#include <Eigen/Core>

// C/C++ includes
#include <memory>

// Own includes
#include "geometry.h"
#include "tracking.h"

namespace gvl {

/**
   * @brief read_normals
   * @param path
   * @return
   */
  cv::Mat read_normals(std::string& path);

  // Read/write partial point cloud from .ply file using vtk
  std::shared_ptr<gvl::Pointcloud> read_pointcloud(std::string pointcloud_path);
  void write_pointcloud(const std::shared_ptr<gvl::Pointcloud>& pointcloud,
                        std::string pointcloud_path);

  /**
   * @brief Read groundplane parameters from file
   * @param[in] path
   * @param[out] a
   * @param[out] b
   * @param[out] c
   * @param[out] d
   */
  void read_groundplane(const std::string& filename,
                        double *a,
                        double *b,
                        double *c,
                        double *d);

  /**
   * @brief read_viewpoints reads specified viewpoint file and returns the parsed viewpoints
   * @param viewpoints_path path to viewpoint file
   * @return vector with the 3D viewpoints
   */
  std::vector<Eigen::Vector4d> read_viewpoints(std::string viewpoints_path);

  std::shared_ptr<gvl::Pointcloud> read_partial(std::string partial_path);

  std::vector<std::shared_ptr<gvl::Pointcloud>> read_partials(std::string& partials_path_prefix,
                                                              unsigned int viewpoints_count);

  /**
   * @brief writeMatrixToSharedMemory
   * @param[in] segmentName
   * @param[in] vectorName
   * @param[in] matrix
   */
  void writeMatrixToSharedMemory(const std::string& segmentName,
                                 const std::string& vectorName,
                                 const Eigen::MatrixXd& matrix);

  /**
   * @brief Read Eigen::Matrix from shared memory
   * @param[in] segmentName
   * @param[in] vectorName
   * @param[out] matrix
   */
  void readMatrixFromSharedMemory(const std::string& segmentName,
                                  const std::string& vectorName,
                                  Eigen::MatrixXd& matrix);

  /**
   * @brief Write Eigen matrix to file
   * @param[in] matrix
   * @param[in] filename
   * @return true on success
   */
  bool writeMatrixToFile(const Eigen::MatrixXd& matrix,
                         const std::string& filename);

  /**
   * @brief Read eigen matrix from file
   * @param[in] filename
   * @param[out] matrix
   * @param[in] block_row
   * @param[in] block_col
   * @return true on success
   */
  bool readMatrixFromFile(const std::string& filename,
                          Eigen::MatrixXd &matrix,
                          int block_row=0, int block_col=0);

  /**
   * @brief writeMatrixToBinaryFile
   * @param[in] filename
   * @param[in] matrix
   */
  void writeMatrixToBinaryFile(const std::string& filename, const Eigen::MatrixXd& matrix);

  /**
   * @brief readMatrixFromBinaryFile
   * @param[in] filename
   * @param[out] matrix
   */
  void readMatrixFromBinaryFile(const std::string& filename, Eigen::MatrixXd& matrix);

  /**
   * @brief readAnnotationsFromFile
   * @param path
   * @param annotations
   * @return
   */
  bool readAnnotationsFromFile(const std::string& path,
                               std::vector<std::shared_ptr<gvl::Annotation>>& annotations);

  /**
   * @brief writeAnnotationsToFile
   * @param path
   * @param annotations
   * @return
   */
  bool writeAnnotationsToFile(const std::string& path,
                              std::vector<std::shared_ptr<gvl::Annotation>>& annotations);

  /**
   * @brief readCalibrationFile
   * @param path
   * @return
   */
  std::vector<Eigen::Matrix4d> readCalibrationFile(std::string& path);

  /**
   * @brief readPosesFromFile
   * @param path
   * @return
   */
  std::vector<Eigen::Matrix4d> readPosesFromFile(std::string& path);

  /**
   * @brief writeDetectionsToFile
   * @param path
   * @param bbs
   */
  void writeDetectionsToFile(std::string& path, std::vector<gvl::Detection> dets);

  /**
   * @brief readDetectionsFromFile
   * @param path
   * @return
   */
  std::vector<gvl::BoundingBox> readDetectionsFromFile(std::string& detections_path);

  /**
   * @brief readGroundPlaneFromFile
   * @param path
   * @return
   */
  Eigen::Vector4d readGroundplaneFromFile(const std::string& path);


}

#endif // GVL_IO_H

// Read cluster centers
//std::cout << "Skipping cluster centers!" << std::endl;
/*std::vector<Eigen::Vector3d> proposals_clusters_centers;
std::string line2;
std::ifstream proposals_clusters_file (proposals_clusters_path.c_str());
if (proposals_clusters_file.is_open()) {
  while ( getline (proposals_clusters_file, line2) ) {
    std::cout << line2 << std::endl;
    std::vector<std::string> values;
    boost::split( values, line2, boost::is_any_of(" ") );

    Eigen::Vector3d cluster_center;
    cluster_center[0] = std::stod(values[0]);
    cluster_center[1] = 1.0;
    cluster_center[2] = std::stod(values[1]);

    proposals_clusters_centers.push_back(cluster_center);
  }
  proposals_clusters_file.close();
} else {
  std::cout << "Unable to open file: " << proposals_clusters_path  << std::endl;
  return 0;
}*/

/**
 * @brief write_vertices_as_ply
 * @param vertices
 * @param output_path
 */
/*void write_vertices_as_ply(const vtkSmartPointer<vtkPoints>& vertices, std::string output_path) {
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(vertices);
  polydata->Squeeze();
  vtkSmartPointer<vtkPLYWriter> plyWriter = vtkSmartPointer<vtkPLYWriter>::New();
  plyWriter->SetFileName(output_path.c_str());
  plyWriter->SetInputData(polydata);
  plyWriter->Write();
}*/
