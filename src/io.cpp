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
#include <stdio.h>
#include <math.h>
#include <float.h>
#include <string>
#include <iostream>
#include <fstream>
#include <cstring>
#include <cstdlib>

// Boost includes
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>

// VTK includes
#include <vtkSmartPointer.h>
#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>
#include <vtkUnsignedCharArray.h>
#include <vtkDataArray.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>

// OpenCV includes
#include <opencv2/highgui/highgui.hpp>

// Own includes
#include "io.h"
#include "tracking.h"

namespace gvl {

std::shared_ptr<gvl::Pointcloud> read_pointcloud(std::string pointcloud_path) {

  // Read ply data
  vtkSmartPointer<vtkPLYReader> plyReader = vtkSmartPointer<vtkPLYReader>::New();
  plyReader->SetFileName(pointcloud_path.c_str());
  plyReader->Update();

  // Extract vertices and color from ply-data
  vtkSmartPointer<vtkPolyData> polyData = plyReader->GetOutput();
  vtkSmartPointer<vtkPoints> pointcloud_vertices = polyData->GetPoints();
  vtkSmartPointer<vtkDataArray> pointcloud_colors = polyData->GetPointData()->GetScalars();

  // Add read data into pointcloud
  auto pointcloud(std::make_shared<gvl::Pointcloud>());
  pointcloud->points.resize(pointcloud_vertices->GetNumberOfPoints());
  for (int i=0; i<pointcloud_vertices->GetNumberOfPoints(); i++) {
    double data[3];
    pointcloud_vertices->GetPoint(i, data);
    double* color = pointcloud_colors->GetTuple3(i);
    gvl::Point p;
    p.x = data[0]; p.y = data[1]; p.z = data[2];
    p.r = color[0]; p.g = color[1]; p.b = color[2];
    pointcloud->points.at(i)=p;
  }
  return pointcloud;
}

void write_pointcloud(const std::shared_ptr<gvl::Pointcloud>& pointcloud,
                      std::string pointcloud_path)
{
  // Prepare color and points array
  vtkSmartPointer<vtkPoints> pointcloud_vertices = vtkSmartPointer< vtkPoints >::New();
  vtkSmartPointer<vtkUnsignedCharArray> pointcloud_colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
  pointcloud_colors->SetNumberOfComponents(3);
  pointcloud_colors->SetName("Colors");

  // Copy data from gvl::pointcloud to vtk containers
  for (unsigned int i=0; i<pointcloud->points.size(); i++){
    gvl::Point* point = &pointcloud->points.at(i);
    pointcloud_vertices->InsertNextPoint(point->x,point->y,point->z);
    pointcloud_colors->InsertNextTuple3(point->r, point->g, point->b);
  }

  // Write pointcloud to binary ply file
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(pointcloud_vertices);
  polydata->GetPointData()->SetScalars(pointcloud_colors);
  vtkSmartPointer<vtkPLYWriter> plyWriter = vtkSmartPointer<vtkPLYWriter>::New();
  plyWriter->SetFileName(pointcloud_path.c_str());
  plyWriter->SetInputData(polydata);
  //plyWriter->SetInputConnection(polydata);
  plyWriter->SetArrayName("Colors");
  plyWriter->SetFileTypeToBinary();
  plyWriter->Write();
}

void read_groundplane(const std::string &path, double *a, double *b, double *c, double *d)
{
  std::ifstream groundplane_file (path.c_str());
  std::string line;
  if (groundplane_file.is_open()) {
    getline(groundplane_file,line);
    getline(groundplane_file,line);
    getline(groundplane_file,line);
    getline(groundplane_file,line);

    std::vector<std::string> values;
    boost::split( values, line, boost::is_any_of(" ") );
    *a = std::stod(values[0]);
    *b = std::stod(values[1]);
    *c = std::stod(values[2]);
    *d = std::stod(values[3]);
    groundplane_file.close();
  } else {
    std::cout << "Unable to open file: " << path  << std::endl;
  }
}

Eigen::Vector4d readGroundplaneFromFile(const std::string &path)
{
  std::ifstream groundplane_file (path.c_str());
  std::string line;
  double a,b,c,d;
  if (groundplane_file.is_open()) {
    getline(groundplane_file,line);
    getline(groundplane_file,line);
    getline(groundplane_file,line);
    getline(groundplane_file,line);

    std::vector<std::string> values;
    boost::split( values, line, boost::is_any_of(" ") );
    a = std::stod(values[0]);
    b = std::stod(values[1]);
    c = std::stod(values[2]);
    d = std::stod(values[3]);
    groundplane_file.close();
  } else {
    std::cout << "Unable to open file: " << path  << std::endl;
  }
  return Eigen::Vector4d(a,b,c,d);
}

void writeMatrixToSharedMemory(const std::string &segmentName,
                               const std::string &vectorName,
                               const Eigen::MatrixXd &matrix)
{
  typedef boost::interprocess::allocator<double, boost::interprocess::managed_shared_memory::segment_manager>  ShmemAllocator;
  typedef std::vector<double, ShmemAllocator> MyVector;

  //Remove shared memory on construction and destruction
  /*struct shm_remove
  {
    shm_remove() { boost::interprocess::shared_memory_object::remove(segmentName.c_str()); }
    ~shm_remove(){ boost::interprocess::shared_memory_object::remove(segmentName.c_str()); }
  } remover;*/
  int cols = matrix.cols();
  int rows = matrix.rows();
  std::cout << rows << "x" << cols << std::endl;
  int factor = 8; // Change this factor if program crashes with bad_alloc, not sure why??

  boost::interprocess::shared_memory_object::remove(segmentName.c_str());

  int buffer_size = cols*rows*sizeof(std::double_t)*factor;
  boost::interprocess::managed_shared_memory segment(boost::interprocess::create_only,
                                                     segmentName.c_str(),
                                                     buffer_size);
  const ShmemAllocator alloc_inst (segment.get_segment_manager());
  MyVector *myvector = segment.construct<MyVector>(vectorName.c_str())(alloc_inst);

  //Insert data in the vector
  std::cout << "putting data..." << std::endl;
  for (int j=0; j<cols; j++) {
    for (int i=0; i<rows; i++) {
      //std::cout << i << "<"<<rows<< " "<< j << "<"<<cols <<  std::endl;
      myvector->push_back((double)(matrix(i,j)));
    }
  }

}

void readMatrixFromSharedMemory(const std::string& segmentName,
                                const std::string& vectorName,
                                Eigen::MatrixXd& matrix)
{
  typedef boost::interprocess::allocator<double,boost::interprocess::managed_shared_memory::segment_manager>  ShmemAllocator;
  typedef std::vector<double, ShmemAllocator> MyVector;

  boost::interprocess::managed_shared_memory segment(boost::interprocess::open_only, segmentName.c_str());
  MyVector *myvector = segment.find<MyVector>(vectorName.c_str()).first;
  matrix = Eigen::Map<Eigen::MatrixXd>(myvector->data(), matrix.rows(), matrix.cols());
}

bool writeMatrixToFile(const Eigen::MatrixXd& matrix,
                       const std::string& filename)
{
  std::ofstream myfile;
  myfile.open (filename.c_str());
  if (myfile.is_open()) {
    Eigen::IOFormat format(Eigen::FullPrecision, 0, ",", ",", "", "", "", "");
    myfile << matrix.format(format);
  } else {
    std::cout << "Unable to open file: " << filename << std::endl;
    return false;
  }
  myfile.close();
  return true;
}

bool readMatrixFromFile(const std::string &filename,
                        Eigen::MatrixXd &matrix,
                        int block_row,
                        int block_col)
{
  int cols = matrix.cols();

  std::string line;
  std::ifstream myfile (filename.c_str());
  if (myfile.is_open()) {
    while ( getline (myfile,line) ) {
      std::vector<std::string> sub_strings;
      boost::split( sub_strings, line, boost::is_any_of(","), boost::token_compress_on );
      for (unsigned int i=0; i<sub_strings.size(); i++) {
        double value = std::stod(sub_strings.at(i));
        matrix(block_row+i/cols, block_col+i%cols) = value;
      }
    }
    myfile.close();
  } else {
    std::cout << "Unable to open file: " << filename << std::endl;
    return false;
  }

  return true;
}

void writeMatrixToBinaryFile(const std::string& filename, const Eigen::MatrixXd& matrix){
  std::ofstream out(filename.c_str(), ios::out | ios::binary | ios::trunc);
  if (out.is_open()) {
    typename Eigen::MatrixXd::Index rows=matrix.rows(), cols=matrix.cols();
    out.write((char*) (&rows), sizeof(typename Eigen::MatrixXd::Index));
    out.write((char*) (&cols), sizeof(typename Eigen::MatrixXd::Index));
    out.write((char*) matrix.data(), rows*cols*sizeof(typename Eigen::MatrixXd::Scalar) );
    out.close();
  } else {
    std::cerr << "Could not open binary file: " << filename << std::endl;
  }
}

void readMatrixFromBinaryFile(const std::string& filename, Eigen::MatrixXd& matrix){
  std::ifstream in(filename.c_str(), ios::in | std::ios::binary);
  if (in.is_open()) {
    typename Eigen::MatrixXd::Index rows=0, cols=0;
    in.read((char*) (&rows),sizeof(typename Eigen::MatrixXd::Index));
    in.read((char*) (&cols),sizeof(typename Eigen::MatrixXd::Index));
    matrix.resize(rows, cols);
    in.read( (char *) matrix.data() , rows*cols*sizeof(typename Eigen::MatrixXd::Scalar) );
    in.close();
  } else {
    std::cerr << "Could not open binary file: " << filename << std::endl;
  }
}

bool readAnnotationsFromFile(const std::string& path,
                             std::vector<std::shared_ptr<gvl::Annotation>>& annotations) {
  // First clear vector
  annotations.clear();

  std::string line;
  std::ifstream file (path.c_str());
  if (file.is_open()) {
    while ( getline (file,line) ) {
      std::vector<std::string> values;
      boost::split( values, line, boost::is_any_of(" ") );

      auto annotation = std::make_shared<gvl::Annotation>();
      annotation->id = std::stoi(values[0]);
      annotation->model_name = values[1];
      annotation->rotation_y = std::stod(values[2]);
      Eigen::Vector3d translation;
      translation[0] = std::stod(values[3]);
      translation[1] = std::stod(values[4]);
      translation[2] = std::stod(values[5]);
      annotation->translation = translation;
      annotation->scale = std::stod(values[6]);
      annotations.push_back(annotation);
    }
    file.close();
  } else {
    std::cout << "Unable to open file: " << path  << std::endl;
    return false;
  }
  return true;
}

bool writeAnnotationsToFile(const std::string& path,
                            std::vector<std::shared_ptr<gvl::Annotation>>& annotations) {
  std::ofstream file;
  file.open (path.c_str());
  if (file.is_open()) {
    for (auto a : annotations) {
      std::string line = "";
      line += std::to_string(a->id) + " ";
      line += a->model_name + " ";
      line += std::to_string(a->rotation_y) + " ";
      line += std::to_string(a->translation[0]) + " ";
      line += std::to_string(a->translation[1]) + " ";
      line += std::to_string(a->translation[2]) + " ";
      line += std::to_string(a->scale);
      file << line << std::endl;
    }
  } else {
    std::cout << "Unable to open file: " << path << std::endl;
    return false;
  }
  file.close();
  return true;
}

std::vector<Eigen::Matrix4d> readCalibrationFile(std::string &calibration_path) {
  std::vector<Eigen::Matrix4d> projection_matrices;
  std::ifstream calibration_file(calibration_path.c_str());
  std::string line;
  if (calibration_file.is_open()) {
    // Each line corresponds to a matrix
    while (getline(calibration_file,line)) {
      std::vector<std::string> values;
      boost::split( values, line, boost::is_any_of(" ") );
      Eigen::Matrix4d matrix;
      matrix << std::stod(values[1]), std::stod(values[2]), std::stod(values[3]),
          std::stod(values[4]), std::stod(values[5]), std::stod(values[6]),
          std::stod(values[7]), std::stod(values[8]), std::stod(values[9]),
          std::stod(values[10]), std::stod(values[11]), std::stod(values[12]),
          0.0, 0.0, 0.0, 1.0;
      projection_matrices.push_back(matrix);
    }
    calibration_file.close();
  } else {
    std::cout << "Unable to open file: " << calibration_path  << std::endl;
  }
  return projection_matrices;
}

std::vector<Eigen::Matrix4d> readPosesFromFile(std::string &path)
{
  std::vector<Eigen::Matrix4d> poses;
  std::ifstream poses_file(path.c_str());
  std::string line;
  if (poses_file.is_open()) {
    // Each line corresponds to a matrix
    while (getline(poses_file,line)) {
      std::vector<std::string> values;
      boost::split( values, line, boost::is_any_of(" ") );
      Eigen::Matrix4d matrix;
      matrix << std::stod(values[0]), std::stod(values[1]), std::stod(values[2]), std::stod(values[3]),
          std::stod(values[4]), std::stod(values[5]), std::stod(values[6]),
          std::stod(values[7]), std::stod(values[8]), std::stod(values[9]),
          std::stod(values[10]), std::stod(values[11]),
          0.0, 0.0, 0.0, 1.0;
      poses.push_back(matrix);
    }
    poses_file.close();
  } else {
    std::cout << "Unable to open file: " << path << std::endl;
  }
  return poses;
}

void writeDetectionsToFile(std::string& path, std::vector<gvl::Detection> dets) {

  std::ofstream file;
  file.open (path.c_str());
  if (file.is_open()) {

    for (auto&det : dets) {

      /*
      1    frame        Frame within the sequence where the object appearers
      1    track id     Unique tracking id of this object within this sequence
      1    type         Describes the type of object: 'Car', 'Van', 'Truck',
                        'Pedestrian', 'Person_sitting', 'Cyclist', 'Tram',
                        'Misc' or 'DontCare'
      1    truncated    Float from 0 (non-truncated) to 1 (truncated), where
                        truncated refers to the object leaving image boundaries.
            Truncation 2 indicates an ignored object (in particular
            in the beginning or end of a track) introduced by manual
            labeling.
      1    occluded     Integer (0,1,2,3) indicating occlusion state:
                        0 = fully visible, 1 = partly occluded
                        2 = largely occluded, 3 = unknown
      1    alpha        Observation angle of object, ranging [-pi..pi]
      4    bbox         2D bounding box of object in the image (0-based index):
                        contains left, top, right, bottom pixel coordinates
      3    dimensions   3D object dimensions: height, width, length (in meters)
      3    location     3D object location x,y,z in camera coordinates (in meters)
      1    rotation_y   Rotation ry around Y-axis in camera coordinates [-pi..pi]
      1    score        Only for results: Float, indicating confidence in
                        detection, needed for p/r curves, higher is better.*/

      std::string line = "";
      line += std::to_string(det.frame_id) + " "; // frame
      line += std::to_string(det.track_id) + " "; // track id
      line += "Car "; // type
      line += "0 "; //truncated
      line += "0 ";  // occluded
      line += std::to_string(0) + " "; // alpha - not set properly
      line += std::to_string(0) + " "; // 2d bb - not set properly
      line += std::to_string(0) + " "; // 2d bb - not set properly
      line += std::to_string(0) + " "; // 2d bb - not set properly
      line += std::to_string(0) + " "; // 2d bb - not set properly
      line += std::to_string(2) + " "; // height
      line += std::to_string(2.) + " "; // width
      line += std::to_string(4.2) + " "; // length
      line += std::to_string(det.translation[0]) + " "; // tx
      line += std::to_string(det.translation[1]) + " "; // ty
      line += std::to_string(det.translation[2]) + " "; // tz
      line += std::to_string(det.rotation_y+M_PI/2) + " "; // ry
      line += std::to_string(1); // score
      file << line << std::endl;
    }

  } else {
    std::cout << "Unable to open file: " << path << std::endl;
    return;
  }

  file.close();
}

std::vector<BoundingBox> readDetectionsFromFile(std::string &detections_path)
{
  std::vector<gvl::BoundingBox> detections;
  std::string line;
  std::ifstream detections_file (detections_path.c_str());
  if (detections_file.is_open()) {
    while ( getline (detections_file,line) ) {
      std::vector<std::string> values;
      boost::split( values, line, boost::is_any_of(" ") );
      bool isTracking = (values.size()>16);
      gvl::BoundingBox bb;
      int offset = 0; // when using tracking offset is 2, else 0
      if (isTracking) {
        offset = 2; // when using tracking offset is 2, else 0
        bb.frameId = std::stod(values[0]);
        bb.trackId = std::stod(values[1]);
      }
      bb.type = values[0+offset];
      bb.truncated = std::stod(values[1+offset]);
      bb.occluded = std::stod(values[2+offset]);
      bb.left = std::stod(values[4+offset]);
      bb.top = std::stod(values[5+offset]);
      bb.right = std::stod(values[6+offset]);
      bb.bottom = std::stod(values[7+offset]);

      bb.height = std::stod(values[8+offset]);
      bb.width = std::stod(values[9+offset]);
      bb.length = std::stod(values[10+offset]);
      bb.x = std::stod(values[11+offset]);
      bb.y = std::stod(values[12+offset]);
      bb.z = std::stod(values[13+offset]);
      bb.rotation_y = std::stod(values[14+offset])-M_PI/2;
      bb.score = std::stod(values[15+offset]);
      /*if (isTracking) {
        bb.r = 1;
      }else{
        bb.r = 0;
      }
      bb.g = 0;
      bb.b = 1;*/

      //if (bb.score > max) max = bb.score;
      detections.push_back(bb);
    }
    detections_file.close();
  } else {
    std::cout << "Unable to open file: " << detections_path  << std::endl;
  }

  // Remove detections that have score below fraction of the median score
  // BAD BAD BAD do not do this, you have to recompute all tracks then
  /*if (filter) {

    std::vector<gvl::BoundingBox> detections_filtered(detections.size());

    // Filter detections based on score
    std::vector<double> bb_scores;
    for (auto bb : detections) {
      bb_scores.push_back(bb.score);
      std::cout << "bb.score = " << bb.score << std::endl;
    }
    const double bb_score_median = gvl::computeMedian(bb_scores);

    // copy only detections with score above threshold:
    auto it = std::copy_if (detections.begin(), detections.end(),
                            detections_filtered.begin(),
                            [m=bb_score_median](gvl::BoundingBox bb) {
                              return (bb.score>0.9);
                            } );
    detections_filtered.resize(std::distance(detections_filtered.begin(),it));  // shrink container to new size

    return detections_filtered;
  }*/

  return detections;
}

cv::Mat read_normals(std::string& normals_path)
{
  cv::Mat normals_in = cv::imread(normals_path, CV_LOAD_IMAGE_COLOR);
  cv::Mat normals(normals_in.rows, normals_in.cols, CV_64FC3, cv::Scalar(0,0,0));

  for (int v=0; v<normals.rows; v++) {
    for (int u=0; u<normals.cols; u++) {
      cv::Vec3b normal_in = normals_in.at<cv::Vec3b>(v,u);
      double nx = ((double)normal_in[0]-(255.0/2.0))/(255.0/2.0);
      double ny = ((double)normal_in[1]-(255.0/2.0))/(255.0/2.0);
      double nz = ((double)normal_in[2]-(255.0/2.0))/(255.0/2.0);
      normals.at<cv::Vec3d>(v,u) = cv::Vec3d(nx, ny, nz);
    }
  }

  return normals;
}

/**
 * @brief read_viewpoints reads specified viewpoint file and returns the parsed viewpoints
 * @param viewpoints_path path to viewpoint file
 * @return vector with the 3D viewpoints
 */
std::vector<Eigen::Vector4d> read_viewpoints(std::string viewpoints_path) {
  // Opening the file
  //std::cout << "Reading viewpoints from file " << viewpoints_path << std::endl;
  std::ifstream myfile;
  myfile.open(viewpoints_path.c_str());
  if(!myfile.is_open()) {
    std::cout << "Exiting: could not open file " << viewpoints_path << std::endl;
  }

  // Read each line and parse it into Vector3d
  std::vector<Eigen::Vector4d> viewpoints;
  for (std::string line; std::getline(myfile, line);) {
    Eigen::Vector4d viewpoint;
    std::string::size_type sz1, sz2;
    viewpoint[0] = std::stod(line, &sz1);
    viewpoint[1] = std::stod(line.substr(sz1), &sz2);
    viewpoint[2] = std::stod((line.substr(sz1)).substr(sz2));
    viewpoint[3] = 1.0;
    viewpoints.push_back(viewpoint);
  }
  myfile.close();
  return viewpoints;
}

std::shared_ptr<gvl::Pointcloud> read_partial(std::string partial_path) {

  // Read ply data
  vtkSmartPointer<vtkPLYReader> model_reader = vtkSmartPointer<vtkPLYReader>::New();
  model_reader->SetFileName(partial_path.c_str());
  model_reader->Update();
  vtkSmartPointer<vtkPolyData> polyData = model_reader->GetOutput();
  vtkSmartPointer<vtkPoints> vertices = polyData->GetPoints();

  // Add read data into partial pointlcoud
  auto partial_pointcloud(std::make_shared<gvl::Pointcloud>());
  partial_pointcloud->points.resize(vertices->GetNumberOfPoints());
  for (int i=0; i<vertices->GetNumberOfPoints(); i++) {
    double data[3];
    vertices->GetPoint(i, data);
    gvl::Point p;
    p.x = data[0];
    p.y = data[1];
    p.z = data[2];
    p.r = 255;
    p.g = 0;
    p.b = 255;
    partial_pointcloud->points.at(i)=p;
  }

  return partial_pointcloud;
}

std::vector<std::shared_ptr<gvl::Pointcloud>> read_partials(std::string& partials_path_prefix,
                                                            unsigned int viewpoints_count){
  std::vector<std::shared_ptr<gvl::Pointcloud>> partials;
  for (unsigned int i=0; i<viewpoints_count; i++) {
    std::string partial_path = partials_path_prefix+std::to_string(1000+i)+".ply";
    auto partial_pointcloud = read_partial(partial_path);
    partials.push_back(partial_pointcloud);
  }
  return partials;
}

}
