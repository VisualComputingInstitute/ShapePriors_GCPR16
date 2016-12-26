#ifndef GVL_VISUALIZATION_H
#define GVL_VISUALIZATION_H

// C/C++ includes
#include <string>

// OpenCV includes
#include <opencv2/core/core.hpp>

// VTK includes
#include <vtkPlane.h>
#include <vtkSmartPointer.h>

// Eigen includes
#include <eigen/Core>
#include <eigen/Dense>

// Own includes
#include "geometry.h"

class vtkRenderer;
class vtkRenderWindow;
class vtkRenderWindowInteractor;
class vtkCallbackCommand;

namespace viz {

class Visualization
{
public:

  // -----------------------------------------------------------------------------------------------
  // Viewer Functions
  // -----------------------------------------------------------------------------------------------
  Visualization(int width=1200, int height=300, int renderers_count=1, bool vertical_split=true);
  void setActiveRenderer(int rendererId);
  void connectCameras(int i, int j);
  void setBackgroundColor(const Eigen::Vector3f& color);
  void setBackgroundColor(float r, float g, float b);
  void setWindowSize(const int width, const int height);
  void hideAllActors();
  void show();
  void draw();

  // -----------------------------------------------------------------------------------------------
  // Render Functions
  // -----------------------------------------------------------------------------------------------
  void renderImage(cv::Mat &image);
  void renderDepth(cv::Mat& depth, bool flipped=true);

  // -----------------------------------------------------------------------------------------------
  // Camera Functions
  // -----------------------------------------------------------------------------------------------
  void setK(const Eigen::Matrix3d& K, const int width, const int height);
  void setCameraPosition(const Eigen::Vector3d& pos, double dist);
  void setCameraPosition(const Eigen::Vector3d& pos);
  void setCameraFocalPoint(const Eigen::Vector3d& fp);
  void printCameraPosition(void);
  void setNearClippingRange(double d);
  void takeScreenshot2(const std::string& path);
  void takeScreenshot(const std::string& path, int width=900, int height=900);
  void takeMovie(const std::string& path,
                 const std::vector<Eigen::Vector3d>& waypoints,
                 int width=900, int height=900);
  void takeMovie2(const std::string& path,
                 const std::vector<Eigen::Vector3d>& waypoints,
                 const std::vector<Eigen::Vector3d>& focalpoints,
                 const std::vector<double>& speeds,
                 int width=900, int height=900);
  void pointToOrigin();
  void pointToScene();
  void pointFromTop();
  void lookFromAt(Eigen::Vector3d from, Eigen::Vector3d at);
  void enableParallelProjection(void);

  // -----------------------------------------------------------------------------------------------
  // Add Functions
  // -----------------------------------------------------------------------------------------------
  void addGrid(double depth_min = 5,
               double depth_max = 30,
               double width_min = -6,
               double width_max = 6,
               double y = 1.5);
  void addGridSmall(void);
  void addAxes();
  void addAxesBig();
  void addTrajectory(const std::vector<Eigen::Vector3d>& trajectory, const Eigen::Vector3f& color);
  void addFrustum(const Eigen::Matrix4d& pose);
  void addPointcloud(const std::vector<Eigen::Vector3d>& vertices,
                     const std::vector<Eigen::Vector3f>& colors,
                     double point_size = 3);
  void addPlane(const double a, const double b, const double c, const double d);
  void addPlane(const Eigen::Vector4d& plane);
  void addPLYModel(const std::string& path);
  void addBoundingBox(const Eigen::Vector3d& size,
                      const Eigen::Vector3d& translation,
                      const double rotation_y,
                      const Eigen::Vector3f& color);
  void addNormals(const cv::Mat& vertices, const cv::Mat& normals);
  void addNormals(const std::vector<Eigen::Vector3d>& vertices,
                  const std::vector<Eigen::Vector3d>& normals);
  void addClusterCenter(const double x, const double y, const double z);
  void addLocalCoordinateAxes(Eigen::Matrix4d& pose);
  void addZeroSurfaceFromGrid(const Eigen::MatrixXd& values,
                              const Eigen::Matrix4d& pose,
                              const Eigen::Vector3f& color,
                              bool alpha = true,
                              double isoValue = 0,
                              double opacity = 0.1,
                              double lineWidth = 1.0);
  void addGridSamples(const Eigen::MatrixXd& values,
                              const Eigen::Matrix4d& pose,
                              const Eigen::Vector3f& color,
                              bool alpha = true);
  void addRectangle2d(double value, double pos_x, double pos_y, double scaling);
  void addPlaneCut(const Eigen::MatrixXd &values,
                   vtkPlane* plane, bool legend=true);
  void addVolumeRenderer(const Eigen::MatrixXd& values);
  void addText(const std::string& text, const Eigen::Vector3d& translation, const Eigen::Vector3f& color = Eigen::Vector3f(0,0,0));
  void addText2d(const std::string& text, double pos_x = 30, double pos_y = 30, const Eigen::Vector3f& color = Eigen::Vector3f(0,0,0));
  void addCallbackCommand(vtkSmartPointer<vtkCallbackCommand> keypressCallback);
  void addCallbackCommand(void (*f)(vtkObject *caller, unsigned long eid,
                          void *clientdata, void *calldata));

  // -----------------------------------------------------------------------------------------------
  // Variables
  // -----------------------------------------------------------------------------------------------
  vtkRenderer* renderer;
  vtkRenderWindow* renderWindow;
  vtkRenderWindowInteractor* renderWindowInteractor;
  int render_id_current;
  std::vector<vtkRenderer*> renderers;
private:
  int renderers_count_;
};

}

#endif // GVL_VISUALIZATION_H

