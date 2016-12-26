// MIT License
//
// Copyright (c) 2016 Francis Engelmann
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// C/C++ includes
#include <chrono>
#include <ctime>

// VTK includes
#include <vtkAxesActor.h>
#include <vtkActor.h>
#include <vtkArrowSource.h>
#include <vtkCamera.h>
#include <vtkCubeSource.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkCutter.h>
#include <vtkCallbackCommand.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkGlyph3D.h>
#include <vtkGlyph2D.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkImageData.h>
#include <vtkJPEGWriter.h>
#include <vtkLine.h>
#include <vtkMarchingCubes.h>
#include <vtkOutlineFilter.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPlane.h>
#include <vtkPlaneSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkPNGWriter.h>
#include <vtkPLYReader.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkScalarBarActor.h>
#include <vtkSliderRepresentation2D.h>
#include <vtkSliderWidget.h>
#include <vtkTransform.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkVectorText.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkWindowToImageFilter.h>
#include <vtkGraphicsFactory.h>
#include <vtkRendererCollection.h>
#include <vtkLookupTable.h>
#include <vtkPolygon.h>
#include <vtkProperty2D.h>

// Own includes
#include "viz.h"

// OpenCV includes
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

namespace viz {

void CameraPositionCallbackFunction ( vtkObject* caller,
                                long unsigned int vtkNotUsed(eventId),
                                void* vtkNotUsed(clientData),
                                void* vtkNotUsed(callData) ) {
  vtkRenderWindowInteractor *iren = static_cast<vtkRenderWindowInteractor*>(caller);

  char* key = iren->GetKeySym();

  if(key[0]==99) { // - 'c' print camera parameters
    vtkSmartPointer<vtkCamera> camera = iren->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera();
    std::cout << "P: " << camera->GetPosition()[0] << ", ";
    std::cout << camera->GetPosition()[1] << ", ";
    std::cout << camera->GetPosition()[2];

    std::cout << "   F: " << camera->GetFocalPoint()[0] << ", ";
    std::cout << camera->GetFocalPoint()[1] << ", ";
    std::cout << camera->GetFocalPoint()[2] << std::endl;
  }

  return;
}

Visualization::Visualization(int width, int height, int renderers_count, bool vertical_split):
  renderers_count_(renderers_count)
{

  // Impossible that we have less then one renderer
  assert(renderers_count>0);
  //assert(renderers_count<5); // more then 4 renderers not supported right now (need to select proper viewports)

  // Initiliaze renderers
  renderers.resize(renderers_count);
  for (int i=0; i<renderers_count; i++){
    renderers.at(i) = vtkRenderer::New();
    renderers.at(i)->SetBackground(1,1,1);
  }

  // Set link cameras between renderers, maybe this is not wanted?
  for (int i=1; i<renderers_count; i++) {
    //renderers.at(i)->SetActiveCamera(renderers.at(0)->GetActiveCamera());
  }

  if (renderers_count==2) {
    if (vertical_split==true) {
      double leftViewport[4] = {0.0, 0.0, 0.5, 1.0};
      double rightViewport[4] = {0.5, 0.0, 1.0, 1.0};
      renderers.at(0)->SetViewport(leftViewport);
      renderers.at(1)->SetViewport(rightViewport);
    } else {
      double bottomViewport[4] = {0.0, 0.0, 1.0, 0.5};
      double topViewport[4] = {0.0, 0.5, 1.0, 1.0};
      renderers.at(0)->SetViewport(topViewport);
      renderers.at(1)->SetViewport(bottomViewport);
    }
  } else if (renderers_count==3) {
    if (vertical_split) {
      double leftTopViewport[4] = {0.0, 0.5, 0.5, 1.0};
      double rightTopViewport[4] = {0.5, 0.5, 1.0, 1.0};
      double bottomViewport[4] = {0.0, 0.0, 1.0, 0.5};
      renderers.at(0)->SetViewport(leftTopViewport);
      renderers.at(1)->SetViewport(rightTopViewport);
      renderers.at(2)->SetViewport(bottomViewport);
    } else {
      double bottomViewport[4] = {0.0, 0.0, 1.0, 0.33};
      double middleViewport[4] = {0.0, 0.33, 1.0, 0.66};
      double topViewport[4] = {0.0, 0.66, 1.0, 1.0};
      renderers.at(0)->SetViewport(topViewport);
      renderers.at(1)->SetViewport(middleViewport);
      renderers.at(2)->SetViewport(bottomViewport);
      }
  } else if (renderers_count==4) {
    double half = 0.5;
    double leftTopViewport[4] = {0.0, half, half, 1.0};
    double rightTopViewport[4] = {half, half, 1.0, 1.0};
    double leftBottomViewport[4] = {0.0, 0.0, half, half};
    double rightBottomViewport[4] = {half, 0.0, 1.0, half};
    renderers.at(0)->SetViewport(leftTopViewport);
    renderers.at(1)->SetViewport(rightTopViewport);
    renderers.at(2)->SetViewport(leftBottomViewport);
    renderers.at(3)->SetViewport(rightBottomViewport);
  } else if (renderers_count>4) {
    std::cerr << "!!! Remember to set viewports for all renderers !!!!!" << std::endl;
  }

  // Set window size
  int size[2] = {width, height};
  renderWindow = vtkRenderWindow::New();
  renderWindow->SetSize(size);

  // Add all renderers to renderWindow
  for (int i=0; i<renderers_count; i++) {
    renderWindow->AddRenderer(renderers.at(i));
  }

  renderWindowInteractor = vtkRenderWindowInteractor::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New(); //like paraview
  renderWindowInteractor->SetInteractorStyle( style );

  for (int i=0; i<renderers.size(); i++) {
    vtkSmartPointer<vtkCamera> camera = renderers.at(i)->GetActiveCamera();
    camera->SetViewUp(0.0,-1.0, 0.0);
    camera->SetPosition(0.0, 0.0, 0.0);
    camera->SetFocalPoint(0.0, 0.0, 10.0);
  }

  // Add callback that print camera position when pressing c
  addCallbackCommand(CameraPositionCallbackFunction);

  // Set default renderer
  renderer = renderers.at(0);
}

void Visualization::setActiveRenderer(int rendererId)
{
  assert(rendererId < renderers.size());
  renderer = renderers.at(rendererId);
}

void Visualization::connectCameras(int i, int j)
{
  assert(i<renderers.size());
  assert(j<renderers.size());
  renderers.at(i)->SetActiveCamera(renderers.at(j)->GetActiveCamera());
}

void Visualization::setBackgroundColor(const Eigen::Vector3f& color) {
  renderer->SetBackground(color[0]/255.0,color[1]/255.0,color[2]/255);
}

void Visualization::setBackgroundColor(float r, float g, float b) {
  this->setBackgroundColor(Eigen::Vector3f(r,g,b));
}

void Visualization::pointToOrigin(void) {
  vtkSmartPointer<vtkCamera> camera = renderer->GetActiveCamera();
  camera->SetViewUp(0.0, -1.0, 0.0);
  camera->SetPosition(4.5, -4.0, -4.5);
  camera->SetFocalPoint(0.0, -0.5, 0.0);
}

void Visualization::pointToScene(void) {
  vtkSmartPointer<vtkCamera> camera = renderer->GetActiveCamera();
  camera->SetViewUp(0.0, -1.0, 0.0);
  camera->SetPosition(0.0, 0.0, 0.0);
  camera->SetFocalPoint(0.0, 0.0, 10.0);
}

void Visualization::pointFromTop(void) {
  vtkSmartPointer<vtkCamera> camera = renderer->GetActiveCamera();
  //camera->SetViewUp(0.0, -1.0, 0.0);
  camera->SetPosition(0.0, -30.0, 11.0);
  camera->SetFocalPoint(0.0, 0.0, 11.1);
  camera->ParallelProjectionOn();
  camera->SetEyeAngle(90);
  camera->SetParallelScale(3.0);
}

void Visualization::enableParallelProjection(void) {
  vtkSmartPointer<vtkCamera> camera = renderer->GetActiveCamera();
  camera->ParallelProjectionOn();
  camera->SetEyeAngle(90);
  camera->SetParallelScale(3.0);
}

void Visualization::lookFromAt(Eigen::Vector3d from, Eigen::Vector3d at) {
  vtkSmartPointer<vtkCamera> camera = renderer->GetActiveCamera();
  camera->SetViewUp(0.0, -1.0, 0.0);
  camera->SetPosition(from[0], from[1], from[2]);
  camera->SetFocalPoint(at[0], at[1], at[2]);
}

void Visualization::hideAllActors()
{
  for (int i=0; i<this->renderers_count_; i++) {
    this->setActiveRenderer(i);
    vtkPropCollection* props = renderer->GetViewProps(); //iterate through and set each visibility to 0
    props->InitTraversal();
    for(int i=0; i<props->GetNumberOfItems(); i++){
      props->GetNextProp()->VisibilityOff();
    }
  }
}

void Visualization::addGrid(double depth_min, double depth_max, double width_min, double width_max, double y) {
  int num_lines = 0;

  // Create a vtkPoints container and store the points in it
  vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
  for (int i=width_min; i<=width_max; i++) {
    pts->InsertNextPoint((double)i, y, depth_min);
    pts->InsertNextPoint((double)i, y, depth_max);
    num_lines++;
  }

  for (int i=depth_min; i<=depth_max; i++) {
    pts->InsertNextPoint(width_min, y, (double)i);
    pts->InsertNextPoint(width_max, y, (double)i);
    num_lines++;
  }

  vtkSmartPointer<vtkPolyData> linesPolyData = vtkSmartPointer<vtkPolyData>::New();
  linesPolyData->SetPoints(pts);
  vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

  for (int i=0; i<num_lines; i++) {
    vtkSmartPointer<vtkLine> line_x = vtkSmartPointer<vtkLine>::New();
    line_x->GetPointIds()->SetId(0, 2*i);
    line_x->GetPointIds()->SetId(1, 2*i+1);
    lines->InsertNextCell(line_x);

  }

  // Add the lines to the polydata container
  linesPolyData->SetLines(lines);

  // Set colors
  unsigned char red[3] = { 125, 125, 125 };
  vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(3);
  for (int i=0; i<num_lines; i++) {
    colors->InsertNextTupleValue(red);
  }
  linesPolyData->GetCellData()->SetScalars(colors);

  // Setup the visualization pipeline
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputData(linesPolyData);
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  renderer->AddActor(actor);
}

void Visualization::show()
{
  renderWindowInteractor->Initialize();
  renderWindow->Render();
  renderWindowInteractor->Start();
}

void Visualization::draw()
{
  renderWindow->Render();
}

void Visualization::addAxes()
{
  vtkSmartPointer<vtkAxesActor> actor_axes = vtkSmartPointer<vtkAxesActor>::New();
  actor_axes->AxisLabelsOff();
  renderer->AddActor(actor_axes);
}

void Visualization::addAxesBig()
{
  vtkSmartPointer<vtkAxesActor> actor_axes = vtkSmartPointer<vtkAxesActor>::New();
  actor_axes->AxisLabelsOff();
  actor_axes->SetShaftTypeToCylinder();
  actor_axes->SetTipTypeToCone();
  actor_axes->SetScale(1);
  actor_axes->SetConeRadius(0.5);
  actor_axes->SetCylinderRadius(0.05);
  renderer->AddActor(actor_axes);
}

void Visualization::addTrajectory(const std::vector<Eigen::Vector3d>& trajectory,
                                  const Eigen::Vector3f& color)
{
  if (trajectory.size()<=1) return; // Nothing to do here
  // Create the polydata where we will store all the geometric data
  vtkSmartPointer<vtkPolyData> linesPolyData = vtkSmartPointer<vtkPolyData>::New();

  // Create a vtkPoints container and store the points in it
  vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
  for (auto v : trajectory) {
    double point[3] = {v(0), v(1), v(2)};
    pts->InsertNextPoint(point);
  }

  // Add the points to the polydata container
  linesPolyData->SetPoints(pts);

  vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
  for (unsigned int i=0; i<trajectory.size()-1; i++) {
    vtkSmartPointer<vtkLine> line =vtkSmartPointer<vtkLine>::New();
    line->GetPointIds()->SetId(0, i); // the second 0 is the index of the Origin in linesPolyData's points
    line->GetPointIds()->SetId(1, i+1); // the second 1 is the index of P0 in linesPolyData's points
    lines->InsertNextCell(line);
  }

  // Add the lines to the polydata container
  linesPolyData->SetLines(lines);

  // Create a vtkUnsignedCharArray container and store the colors in it
  vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(3);
  for (unsigned int i=0; i<trajectory.size(); i++) {
    unsigned char c[3] = { (uchar)color(0), (uchar)color(1), (uchar)color(2) };
    colors->InsertNextTupleValue(c);
  }

  // Color the lines.
  // SetScalars() automatically associates the values in the data array passed as parameter
  // to the elements in the same indices of the cell data array on which it is called.
  // This means the first component (red) of the colors array
  // is matched with the first component of the cell array (line 0)
  // and the second component (green) of the colors array
  // is matched with the second component of the cell array (line 1)`
  linesPolyData->GetCellData()->SetScalars(colors);

  // Setup the visualization pipeline
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputData(linesPolyData);

  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetLineWidth(5);
  renderer->AddActor(actor);
}

void Visualization::addFrustum(const Eigen::Matrix4d &pose)
{
  vtkSmartPointer<vtkPolyData> linesPolyData = vtkSmartPointer<vtkPolyData>::New();

  std::vector<Eigen::Vector4d> points;
  points.push_back(Eigen::Vector4d(7,0,10,1));
  points.push_back(Eigen::Vector4d(0,0,0,1));
  points.push_back(Eigen::Vector4d(-7,0,10,1));
  // Create a vtkPoints container and store the points in it
  vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
  for (auto v : points) {
    auto u = pose*v;
    double point[3] = {u(0)/u(3), u(1)/u(3), u(2)/u(3)};
    pts->InsertNextPoint(point);
  }

  // Add the points to the polydata container
  linesPolyData->SetPoints(pts);

  vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
  for (unsigned int i=0; i<points.size()-1; i++) {
    vtkSmartPointer<vtkLine> line =vtkSmartPointer<vtkLine>::New();
    line->GetPointIds()->SetId(0, i); // the second 0 is the index of the Origin in linesPolyData's points
    line->GetPointIds()->SetId(1, i+1); // the second 1 is the index of P0 in linesPolyData's points
    lines->InsertNextCell(line);
  }

  // Add the lines to the polydata container
  linesPolyData->SetLines(lines);

  // Create a vtkUnsignedCharArray container and store the colors in it
  vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(3);
  for (unsigned int i=0; i<points.size(); i++) {
    unsigned char c[3] = { 0, 128, 0 };
    colors->InsertNextTupleValue(c);
  }
  linesPolyData->GetCellData()->SetScalars(colors);

  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputData(linesPolyData);

  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetLineWidth(5);
  renderer->AddActor(actor);
}

//void Visualization::addBoundingBox(const gvl::BoundingBox &bb)
void Visualization::addBoundingBox(const Eigen::Vector3d& size,
                                   const Eigen::Vector3d& translation,
                                   const double rotation_y,
                                   const Eigen::Vector3f& color)
{

  vtkSmartPointer<vtkCubeSource> cubeSource =  vtkSmartPointer<vtkCubeSource>::New();
  vtkSmartPointer<vtkPolyDataMapper> mapper_bb =  vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper_bb->SetInputConnection(cubeSource->GetOutputPort());

  vtkSmartPointer<vtkActor> actor_bb =  vtkSmartPointer<vtkActor>::New();
  actor_bb->SetMapper(mapper_bb);
  actor_bb->GetProperty()->SetRepresentationToWireframe();
  actor_bb->GetProperty()->SetOpacity(1.0);
  actor_bb->GetProperty()->SetLineWidth(3);
  actor_bb->GetProperty()->SetColor(color[0],color[1],color[2]);
  actor_bb->GetProperty()->SetLighting(false);

  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  transform->PostMultiply(); //this is the key line
  transform->Scale(size[0], size[1], size[2]);
  transform->RotateY(rotation_y/M_PI*180.0);
  transform->Translate(translation[0], translation[1]-size[1]/2.0, translation[2]);
  actor_bb->SetUserTransform(transform);

  renderer->AddActor(actor_bb);

  // Direction Arrow
  vtkSmartPointer<vtkArrowSource> arrowSource = vtkSmartPointer<vtkArrowSource>::New();
  arrowSource->SetTipRadius(0.15);
  arrowSource->SetShaftRadius(0.05);
  vtkSmartPointer<vtkPolyDataMapper> mapper_arrow =  vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper_arrow->SetInputConnection(arrowSource->GetOutputPort());

  vtkSmartPointer<vtkActor> actor_arrow =  vtkSmartPointer<vtkActor>::New();
  actor_arrow->SetMapper(mapper_arrow);
  actor_arrow->GetProperty()->SetColor(color[0],color[1],color[2]);

  vtkSmartPointer<vtkTransform> transform_arrow = vtkSmartPointer<vtkTransform>::New();
  transform_arrow->PostMultiply(); //this is the key line
  transform_arrow->RotateY(90);
  transform_arrow->Scale(1, 1, size[2]/2.0);
  transform_arrow->RotateY(rotation_y/M_PI*180.0);
  transform_arrow->Translate(translation[0], -0.5, translation[2]);
  actor_arrow->SetUserTransform(transform_arrow);
  actor_arrow->GetProperty()->SetLighting(false);

  renderer->AddActor(actor_arrow);
}

void Visualization::addPointcloud(const std::vector<Eigen::Vector3d>& ver,
                                  const std::vector<Eigen::Vector3f>& col,
                                  double point_size)
{

  vtkSmartPointer<vtkPoints> vertices = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(3);
  colors->SetName("Colors");

  for (unsigned int i=0; i<ver.size(); i++) {
    auto& p = ver.at(i);
    auto& c = col.at(i);
    vertices->InsertNextPoint(p[0], p[1], p[2]);

    unsigned char r,g,b;
    r = c[0];
    g = c[1];
    b = c[2];
    unsigned char co[3] = {r, g, b};
    colors->InsertNextTupleValue(co);
  }

  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(vertices);
  polydata->Squeeze();

  vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
  glyphFilter->SetInputData(polydata);
  glyphFilter->Update();

  vtkSmartPointer<vtkPolyData> polydata_color = vtkSmartPointer<vtkPolyData>::New();
  polydata_color->ShallowCopy(glyphFilter->GetOutput());
  polydata_color->GetPointData()->SetScalars(colors);

  vtkSmartPointer<vtkPolyDataMapper> mapper_data = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper_data->SetInputData(polydata_color);

  vtkSmartPointer<vtkActor> actor_data = vtkSmartPointer<vtkActor>::New();
  actor_data->SetMapper(mapper_data);
  actor_data->GetProperty()->SetPointSize(point_size);

  renderer->AddActor(actor_data);
}


void Visualization::addPlane(const double a, const double b, const double c, const double d)
{
  vtkSmartPointer<vtkPlaneSource> groundplane = vtkSmartPointer<vtkPlaneSource>::New();
  groundplane->SetOrigin(-40, (-d-a*(-40))/b, 0);
  groundplane->SetPoint1( 40, (-d-a*( 40))/b, 0);
  groundplane->SetPoint2(-40, (-d-a*(-40)-c*80)/b, 80);

  vtkSmartPointer<vtkPolyDataMapper> mapper_groundplane = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper_groundplane->SetInputConnection(groundplane->GetOutputPort());

  vtkSmartPointer<vtkActor> actor_groundplane = vtkSmartPointer<vtkActor>::New();
  actor_groundplane->SetMapper(mapper_groundplane);
  actor_groundplane->GetProperty()->SetLighting(false);
  actor_groundplane->GetProperty()->SetColor(0.1,0.8,0.1);
  renderer->AddActor(actor_groundplane);
}

void Visualization::addPlane(const Eigen::Vector4d& plane)
{
  this->addPlane(plane[0], plane[1], plane[2], plane[3]);
}

void Visualization::addPLYModel(const std::string &model_path)
{
  //Read input model
  vtkSmartPointer<vtkPLYReader> model_reader = vtkSmartPointer<vtkPLYReader>::New();
  model_reader->SetFileName(model_path.c_str());
  model_reader->Update();

  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(model_reader->GetOutputPort());

  vtkSmartPointer<vtkActor> actor_model = vtkSmartPointer<vtkActor>::New();
  actor_model->SetMapper(mapper);
  actor_model->GetProperty()->SetAmbient(0.2);
  actor_model->GetProperty()->SetDiffuse(0.5);
  actor_model->GetProperty()->SetSpecular(0.2);
  renderer->AddActor(actor_model);

}

void Visualization::addNormals(const cv::Mat& vertices, const cv::Mat& normals)
{
  int skip = 2;
  double depth_min = 10;
  double depth_max = 25;
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  for (int v=0; v<vertices.rows; v+=skip) {
    for (int u=0; u<vertices.cols; u+=skip) {
      double depth = vertices.at<cv::Vec3d>(v,u)[2];
      if (depth > depth_max || depth < depth_min) continue;
      points->InsertNextPoint(vertices.at<cv::Vec3d>(v,u)[0],
                              vertices.at<cv::Vec3d>(v,u)[1],
                              vertices.at<cv::Vec3d>(v,u)[2]);

    }
  }

  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);

  // Set point normals
  vtkSmartPointer<vtkDoubleArray> pointNormalsArray = vtkSmartPointer<vtkDoubleArray>::New();
  pointNormalsArray->SetNumberOfComponents(3); //3d normals (ie x,y,z)
  pointNormalsArray->SetNumberOfTuples( polydata->GetNumberOfPoints());

  int index = 0;
  for (int v=0; v<normals.rows; v+=skip) {
    for (int u=0; u<normals.cols; u+=skip) {
      double depth = vertices.at<cv::Vec3d>(v,u)[2];
      if (depth > depth_max || depth < depth_min) continue;
      double normal[3] = {normals.at<cv::Vec3d>(v,u)[0],
                          normals.at<cv::Vec3d>(v,u)[1],
                          normals.at<cv::Vec3d>(v,u)[2]};
      pointNormalsArray->SetTuple(index++, normal) ;

    }
  }

  // Add the normals to the points in the polydata
  polydata->GetPointData()->SetNormals(pointNormalsArray);

  // Arrow that will be applied to each point
  vtkSmartPointer<vtkArrowSource> arrowSource = vtkSmartPointer<vtkArrowSource>::New();
  arrowSource->SetTipResolution(2);
  arrowSource->SetTipRadius(0.05);
  arrowSource->SetTipLength(0.2);
  arrowSource->SetShaftResolution(3);
  arrowSource->SetShaftRadius(0.01);

  vtkSmartPointer<vtkGlyph3D> glyph3D = vtkSmartPointer<vtkGlyph3D>::New();
  // Set 3D model to reader at each point
  glyph3D->SetSourceConnection(arrowSource->GetOutputPort());
  glyph3D->SetVectorModeToUseNormal();
  glyph3D->SetInputData(polydata);
  glyph3D->SetScaleFactor(0.2);
  //glyph3D->SetColorModeToColorByScalar();
  //glyph3D->SetScaleModeToScaleByVector();
  glyph3D->OrientOn();
  glyph3D->Update();

  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(glyph3D->GetOutputPort());

  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  renderer->AddActor(actor);
}

void Visualization::addNormals(const std::vector<Eigen::Vector3d>& vertices,
                               const std::vector<Eigen::Vector3d>& normals)
{

  int skip = 1;
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  for (unsigned int i=0; i<vertices.size(); i+=skip) {
    auto v = vertices.at(i);
    points->InsertNextPoint(v[0], v[1], v[2]);
  }

  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);

  // Set point normals
  vtkSmartPointer<vtkDoubleArray> pointNormalsArray = vtkSmartPointer<vtkDoubleArray>::New();
  pointNormalsArray->SetNumberOfComponents(3); //3d normals (ie x,y,z)
  pointNormalsArray->SetNumberOfTuples( (int)polydata->GetNumberOfPoints()/skip);

  int index = 0;
  for (unsigned int i=0; i<normals.size(); i+=skip) {
    auto n = normals.at(i);
    double normal[3] = {n[0], n[1], n[2]};
    pointNormalsArray->SetTuple(index++, normal) ;
  }

  // Add the normals to the points in the polydata
  polydata->GetPointData()->SetNormals(pointNormalsArray);

  // Arrow that will be applied to each point
  vtkSmartPointer<vtkArrowSource> arrowSource = vtkSmartPointer<vtkArrowSource>::New();
  arrowSource->SetTipResolution(3);
  arrowSource->SetTipRadius(0.04);
  arrowSource->SetTipLength(1.12);
  arrowSource->SetShaftResolution(3);
  arrowSource->SetShaftRadius(0.01);

  vtkSmartPointer<vtkGlyph3D> glyph3D = vtkSmartPointer<vtkGlyph3D>::New();
  // Set 3D model to reader at each point
  glyph3D->SetSourceConnection(arrowSource->GetOutputPort());
  glyph3D->SetVectorModeToUseNormal();
  glyph3D->SetInputData(polydata);
  glyph3D->SetScaleModeToScaleByVector();
  glyph3D->SetScaleFactor(1.0);
  glyph3D->SetColorModeToColorByVector();
  glyph3D->OrientOn();
  glyph3D->Update();

  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(glyph3D->GetOutputPort());

  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  //actor->GetProperty()->SetLighting(false);

  renderer->AddActor(actor);
}

void Visualization::addClusterCenter(const double x, const double y, const double z)
{
  vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->SetCenter(x, y, z);
  sphereSource->SetRadius(1.5);

  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(sphereSource->GetOutputPort());

  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  renderer->AddActor(actor);
}

void Visualization::addLocalCoordinateAxes(Eigen::Matrix4d &pose)
{
  Eigen::VectorXd x(4,1); x << 1,0,0,1;
  Eigen::VectorXd y(4,1); y << 0,1,0,1;
  Eigen::VectorXd z(4,1); z << 0,0,1,1;
  Eigen::VectorXd o(4,1); o << 0,0,0,1;

  Eigen::VectorXd xt,yt,zt,ot;
  xt = pose*x;
  yt = pose*y;
  zt = pose*z;
  ot = pose*o;

  // Create a vtkPoints container and store the points in it
  vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
  pts->InsertNextPoint(ot[0]/ot[3], ot[1]/ot[3], ot[2]/ot[3]);
  pts->InsertNextPoint(xt[0]/xt[3], xt[1]/xt[3], xt[2]/xt[3]);
  pts->InsertNextPoint(yt[0]/yt[3], yt[1]/yt[3], yt[2]/yt[3]);
  pts->InsertNextPoint(zt[0]/zt[3], zt[1]/zt[3], zt[2]/zt[3]);

  vtkSmartPointer<vtkPolyData> linesPolyData = vtkSmartPointer<vtkPolyData>::New();
  linesPolyData->SetPoints(pts);

  // Create the first line x
  vtkSmartPointer<vtkLine> line_x = vtkSmartPointer<vtkLine>::New();
  line_x->GetPointIds()->SetId(0, 0);
  line_x->GetPointIds()->SetId(1, 1);
  vtkSmartPointer<vtkLine> line_y = vtkSmartPointer<vtkLine>::New();
  line_y->GetPointIds()->SetId(0, 0);
  line_y->GetPointIds()->SetId(1, 2);
  vtkSmartPointer<vtkLine> line_z = vtkSmartPointer<vtkLine>::New();
  line_z->GetPointIds()->SetId(0, 0);
  line_z->GetPointIds()->SetId(1, 3);

  // Create a vtkCellArray container and store the lines in it
  vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
  lines->InsertNextCell(line_x);
  lines->InsertNextCell(line_y);
  lines->InsertNextCell(line_z);

  // Add the lines to the polydata container
  linesPolyData->SetLines(lines);

  // Set colors
  unsigned char red[3] = { 255, 0, 0 };
  unsigned char green[3] = { 0, 255, 0 };
  unsigned char blue[3] = {0, 0, 255};
  vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(3);
  colors->InsertNextTupleValue(red);
  colors->InsertNextTupleValue(green);
  colors->InsertNextTupleValue(blue);
  linesPolyData->GetCellData()->SetScalars(colors);

  // Setup the visualization pipeline
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputData(linesPolyData);
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetLineWidth(3.0);
  renderer->AddActor(actor);
}

void  Visualization::addZeroSurfaceFromGrid(const Eigen::MatrixXd& values,
                                            const Eigen::Matrix4d& pose,
                                            const Eigen::Vector3f& color,
                                            bool alpha,
                                            double isoValue,
                                            double opacity,
                                            double lineWidth) {

  double bin_size = 0.1;
  Eigen::Vector3d origin{0, 0, 0};
  int dx = 60;
  int dy = 40;
  int dz = 60;
  // Initialize the 3D grid needed by Marching Cubes
  vtkSmartPointer<vtkImageData> volume = vtkSmartPointer<vtkImageData>::New();
  volume->SetExtent(-30, 29,
                    -30, 9,
                    -30, 29);
  volume->SetSpacing(bin_size, bin_size, bin_size);
  volume->SetOrigin(origin[0], origin[1], origin[2]);
  volume->AllocateScalars(VTK_FLOAT, 1);

  double bounds[6];
  volume->GetBounds(bounds);

  // Initilaize all voxels as "outside"
  double default_sdf_value = 0.2;//tsdf->_truncation_value_pos;
  for (vtkIdType i=0; i<volume->GetNumberOfPoints(); i++) {
    volume->GetPointData()->GetScalars()->SetTuple1(i, default_sdf_value);
  }

  // Put the data from the tsdf-grid into the regular voxel-grid

  // Convert 1D-index to 3D-index
  for (int i=0; i<dx*dy*dz; i++) {
    int x = ((i/dy/dz)%dx)-30;
    int y = (i/dz)%dy-30;
    int z = (i%dz)-30;
    float value = values(i,0);
    volume->SetScalarComponentFromFloat(x, y, z, 0, value);
    //std::cout << values(i,0) << " - " << x << "x" << y << "x" << z <<  std::endl;
  }

  // Compute the surface at zero-crossing using marching cubes
  vtkSmartPointer<vtkMarchingCubes> surface = vtkSmartPointer<vtkMarchingCubes>::New();
  surface->SetInputData(volume);
  surface->ComputeNormalsOn();
  surface->SetValue(0, isoValue);

  // Pass the surface to vtk render pipeline: mapper -> actor ...
  vtkSmartPointer<vtkPolyDataMapper> mapper_isosurface = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper_isosurface->SetInputConnection(surface->GetOutputPort());
  mapper_isosurface->ScalarVisibilityOff();
  vtkSmartPointer<vtkActor> actor_isosurface = vtkSmartPointer<vtkActor>::New();
  actor_isosurface->SetMapper(mapper_isosurface);

  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  transform->PostMultiply(); //this is the key line
  vtkSmartPointer<vtkMatrix4x4> m = vtkSmartPointer<vtkMatrix4x4>::New();
  for (int i=0; i<4; i++) {
    for (int j=0; j<4; j++) {
      m->SetElement(i,j,pose(i,j));
    }
  }
  transform->SetMatrix(m);
  actor_isosurface->SetUserTransform(transform);
  if (alpha) {
    actor_isosurface->GetProperty()->SetOpacity(opacity); //0.2
    actor_isosurface->GetProperty()->SetRepresentationToWireframe();
    actor_isosurface->GetProperty()->SetLineWidth(lineWidth); // 2.0
    actor_isosurface->GetProperty()->SetLighting(false);
  }
  actor_isosurface->GetProperty()->SetColor(color[0]/255.0,color[1]/255.0,color[2]/255.0);

  // Outline box
  vtkSmartPointer<vtkOutlineFilter> outline =  vtkSmartPointer<vtkOutlineFilter>::New();
  outline->SetInputData(volume);
  vtkSmartPointer<vtkPolyDataMapper> outlineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  outlineMapper->SetInputConnection(outline->GetOutputPort());
  vtkSmartPointer<vtkActor> outlineActor = vtkSmartPointer<vtkActor>::New();
  outlineActor->SetMapper(outlineMapper);
  outlineActor->SetUserTransform(transform);
  outlineActor->GetProperty()->SetColor(0.,0.,0.);
  outlineActor->GetProperty()->SetLineWidth(2);

  renderer->AddActor(actor_isosurface);
  //renderer->AddActor(outlineActor);
}

void  Visualization::addGridSamples(const Eigen::MatrixXd& values,
                                    const Eigen::Matrix4d& pose,
                                    const Eigen::Vector3f& color,
                                    bool alpha) {
  double bin_size = 0.1;
  Eigen::Vector3d origin{0, 0, 0};
  int dx = 60;
  int dy = 40;
  int dz = 60;
  // Initialize the 3D grid needed by Marching Cubes
  vtkSmartPointer<vtkImageData> volume = vtkSmartPointer<vtkImageData>::New();
  volume->SetExtent(-30, 30,
                    -30, 10,
                    -30, 30);
  volume->SetSpacing(bin_size, bin_size, bin_size);
  volume->SetOrigin(origin[0], origin[1], origin[2]);
  volume->AllocateScalars(VTK_FLOAT, 1);

  double bounds[6];
  volume->GetBounds(bounds);

  // Initilaize all voxels as "outside"
  double default_sdf_value = 0.2;//tsdf->_truncation_value_pos;
  for (vtkIdType i=0; i<volume->GetNumberOfPoints(); i++) {
    volume->GetPointData()->GetScalars()->SetTuple1(i, default_sdf_value);
  }

  // Put the data from the tsdf-grid into the regular voxel-grid

  // Convert 1D-index to 3D-index
  for (int i=0; i<dx*dy*dz; i++) {
    int x = ((i/dy/dz)%dx)-30;
    int y = (i/dz)%dy-30;
    int z = (i%dz)-30;
    float value = values(i,0);
    volume->SetScalarComponentFromFloat(x, y, z, 0, value);
    //std::cout << values(i,0) << " - " << x << "x" << y << "x" << z <<  std::endl;
  }

  // Compute the surface at zero-crossing using marching cubes
  vtkSmartPointer<vtkMarchingCubes> surface = vtkSmartPointer<vtkMarchingCubes>::New();
  surface->SetInputData(volume);
  surface->ComputeNormalsOn();
  double isoValue[] = {0.0, 0.999};
  surface->SetValue(0, isoValue[0]);
  surface->SetValue(1, isoValue[1]);

  // Pass the surface to vtk render pipeline: mapper -> actor ...
  vtkSmartPointer<vtkPolyDataMapper> mapper_isosurface = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper_isosurface->SetInputConnection(surface->GetOutputPort());
  mapper_isosurface->ScalarVisibilityOff();
  vtkSmartPointer<vtkActor> actor_isosurface = vtkSmartPointer<vtkActor>::New();
  actor_isosurface->SetMapper(mapper_isosurface);

  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  transform->PostMultiply(); //this is the key line
  vtkSmartPointer<vtkMatrix4x4> m = vtkSmartPointer<vtkMatrix4x4>::New();
  for (int i=0; i<4; i++) {
    for (int j=0; j<4; j++) {
      m->SetElement(i,j,pose(i,j));
    }
  }
  transform->SetMatrix(m);
  actor_isosurface->SetUserTransform(transform);
  if (alpha) {
    actor_isosurface->GetProperty()->SetOpacity(0.2);
    actor_isosurface->GetProperty()->SetRepresentationToWireframe();
    actor_isosurface->GetProperty()->SetLineWidth(2.0);
    actor_isosurface->GetProperty()->SetLighting(false);
  }
  actor_isosurface->GetProperty()->SetColor(color[0]/255.0,color[1]/255.0,color[2]/255.0);

  // Outline box
  vtkSmartPointer<vtkOutlineFilter> outline =  vtkSmartPointer<vtkOutlineFilter>::New();
  outline->SetInputData(volume);
  vtkSmartPointer<vtkPolyDataMapper> outlineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  outlineMapper->SetInputConnection(outline->GetOutputPort());
  vtkSmartPointer<vtkActor> outlineActor = vtkSmartPointer<vtkActor>::New();
  outlineActor->SetMapper(outlineMapper);
  outlineActor->SetUserTransform(transform);
  outlineActor->GetProperty()->SetColor(color[0]/255.0,color[1]/255.0,color[2]/255.0);

  renderer->AddActor(actor_isosurface);
  //renderer->AddActor(outlineActor);
}


void Visualization::addRectangle2d( double value, double pos_x, double pos_y, double scaling ) {
  addText2d("____________________", pos_x, pos_y,Eigen::Vector3f(0.5,0.5,0.5));
  double offset_y = -9;
  double offset_x = 110;
  addText2d("|", pos_x+offset_x-1, pos_y+offset_y-2,Eigen::Vector3f(0.5,0.5,0.5));
  addText2d("•", pos_x+offset_x + value*scaling, pos_y+offset_y);
}

void Visualization::addPlaneCut(const Eigen::MatrixXd &values,
                                vtkPlane* plane, bool legend)
{ 
  double bin_size = 0.1;
  int dx = 60;
  int dy = 40;
  int dz = 60;
  // Initialize the 3D grid needed by Marching Cubes
  vtkSmartPointer<vtkImageData> volume = vtkSmartPointer<vtkImageData>::New();
  volume->SetExtent(-30, 29,
                    -30, 9,
                    -30, 29);
  volume->SetSpacing(bin_size, bin_size, bin_size);
  volume->SetOrigin(0, 0, 0);
  volume->AllocateScalars(VTK_FLOAT, 1);

  double bounds[6];
  volume->GetBounds(bounds);

  // Initilaize all voxels as "outside"
  double default_sdf_value = 0.2;//tsdf->_truncation_value_pos;
  for (vtkIdType i=0; i<volume->GetNumberOfPoints(); i++) {
    volume->GetPointData()->GetScalars()->SetTuple1(i, default_sdf_value);
  }

  // Put the data from the tsdf-grid into the regular voxel-grid

  // Convert 1D-index to 3D-index
  for (int i=0; i<dx*dy*dz; i++) {
    int x = ((i/dy/dz)%dx)-30;
    int y = (i/dz)%dy-30;
    int z = (i%dz)-30;
    float value = values(i,0);
    volume->SetScalarComponentFromFloat(x, y, z, 0, value);
  }

  vtkSmartPointer<vtkCutter> planeCut = vtkSmartPointer<vtkCutter>::New();
  planeCut->SetInputData(volume);
  planeCut->SetCutFunction(plane);
  planeCut->GenerateCutScalarsOff();

  vtkSmartPointer<vtkLookupTable> lut =vtkSmartPointer<vtkLookupTable>::New();
  int tableSize = std::max(0.2 + 10, 10.0);
  lut->SetNumberOfTableValues(tableSize);
  lut->Build();

  // Fill in a few known colors, the rest will be generated if needed
  /*lut->SetTableValue(0     , 0     , 0     , 0, 1);  //Black
  lut->SetTableValue(1, 0.8900, 0.8100, 0.3400, 1); // Banana
  lut->SetTableValue(2, 1.0000, 0.3882, 0.2784, 1); // Tomato
  lut->SetTableValue(3, 0.9608, 0.8706, 0.7020, 1); // Wheat
  lut->SetTableValue(4, 0.9020, 0.9020, 0.9804, 1); // Lavender
  lut->SetTableValue(5, 1.0000, 0.4900, 0.2500, 1); // Flesh
  lut->SetTableValue(6, 0.5300, 0.1500, 0.3400, 1); // Raspberry
  lut->SetTableValue(7, 0.9804, 0.5020, 0.4471, 1); // Salmon
  lut->SetTableValue(8, 0.7400, 0.9900, 0.7900, 1); // Mint
  lut->SetTableValue(9, 0.2000, 0.6300, 0.7900, 1); // Peacock*/

  vtkSmartPointer<vtkPolyDataMapper> cutMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  cutMapper->SetInputConnection(planeCut->GetOutputPort());
  cutMapper->SetScalarRange(values.minCoeff(), values.maxCoeff());
  //cutMapper->SetLookupTable(lut);

  vtkSmartPointer<vtkActor> cutActor = vtkSmartPointer<vtkActor>::New();
  cutActor->SetMapper(cutMapper);
  cutActor->GetProperty()->SetAmbient(1.0);
  cutActor->GetProperty()->SetDiffuse(0.0);
  //cutActor->GetProperty()->SetEdgeColor(255,0,0);
  //cutActor->GetProperty()->SetEdgeVisibility(true);
  //cutActor->GetProperty()->SetSpecular(0.2);
  renderer->AddActor(cutActor);

  // Outline box
  vtkSmartPointer<vtkOutlineFilter> outline =  vtkSmartPointer<vtkOutlineFilter>::New();
  outline->SetInputConnection(planeCut->GetOutputPort());
  vtkSmartPointer<vtkPolyDataMapper> outlineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  outlineMapper->SetInputConnection(outline->GetOutputPort());
  vtkSmartPointer<vtkActor> outlineActor = vtkSmartPointer<vtkActor>::New();
  outlineActor->SetMapper(outlineMapper);
  //outlineActor->SetUserTransform(transform);
  outlineActor->GetProperty()->SetColor(0,0,0);
  outlineActor->GetProperty()->SetLineWidth(2);
  renderer->AddActor(outlineActor);

  if(legend){
    vtkSmartPointer<vtkScalarBarActor> scalarBar = vtkSmartPointer<vtkScalarBarActor>::New();
    scalarBar->SetLookupTable(cutMapper->GetLookupTable());
    scalarBar->SetTitle("");
    scalarBar->SetNumberOfLabels(3);
    renderer->AddActor2D(scalarBar);
  }
}

void Visualization::addText(const std::string& text, const Eigen::Vector3d& translation, const Eigen::Vector3f& color)
{
  vtkSmartPointer<vtkVectorText> textSource = vtkSmartPointer<vtkVectorText>::New();
  textSource->SetText(text.c_str());
  textSource->Update();
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(textSource->GetOutputPort());
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(color(0)/255, color(1)/255, color(2)/255);
  vtkSmartPointer<vtkTransform> text_transform = vtkSmartPointer<vtkTransform>::New();
  text_transform->PostMultiply();
  text_transform->Scale(0.5,0.5,0.5);
  text_transform->RotateZ(180);
  text_transform->RotateY(180);
  text_transform->Translate(translation[0],
                            translation[1],
                            translation[2]);
  actor->SetUserTransform(text_transform);
  renderer->AddActor(actor);
}

void Visualization::addText2d(const std::string &text, double pos_x, double pos_y, const Eigen::Vector3f& color)
{
  vtkSmartPointer<vtkTextActor> textActor = vtkSmartPointer<vtkTextActor>::New();
  textActor->SetInput (text.c_str());
  textActor->SetPosition ( pos_x, pos_y ); // Distance from lower left corner
  textActor->GetTextProperty()->SetFontSize ( 30 );
  textActor->GetTextProperty()->SetColor (color[0]/255, color[1]/255, color[2]/255);
  renderer->AddActor2D ( textActor );
}

void Visualization::addCallbackCommand(vtkSmartPointer<vtkCallbackCommand> keypressCallback)
{
  renderWindowInteractor->AddObserver(vtkCommand::KeyPressEvent, keypressCallback);
}

void Visualization::addCallbackCommand(void (*f)(vtkObject *caller, unsigned long eid,
                                                 void *clientdata, void *calldata))
{
  vtkSmartPointer<vtkCallbackCommand> keypressCallback = vtkSmartPointer<vtkCallbackCommand>::New();
  keypressCallback->SetCallback(f);
  this->addCallbackCommand(keypressCallback);
  renderWindowInteractor->AddObserver(vtkCommand::KeyPressEvent, keypressCallback);
}

void Visualization::renderImage(cv::Mat &image) {

  //renderWindow->SetOffScreenRendering(1);

  int height = image.rows;
  int width = image.cols;

  vtkSmartPointer<vtkPoints> Vertices = vtkSmartPointer<vtkPoints>::New();

  renderWindow->Render();
  renderWindow->WaitForCompletion(); // Very important, otherwise the Vertices array can sometimes be empty!

  unsigned char *pixels = renderWindow->GetRGBACharPixelData(0, 0, width-1, height-1, true);
  image = cv::Mat(height, width, CV_8UC4, pixels);
  cv::flip(image,image, 0);
  cv::cvtColor(image, image, CV_RGBA2BGRA);
  //renderWindow->SetOffScreenRendering(0);
  //renderWindow->Finalize();
}

void Visualization::renderDepth(cv::Mat &depth, bool flipped)
{
  renderWindow->OffScreenRenderingOn();

  if (!flipped) {
    renderer->GetActiveCamera()->SetViewUp(0,1,0);
  }

  int height = depth.rows;
  int width = depth.cols;

  vtkSmartPointer<vtkPoints> Vertices = vtkSmartPointer<vtkPoints>::New();
  //do {
    renderWindow->Render();
    renderWindow->WaitForCompletion(); // Very important, otherwise the Vertices array can sometimes be empty!

    // Read depth buffer
    vtkSmartPointer<vtkFloatArray> depths = vtkSmartPointer<vtkFloatArray>::New();
    renderWindow->GetZbufferData(0,0, width-1, height-1, depths);

    double point[4];
    unsigned row = height-1;
    for (int i=0; i<height; i++) {
      for (int j=0; j<width; j++) {
        unsigned indx = row * width + j;
        double d = depths->GetValue(indx);
        renderer->SetDisplayPoint(j,i,d);
        renderer->DisplayToWorld();
        renderer->GetWorldPoint(point);
        d = point[2]/point[3];
        if (d>350) continue;

        double x = point[0]/point[3];
        double y = point[1]/point[3];
        double z = point[2]/point[3];
        if (z>350) continue;
        cv::Vec3d po;
        po[0] = x; po[1] = y; po[2] = z;
        depth.at<cv::Vec3d>(i,j) = po;
        //Vertices->InsertNextPoint(x,y,z);
      }
      row--;
    }

  //} while (Vertices->GetNumberOfPoints()==0);
  //std::cout << " Vertices: " << Vertices->GetNumberOfPoints() << std::endl;
}

void Visualization::setK(const Eigen::Matrix3d &K, const int width, const int height)
{
  double principal_pt_x = K(0,2);
  double principal_pt_y = K(1,2);
  int nx = width;
  int ny = height;
  double focal_len = K(0,0);

  vtkSmartPointer<vtkCamera> camera = renderer->GetActiveCamera();

  // convert the principal point to window center (normalized coordinate system) and set it
  double wcx = -2*(principal_pt_x - double(nx)/2) / nx;
  double wcy =  2*(principal_pt_y - double(ny)/2) / ny;
  camera->SetWindowCenter(wcx, wcy);

  // convert the focal length to view angle and set it
  double view_angle = (2.0 * std::atan2( ny/2.0, focal_len )) * 180.0/M_PI;
  //std::cout << "view_angle = " << view_angle << std::endl;
  camera->SetViewAngle( view_angle );
}

void Visualization::setCameraPosition(const Eigen::Vector3d& pos, double dist)
{
  vtkSmartPointer<vtkCamera> camera = renderer->GetActiveCamera();
  camera->SetPosition(pos[0], pos[1], pos[2]);
  camera->SetDistance(dist);
}

void Visualization::setCameraPosition(const Eigen::Vector3d& pos)
{
  vtkSmartPointer<vtkCamera> camera = renderer->GetActiveCamera();
  camera->SetPosition(pos[0], pos[1], pos[2]);
}

void Visualization::setCameraFocalPoint(const Eigen::Vector3d& fp)
{
  vtkSmartPointer<vtkCamera> camera = renderer->GetActiveCamera();
  camera->SetFocalPoint(fp[0], fp[1], fp[2]);
}

void Visualization::setWindowSize(const int width, const int height)
{
  int size[2] = {width, height};
  renderWindow->SetSize(size);
}

void Visualization::printCameraPosition()
{
  vtkSmartPointer<vtkCamera> camera = renderer->GetActiveCamera();
  double* camPos = camera->GetPosition();
  double* camFP = camera->GetFocalPoint();
  std::cout << "Position: \t" << camPos[0] << " " << camPos[1] << " " << camPos[2] << std::endl;
  std::cout << "Focal Point: \t" << camFP[0] << " " << camFP[1] << " " << camFP[2] << std::endl;
  std::cout << "Distance: \t" << camera->GetDistance() << std::endl;
}

void Visualization::setNearClippingRange(double d)
{
  renderer->GetActiveCamera()->SetClippingRange(d,10000);
}

void Visualization::takeScreenshot2(const std::string &path)
{
  renderWindow->Render();
  renderWindow->WaitForCompletion();

  vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
  windowToImageFilter->SetInput(renderWindow);
  windowToImageFilter->SetInputBufferTypeToRGB(); //also record the alpha (transparency) channel
  windowToImageFilter->ReadFrontBufferOff(); // read from the back buffer
  windowToImageFilter->Update();

  vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
  writer->SetFileName(path.c_str());
  writer->SetInputConnection(windowToImageFilter->GetOutputPort());
  writer->Write();
}

void Visualization::takeMovie(const std::string &path,
                              const std::vector<Eigen::Vector3d>& waypoints,
                              int width, int height)
{
  int index = 10000;
  for (unsigned int w=0; w<waypoints.size()-1; w++) {
    Eigen::Vector3d start = waypoints[w];
    Eigen::Vector3d end = waypoints[w+1];
    Eigen::Vector3d diff = end-start;
    double distance = (end-start).norm();
    diff.normalize();
    double step_size = distance/100.0;
    for (double i=0; i<distance; i+=step_size) {
      // Move camera
      // double pos[3];
      // renderer->GetActiveCamera()->GetPosition(pos);
      // pos[2]+=0.05;

      Eigen::Vector3d curr_pos = start+diff*i;
      renderer->GetActiveCamera()->SetPosition(curr_pos[0],curr_pos[1],curr_pos[2]);
      renderWindow->Render();
      renderWindow->WaitForCompletion();

      vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
      windowToImageFilter->SetInput(renderWindow);
      //windowToImageFilter->SetMagnification(3); //set the resolution of the output image (3 times the current resolution of vtk render window)
      windowToImageFilter->SetInputBufferTypeToRGB(); //also record the alpha (transparency) channel
      windowToImageFilter->ReadFrontBufferOff(); // read from the back buffer

      // Write image
      std::string output_path = path+std::to_string(index++)+std::string(".png");
      vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
      windowToImageFilter->Update();
      writer->SetInputConnection(windowToImageFilter->GetOutputPort());
      writer->SetFileName(output_path.c_str());
      writer->SetInputConnection(windowToImageFilter->GetOutputPort());
      writer->Write();
    }
  }
}

void Visualization::takeMovie2(const std::string &path,
                              const std::vector<Eigen::Vector3d>& waypoints,
                              const std::vector<Eigen::Vector3d>& focalpoints,
                              const std::vector<double>& speeds,
                              int width, int height)
{
  std::cout << "recording movie..."<< std::endl;
  int index = 0;
  for (unsigned int w=0; w<waypoints.size()-1; w++) {
    Eigen::Vector3d start_wp = waypoints[w];
    Eigen::Vector3d end_wp = waypoints[w+1];
    Eigen::Vector3d diff_wp = end_wp-start_wp;
    double diff_wp_norm = diff_wp.norm();

    Eigen::Vector3d start_fp = focalpoints[w];
    Eigen::Vector3d end_fp = focalpoints[w+1];
    Eigen::Vector3d diff_fp = end_fp-start_fp;
    double diff_fp_norm = diff_fp.norm();

    //double step_size = distance/100.0;
    for (double i=0; i<1; i+=speeds.at(w)) {
      // Move camera
      // double pos[3];
      // renderer->GetActiveCamera()->GetPosition(pos);
      // pos[2]+=0.05;

      Eigen::Vector3d curr_pos = start_wp + diff_wp*i;
      Eigen::Vector3d curr_focal = start_fp + diff_fp*i;

      renderer->GetActiveCamera()->SetPosition(curr_pos[0],curr_pos[1],curr_pos[2]);
      renderer->GetActiveCamera()->SetFocalPoint(curr_focal[0],curr_focal[1],curr_focal[2]);
      renderWindow->Render();
      renderWindow->WaitForCompletion();

      //show();
      //continue;

      vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
      windowToImageFilter->SetInput(renderWindow);
      //windowToImageFilter->SetMagnification(3); //set the resolution of the output image (3 times the current resolution of vtk render window)
      windowToImageFilter->SetInputBufferTypeToRGB(); //also record the alpha (transparency) channel
      windowToImageFilter->ReadFrontBufferOff(); // read from the back buffer

      // Write image
      std::string str = std::to_string(i++);
      str = std::string (8-str.length(), '0') + str;
      std::string output_path = path + str + std::string(".png");
      vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
      windowToImageFilter->Update();
      writer->SetInputConnection(windowToImageFilter->GetOutputPort());
      writer->SetFileName(output_path.c_str());
      writer->SetInputConnection(windowToImageFilter->GetOutputPort());
      writer->Write();
    }
  }
}

}
