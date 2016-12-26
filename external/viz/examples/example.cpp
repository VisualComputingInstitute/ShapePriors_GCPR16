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

#include "../src/viz.h"

// Hello World Example
int main(int argc, char** argv) {
  viz::Visualization v(800,600);
  v.lookFromAt(Eigen::Vector3d(-5,-3,-12), Eigen::Vector3d(5,0,0));
  v.addText("VIZ - lightweight wrapper for VTK", Eigen::Vector3d(0,-0.3,0));
  v.addAxes();

  // Bounding Box Example
  Eigen::Vector3d size(1,2,3);
  Eigen::Vector3d translation(0,0,1);
  Eigen::Vector3f color(255,0,0);
  double rot = 0;
  v.addBoundingBox(size,translation,rot,color);

  v.show();
  return 0;
}
