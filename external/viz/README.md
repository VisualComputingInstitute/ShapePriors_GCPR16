# What is VIZ?
VIZ is a lightweight 3D visualization library based on VTK.

![alt tag](viz_logo.png)

Inspired by the functionality of the famouse PCL (Pointcloud Library),
VIZ tries to be a more lightweight visualiation library that comes without a large list of dependencies.
Its only required dependencies are VTK and Eigen. If desired, it can also output rendered views into OpenCV.

# Motivation
Visualizations are en essantial part of debugging computer vision code.
Hence, it should be as easy and as convenient as possible.

# Installation
Go to root dir of the repository, then type:
* `mkdir build; cd build;`
* Configure the build using [cmake](http://www.cmake.org/cmake/resources/software.html): `cmake ..`
* `make; sudo make install`

# Standards
* Coordinate sytem orientation
* Color scaling and type: always in 0..255 
