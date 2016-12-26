# Dependencies
The following libs are rquired:
* VTK 7
* OpenCV
* Eigen
* CeresSolver

# Compiling
First build VIZ in './external/viz' following the instructions given there.
Then build using normal cmake procedure.
```
mkdir build; cd build;
cmake ..; make
```

# Usage example:
The following is an example run. All needed precomputations are already given in /data.
```
‘./ShapePrior ../data/kitti/image_2/000046_ 10 10 2 000046 ../data/kitti/poses/000046.txt ../data/kitti/detections/000046_ ../data/kitti/disparity/000046_ ../data/kitti/calib/000046.txt ../data/kitti/planes/000046_ ../data/kitti/results/kapp3/ 1' 
```

Once the optimization has converged, a window will appear showing the input on the left and our result on the right.
You can zoom in into the car, by pointing with the mouse at it an pressing 'F'.
Press 'esc' to close.

# Citation
If you find this code useful please cite us:
@inproceedings{EngelmannGCPR16_shapepriors, 
title = {Joint Object Pose Estimation and Shape Reconstruction in Urban Street Scenes Using {3D} Shape Priors},
author = {Francis Engelmann and J\"org St\"uckler and Bastian Leibe},
booktitle = {Proc. of the German Conference on Pattern Recognition (GCPR)},
year = {2016}}
