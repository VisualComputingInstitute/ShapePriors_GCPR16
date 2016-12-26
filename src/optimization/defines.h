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

#ifndef GVL_OPTIMIZATION_DEFINES_H
#define GVL_OPTIMIZATION_DEFINES_H

#include <string>

const int dx = 60; // Number of voxels along x direction
const int dy = 40; // Number of voxels along y direction
const int dz = 60; // Number of voxels along z direction

const double limits[] = {-3.0, 3.0, -3.0, 1.0, -3.0, 3.0};

const int r = 5;        // Dimension of subspace
const int m = 37;       // Number of training samples
const int p = dx*dy*dz; // Dimension of original space

const double bin_size = 0.1; // Voxel grid bin size
const double lambda_dist2cam = 50; // Max distance to camera of considering points

const std::string path_S = "../data/matrices/S.bin";
const std::string path_V = "../data/matrices/V.bin";
const std::string path_mean_shape = "../data/matrices/mean_shape.bin";

#endif
