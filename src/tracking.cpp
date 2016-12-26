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

// Own includes
#include "tracking.h"
#include "geometry.h"

namespace gvl {

double compute_IoU_2D(gvl::BoundingBox& bb1, gvl::BoundingBox& bb2) {

  double intersection_width = std::max(0.0,std::min(bb1.right, bb2.right) - std::max(bb1.left,bb2.left));
  double intersection_height = std::max(0.0,std::min(bb1.bottom,bb2.bottom) - std::max(bb1.top,bb2.top));
  double intersection = intersection_width*intersection_height;

  if (bb1.right < bb2.left || bb2.right < bb1.left) intersection = 0;
  if (bb1.bottom < bb2.top || bb2.bottom < bb1.top) intersection = 0;

  double bb1_width = bb1.right - bb1.left;
  double bb2_width = bb2.right - bb2.left;

  double bb1_height = bb1.bottom - bb1.top;
  double bb2_height = bb2.bottom - bb2.top;

  double union_ = bb1_width*bb1_height + bb2_width*bb2_height - intersection;
  double IoU = intersection/union_;

  return IoU;
}

void solve_associations_greedy(Eigen::MatrixXd &association_costs, Eigen::MatrixXi &associations)
{
  // Iterate until all assignments are done
  for (int i=0; i<associations.rows(); i++) associations(i,0) = -1; // Setting all associations to

  // Execute as many times as we have tracks, then all cells should be marked as visited
  for (int it=0; it<association_costs.rows(); it++) {
    std::tuple<unsigned int, unsigned int> min_id;
    double min_cost = DBL_MAX;

    // Solve association problem using cost matrix.
    for (int r=0; r<association_costs.rows(); r++) {
      for (int c=0; c<association_costs.cols(); c++) {
        double curr_cost = association_costs(r,c);
        if (curr_cost < min_cost) {
          min_cost = curr_cost;
          min_id = std::make_tuple<unsigned int, unsigned int>(r,c);
        }
      }
    }

    if (min_cost == DBL_MAX) break;

    // Assign cheapest detection
    associations(std::get<0>(min_id), 0) = std::get<1>(min_id);

    // Set column and row of assignment to Max values so we ignore them in the next round
    for (unsigned int c=0; c<association_costs.cols(); c++) association_costs(std::get<0>(min_id),c) = DBL_MAX;
    for (unsigned int r=0; r<association_costs.rows(); r++) association_costs(r,std::get<1>(min_id)) = DBL_MAX;
  }
}

}

