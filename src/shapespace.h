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

#ifndef GVL_SHAPESPACE_H
#define GVL_SHAPESPACE_H

// Eigen includes
#include <Eigen/Core>
#include <Eigen/Dense>

// C/C++ includes
#include <vector>

// Own includes
#include "optimization/defines.h"
#include "utils.h"

namespace gvl {

/**
 * @brief The ShapeSpacePCA class implements PCA shape space
 */

class ShapeSpacePCA
{
public:

    static ShapeSpacePCA& instance() {
        static ShapeSpacePCA m_instance;
        return m_instance;
    }
    
    Eigen::MatrixXd S;    // Diagonal-matrix stored as vector
    Eigen::MatrixXd V;    // Vectors
    Eigen::MatrixXd mean; // Mean shape
    std::vector<Eigen::Vector3i> n_ids;

private:
    
    ShapeSpacePCA() {
        S = Eigen::MatrixXd(m,1);    // Diagonal-matrix stored as vector
        V = Eigen::MatrixXd(p,m);    // Vectors
        mean = Eigen::MatrixXd(p,1); // Mean shape
        gvl::populate_neighborhood(n_ids);
    }
    
public:
    ShapeSpacePCA(ShapeSpacePCA const&) = delete;
    void operator=(ShapeSpacePCA const&) = delete;
        
};    


}

#endif // GVL_SHAPESPACE_H
