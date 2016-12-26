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

#include "shape.h"

namespace gvl {

double optimize_shape(gvl::Detection& detection, bool visualize, int* it) {

  Eigen::MatrixXd& S = gvl::ShapeSpacePCA::instance().S;
  Eigen::MatrixXd& V = gvl::ShapeSpacePCA::instance().V;
  Eigen::MatrixXd& mean = gvl::ShapeSpacePCA::instance().mean;
  std::vector<Eigen::Vector3i>& n_ids = gvl::ShapeSpacePCA::instance().n_ids;
    
  ceres::Problem problem;
  auto& pose = detection.pose;
  auto poseInv = pose.inverse();

  // For each point in the cloud add a residual
  for (unsigned int i = 0; i < detection.pointcloud->points.size(); i++) {
    auto& p = detection.pointcloud->points.at(i);
    Eigen::Vector4d P = poseInv*Eigen::Vector4d(p.x, p.y, p.z, 1.0); P/=P[3];
    double d_std_dev = 1;
    double N = detection.pointcloud->points.size();

    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<SDF_Residual, 1, r>(
          new SDF_Residual(P[0], P[1], P[2], d_std_dev));
    problem.AddResidualBlock(cost_function,
                             new ceres::ScaledLoss(new ceres::HuberLoss(0.1),1.0/(N*0.03),ceres::TAKE_OWNERSHIP),
                             detection.z.data());
  }

  double lambda_shape_reg = 1;
  // Add regularizer for each component of z
  ceres::CostFunction* regularizer_function =
     new ceres::AutoDiffCostFunction<Regularizer_z, r, r>
      (new Regularizer_z(1, lambda_shape_reg));
  problem.AddResidualBlock(regularizer_function, NULL, detection.z.data());

  viz::Visualization visualizer(1024,768);
  visualizer.addAxes();
  visualizer.lookFromAt(Eigen::Vector3d(4,-4,-10), Eigen::Vector3d(0.5,-0.2,0));

  ceres::Solver::Options options;
  options.update_state_every_iteration = true;
  bool bfgs = false;
  if( bfgs ) {
    options.minimizer_type = ceres::LINE_SEARCH;
    options.line_search_direction_type = ceres::LBFGS;
  }
  else {
    options.minimizer_type = ceres::TRUST_REGION; // Trust region options
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT; // Trust region options
    options.linear_solver_type = ceres::DENSE_QR;
  }
  options.max_num_iterations = 100;

  if (visualize) {
    ShapeVisualizerCallback callback(detection, visualizer, it);
    options.callbacks.push_back(&callback);
    options.minimizer_progress_to_stdout = true;
  }

  ceres::Solver::Summary summary;
  Solve(options, &problem, &summary);
  if (visualize) std::cout << summary.FullReport() << std::endl;
  else std::cout << summary.BriefReport() << std::endl;

  // Reconstruct high-dimension shape from -------------------------------------------------------
  detection.shape = V.block<p, r>(0,0)*detection.z + mean;

  return summary.final_cost;
}

}
