/**
* This file is part of COVINS.
*
* Copyright (C) 2018-2021 Patrik Schmuck / Vision for Robotics Lab
* (ETH Zurich) <collaborative (dot) slam (at) gmail (dot) com>
* For more information see <https://github.com/VIS4ROB-lab/covins>
*
* COVINS is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the license, or
* (at your option) any later version.
*
* COVINS is distributed to support research and development of
* multi-agent system, but WITHOUT ANY WARRANTY; without even the
* implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. In no event will the authors be held liable for any damages
* arising from the use of this software. See the GNU General Public
* License for more details.
*
* You should have received a copy of the GNU General Public License
* along with COVINS. If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * FrameAbsolutePoseSacProblem.hpp
 * @brief Header for the FrameAbsolutePoseSacProblem class
 * @author: Marco Karrer
 * Created on: Jan 23, 2017
 */

#pragma once

// The opengv includes
#include <opengv/types.hpp>
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>
#include "matcher/opengv/absolute_pose/FrameNoncentralAbsoluteAdapter.h"

namespace opengv {

namespace sac_problems {

namespace absolute_pose {

  /// @brief Provides functions for fitting an absolute-pose model to a set of
  ///        bearing-vector to point correspondences, using different algorithms (central
  ///        and non-central ones). Used in a sample-consenus paradigm for rejecting
  ///        outlier correspondences.
class FrameAbsolutePoseSacProblem : public AbsolutePoseSacProblem {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef AbsolutePoseSacProblem base_t;

  /** The type of adapter that is expected by the methods */
  using base_t::adapter_t;
  /** The possible algorithms for solving this problem */
  using base_t::algorithm_t;
  /** The model we are trying to fit (transformation) */
  using base_t::model_t;


  /// @brief Constructor.
  /// @param adapter Visitor holding bearing vectors, world points, etc.
  /// @param algorithm The algorithm we want to use.
  /// @warning Only absolute_pose::FrameNoncentralAbsoluteAdapter supported.
  FrameAbsolutePoseSacProblem(adapter_t & adapter, algorithm_t algorithm)
      : base_t(adapter, algorithm),
        adapterDerived_(
            *static_cast<opengv::absolute_pose::FrameNoncentralAbsoluteAdapter*>(&_adapter)) {
  }

  /// @brief Constructor.
  /// @param adapter Visitor holding bearing vectors, world points, etc.
  /// @param algorithm The algorithm we want to use.
  /// @param indices A vector of indices to be used from all available
  ///                    correspondences.
  /// @warning Only absolute_pose::FrameNoncentralAbsoluteAdapter supported.
  FrameAbsolutePoseSacProblem(adapter_t & adapter, algorithm_t algorithm,
                              const std::vector<int> & indices)
      : base_t(adapter, algorithm, indices),
        adapterDerived_(
            *static_cast<opengv::absolute_pose::FrameNoncentralAbsoluteAdapter*>(&_adapter)) {
  }

  virtual ~FrameAbsolutePoseSacProblem() {
  }

  /// @brief Compute the distances of all samples whith respect to given model
  ///        coefficients.
  /// @param model The coefficients of the model hypothesis.
  /// @param indices The indices of the samples of which we compute distances.
  /// @param scores The resulting distances of the selected samples. Low
  ///                    distances mean a good fit.
  virtual void getSelectedDistancesToModel(const model_t & model,
                                           const std::vector<int> & indices,
                                           std::vector<double> & scores) const {
    //compute the reprojection error of all points
    //compute inverse transformation
    model_t inverseSolution;
    inverseSolution.block<3, 3>(0, 0) = model.block<3, 3>(0, 0).transpose();
    inverseSolution.col(3) = -inverseSolution.block<3, 3>(0, 0) * model.col(3);

    Eigen::Matrix<double, 4, 1> p_hom;
    p_hom[3] = 1.0;

    for (size_t i = 0; i < indices.size(); i++) {
      //get point in homogeneous form
      p_hom.block<3, 1>(0, 0) = adapterDerived_.getPoint(indices[i]);

      //compute the reprojection (this is working for both central and
      //non-central case)
      point_t bodyReprojection = inverseSolution * p_hom;
      point_t reprojection = adapterDerived_.getCamRotation(indices[i])
          .transpose()
          * (bodyReprojection - adapterDerived_.getCamOffset(indices[i]));
      reprojection = reprojection / reprojection.norm();

      //compute the score
      point_t error = (reprojection
          - adapterDerived_.getBearingVector(indices[i]));
      double error_squared = error.transpose() * error;
      scores.push_back(
          error_squared / adapterDerived_.getSigmaAngle(indices[i]));
    }
  }

 protected:
  /// The adapter holding the bearing, correspondences etc.
  opengv::absolute_pose::FrameNoncentralAbsoluteAdapter & adapterDerived_;

};

} // namespace absolute_pose
} // namespace sac_problems
} // namespace opengv
