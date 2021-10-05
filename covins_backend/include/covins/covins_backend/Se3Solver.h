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

#pragma once

// C++
#include <memory>
#include <vector>

#include <eigen3/Eigen/Core>

// COVINS
#include <covins/covins_base/typedefs_base.hpp>

namespace covins {

//forward decs
class Keyframe;
class Landmark;
//---------------

class Se3Solver {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using KeyframePtr                   = TypeDefs::KeyframePtr;
  using LandmarkPtr                   = TypeDefs::LandmarkPtr;

  using LandmarkVector                = TypeDefs::LandmarkVector;

public:
  /// @brief The Class constructor
  /// @param minInliers The minimal number of inliers required.
  /// @param ransacProb The ransac probability.
  /// @param maxIter The maximal number of iterations
  Se3Solver(const size_t minInliers = 30, const double ransacProb = 0.999,
    const size_t maxIter = 300);

  /// @brief Set the parameters used for RANSAC
  /// @param minInliers The minimal number of inliers needed.
  /// @param ransacProb The probability of all inliers that should be reached.
  /// @param maxIter The maximal number of iterations.
  void setRansacParams(const int minInliers, const double ransacProb,
    const int maxIter);

  /// @brief Perform a 3d-2d projective RANSAC to find a 6DoF transformation.
  /// @param keyframePtr The keyframe that should be aligned.
  /// @param mapPointMatches The matched map points against which the alignment should be done.
  /// @param threshold The threshold used for decide inliers.
  /// @param Tws The computed transformation.
  /// @return Whether a transformation was found or nod.
  bool projectiveAlignment(const KeyframePtr keyframePtr,
    LandmarkVector& mapPointMatches, const double threshold, Eigen::Matrix4d& Tws);

private:
  // The RANSAC parameters
  size_t mMinInliers;
  double mRansacProb;
  size_t mMaxIter;
};

} // namespace cvislam
