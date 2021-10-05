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

#include "covins_backend/Se3Solver.h"

// C++
#include <cmath>
#include <vector>
#include <eigen3/Eigen/Core>

// COVINS
#include "covins_backend/keyframe_be.hpp"

// OpenGV related includes
#include <opengv/sac/Ransac.hpp>
#include "matcher/opengv/sac_problems/FrameAbsolutePoseSacProblem.h"

namespace covins {

// The class constructor
Se3Solver::Se3Solver(const size_t minInliers, const double ransacProb,
                     const size_t maxIter):
  mMinInliers(minInliers),
  mRansacProb(ransacProb),
  mMaxIter(maxIter)
{
  //...
}

// Set the parameters used for RANSAC
void Se3Solver::setRansacParams(const int minInliers,
    const double ransacProb, const int maxIter) {
  mMinInliers = minInliers;
  mRansacProb = ransacProb;
  mMaxIter = maxIter;
}

// Solve the 3d-2d problem
bool Se3Solver::projectiveAlignment(
    const KeyframePtr keyframePtr,
    LandmarkVector& mapPointMatches,
    const double threshold, Eigen::Matrix4d& Tws) {
  // Keep track of the initial order
  std::vector<size_t> indMap;
  for (size_t i = 0; i < mapPointMatches.size(); ++i) {
    if (!mapPointMatches[i]) {
      continue;
    }
    if (mapPointMatches[i]->IsInvalid()) {
      continue;
    }
    indMap.push_back(i);
  }

  // Setup the openGV problem to solve
  opengv::absolute_pose::FrameNoncentralAbsoluteAdapter adapter(keyframePtr,mapPointMatches);
  opengv::sac::Ransac<
  opengv::sac_problems::absolute_pose::FrameAbsolutePoseSacProblem> sacProb;
   std::shared_ptr<opengv::sac_problems::absolute_pose::FrameAbsolutePoseSacProblem> absposeproblemPtr(
      new opengv::sac_problems::absolute_pose::FrameAbsolutePoseSacProblem(
        adapter, opengv::sac_problems::absolute_pose::FrameAbsolutePoseSacProblem::Algorithm::GP3P));
  sacProb.sac_model_ = absposeproblemPtr;
  sacProb.threshold_ = threshold;
  sacProb.max_iterations_ = mMaxIter;
  sacProb.computeModel(0);

  if (sacProb.inliers_.size() < mMinInliers) {
    return false;
  }

  // Classify the points
  std::vector<int> inlierInd = sacProb.inliers_;
  std::vector<bool> inliers;
  inliers.resize(mapPointMatches.size(), false);
  for (size_t i = 0; i < inlierInd.size(); ++i) {
    size_t origInd = indMap[inlierInd[i]];
    inliers[origInd] = true;
  }

  for (size_t i = 0; i < mapPointMatches.size(); ++i) {
    if (!inliers[i]) {
      mapPointMatches[i] = static_cast<LandmarkPtr>(NULL);;
    }
  }

  // Extract the pose from RANSAC
  Tws = Eigen::Matrix4d::Identity();
  Tws.block<3,4>(0,0) = sacProb.model_coefficients_;
  return true;
}

} //end ns
