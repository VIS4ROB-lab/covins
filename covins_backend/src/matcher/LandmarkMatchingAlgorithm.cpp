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
 * LandmarkMatchingAlgorithm.cpp
 * @brief Source for the LandmarkMatchingAlgorithm class (3d-3d matching).
 * @author: Marco Karrer
 * Created on: Jun 12, 2017
 */

#include "matcher/LandmarkMatchingAlgorithm.h"

// C)VINS
#include "covins_backend/landmark_be.hpp"

// The main namespace of this package
namespace covins {

// Constructor
LandmarkMatchingAlgorithm::LandmarkMatchingAlgorithm(
    float distanceThreshold) :
  distanceThreshold_(distanceThreshold) {
    //...
}

// Destructor
LandmarkMatchingAlgorithm::~LandmarkMatchingAlgorithm() {
    //...
}

// Set which frames to match.
void LandmarkMatchingAlgorithm::setFrames(KeyframePtr kfPtr1,
    KeyframePtr kfPtr2) {
  // Remember the frames and indices.
  kfPtrA_ = kfPtr1;
  kfPtrB_ = kfPtr2;

  // Set the focal length
  fA_ = (kfPtr1->calibration_.intrinsics(0) + kfPtr1->calibration_.intrinsics(1))/2.0;
  fB_ = (kfPtr2->calibration_.intrinsics(0) + kfPtr2->calibration_.intrinsics(1))/2.0;
}

// This will be called exactly once for each call to DenseMatcher::match().
void LandmarkMatchingAlgorithm::doSetup() {
  // Reset the match counter
  numMatches_ = 0;
  numUncertainMatches_ = 0;

  // Prepare the bookkeeping for frame A
  const size_t numA = kfPtrA_->keypoints_distorted_.size();
  skipA_.clear();
  skipA_.resize(numA, false);

  // Compute the ray uncertainty for frame A
  raySigmasA_.resize(numA);
  const double sqrtOfsqrtOf2 = std::sqrt(std::sqrt(2.0));
  for (size_t k = 0; k < numA; ++k) {
    double keypointAStdDev = 0.8 * (kfPtrA_->keypoints_aors_[k][1] + 1);
    raySigmasA_[k] = sqrtOfsqrtOf2 * keypointAStdDev / fA_;
    if (!kfPtrA_->GetLandmark(k)) {
      skipA_[k] = true;
    } else if (kfPtrA_->GetLandmark(k)->IsInvalid()) {
      skipA_[k] = true;
    }
  }

  // Prepare the bookkeeping for frame B
  const size_t numB = kfPtrB_->keypoints_distorted_.size();
  skipB_.clear();
  skipB_.resize(numB, false);
  // Compute the ray uncertainty for frame B
  raySigmasB_.resize(numB);
  for (size_t k = 0; k < numB; ++k) {
    double keypointBStdDev = 0.8 * (kfPtrA_->keypoints_aors_[k][1] + 1);
    raySigmasB_[k] = sqrtOfsqrtOf2 * keypointBStdDev / fB_;
    if (!kfPtrB_->GetLandmark(k)) {
      skipB_[k] = true;
    } else if (kfPtrB_->GetLandmark(k)->IsInvalid()) {
      skipB_[k] = true;
    }
  }
}

// What is the size of list A?
size_t LandmarkMatchingAlgorithm::sizeA() const {
    return kfPtrA_->keypoints_distorted_.size();
}
// What is the size of list B?
size_t LandmarkMatchingAlgorithm::sizeB() const {
    return kfPtrB_->keypoints_distorted_.size();
}

// Set the distance threshold for which matches exceeding it will not be returned as matches.
void LandmarkMatchingAlgorithm::setDistanceThreshold(
    float distanceThreshold) {
  distanceThreshold_ = distanceThreshold;
}

// Get the distance threshold for which matches exceeding it will not be returned as matches.
float LandmarkMatchingAlgorithm::distanceThreshold() const {
  return distanceThreshold_;
}

// Geometric verification of a match.
bool LandmarkMatchingAlgorithm::verifyMatch(
    size_t indexA, size_t indexB) const {
  // potential 2d2d match - verify by triangulation
  Eigen::Vector4d hP;
//  bool isParallel;
  // TODO: Actually check the result!
  bool valid = true;// = probabilisticStereoTriangulator_.stereoTriangulate(
   // indexA, indexB, hP, isParallel, std::max(raySigmasA_[indexA], raySigmasB_[indexB]));
  if (valid) {
    return true;
  }
  return false;
}

// A function that tells you how many times setMatching() will be called.
void LandmarkMatchingAlgorithm::reserveMatches(
    size_t /*numMatches*/) {
  //_triangulatedPoints.clear();
}

// Get the number of matches.
size_t LandmarkMatchingAlgorithm::numMatches() {
  return numMatches_;
}

// Get the number of uncertain matches.
size_t LandmarkMatchingAlgorithm::numUncertainMatches() {
  return numUncertainMatches_;
}

// At the end of the matching step, this function is called once
// for each pair of matches discovered.
void LandmarkMatchingAlgorithm::setBestMatch(
    size_t indexA, size_t indexB, double distance) {
  // Create match object and push it to the matches
  Match match(indexA, indexB, distance);
  matches_.push_back(match);

  // Increase matcher count
  numMatches_++;
}

} // namespace covins
