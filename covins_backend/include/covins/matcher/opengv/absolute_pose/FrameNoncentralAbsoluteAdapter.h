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
 * FrameNoncentralAbsoluteAdapter.hpp
 * @brief Header for the FrameNoncentralAbsoluteAdapter class
 * @author: Marco Karrer
 * Created on: Jan 23, 2017
 */

#pragma once

// Standard includes
#include <stdlib.h>
#include <vector>
#include <memory>

// The OpenGV includes
#include <opengv/types.hpp>
#include <opengv/absolute_pose/NoncentralAbsoluteAdapter.hpp>

// Package related includes
#include <covins/covins_base/typedefs_base.hpp>

#include "covins_backend/keyframe_be.hpp"
#include "covins_backend/landmark_be.hpp"

namespace opengv {

namespace absolute_pose {

using namespace covins;

/// @brief Adapter for absolute pose RANSAC (3D2D) with non-central cameras,
///        i.e. could be a multi-camera-setup.
class FrameNoncentralAbsoluteAdapter : public AbsoluteAdapterBase {
private:
  using AbsoluteAdapterBase::_t;
  using AbsoluteAdapterBase::_R;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using KeyframePtr                   = TypeDefs::KeyframePtr;
  using LandmarkPtr                   = TypeDefs::LandmarkPtr;
  using LandmarkVector                = TypeDefs::LandmarkVector;

  /// @brief type for describing matches.
  typedef std::vector<int> matches_t;

  /// @brief Constructor.
  /// @param keyframePtr   The keyframe.
  /// @param matchedPoints The matched 3d points
  FrameNoncentralAbsoluteAdapter(
      const KeyframePtr keyframePtr,
      const LandmarkVector& matchedPoints) {
    // Only a single camera without offset
    camOffsets_.push_back(Eigen::Vector3d::Zero());
    camRotations_.push_back(Eigen::Matrix3d::Identity());
    const double fu = (keyframePtr->calibration_.intrinsics(0) + keyframePtr->calibration_.intrinsics(1))/2.0;
    // Extract the geometrical information
//    const size_t numKPs = keyframePtr->mvBearings.size();
    const size_t numKPs = std::min(keyframePtr->bearings_.size(),matchedPoints.size()); //mvBearings not always exactly 500
    int noCorrespondences = 0;
    for (size_t i = 0; i < numKPs; ++i) {
      LandmarkPtr tmpMPi = matchedPoints[i];
      if (!tmpMPi) {
        continue;
      }
      if (tmpMPi->IsInvalid()) {
        continue;
      }

      // Add the landmark
      Eigen::Vector3d p3Dw = tmpMPi->GetWorldPos();
      points_.push_back(p3Dw);

      // Add the bearing vector and the sigma of the angle
      double keypointStdDev = 0.8*(double)(keyframePtr->keypoints_aors_[i][1] + 1);
      bearingVectors_.push_back(keyframePtr->bearings_[i]);
      sigmaAngles_.push_back(sqrt(2) * keypointStdDev * keypointStdDev / (fu * fu));

      // count
      ++noCorrespondences;

      // store camera index (only a single camera --> 0)
      camIndices_.push_back(0);

      // store keypoint index
      keypointIndices_.push_back(noCorrespondences - 1);//i);
    }
  }

  virtual ~FrameNoncentralAbsoluteAdapter() {
  }

  /// @brief Retrieve the bearing vector of a correspondence.
  /// @param index The serialized index of the correspondence.
  /// @return The corresponding bearing vector.
  virtual opengv::bearingVector_t getBearingVector(size_t index) const {
    assert(index < bearingVectors_.size());
    return bearingVectors_[index];
  }

  /// @brief Retrieve the position of a camera of a correspondence
  ///        seen from the viewpoint origin.
  /// @param index The serialized index of the correspondence.
  /// @return The position of the corresponding camera seen from the viewpoint
  /// origin.
  virtual opengv::translation_t getCamOffset(size_t index) const {
    return camOffsets_[camIndices_[index]];
  }

  /// @brief Retrieve the rotation from a camera of a correspondence to the
  ///        viewpoint origin.
  /// @param index The serialized index of the correspondence.
  ///  @return The rotation from the corresponding camera back to the viewpoint
  ///       origin.
  virtual opengv::rotation_t getCamRotation(size_t index) const {
    return camRotations_[camIndices_[index]];
  }

  /// @brief Retrieve the world point of a correspondence.
  /// @param index The serialized index of the correspondence.
  /// @return The corresponding world point.
  virtual opengv::point_t getPoint(size_t index) const {
    assert(index < bearingVectors_.size());
    return points_[index];
  }

  /// @brief Get the number of correspondences. These are keypoints that have a
  ///        corresponding landmark which is added to the estimator,
  ///        has more than one observation and not at infinity.
  /// @return Number of correspondences.
  virtual size_t getNumberCorrespondences() const {
    return points_.size();
  }

  //// @brief Get the camera index for a specific correspondence.
  /// @param index The serialized index of the correspondence.
  /// @return Camera index of the correspondence.
  int camIndex(size_t index) const {
    return camIndices_.at(index);}

  /// @brief Get the keypoint index for a specific correspondence
  /// @param index The serialized index of the correspondence.
  /// @return Keypoint index belonging to the correspondence.
  int keypointIndex(size_t index) const {
    return keypointIndices_.at(index);}

  /// @brief Retrieve the weight of a correspondence. The weight is supposed to
  ///       reflect the quality of a correspondence, and typically is between
  ///        0 and 1.
  /// @warning This is not implemented and always returns 1.0.
  virtual double getWeight(size_t) const {
    return 1.0;
  }

  /// @brief Obtain the angular standard deviation in [rad].
  /// @param index The index of the correspondence.
  /// @return The standard deviation in [rad].
  double getSigmaAngle(size_t index) {
    return sigmaAngles_[index];
  }

 private:
  /// The bearing vectors of the correspondences.
  opengv::bearingVectors_t bearingVectors_;

  /// The world coordinates of the correspondences.
  opengv::points_t points_;

  /// The camera indices of the correspondences.
  std::vector<size_t> camIndices_;

  /// The keypoint indices of the correspondences.
  std::vector<size_t> keypointIndices_;

  /// The position of the cameras seen from the viewpoint origin
  opengv::translations_t camOffsets_;

  /// The rotation of the cameras to the viewpoint origin.
  opengv::rotations_t camRotations_;

  /// The standard deviations of the bearing vectors in [rad].
  std::vector<double> sigmaAngles_;

};

} // namespace absolute_pose
} // namespace opengv
