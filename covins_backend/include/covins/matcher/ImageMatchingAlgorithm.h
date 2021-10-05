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
 * ImageMatchingAlgorithm.hpp
 * @brief Header for the ImageMatchingAlgorithm class (2d-2d matching).
 * @author: Marco Karrer
 * Created on: Jun 12, 2017
 */

#pragma once

// C++
#include <iostream>
#include <memory>
#include <vector>

#include <eigen3/Eigen/Core>

// COVINS
#include <covins/covins_base/config_comm.hpp>
#include <covins/covins_base/config_backend.hpp>
#include <covins/covins_base/typedefs_base.hpp>


#include "covins_backend/keyframe_be.hpp"
#include "matcher/MatchingAlgorithm.h"
#include "covins_backend/feature_matcher_be.hpp"

// Thirdparty
#include "brisk/hamming.h"
#include "covins/dense_matcher/DenseMatcher.hpp"

/// \brief cvislam Main namespace of this package.
namespace covins {

/**
 * \brief A MatchingAlgorithm implementation
 */
class ImageMatchingAlgorithm : public MatchingAlgorithm {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using KeyframePtr                   = TypeDefs::KeyframePtr;

  ///@brief Constructor.
  ///@param distanceThreshold   Descriptor distance threshold.
  ImageMatchingAlgorithm(float distanceThreshold,
                         bool epipoleCheck = false,
                         bool triangCheck = false);

  virtual ~ImageMatchingAlgorithm();

  ///@brief Set which frames to match.
  ///@param kfPtr1  The multiframe whose frames should be matched.
  ///@param kfPtr2  ID of the frame inside multiframe to match.
  void setFrames(KeyframePtr kfPtr1,
                 KeyframePtr kfPtr2);

  ///@brief Set the Fundamental matrix.
  ///@param Fab The fundamental matrix between A and B.
  void setFundamentalMat(const Eigen::Matrix3d& Fab) {
    mFab = Fab;
  }

  /// \brief This will be called exactly once for each call to DenseMatcher::match().
  virtual void doSetup();

  /// \brief What is the size of list A?
  virtual size_t sizeA() const;
  /// \brief What is the size of list B?
  virtual size_t sizeB() const;

  /// \brief Get the distance threshold for which matches exceeding it will not be returned as matches.
  virtual float distanceThreshold() const;
  /// \brief Set the distance threshold for which matches exceeding it will not be returned as matches.
  void setDistanceThreshold(float distanceThreshold);

  /// \brief Should we skip the item in list A? This will be called once for each item in the list
  virtual bool skipA(size_t indexA) const {
    return skipA_[indexA];
  }

  /// \brief Should we skip the item in list B? This will be called many times.
  virtual bool skipB(size_t indexB) const {
    return skipB_[indexB];
  }

  /**
   * @brief Calculate the distance between two keypoints.
   * @param indexA Index of the first keypoint.
   * @param indexB Index of the other keypoint.
   * @return Distance between the two keypoint descriptors.
   * @remark Points that absolutely don't match will return float::max.
   */
  virtual float distance(size_t indexA, size_t indexB) const {
    const float dist = static_cast<float>(specificDescriptorDistance(
        kfPtrA_->GetDescriptor(indexA),
        kfPtrB_->GetDescriptor(indexB)));

    if (dist < distanceThreshold_) {
      if (verifyMatch(indexA, indexB))
        return dist;
    }
    return std::numeric_limits<float>::max();
  }

  /// \brief Geometric verification of a match.
  bool verifyMatch(size_t indexA, size_t indexB) const;

  /// \brief A function that tells you how many times setMatching() will be called.
  /// \warning Currently not implemented to do anything.
  virtual void reserveMatches(size_t numMatches);

  /// \brief At the end of the matching step, this function is called once
  ///        for each pair of matches discovered.
  virtual void setBestMatch(size_t indexA, size_t indexB, double distance);

  /// \brief Get the number of matches.
  size_t numMatches();
  /// \brief Get the number of uncertain matches.
  size_t numUncertainMatches();

  /// \brief access the matching result.
  const Matches & getMatches() const {
    return matches_;
  };

  /// \brief assess the validity of the relative uncertainty computation.
  bool isRelativeUncertaintyValid() {
    return validRelativeUncertainty_;
  }

 private:
  bool EpipolarDistanceCheck(const size_t idxA,
                             const size_t idxB) const;

  /// \name Which frames to take
  /// \{
  KeyframePtr kfPtrA_;
  KeyframePtr kfPtrB_;
  /// \}

  /// Check related members
  bool mbCheckEpi;
  bool mbCheckTriang;
  Eigen::Matrix3d mFab;

  /// Distances above this threshold will not be returned as matches.
  float distanceThreshold_;

  /// The number of matches.
  size_t numMatches_ = 0;
  /// The number of uncertain matches.
  size_t numUncertainMatches_ = 0;

  /// Focal length of camera used in frame A.
  double fA_ = 0;
  /// Focal length of camera used in frame B.
  double fB_ = 0;

  /// Should keypoint[index] in frame A be skipped
  std::vector<bool> skipA_;
  /// Should keypoint[index] in frame B be skipped
  std::vector<bool> skipB_;

  /// Camera center of frame A.
  Eigen::Vector3d pA_W_;
  /// Camera center of frame B.
  Eigen::Vector3d pB_W_;



  /// Temporarily store ray sigmas of frame A.
  std::vector<double> raySigmasA_;
  /// Temporarily store ray sigmas of frame B.
  std::vector<double> raySigmasB_;

  /// Store the matches
  Matches matches_;

  bool validRelativeUncertainty_ = false;

  /// \brief Calculates the distance between two descriptors.
  // copy from BriskDescriptor.hpp
  u_int32_t specificDescriptorDistance(
      const unsigned char * descriptorA,
      const unsigned char * descriptorB) const {
      if((covins_params::features::desc_length % 16) != 0) {
          std::cout << "ERROR: descriptor length is not a multiple of 16 " << std::endl;
      }
      int desc_length = covins_params::features::desc_length % 16;
      return brisk::Hamming::PopcntofXORed(descriptorA, descriptorB, desc_length);
  }
};

} // namespace covins
