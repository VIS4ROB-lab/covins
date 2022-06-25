/*
 * frame-relative-adapter.hpp
 * @brief Adapter for relative pose RANSAC (2D-2D)
 * @author: Marco Karrer
 * Created on: Apr 09, 2019
 */

#pragma once

#include <vector>
#include <opengv/types.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>


#include "covins_backend/keyframe_be.hpp"
#include "matcher/MatchingAlgorithm.h"

namespace opengv {

namespace relative_pose {

class FrameRelativeAdapter : public RelativeAdapterBase {
private:
  using RelativeAdapterBase::_t12;
  using RelativeAdapterBase::_R12;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief Constructor.
  /// @param frame_A The first frame.
  /// @param frame_B The second frame.
  /// @param matches_A_B The matches between frame_A and frame_B
  FrameRelativeAdapter(
      covins::TypeDefs::KeyframePtr frame_A,
      covins::TypeDefs::KeyframePtr frame_B,
      const covins::Matches& matches_A_B);

  virtual ~FrameRelativeAdapter() {};

  /// \brief Retrieve the bearing vecotr of a correspondence in viewpoint 1 (A)
  /// @param index The serialized index of the correspondence.
  /// @return The corresponding bearing vector.
  virtual opengv::bearingVector_t getBearingVector1(size_t index) const;

  /// \brief Retrieve the bearing vector of a correspondence in viewpoint 2 (B)
  /// @param index The serialized index of the correspondence.
  /// @return The corresponding bearing vector.
  virtual opengv::bearingVector_t getBearingVector2(size_t index) const;

  /// \brief Retrieve the position of a camera of a correspondence in viewpoint
  ///        1 seen from the origin of the viewpoint.
  /// @param index The serialized index of the correspondence.
  /// @return The position of the corresponding camera seen from the viewpoint
  ///          origin.
  virtual opengv::translation_t getCamOffset1(size_t index) const;

  /// \brief Retrieve the rotation from a camera of a correspondence in
  ///        viewpoint 1 to the viewpoint origin.
  /// @param index The serialized index of the correspondence.
  /// @return The rotation from the corresponding camera back to the viewpoint
  ///         origin.
  virtual opengv::rotation_t getCamRotation1(size_t index) const;

  /// \brief Retrieve the position of a camera of a correspondence in viewpoint
  ///        2 seen from the origin of the viewpoint.
  /// @param index The serialized index of the correspondence.
  /// @return The position of the corresponding camera seen from the viewpoint
  ///          origin.
  virtual opengv::translation_t getCamOffset2(size_t index) const;

  /// \brief Retrieve the rotation from a camera of a correspondence in
  ///        viewpoint 2 to the viewpoint origin.
  /// @param index The serialized index of the correspondence.
  /// @return The rotation from the corresponding camera back to the viewpoint
  ///         origin.
  virtual opengv::rotation_t getCamRotation2(size_t index) const;

  /// \brief Retrieve the number of correspondences.
  /// @return The number of correspondences.
  virtual size_t getNumberCorrespondences() const;

  /// \brief Retrieve the weight of a correspondence. The weight is supposed to
  ///        reflect the quality of a correspondence, and typically is between
  ///        0 and 1.
  /// \warning This is not implemented and always returns 1.0.
  virtual double getWeight(size_t) const {
    return 1.0;
  }

  /// \brief Obtain the angular standard deviation of the correspondence in frame 1 in [rad].
  /// @param index The index of the correspondence.
  /// @return The standard deviation in [rad].
  double getSigmaAngle1(size_t index) const;

  /// \brief Obtain the angular standard deviation of the correspondence in frame 2 in [rad].
  /// @param index The index of the correspondence.
  /// @return The standard deviation in [rad].
  double getSigmaAngle2(size_t index) const;

private:
  // The bearing vectors of the correspondences in frame 1.
  opengv::bearingVectors_t bearing_vectors1_;

  // The bearing vectors of the correspondences in frame 2.
  opengv::bearingVectors_t bearing_vectors2_;

  // also store individual uncertainties
  // The standard deviations of the bearing vectors of frame 1 in [rad].
  std::vector<double> sigma_angles1_;
  // The standard deviations of the bearing vectors of frame 2' in [rad].
  std::vector<double> sigma_angles2_;

  // The matching keypoints of both frames.
  covins::Matches matches_A_B_;
};

} // namespace relative_pose

} // namespace opengv
