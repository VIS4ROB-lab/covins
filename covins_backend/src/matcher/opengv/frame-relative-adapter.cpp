/*
 * frame-relative-adapter.cpp
 * @brief Adapter for relative pose RANSAC (2D-2D)
 * @author: Marco Karrer
 * Created on: Apr 09, 2019
 */

#include "covins/matcher/opengv/rel_pose/frame-relative-adapter.hpp"

namespace opengv {

namespace relative_pose {

FrameRelativeAdapter::FrameRelativeAdapter(
    covins::TypeDefs::KeyframePtr frame_A,
    covins::TypeDefs::KeyframePtr frame_B,
    const covins::Matches& matches_A_B)
{ 
  matches_A_B_ = matches_A_B;
  const size_t num_matches = matches_A_B_.size();
  // std::cout << "Number of matches before ransac" << num_matches << std::endl;
  bearing_vectors1_.reserve(num_matches);
  bearing_vectors2_.reserve(num_matches);
  sigma_angles1_.reserve(num_matches);
  sigma_angles2_.reserve(num_matches);

  for (covins::Matches::iterator itr = matches_A_B_.begin(); itr !=matches_A_B_.end(); ++itr) {
      const size_t idxA = (*itr).idxA;
      const size_t idxB = (*itr).idxB;
      Eigen::Vector3d tmp_bearing_A = frame_A->bearings_add_[idxA];
      Eigen::Vector3d tmp_bearing_B = frame_B->bearings_add_[idxB];
      bearing_vectors1_.push_back(tmp_bearing_A.normalized());
      bearing_vectors2_.push_back(tmp_bearing_B.normalized());
  //   // For now, just say all the bearing vectors have the same uncertainty
      sigma_angles1_.push_back(4.5E-06);
      sigma_angles2_.push_back(4.5E-06);

  }
  
  // for (size_t i = 0; i < num_matches; ++i) {
  //   Eigen::Vector3d tmp_bearing_A = frame_A->bearings_add_[
  //         matches_A_B_[i].getKeypointIndexAppleFrame()];
  //   Eigen::Vector3d tmp_bearing_B = frame_B->bearings_add_[
  //         matches_A_B_[i].getKeypointIndexBananaFrame()];
  //   bearing_vectors1_.push_back(tmp_bearing_A.normalized());
  //   bearing_vectors2_.push_back(tmp_bearing_B.normalized());

  //   // For now, just say all the bearing vectors have the same uncertainty
  //   // (sqrt(2) * sigmaKP(=0.8)^2 / (focal_length(=450)^2)
  //   sigma_angles1_.push_back(4.5E-06);
  //   sigma_angles2_.push_back(4.5E-06);
  // }


}

opengv::bearingVector_t FrameRelativeAdapter::getBearingVector1(
    size_t index) const {
  return bearing_vectors1_[index];
}

opengv::bearingVector_t FrameRelativeAdapter::getBearingVector2(
    size_t index) const {
  return bearing_vectors2_[index];
}

opengv::translation_t FrameRelativeAdapter::getCamOffset1(size_t index) const {
  return Eigen::Vector3d::Zero();
}

opengv::rotation_t FrameRelativeAdapter::getCamRotation1(size_t index) const {
  return Eigen::Matrix3d::Identity();
}

opengv::translation_t FrameRelativeAdapter::getCamOffset2(size_t index) const {
  return Eigen::Vector3d::Zero();
}

opengv::rotation_t FrameRelativeAdapter::getCamRotation2(size_t index) const {
  return Eigen::Matrix3d::Identity();
}

size_t FrameRelativeAdapter::getNumberCorrespondences() const {
  return matches_A_B_.size();
}

double FrameRelativeAdapter::getSigmaAngle1(size_t index) const {
  return sigma_angles1_[index];
}

double FrameRelativeAdapter::getSigmaAngle2(size_t index) const {
  return sigma_angles1_[index];
}

} // namespace relative_pose

} // namespace opengv
