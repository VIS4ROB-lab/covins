/*
 * FrameNoncentralRelativeAdapter.cpp
 * @brief Adapter for Non-central relative pose RANSAC (2D-2D)
 * @author: Manthan Patel
 * Created on: Apr 12,2022
 */



#include "covins/matcher/opengv/rel_pose/FrameNoncentralRelativeAdapter.hpp"

namespace opengv {

namespace relative_pose {

FrameNoncentralRelativeAdapter::FrameNoncentralRelativeAdapter(
    covins::TypeDefs::KeyframeVector view_A,
    covins::TypeDefs::KeyframeVector view_B,
    std::vector<covins::Matches> match_vect,
    std::vector<Eigen::Matrix4d> TF_vect,
      std::vector<std::vector<int>> inliers_vect, Eigen::Matrix4d T_init) {

  // For 3v1 CKF-QKF
  // Fill up the bearing Vectors, Camera Correspondences and Cam TFs
  covins::TypeDefs::KeyframePtr KF1 = view_A[0];
  this->setR12(T_init.block<3, 3>(0, 0));
  _bearingVectors1.reserve((inliers_vect.size()+1) * inliers_vect[0].size());
  _bearingVectors2.reserve((inliers_vect.size() + 1) * inliers_vect[0].size());
  _camCorrespondences1.reserve((inliers_vect.size() + 1) *
                               inliers_vect[0].size());
  _camCorrespondences2.reserve((inliers_vect.size() + 1) *
                               inliers_vect[0].size());


  for (size_t i = 0; i < match_vect.size(); ++i) {

    covins::Matches matches = match_vect[i];
    std::vector<int> inlierInd = inliers_vect[i];
    covins::TypeDefs::KeyframePtr KF2 = view_B[i];

    // Iterate through the Matches for the current pair and add the Bearings and
    // Cam indices

    for (size_t j : inlierInd) {

      Eigen::Vector3d tmp_bearing_A = KF1->bearings_add_[matches[j].idxA];
      Eigen::Vector3d tmp_bearing_B = KF2->bearings_add_[matches[j].idxB];
      _bearingVectors1.push_back(tmp_bearing_A.normalized());
      _bearingVectors2.push_back(tmp_bearing_B.normalized());

      _camCorrespondences1.push_back(0);
      _camCorrespondences2.push_back(i);

    }

    //Push the Camera Translation and Rotations
    _camOffsets.push_back(TF_vect[i].block<3, 1>(0, 3));
    _camRotations.push_back(TF_vect[i].block<3, 3>(0, 0));

  }
}

FrameNoncentralRelativeAdapter::FrameNoncentralRelativeAdapter(
    covins::TypeDefs::KeyframeVector view_A,
    covins::TypeDefs::KeyframeVector view_B,
    std::vector<std::vector<covins::Matches>> match_vect,
    std::vector<std::vector<Eigen::Matrix4d>> TF_vect,
    std::vector<std::vector<std::vector<int>>> inliers_vect, Eigen::Matrix4d T_init) {

  // For Arbitrary Number of CKFs and QKFs
  // Fill up the bearing Vectors, Camera Correspondences and Cam TFs

  size_t n_ckfs = match_vect[0].size();
  size_t n_qkfs = match_vect.size();
  
  this->setR12(T_init.block<3, 3>(0, 0));

  _bearingVectors1.reserve((inliers_vect[0].size()+1) * inliers_vect[0][0].size());
  _bearingVectors2.reserve((inliers_vect[0].size()+1) * inliers_vect[0][0].size());
  _camCorrespondences1.reserve((inliers_vect[0].size()+1) * inliers_vect[0][0].size());
  _camCorrespondences2.reserve((inliers_vect[0].size()+1) * inliers_vect[0][0].size());


  for (size_t i = 0; i < n_qkfs; ++i) {

    covins::TypeDefs::KeyframePtr KF1 = view_A[i];
    
    for (size_t j = 0; j < n_ckfs; ++j) {

      covins::Matches matches = match_vect[i][j];
      std::vector<int> inlierInd = inliers_vect[i][j];
      covins::TypeDefs::KeyframePtr KF2 = view_B[j];

      // Iterate through the Matches for the current pair and add the Bearings
      // and Cam indices
      
      for (size_t k : inlierInd) {

      Eigen::Vector3d tmp_bearing_A = KF1->bearings_add_[matches[k].idxA];
      Eigen::Vector3d tmp_bearing_B = KF2->bearings_add_[matches[k].idxB];
      _bearingVectors1.push_back(tmp_bearing_A.normalized());
      _bearingVectors2.push_back(tmp_bearing_B.normalized());

      _camCorrespondences1.push_back(i);
      _camCorrespondences2.push_back(n_qkfs + j);

      }
    }
  }

  // Build the Cam OFfset Matrix (First push all at index 0 and then all at
  // index 1)
  //
  // Push the Camera Translation and Rotations
  for (size_t i = 0; i < n_qkfs; ++i) {
    _camOffsets.push_back(TF_vect[0][i].block<3, 1>(0, 3));
    _camRotations.push_back(TF_vect[0][i].block<3, 3>(0, 0));
  }
  for (size_t i = 0; i < n_ckfs; ++i) {
    _camOffsets.push_back(TF_vect[1][i].block<3, 1>(0, 3));
    _camRotations.push_back(TF_vect[1][i].block<3, 3>(0, 0));
  }

}


bearingVector_t
FrameNoncentralRelativeAdapter::getBearingVector1(size_t index) const {
  assert(index < _bearingVectors1.size());
  return _bearingVectors1[index];
}

bearingVector_t
FrameNoncentralRelativeAdapter::getBearingVector2(size_t index) const {
  assert(index < _bearingVectors2.size());
  return _bearingVectors2[index];
}

double FrameNoncentralRelativeAdapter::getWeight(size_t index) const {
  return 1.0;
}

translation_t FrameNoncentralRelativeAdapter::
    getCamOffset1( size_t index ) const
{
  assert(_camCorrespondences1[index] < _camOffsets.size());
  return _camOffsets[_camCorrespondences1[index]];
}

rotation_t FrameNoncentralRelativeAdapter::getCamRotation1(size_t index) const {
  
  assert(_camCorrespondences1[index] < _camRotations.size());
  return _camRotations[_camCorrespondences1[index]];
}

translation_t
FrameNoncentralRelativeAdapter::getCamOffset2(size_t index) const {

  assert(_camCorrespondences2[index] < _camOffsets.size());
  return _camOffsets[_camCorrespondences2[index]];
}

rotation_t FrameNoncentralRelativeAdapter::getCamRotation2(size_t index) const {

  assert(_camCorrespondences2[index] < _camRotations.size());
  return _camRotations[_camCorrespondences2[index]];
}

size_t FrameNoncentralRelativeAdapter::getNumberCorrespondences() const {

  return _bearingVectors2.size();
}

} // namespace relative_pose

} // namespace opengv