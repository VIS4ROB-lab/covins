/*
 * FrameNoncentralRelativeAdapter.hpp
 * @brief Adapter for Non-central relative pose RANSAC (2D-2D)
 * @author: Manthan Patel
 * Created on: Apr 12,2022
 */


#pragma once

#include <stdlib.h>
#include <vector>
#include <opengv/types.hpp>
#include <opengv/relative_pose/RelativeAdapterBase.hpp>

#include "covins_backend/keyframe_be.hpp"
#include "matcher/MatchingAlgorithm.h"

/**
 * \brief The namespace of this library.
 */
namespace opengv
{
/**
 * \brief The namespace for the relative pose methods.
 */
namespace relative_pose
{


class FrameNoncentralRelativeAdapter : public RelativeAdapterBase
{
private:
  using RelativeAdapterBase::_t12;
  using RelativeAdapterBase::_R12;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** A type defined for the camera-correspondences, see protected
   *  class-members
   */
  typedef std::vector<int> camCorrespondences_t;

  /// \brief Constructor. (Arbitrary KFs in View)
  /// @param view_A The Query View 
  /// @param view_B The Candidate View 
  /// @param match_vect The vector consisting matches between View A and frames
  /// of View B
  /// @param TF_vect Consists relative TFs between cameras of View B
  /// @param T_init Initial estimate of Transformation

  FrameNoncentralRelativeAdapter(
      covins::TypeDefs::KeyframeVector view_A,
      covins::TypeDefs::KeyframeVector view_B,
      std::vector<std::vector<covins::Matches>> match_vect,
      std::vector<std::vector<Eigen::Matrix4d>> TF_vect,
      std::vector<std::vector<std::vector<int>>> inliers_vect,
      Eigen::Matrix4d T_init);

  /// \brief Constructor. (3v1 Query-Candidate)
  /// @param view_A The Query View 
  /// @param view_B The Candidate View 
  /// @param match_vect The vector consisting matches between View A and frames
  /// of View B
  /// @param TF_vect Consists relative TFs between cameras of View B
  /// @param T_init Initial estimate of Transformation

  FrameNoncentralRelativeAdapter(
      covins::TypeDefs::KeyframeVector view_A,
      covins::TypeDefs::KeyframeVector view_B,
      std::vector<covins::Matches> match_vect,
      std::vector<Eigen::Matrix4d> TF_vect,
      std::vector<std::vector<int>> inliers_vect,
      Eigen::Matrix4d T_init);
  

  /// \brief Destructor.
  virtual ~FrameNoncentralRelativeAdapter() {};
 
  //Access of correspondences
  
  /** See parent-class */
  virtual bearingVector_t getBearingVector1( size_t index ) const;
  /** See parent-class */
  virtual bearingVector_t getBearingVector2( size_t index ) const;
  /** See parent-class */
  virtual double getWeight( size_t index ) const;
  /** See parent-class */
  virtual translation_t getCamOffset1( size_t index ) const;
  /** See parent-class */
  virtual rotation_t getCamRotation1( size_t index ) const;
  /** See parent-class */
  virtual translation_t getCamOffset2( size_t index ) const;
  /** See parent-class */
  virtual rotation_t getCamRotation2( size_t index ) const;
  /** See parent-class */
  virtual size_t getNumberCorrespondences() const;

private:
  /** Reference to bearing-vectors in viewpoint 1.
   *  (expressed in their individual cameras)
   */
  bearingVectors_t _bearingVectors1;
  /** Reference to bearing-vectors in viewpoint 2.
   *  (expressed in their individual cameras)
   */
  bearingVectors_t _bearingVectors2;
  /** Reference to an array of camera-indices for the bearing vectors in
   *  viewpoint 1. Length equals to number of bearing-vectors in viewpoint 1,
   *  and elements are indices of cameras in the _camOffsets and _camRotations
   *  arrays.
   */
  camCorrespondences_t _camCorrespondences1;
  /** Reference to an array of camera-indices for the bearing vectors in
   *  viewpoint 2. Length equals to number of bearing-vectors in viewpoint 2,
   *  and elements are indices of cameras in the _camOffsets and _camRotations
   *  arrays.
   */
  camCorrespondences_t _camCorrespondences2;

  /** Reference to positions of the different cameras seen from their
   *  viewpoint.
   */
  translations_t _camOffsets;
  /** Reference to rotations from the different cameras back to their
   *  viewpoint.
   */
  rotations_t _camRotations;
};

}
}

