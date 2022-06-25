#pragma once

// C++

#include <memory>
#include <vector>

#include <eigen3/Eigen/Core>

// CoVINS
#include <covins/covins_base/typedefs_base.hpp>

#include "covins/matcher/MatchingAlgorithm.h"

namespace covins {

// forward decs
class Keyframe;

//-------------


class RelPosSolver {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using KeyframePtr = std::shared_ptr<Keyframe>;
  using KeypointVector = TypeDefs::KeypointVector;
  
public:
  /// @brief The class constructor
  /// @param minInliers The minimal number of inliers required
  /// @param ransacProb The ransac Probability
  /// @param maxIter The maximal number of iterations
  RelPosSolver(const size_t minInliers = 50, const double ransacProb = 0.99,
               const size_t maxIter = 300);

  /// @brief Set the parameters used for RANSAC
  /// @param minInliers The minimal number of inliers needed
  /// @param ransacProb The ransac probability
  /// @param maxIter The maximal number of iterations
  void setRansacParams(const int minInliers, const double ransacProb,
                       const int maxIter);

  /// @brief Perform 5 Point Ransac using 2D-2D correspondces to get pose.
  /// @param KeyframePtr1 The first keyframe (query KF)
  /// @param KeyframePtr2 The second keyframe (candidate KF)
  /// @param ImgMatches Contains the correspondences
  /// @param threshold Ransac threshold for inliers
  /// @param Tc1c2 Relative Transfomation from Frame 2 to Frame 1
  /// @return Whether a Transformation with sufficients inliers was found
  bool computePose(const KeyframePtr KeyframePtr1,
                   const KeyframePtr KeyframePtr2, Matches &ImgMatches,
                   const double threshold, Eigen::Matrix4d &Tc1c2);

private:
// The RANSAC parameters
  size_t mMinInliers;
  double mRansacProb;
  size_t mMaxIter;
  KeypointVector kp_vect1_;
  KeypointVector kp_vect2_;
  std::vector<cv::KeyPoint> kp_vect1_in_;
  std::vector<cv::KeyPoint> kp_vect2_in_;
  std::vector<cv::DMatch> matches_in_;

};

} // end ns