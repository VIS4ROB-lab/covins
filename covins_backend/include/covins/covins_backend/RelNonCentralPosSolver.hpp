#pragma once

// C++

#include <memory>
#include <vector>

#include <eigen3/Eigen/Core>

// CoVINS
#include <covins/covins_base/typedefs_base.hpp>

#include "covins/matcher/MatchingAlgorithm.h"

// Third Party
#include "matcher/MatchingAlgorithm.h"
#include "matcher/ImageMatchingAlgorithm.h"
#include "covins_backend/optimization_be.hpp"
#include <covins/covins_base/utils_base.hpp>

namespace covins {

// forward decs
class Keyframe;

//-------------


class RelNonCentralPosSolver {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using KeyframePtr = std::shared_ptr<Keyframe>;
  using KeypointVector = TypeDefs::KeypointVector;
  using KeyframeVector = TypeDefs::KeyframeVector;
  using Vector3Type = TypeDefs::Vector3Type;
  using TransformType = TypeDefs::TransformType;
  using Vector3Vector = TypeDefs::Vector3Vector;
  using KfObservations = TypeDefs::KfObservations;
  using LoopVector = TypeDefs::LoopVector;
  using Matrix6Type = TypeDefs::Matrix6Type;
  
public:
  /// @brief The class constructor
  /// @param minInliers The minimal number of inliers required
  /// @param ransacProb The ransac Probability
  /// @param maxIter The maximal number of iterations
  RelNonCentralPosSolver(const size_t minInliers = 50, const double ransacProb = 0.99,
               const size_t maxIter = 300);

  /// @brief Set the parameters used for RANSAC
  /// @param minInliers The minimal number of inliers needed
  /// @param ransacProb The ransac probability
  /// @param maxIter The maximal number of iterations
  void setRansacParams(const int minInliers, const double ransacProb,
                       const int maxIter);


  /// @brief Perform 17 Point Ransac using 2D-2D correspondces in non central
  /// camera frame.
  /// @param KeyframePtr1 The first keyframe (query KF)
  /// @param KeyframePtr2 The second keyframe (candidate KF)
  /// @param threshold Ransac threshold for inliers
  /// @param Tc1c2 Relative Transfomation from Frame 2 to Frame 1
  /// @param cov_loop Covariance Matrix (6x6) of the loop transformation
  /// @return Whether a Transformation with sufficients inliers was found
  bool computeNonCentralRelPose(const KeyframePtr KeyframePtr1,
                                const KeyframePtr KeyframePtr2,
                                const double threshold, TransformType &Tc1c2,
                                Matrix6Type &cov_loop);


  /// @brief Perform 5 Point Ransac using 2D-2D correspondces to get pose.
  /// @param KeyframePtr1 The first keyframe (query KF)
  /// @param KeyframePtr2 The second keyframe (candidate KF)
  /// @param ImgMatches Contains the correspondences
  /// @param threshold Ransac threshold for inliers
  /// @param Tc1c2 Relative Transfomation from Frame 2 to Frame 1
  /// @param inlierInd Vector containing the inlier indices
  /// @return Whether a Transformation with sufficients inliers was found
  bool computePose(const KeyframePtr KeyframePtr1,
                   const KeyframePtr KeyframePtr2, Matches &ImgMatches,
                   const double threshold, TransformType &Tc1c2, std::vector<int> &inlierInd);

  auto findMatches(const KeyframePtr KeyframePtr1,
                   const KeyframePtr KeyframePtr2) -> Matches;

  void plotMatches(const KeyframePtr KfPtr1,
                                           const KeyframePtr KfPtr2,
                                           Matches &ImgMatches,
                                           std::vector<int> &inlierInd, std::string s = "");
      
private:
// The RANSAC parameters
  size_t mMinInliers;
  double mRansacProb;
  size_t mMaxIter;
  size_t mImgMatches;
  size_t mMaxIter_17PT;
  size_t mMinInliers_17PT;
  double mMax_cov;
  size_t mCov_iter;
  int mCov_max_iter;
  double mRP_err;
  double mMatchThreshold;
  double mThres_17PT;
  KeypointVector kp_vect1_;
  KeypointVector kp_vect2_;
  std::vector<cv::KeyPoint> kp_vect1_in_;
  std::vector<cv::KeyPoint> kp_vect2_in_;
  std::vector<cv::DMatch> matches_in_;

};

} // end ns