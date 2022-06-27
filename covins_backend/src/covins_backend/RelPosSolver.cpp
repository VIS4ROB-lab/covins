// Header file
#include "covins_backend/RelPosSolver.hpp"

// Standard includes
#include <cmath>
#include <vector>

#include <eigen3/Eigen/Core>

// CoVINS
#include "covins_backend/keyframe_be.hpp"

// opengv related includes
#include <opengv/sac/Ransac.hpp>
#include "matcher/opengv/sac_problems/frame-relative-pose-sac-problem.hpp"

#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

namespace covins {

// The class constructor
RelPosSolver::RelPosSolver(const size_t minInliers, const double ransacProb,
                           const size_t maxIter)
    : mMinInliers(minInliers),
    mRansacProb(ransacProb),
    mMaxIter(maxIter)
{

}

// Set the parameters used for RANSAC
void RelPosSolver::setRansacParams(const int minInliers,
    const double ransacProb, const int maxIter) {
  mMinInliers = minInliers;
  mRansacProb = ransacProb;
  mMaxIter = maxIter;
}

// Solve the 2d-2d problem

bool RelPosSolver::computePose(const KeyframePtr KfPtr1,
                               const KeyframePtr KfPtr2,
                               Matches &ImgMatches, const double threshold,
                               Eigen::Matrix4d &Tc1c2) {

  // Find the Macthing keypoint indices (For Debugging)
  const size_t num_matches = ImgMatches.size();
  
kp_vect1_.reserve(num_matches);
kp_vect2_.reserve(num_matches);


for (Matches::iterator itr = ImgMatches.begin(); itr !=ImgMatches.end(); ++itr) {
      const size_t idxA = (*itr).idxA;
      const size_t idxB = (*itr).idxB;
      kp_vect1_.push_back(KfPtr1->keypoints_undistorted_add_[idxA]);
      kp_vect2_.push_back(KfPtr2->keypoints_undistorted_add_[idxB]);
}

  // Setup the openGV problem to solve
  opengv::relative_pose::FrameRelativeAdapter adapter(KfPtr1,
                                                      KfPtr2, ImgMatches);
  opengv::sac::Ransac<
      opengv::sac_problems::relative_pose::FrameRelativePoseSacProblem>
      sacProb;
  std::shared_ptr<
      opengv::sac_problems::relative_pose::FrameRelativePoseSacProblem>
      relposeprobelmPtr(
          new opengv::sac_problems::relative_pose::FrameRelativePoseSacProblem(
              adapter, opengv::sac_problems::relative_pose::
                           FrameRelativePoseSacProblem::Algorithm::STEWENIUS));
  sacProb.sac_model_ = relposeprobelmPtr;
  sacProb.threshold_ = threshold;
  sacProb.max_iterations_ = mMaxIter;
  sacProb.computeModel(0);

  opengv::transformation_t optimized_pose;
  sacProb.sac_model_->optimizeModelCoefficients(sacProb.inliers_, sacProb.model_coefficients_, optimized_pose);

  std::cout << "sacProb.inliers_.size(): " << sacProb.inliers_.size()
            << std::endl;
  
    if (sacProb.inliers_.size() < mMinInliers) {
//      std::cout << "sacProb.inliers_.size(): " << sacProb.inliers_.size() << std::endl;
    return false;
    }
    std::cout << "Ransac needed " << sacProb.iterations_ << " iterations and ";
    std::cout << std::endl;
    std::cout << "the number of inliers is: " << sacProb.inliers_.size();
    std::cout << std::endl << std::endl;

    //Inlier keypoints
    std::vector<int> inlierInd = sacProb.inliers_;
    kp_vect1_in_.reserve(sacProb.inliers_.size());
    kp_vect2_in_.reserve(sacProb.inliers_.size());

    size_t j=0;
    for (size_t i : inlierInd) {
      cv::KeyPoint temp_kp1(float(KfPtr1->keypoints_undistorted_add_[ImgMatches[i].idxA].x()),
                            float(KfPtr1->keypoints_undistorted_add_[ImgMatches[i].idxA].y()), 1);
      cv::KeyPoint temp_kp2(float(KfPtr2->keypoints_undistorted_add_[ImgMatches[i].idxB].x()),
                            float(KfPtr2->keypoints_undistorted_add_[ImgMatches[i].idxB].y()), 1);
      cv::DMatch temp_match(j,j,0);
      kp_vect1_in_.push_back(temp_kp1);
      kp_vect2_in_.push_back(temp_kp2);
      matches_in_.push_back(temp_match);
      ++j;
    }

    // KfPtr1->img_
    cv::Mat img_out;
    // double cx = KfPtr1->calibration_.intrinsics[2];
    // double cy = KfPtr1->calibration_.intrinsics[3];
    // double fx = KfPtr1->calibration_.intrinsics[0];
    // double fy = KfPtr1->calibration_.intrinsics[1];

    // Mat Camera_Matrix;
    // Mat Distortion_Coefficients;
    // cv::eigen2cv(KfPtr1->calibration_.K, Camera_Matrix);
    // cv::eigen2cv(KfPtr1->calibration_.dist_coeffs, Distortion_Coefficients);
    

    // cv::Mat img_und_1; // Will be the undistorted version of the above image.
    // cv::Mat img_und_2;

    // cv::undistort(KfPtr1->img_, img_und_1, Camera_Matrix,
    //               Distortion_Coefficients);
    // cv::undistort(KfPtr2->img_, img_und_2, Camera_Matrix,
    //               Distortion_Coefficients);
    cv::drawMatches(KfPtr1->img_, kp_vect1_in_, KfPtr2->img_,
                    kp_vect2_in_, matches_in_, img_out);
    // cv::drawMatches(img_und_1, kp_vect1_in_, img_und_2, kp_vect2_in_,
    //                 matches_in_, img_out);
    
    std::stringstream ss;
    ss << "/home/manthan/ws_vins/covins_ws/results/imgs/" << KfPtr1->id_.first << "_" << KfPtr2->id_.first  << ".jpg";
    cv::imwrite(ss.str(), img_out);
    
    Tc1c2 = Eigen::Matrix4d::Identity();
    Tc1c2.block<3, 4>(0, 0) = optimized_pose;
    // Tc1c2.block<3, 4>(0, 0) = sacProb.model_coefficients_;
    Eigen::Quaterniond Rotquat(Tc1c2.block<3, 3>(0, 0));

    std::ofstream myfile(
        "/home/manthan/ws_vins/covins_ws/results/results_tf.csv",
        std::ios::app);
    
    // Write to File
    // Query KF Timestamp (x1e9), Query KF ID,  QKF Agent ID,
    // Candidate KF Timestamp (x1e9),Candidate KF ID, CKF Agent ID, 
    // x, y, z, qx, qy, qz, qw, Number of Inliers
    // Extract the pose from RANSAC

    myfile << std::setprecision(19) << (KfPtr1->timestamp_) * 1e9 << "," << KfPtr1->id_.first << ","
           << KfPtr1->id_.second << "," << std::setprecision(19) << (KfPtr2->timestamp_) * 1e9 << ","
           << KfPtr2->id_.first << "," << KfPtr2->id_.second << ","
           << Tc1c2(0, 3) << "," << Tc1c2(1, 3) << "," << Tc1c2(2, 3) << ","
           << Rotquat.x() << "," << Rotquat.y() << "," << Rotquat.z() << ","
           << Rotquat.w() << "," << sacProb.inliers_.size() << std::endl;
    return true;

}

} //ns ends