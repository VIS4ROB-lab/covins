// Header file
#include "covins_backend/RelNonCentralPosSolver.hpp"

// Standard includes
#include <cmath>
#include <vector>

#include <eigen3/Eigen/Core>

// CoVINS
#include "covins_backend/keyframe_be.hpp"
#include "covins_base/config_backend.hpp"

// opengv related includes
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/NoncentralRelativePoseSacProblem.hpp>
#include "matcher/opengv/rel_pose/frame-relative-adapter.hpp"
#include "matcher/opengv/rel_pose/FrameNoncentralRelativeAdapter.hpp"
#include "matcher/opengv/sac_problems/frame-relative-pose-sac-problem.hpp"
#include "opengv/relative_pose/methods.hpp"

#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>



namespace covins {

// The class constructor
RelNonCentralPosSolver::RelNonCentralPosSolver(const size_t minInliers, const double ransacProb,
                           const size_t maxIter)
    : mMinInliers(minInliers),
    mRansacProb(ransacProb),
    mMaxIter(maxIter)
{
  mMinInliers_17PT = covins_params::placerec::nc_rel_pose::min_inliers;
  mMaxIter_17PT = covins_params::placerec::nc_rel_pose::max_iters;
  mImgMatches = covins_params::placerec::rel_pose::min_img_matches;
  mMatchThreshold = covins_params::features::img_match_thres;
  mMinInliers = covins_params::placerec::rel_pose::min_inliers;
  mMaxIter = covins_params::placerec::rel_pose::max_iters;
  mRansacProb = covins_params::placerec::ransac::probability;
  mCov_iter = covins_params::placerec::nc_rel_pose::cov_iters;
  mCov_max_iter = covins_params::placerec::nc_rel_pose::cov_max_iters;
  mMax_cov = covins_params::placerec::nc_rel_pose::cov_thres;
  mRP_err = covins_params::placerec::nc_rel_pose::rp_error_cov;
  double thres_17pt = covins_params::placerec::nc_rel_pose::rp_error;
  mThres_17PT = 2.0 * (1.0 - cos(atan(thres_17pt * 1 / 800.0)));
  
}

// Set the parameters used for RANSAC
void RelNonCentralPosSolver::setRansacParams(const int minInliers,
    const double ransacProb, const int maxIter) {
  mMinInliers = minInliers;
  mRansacProb = ransacProb;
  mMaxIter = maxIter;
}

bool RelNonCentralPosSolver::computeNonCentralRelPose(
    const KeyframePtr QKF, const KeyframePtr CKF,
    const double threshold, Eigen::Matrix4d &Tc1c2, Eigen::Matrix<double, 6, 6> &cov_loop) {

  // Find matches between CKF and QKF
  std::cout << "Attempting NON-Relative Pose Solver" << std::endl;

  // 3 v 2 Matching of Multicamera Systems (See publication for reference)
  size_t n_ckfs = 3;
  size_t n_qkfs = 2;
  TransformType T_QKF_cw = QKF->GetPoseTcw();
  TransformType T_CKF_cw = CKF->GetPoseTcw();
  TransformType T_init;
  cov_loop = Eigen::Matrix<double,6,6>::Identity();
  // Initialize the adapter vectors
  KeyframeVector view_A;
  KeyframeVector view_B;
  std::vector<std::vector<Matches>> match_vect(n_qkfs);
  std::vector<std::vector<TransformType>> TF_vect(2);
  std::vector<std::vector<std::vector<int>>> inliers_vect(n_qkfs);
  std::vector<int> inliers_size;

  for (size_t i = 0; i < n_qkfs; ++i) {

    KeyframePtr curr_QKF = QKF;
    std::vector<Matches> curr_match_vect;
    std::vector<std::vector<int>> curr_inliers_vect;

    // Get the Neighboring KF according to iteration
    for (size_t j = 0; j < i; ++j) {
      curr_QKF = curr_QKF->GetPredecessor();
    }

    view_A.push_back(curr_QKF);
    Eigen::Matrix4d temp_TF = T_QKF_cw * curr_QKF->GetPoseTwc();

    TF_vect[0].push_back(temp_TF);

    for (size_t j = 0; j < n_ckfs; ++j) {

      // Do Computations for the QKF[i] vs all NCKFs and push the entire row in
      // adapter
      
      KeyframePtr curr_CKF = CKF;

      // Get the Neighboring KF according to iteration
      for (size_t k = 0; k < j; ++k) {
        curr_CKF = curr_CKF->GetPredecessor();
      }

      if (i == 0) {
        // Add the CKFs and TF to view for the first time
        view_B.push_back(curr_CKF);
        Eigen::Matrix4d temp_TF = T_CKF_cw * curr_CKF->GetPoseTwc();
        TF_vect[1].push_back(temp_TF);
      }

      Matches matches_QC = this->findMatches(curr_QKF, curr_CKF);

      if (matches_QC.size() < mImgMatches) {
          return false;
      }
      Eigen::Matrix4d T12_5pt;
      // Solve 5 Point Ransac between them
      Tc1c2 = Eigen::Matrix4d::Identity();
      std::vector<int> inlierInd_QC;

      if (this->computePose(curr_QKF, curr_CKF, matches_QC, threshold, T12_5pt,
                            inlierInd_QC)) {
        if (i == 0 && j == 0) {
          T_init = T12_5pt;
        }

        curr_inliers_vect.push_back(inlierInd_QC);
        curr_match_vect.push_back(matches_QC);
        inliers_size.push_back(inlierInd_QC.size());

      } else {
        return false;
      }
    }

    match_vect[i] = curr_match_vect;
    inliers_vect[i] = curr_inliers_vect;
  }

  opengv::relative_pose::FrameNoncentralRelativeAdapter adapter(
      view_A, view_B, match_vect, TF_vect, inliers_vect, T_init);
  
        opengv::sac::Ransac<opengv::sac_problems::relative_pose::NoncentralRelativePoseSacProblem> sacProb;
          std::shared_ptr<opengv::sac_problems::relative_pose::
                              NoncentralRelativePoseSacProblem>
              relposeproblem_ptr(
                  new opengv::sac_problems::relative_pose::
                      NoncentralRelativePoseSacProblem(
                          adapter,
                          opengv::sac_problems::relative_pose::
                              NoncentralRelativePoseSacProblem::SEVENTEENPT));
          
        sacProb.sac_model_ = relposeproblem_ptr;
        
        sacProb.threshold_ = mThres_17PT;
        sacProb.max_iterations_ = mMaxIter_17PT;
        sacProb.computeModel(0);

        std::cout << "17 POINT Ransac needed " << sacProb.iterations_ << " iterations and ";
        std::cout << std::endl;
        std::cout << "the number of inliers is: " << sacProb.inliers_.size();
        std::cout << std::endl << std::endl;

        if (sacProb.inliers_.size() < mMinInliers_17PT ) {
        return false;
        }

        std::vector<int> inlierInd_17PT = sacProb.inliers_;

        opengv::transformation_t optimized_pose;
        sacProb.sac_model_->optimizeModelCoefficients(
            sacProb.inliers_, sacProb.model_coefficients_, optimized_pose);

        Tc1c2 = Eigen::Matrix4d::Identity();
        Tc1c2.block<3, 4>(0, 0) = optimized_pose;
        
        TransformType Ts1s2 = Eigen::Matrix4d::Identity();
        TransformType Twc2 = CKF->GetPoseTwc();
        TransformType Tws2 = CKF->GetPoseTws();
        TransformType Twc1corr = Twc2 * Tc1c2.inverse();
        TransformType Tws1 = (Twc1corr * QKF->GetStateExtrinsics().inverse());
        Ts1s2 = Tws1.inverse() * Tws2;

        std::cout << "T_s2s1 :" << std::endl << Ts1s2.inverse() << std::endl;

        ////////////////////////////////////////////////////
        // TODO Shift the Covariance Computation to Seperate Function

        // Compute Observed Covariance
        std::vector<std::vector<int>> inliers_17PT_vect(inliers_size.size());
        std::vector<int> inliers_postion;
        inliers_postion.push_back(0);
        Eigen::Vector3d trans_vect = Tc1c2.block<3, 1>(0, 3);
        trans_vect.normalize();
        
        size_t cov_rows = mCov_iter;
        std::vector<int> inliers_cov;
        Eigen::Matrix<double, Eigen::Dynamic, 6> m_s(cov_rows, 6); // Obs Matrix: First 3 cols-Rot, next 3 cols-Trans 
        
        std::random_device rd;
        TypeDefs::QuaternionType q_ref(Tc1c2.block<3, 3>(0, 0));
        TypeDefs::QuaternionType q_ref_s(Ts1s2.block<3, 3>(0, 0));

        // Build a vector for knowing the positions of the inliers
        for (size_t i = 0; i < inliers_size.size(); ++i) {
          inliers_postion.push_back(inliers_postion[i] + inliers_size[i]);
        }

        for (int i : inlierInd_17PT) {
          for (size_t j = 0; j < inliers_postion.size() - 1; ++j) {   
            if (i >= inliers_postion[j] && i < inliers_postion[j + 1]) {
              inliers_17PT_vect[j].push_back(i);
              break;
            }
          }
        }

        int num_samples = 4;
        int num_iter_good = 0;
        int num_inl = 0;

        // Build the Covariance Matrix
        // Check if enough samples in each set
        for (size_t j = 0; j < inliers_17PT_vect.size(); ++j) {
          if (inliers_17PT_vect[j].size() < 2 * num_samples) {
            std::cout << "Not enough Samples in set "<< j << std::endl;
            return false;
          }  
        }

        int iterations = 0;
        for (size_t i = 0; i < cov_rows; ++i) {

          if (iterations > mCov_max_iter)
            break;
          
          inliers_cov.clear();
          std::mt19937 g(rd());

          // Do equal sampling from each set
          for (size_t j = 0; j < inliers_17PT_vect.size(); ++j) {
            std::shuffle(inliers_17PT_vect[j].begin(),
                         inliers_17PT_vect[j].end(), g);
            for (int k = 0; k < num_samples; ++k) {
              inliers_cov.push_back(inliers_17PT_vect[j][k]);
            }
          }

          adapter.setR12(Tc1c2.block<3, 3>(0, 0));
        
          opengv::transformation_t temp_pose =
              opengv::relative_pose::seventeenpt(adapter, inliers_cov);

          // Find inlier ratio of inliers for current pose estimate
          std::vector<double> scores;
          sacProb.sac_model_->getSelectedDistancesToModel(
                temp_pose, inlierInd_17PT, scores);

          num_inl = 0;

          for (double i : scores) {
            if (i < 2.0*(1.0 - cos(atan(mRP_err*1/800.0))))
              num_inl++;
          }

          if (float(num_inl) / inlierInd_17PT.size() > 0.8) {
            num_iter_good++;

            // Convert to IMU Frame
            Eigen::Matrix4d temp_pose_s = Eigen::Matrix4d::Identity();
            temp_pose_s.block<3,4>(0,0) = temp_pose;
            Eigen::Matrix4d Twc1corr = Twc2 * temp_pose_s.inverse();
            Eigen::Matrix4d Tws1 = (Twc1corr * QKF->GetStateExtrinsics().inverse());
            Eigen::Matrix4d Ts1s2_temp = Tws1.inverse() * Tws2;

            m_s.block<1, 3>(i, 3) = Ts1s2_temp.block<3, 1>(0, 3).transpose(); //Translation Terms

            TypeDefs::QuaternionType q_iter_s(Ts1s2_temp.block<3, 3>(0, 0));
            Eigen::Vector3d rotation_sample_s;
            robopt::common::quaternion::Minus(q_iter_s, q_ref_s, &rotation_sample_s);
            m_s.block<1, 3>(i, 0) = rotation_sample_s.transpose(); //Rotation Terms
            
          } else {
            --i;
          }
          ++iterations;
        }

        Eigen::Matrix<double, 1, 6> x_mean_s = Eigen::Matrix<double, 1, 6>::Zero();
        x_mean_s.block<1, 3>(0, 3) = Ts1s2.block<3, 1>(0, 3).transpose(); //Translation Reference (mean)

        cov_loop = ((m_s.rowwise() - x_mean_s).matrix().transpose() *
                    (m_s.rowwise() - x_mean_s).matrix()) /
                   (m_s.rows() - 1);
        
        std::cout << "Cov Mat Trace: " << cov_loop.trace() << std::endl;

        if (cov_loop.trace() < mMax_cov) {
          return true;
        }
  	
 ///////////////////////////////////////////////////////////////////////////////////////////

	return false;
}

auto RelNonCentralPosSolver::findMatches(const KeyframePtr KFPtr1,
                                         const KeyframePtr KFPtr2) -> Matches {

  std::shared_ptr<cv::BFMatcher> matcher(nullptr);

  if (covins_params::features::type == "ORB") {
      matcher = make_shared<cv::BFMatcher>(cv::NORM_HAMMING);
  } else if (covins_params::features::type == "SIFT") {
      matcher = make_shared<cv::BFMatcher>(cv::NORM_L2);
  } else {
      std::cout << COUTERROR
              << "Wrong Feature Type: Only ORB or SIFT is supported currently"
              << std::endl;
      exit(-1);
  }
  
  std::vector<cv::DMatch> matches;
  std::vector<std::vector<cv::DMatch>> matches_vect;
  Matches img_bf_matches;

  matcher->knnMatch(KFPtr1->descriptors_add_, KFPtr2->descriptors_add_,
                   matches_vect, 2);

  for (size_t i = 0; i < matches_vect.size(); ++i) {
    cv::DMatch curr_m = matches_vect[i][0];
    cv::DMatch curr_n = matches_vect[i][1];

    // Distance threshold + Ratio Test
    if (curr_m.distance <= covins_params::features::img_match_thres) {
      if (curr_m.distance < 0.8 * curr_n.distance) {
        Match temp_match(curr_m.queryIdx, curr_m.trainIdx, curr_m.distance);
        img_bf_matches.push_back(temp_match);
      }
    }
  }

  return (img_bf_matches);
}

// Solve the 2d-2d problem
bool RelNonCentralPosSolver::computePose(const KeyframePtr KfPtr1,
                               const KeyframePtr KfPtr2,
                               Matches &ImgMatches, const double threshold,
                               Eigen::Matrix4d &Tc1c2, std::vector<int> &inlierInd) {

// Find the Matching keypoint indices (For Debugging)
const size_t num_matches = ImgMatches.size();

kp_vect1_.clear();
kp_vect2_.clear();
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

  //Inlier keypoints
  inlierInd = sacProb.inliers_;

  kp_vect1_in_.clear();
  kp_vect2_in_.clear();
  matches_in_.clear();

  std::cout << "Inliers: "<< sacProb.inliers_.size() << " Iters: "<< sacProb.iterations_ << std::endl;

  if (sacProb.inliers_.size() < mMinInliers || sacProb.iterations_ >=
  sacProb.max_iterations_) {
    return false;
  }

    Tc1c2 = TransformType::Identity();
    Tc1c2.block<3, 4>(0, 0) = sacProb.model_coefficients_;

    return true;
}

} //ns ends