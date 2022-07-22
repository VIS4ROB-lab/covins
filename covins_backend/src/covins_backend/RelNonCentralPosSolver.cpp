// Header file
#include "covins_backend/RelNonCentralPosSolver.hpp"

// Standard includes
#include <cmath>
#include <vector>

#include <eigen3/Eigen/Core>

// CoVINS
#include "covins_backend/keyframe_be.hpp"
#include "covins_base/config_backend.hpp"
#include <covins/covins_base/timer_utils.hpp>

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
  mCov_iter = covins_params::placerec::nc_rel_pose::cov_iters;
  mMax_cov = covins_params::placerec::nc_rel_pose::cov_thres;
  mRP_err = covins_params::placerec::nc_rel_pose::rp_error;
  mThres_17PT = 2.0 * (1.0 - cos(atan(1.5 * 1 / 800.0)));
  
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
  time_utils::Timer timer;
  time_utils::Timer timer_local;
  time_utils::Timer timer_global;
  std::ofstream file_img_matches("/home/manthan/ws/covins_ws/src/covins/covins_backend/"
                       "output/results_img_match.csv",
                       std::ios::app);
  std::ofstream file_17PT("/home/manthan/ws/covins_ws/src/covins/covins_backend/"
                       "output/results_17PT.csv",
                       std::ios::app);

  std::ofstream file_lms("/home/manthan/ws/covins_ws/src/covins/covins_backend/"
                       "output/lms_local.csv",
                       std::ios::app);

  size_t n_ckfs = 3;
  size_t n_qkfs = 2;
  size_t lm_id = 0;
  Eigen::Matrix4d T_QKF_cw = QKF->GetPoseTcw();
  Eigen::Matrix4d T_CKF_cw = CKF->GetPoseTcw();
  Eigen::Matrix4d T_init;
  cov_loop = Eigen::Matrix<double,6,6>::Identity();
  // Initialize the adapter vectors
  KeyframeVector view_A;
  KeyframeVector view_B;
  std::vector<std::vector<Matches>> match_vect(n_qkfs);
  std::vector<std::vector<Eigen::Matrix4d>> TF_vect(2);
  std::vector<std::vector<std::vector<int>>> inliers_vect(n_qkfs);
  std::vector<int> inliers_size;
  std::vector<float> baseline_vect;
  std::vector<std::shared_ptr<LocalLM>> lm_vect;

  for (size_t i = 0; i < n_qkfs; ++i) {

    // std::cout << "Outside LOOP: " << i << std::endl;
    KeyframePtr curr_QKF = QKF;
    std::vector<Matches> curr_match_vect;
    // std::vector<Eigen::Matrix4d> TF_vect;
    std::vector<std::vector<int>> curr_inliers_vect;

    

    // Get the Neighboring KF according to iteration
    for (size_t j = 0; j < i; ++j) {
      // std::cout << "Fetching NQKF: " << j << std::endl;
      curr_QKF = curr_QKF->GetPredecessor();
    }

    view_A.push_back(curr_QKF);
    Eigen::Matrix4d temp_TF = T_QKF_cw * curr_QKF->GetPoseTwc();
    // std::cout << "Before Pushing TF" << std::endl;

    TF_vect[0].push_back(temp_TF);
    baseline_vect.push_back(temp_TF.block<3,1>(0,3).norm());

    for (size_t j = 0; j < n_ckfs; ++j) {

      fprintf(stderr, "INFO: Resetting Timer: %lu us\n",
              timer_local.measure());
      
      // Do Computations for the QKF[i] vs all NCKFs and push the entire row in
      // adapter

      KeyframePtr curr_CKF = CKF;

      // Get the Neighboring KF according to iteration
      for (size_t k = 0; k < j; ++k) {
        curr_CKF = curr_CKF->GetPredecessor();
      }

      if (i == 0) {
        // Add the CKFs to view for the first time
        // Also need to add the TF (TO DO)
        view_B.push_back(curr_CKF);
        Eigen::Matrix4d temp_TF = T_CKF_cw * curr_CKF->GetPoseTwc();
        TF_vect[1].push_back(temp_TF);
        baseline_vect.push_back(temp_TF.block<3,1>(0,3).norm());
      }

      // Print the NKF and CKF IDs
      // std::cout << "The KF IDs are curr_CKF | curr_QKF  :" << curr_CKF->id_.first
      //         << " | " << curr_QKF->id_.first << std::endl;

      Matches matches_QC = this->findMatches(curr_QKF, curr_CKF);

      // fprintf(stderr, "INFO: Image Matching took: %lu us\n",
      //         timer_local.measure());
      file_img_matches << timer_local.measure() << std::endl;
      
      // std::cout << "Matches in QKF-CKF[" << i << "," << j
      //           << "]: " << matches_QC.size() << std::endl;
      if (matches_QC.size() < mImgMatches) {
          return false;
      }
      Eigen::Matrix4d T12_5pt;
      // Solve 5 Point Ransac between them
      Tc1c2 = Eigen::Matrix4d::Identity();
      std::vector<int> inlierInd_QC;
      Vector3Vector pose_vect;

      if (this->computePose(curr_QKF, curr_CKF, matches_QC, threshold, T12_5pt,
                            inlierInd_QC, pose_vect)) {
        // std::cout << "5 Point T12: "<< "\n" << T12_5pt << std::endl;

        // fprintf(stderr, "INFO: 5 PT Ransac took: %lu us\n",
        //         timer_local.measure());
        if (i == 0 && j == 0) {
          T_init = T12_5pt;
        }

        curr_inliers_vect.push_back(inlierInd_QC);
        curr_match_vect.push_back(matches_QC);
        // std::cout << "Inliers in Neighbouring KF[" << i << "," << j
        //           << "]: " << inlierInd_QC.size() << std::endl;
        inliers_size.push_back(inlierInd_QC.size());

        // Triangulation Modifications
        TransformType TF_QQi = QKF->GetStateExtrinsics()*TF_vect[0][i];
        
        for (size_t l = 0; l < pose_vect.size(); ++l) {
          auto poseQi = pose_vect[l];
          KfObservations temp_obs;
          auto ind = inlierInd_QC[l];
          temp_obs[curr_QKF] = matches_QC[ind].idxA;
          temp_obs[curr_CKF] = matches_QC[ind].idxB;

          // Transform the Landmark Pose to QKF IMU frame
          Vector3Type pose_Q = TF_QQi.block<3,3>(0,0)*poseQi + TF_QQi.block<3,1>(0,3);
          // Create Landmark
          std::shared_ptr<LocalLM> curr_lm(new LocalLM(lm_id, pose_Q, temp_obs));
          lm_vect.push_back(curr_lm);
          lm_id++;
        }

      } else {
        return false;
      }
    }

    match_vect[i] = curr_match_vect;
    inliers_vect[i] = curr_inliers_vect;
  }
  file_lms << std::endl;

  file_17PT << std::endl;
  float bef_seventeen = timer.measure();
  float seventeen_time = 0;
  float seventeen_opt = 0;
  float seventeen_cov = 0;
  file_17PT << "," << bef_seventeen;
  // fprintf(stderr, "INFO: Total Time before setting up 17 PT took: %lu us\n",
  //               timer.measure());

  std::cout << lm_vect.size() << "|" << inliers_size.size() << std::endl;

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

        // std::cout << "17 POINT Ransac needed " << sacProb.iterations_ << " iterations and ";
        // std::cout << std::endl;
        // std::cout << "the number of inliers is: " << sacProb.inliers_.size();
        // std::cout << std::endl << std::endl;

        // fprintf(stderr,
        //         "INFO: 17 PT took: %lu us\n",
        //         timer.measure());
        seventeen_time = timer.measure();
        file_17PT << "," << seventeen_time;

        // if (sacProb.inliers_.size() < mMinInliers_17PT ||
        //     sacProb.iterations_ >= sacProb.max_iterations_) {
        if (sacProb.inliers_.size() < mMinInliers_17PT ) {
        // std::cout << "17 point FAILED - Inliers Found " << sacProb.inliers_.size() << std::endl;
        return false;
        }

        std::vector<int> inlierInd_17PT = sacProb.inliers_;

        opengv::transformation_t optimized_pose;
        sacProb.sac_model_->optimizeModelCoefficients(
            sacProb.inliers_, sacProb.model_coefficients_, optimized_pose);

        Tc1c2 = Eigen::Matrix4d::Identity();
        Tc1c2.block<3, 4>(0, 0) = optimized_pose;
        
        Eigen::Matrix4d Ts1s2 = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d Twc2 = CKF->GetPoseTwc();
        Eigen::Matrix4d Tws2 = CKF->GetPoseTws();
        Eigen::Matrix4d Twc1corr = Twc2 * Tc1c2.inverse();
        Eigen::Matrix4d Tws1 = (Twc1corr * QKF->GetStateExtrinsics().inverse());
        Ts1s2 = Tws1.inverse() * Tws2;

        std::cout << "T_s2s1 :" << std::endl << Ts1s2.inverse() << std::endl;

        // fprintf(stderr, "INFO: Optimized 17 PT took: %lu us\n",
        //         timer.measure());
        seventeen_opt = timer.measure();
        file_17PT << "," << seventeen_opt;
        // std::cout << "T12 (17 Point RANSAC):\n " << std::setprecision(6)
        //           << Tc1c2 << std::endl;
        // return true;

        // for (auto i : inlierInd_17PT) {
        //   file_lms << lm_vect[i]->pos_l_.x() << "," << lm_vect[i]->pos_l_.y() << ","<< lm_vect[i]->pos_l_.z() << ","<< std::endl;
        // }


        ////////
        
        // Covariance Matrix ////////////////
        // 
        std::vector<std::vector<int>> inliers_17PT_vect(inliers_size.size());
        std::vector<int> inliers_postion;
        inliers_postion.push_back(0);
        Eigen::Vector3d trans_vect = Tc1c2.block<3, 1>(0, 3);
        trans_vect.normalize();
        
        size_t cov_rows = mCov_iter;
        std::vector<int> inliers_cov;
        Eigen::MatrixX3d m(cov_rows, 3);
        Eigen::Matrix<double, Eigen::Dynamic, 6> m_full(cov_rows, 6); //First 3 cols-Rot, next 3 cols-Trans
        Eigen::Matrix<double, Eigen::Dynamic, 6> m_full_s(cov_rows, 6); //First 3 cols-Rot, next 3 cols-Trans
        Eigen::VectorXd m_trans(cov_rows);
        
        std::random_device rd;
        TypeDefs::QuaternionType q_ref(Tc1c2.block<3, 3>(0, 0));
        TypeDefs::QuaternionType q_ref_s(Ts1s2.block<3, 3>(0, 0));

        // Build a vector for knowing the positions of the inliers
        for (size_t i = 0; i < inliers_size.size(); ++i) {
          inliers_postion.push_back(inliers_postion[i] + inliers_size[i]);
        }

        // for (auto i : inliers_postion) {
        //   std::cout << i << std::endl;
        // }
        // std::cout << adapter.getNumberCorrespondences() << std::endl;


        for (int i : inlierInd_17PT) {

          for (size_t j = 0; j < inliers_postion.size() - 1; ++j) {
            // std::cout << "Entered" << i << " " << j << " " << inliers_postion[j]
            //           << " " << inliers_postion[j + 1] << std::endl;       
            if (i >= inliers_postion[j] && i < inliers_postion[j + 1]) {
              inliers_17PT_vect[j].push_back(i);
              break;
            }
          }
        }

        // int num_samples = std::ceil(17.0 / inliers_17PT_vect.size());
        int num_samples = 4;
        int num_iter_good = 0;
        int num_inl = 0;
        // Build the Covariance Matrix

        std::cout << "Building Covariance Matrix" << std::endl;

        // Check if enough samples in each set
        for (size_t j = 0; j < inliers_17PT_vect.size(); ++j) {
          if (inliers_17PT_vect[j].size() < 2 * num_samples) {
            std::cout << "Not enough Samples in set "<< j << std::endl;
            return false;
          }  
        }

        int iterations = 0;
        for (size_t i = 0; i < cov_rows; ++i) {

          if (iterations > 300) {
            break;
          }
          
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

          // for (auto i: inliers_cov)
          //   std::cout << i << ' ';
          // std::cout << std::endl;

          adapter.setR12(Tc1c2.block<3, 3>(0, 0));

          opengv::transformation_t temp_pose =
              opengv::relative_pose::seventeenpt(adapter, inliers_cov);

          // if(iterations%20==0)
          //   std::cout << "iter" << i << std::endl;

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
            // std::cout << "good Iter Start" << std::endl;
            num_iter_good++;

            // Convert to IMU Frame
            Eigen::Matrix4d temp_pose_s = Eigen::Matrix4d::Identity();
            temp_pose_s.block<3,4>(0,0) = temp_pose;
            Eigen::Matrix4d Twc1corr = Twc2 * temp_pose_s.inverse();
            Eigen::Matrix4d Tws1 = (Twc1corr * QKF->GetStateExtrinsics().inverse());
            Eigen::Matrix4d Ts1s2_temp = Tws1.inverse() * Tws2;


            m.block<1, 3>(i, 0) = temp_pose.block<3, 1>(0, 3).transpose();
            m_full.block<1, 3>(i, 3) = temp_pose.block<3, 1>(0, 3).transpose();
            m_full_s.block<1, 3>(i, 3) =
                Ts1s2_temp.block<3, 1>(0, 3).transpose();

            TypeDefs::QuaternionType q_iter(temp_pose.block<3, 3>(0, 0));
            TypeDefs::QuaternionType q_iter_s(Ts1s2_temp.block<3, 3>(0, 0));
            Eigen::Vector3d rotation_sample;
            Eigen::Vector3d rotation_sample_s;

            robopt::common::quaternion::Minus(q_iter, q_ref, &rotation_sample);
            robopt::common::quaternion::Minus(q_iter_s, q_ref_s, &rotation_sample_s);

            rotation_sample *= 10;

            m_full.block<1, 3>(i, 0) = rotation_sample.transpose();
            m_full_s.block<1, 3>(i, 0) = rotation_sample_s.transpose();

            Eigen::Vector3d trans_vect_sample = temp_pose.block<3, 1>(0, 3);
            trans_vect_sample.normalize();
            // std::cout << "Trans Vect Sample " << trans_vect_sample <<
            // std::endl; std::cout << "Trans Vect " << trans_vect << std::endl;
            // std::cout << "Dot Product " << trans_vect_sample.dot(trans_vect)
            //           << std::endl;
            // std::cout << "ArcCos " << acos(trans_vect_sample.dot(trans_vect))
            // << std::endl;
            
            m_trans(i) = acos(trans_vect_sample.dot(trans_vect));
            // std::cout << "good Iter" << std::endl;
          } else {
            i--;
          }
          iterations++;
          
        }
        std::cout << "Total Covariance Iterations: " << iterations  << "Good iter: " << num_iter_good << std::endl;

        auto x_mean_2 = m.colwise().mean();
        Eigen::Matrix<double, 1, 3> x_mean = Tc1c2.block<3, 1>(0, 3).transpose();
        Eigen::Matrix<double, 1, 3> x_mean_s = Ts1s2.block<3, 1>(0, 3).transpose();
        Eigen::Matrix<double, 1, 6> x_mean_full = Eigen::Matrix<double, 1, 6>::Zero();
        Eigen::Matrix<double, 1, 6> x_mean_full_s = Eigen::Matrix<double, 1, 6>::Zero();
        x_mean_full.block<1, 3>(0, 3) = x_mean;
        x_mean_full_s.block<1, 3>(0, 3) = x_mean_s;

        // std::cout << " X_mean_full is: " << x_mean_full << std::endl;

        // std::cout << " Trans Matrix is: " << m_trans << std::endl;
        
        Eigen::Matrix3d cov_mat = ((m.rowwise() - x_mean).matrix().transpose() *
                                   (m.rowwise() - x_mean).matrix()) /
                                  (m.rows() - 1);

        Eigen::Matrix3d cov_mat_2 = ((m.rowwise() - x_mean_2).matrix().transpose() *
                                   (m.rowwise() - x_mean_2).matrix()) /
                                  (m.rows() - 1);

        Eigen::Matrix<double, 6, 6> cov_mat_full =
            ((m_full.rowwise() - x_mean_full).matrix().transpose() *
             (m_full.rowwise() - x_mean_full).matrix()) /
            (m_full.rows() - 1);

        Eigen::Matrix<double, 6, 6> cov_mat_full_s =
            ((m_full_s.rowwise() - x_mean_full_s).matrix().transpose() *
             (m_full_s.rowwise() - x_mean_full_s).matrix()) /
            (m_full_s.rows() - 1);
        
        Eigen::Matrix<double, 6, 6> cov_mat_full_t =
            Eigen::Matrix<double, 6, 6>::Identity();
        
        cov_mat_full_t(3, 3) = cov_mat_full(3, 3);
        cov_mat_full_t(4, 4) = cov_mat_full(4, 4);
        cov_mat_full_t(5, 5) = cov_mat_full(5, 5);
        
        double sd_trans = sqrt((m_trans.array()).square().sum() / (m_trans.size()));

        // std::cout << "SD of Trans vector" << sd_trans << std::endl;
        cov_loop = cov_mat_full_s;

        std::cout << "Full COV Matrix IMU Frame is: " << std::endl;
        std::cout << cov_mat_full_s << std::endl;

        // std::cout << "Full COV Matrix Camera Frame is: " << std::endl;
        // std::cout << cov_mat_full << std::endl;

        // std::cout << cov_mat << std::endl;
        std::cout << "Cov_Mat Trace: " << cov_mat.trace() << std::endl;

        // std::cout << "Full COV Matrix is: " << std::endl;
        // std::cout << cov_loop << std::endl;
        // std::cout << "Cov_Mat Trace: " << cov_mat_full.block<3,3>(3,3).trace() << std::endl;

        Eigen::Quaterniond Rotquat(Tc1c2.block<3, 3>(0, 0));
        Eigen::Quaterniond Rotquat_5pt(T_init.block<3, 3>(0, 0));

        std::ofstream myfile("/home/manthan/ws/covins_ws/src/covins/"
                             "covins_backend/output/results_tf.csv",
                             std::ios::app);
        std::ofstream file_covr("/home/manthan/ws/covins_ws/src/covins/"
                             "covins_backend/output/results_cov_ransac.csv",
                             std::ios::app);

        ///// LOCAL BA
        // TypeDefs::Matrix6Type cov_BA = TypeDefs::Matrix6Type::Identity();
        // Optimization::LocalBundleAdjustment(lm_vect, view_A, view_B, Ts1s2,
        //                                     cov_BA);

        // // BundleAdjusted TF

        // TransformType T_s1s2_BA =
        //     Utils::Ceres2Transform(CKF->ceres_pose_local_);
        // Eigen::Quaterniond Rotquat_BA(T_s1s2_BA.block<3, 3>(0, 0));

        // TransformType T_c1c2_BA = QKF->GetStateExtrinsics().inverse() *
        //                           T_s1s2_BA * CKF->GetStateExtrinsics();

        // std::cout << "T_s2_s1_LBA: " << std::endl << T_s1s2_BA.inverse() << std::endl;

        // std::cout << "T_c1_c2_LBA: " << std::endl << T_c1c2_BA << std::endl;

        // std::cout << "Tc1c2 17 PT: " << std::endl << Tc1c2 << std::endl;

        // cov_loop = cov_BA;
        // Tc1c2 = T_c1c2_BA;
        
        // Write to File
        // Query KF Timestamp (x1e9), Query KF ID,  QKF Agent ID,
        // Candidate KF Timestamp (x1e9),Candidate KF ID, CKF Agent ID,
        // x, y, z, qx, qy, qz, qw, Number of Inliers, Num Iterations,
        // 5PT x, y, z, qx, qy, qz, qw, Cov_trace, numGoodIter, Inliers17PT
        // 0,1,2,3,4,5
        // const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ",");
        // file_covr << cov_mat_full_s.format(CSVFormat);
        // file_covr << "\n";

        myfile << std::setprecision(19) << (QKF->timestamp_) * 1e9 << ","
               << QKF->id_.first << "," << QKF->id_.second << ","
               << std::setprecision(19) << (CKF->timestamp_) * 1e9 << ","
               << CKF->id_.first << "," << CKF->id_.second << "," << Tc1c2(0, 3)
               << "," << Tc1c2(1, 3) << "," << Tc1c2(2, 3) << "," << Rotquat.x()
               << "," << Rotquat.y() << "," << Rotquat.z() << "," << Rotquat.w()
               << "," << sacProb.inliers_.size() << "," << sacProb.iterations_
               << "," << T_init(0, 3) << "," << T_init(1, 3) << ","
               << T_init(2, 3) << "," << Rotquat_5pt.x() << ","
               << Rotquat_5pt.y() << "," << Rotquat_5pt.z() << ","
               << Rotquat_5pt.w() << "," << cov_mat.trace() << ","
               << num_iter_good << "," << iterations << "," << inliers_size[0]
               << "," << inliers_size[1] << "," << inliers_size[2] << ","
               << inliers_size[3] << "," << inliers_size[4] << ","
               << inliers_size[5] << "," << cov_mat(0, 0) << ","
               << cov_mat(1, 1) << "," << cov_mat(2, 2) << ","
               << cov_mat_2(0, 0) << "," << cov_mat_2(1, 1) << ","
               << cov_mat_2(2, 2) << "," << sd_trans << ","
               << cov_mat_full_s(0, 0) << "," << cov_mat_full_s(1, 1) << ","
               << cov_mat_full_s(2, 2) << "," << cov_mat_full_s(3, 3) << ","
               << cov_mat_full_s(4, 4) << "," << cov_mat_full_s(5, 5);
        // << ","
              //  << T_s1s2_BA(0, 3) << "," << T_s1s2_BA(1, 3) << ","
              //  << T_s1s2_BA(2, 3) << "," << Rotquat_BA.x() << ","
              //  << Rotquat_BA.y() << "," << Rotquat_BA.z() << ","
              //  << Rotquat_BA.w();
        
        
        for (auto i : baseline_vect)
          myfile << "," << i;

        myfile << std::endl;

        // Create 2D vector for easy plotting of each set Matches
        std::vector<std::vector<int>> inliers_17PT_vect_plot(
            inliers_size.size());

        for (int i : inlierInd_17PT) {
          for (size_t j = 0; j < inliers_postion.size() - 1; ++j) {
            if (i >= inliers_postion[j] && i < inliers_postion[j + 1]) {
              inliers_17PT_vect_plot[j].push_back(i - inliers_postion[j]);
              break;
            }
          }
        }

        std::stringstream s;
        s << cov_mat.trace() << "_" << sacProb.inliers_.size();
        size_t k =0;
        for (size_t i = 0; i < n_qkfs; ++i) {
          for (size_t j = 0; j < n_ckfs; ++j) {
            this->plotMatches(view_A[i], view_B[j], match_vect[i][j], inliers_17PT_vect_plot[k], s.str());
            ++k;
          }
        }

        // fprintf(stderr,
        //         "INFO: Covariance 17 PT took: %lu us\n",
        //         timer.measure());

        seventeen_cov = timer.measure();
        file_17PT << "," << seventeen_cov;

        float tot_seventeen = timer_global.measure();
        file_17PT << "," << tot_seventeen;

        // std::cout << mMax_cov << std::endl;
        // if (cov_mat.trace() < mMax_cov && num_iter_good >= cov_rows) {
        if (cov_mat.trace() < mMax_cov) {
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
      
  // for (auto curr_m : matches) {
  //   if (curr_m.distance <= covins_params::features::img_match_thres) {
  //     Match temp_match(curr_m.queryIdx, curr_m.trainIdx, curr_m.distance);
  //     img_bf_matches.push_back(temp_match);
  //   }
  // }

  return (img_bf_matches);
        
  // // Setup a Threaded Matcher between QKF and CKF
  // std::shared_ptr<ImageMatchingAlgorithm> matchingAlgorithmImage(
  //     new ImageMatchingAlgorithm(mMatchThreshold)); // default: 50.0
  
  // std::unique_ptr<estd2::DenseMatcher> imgMatcherThreaded (new estd2::DenseMatcher(8, 1, false));
  // matchingAlgorithmImage->setFrames(KFPtr1, KFPtr2);
  // imgMatcherThreaded->match<ImageMatchingAlgorithm>(*matchingAlgorithmImage);
  // Matches ImgMatches = matchingAlgorithmImage->getMatches();
  // return(ImgMatches);
}

void RelNonCentralPosSolver::plotMatches(const KeyframePtr KfPtr1,
                                         const KeyframePtr KfPtr2,
                                         Matches &ImgMatches,
                                         std::vector<int> &inlierInd, std::string s) {
  kp_vect1_in_.clear();
  kp_vect2_in_.clear();
  matches_in_.clear();

  size_t j = 0;
  
    for (size_t i : inlierInd) {
      cv::KeyPoint temp_kp1(float(KfPtr1->keypoints_undistorted_add_[ImgMatches[i].idxA].x()),
                            float(KfPtr1->keypoints_undistorted_add_[ImgMatches[i].idxA].y()), 1);
      cv::KeyPoint temp_kp2(float(KfPtr2->keypoints_undistorted_add_[ImgMatches[i].idxB].x()),
                            float(KfPtr2->keypoints_undistorted_add_[ImgMatches[i].idxB].y()), 1);
      cv::DMatch temp_match(j, j, 0);

      kp_vect1_in_.push_back(temp_kp1);
      kp_vect2_in_.push_back(temp_kp2);
      matches_in_.push_back(temp_match);
      ++j;
    }
    
    // cv::Mat img_out;
    
    // cv::drawMatches(KfPtr1->img_, kp_vect1_in_, KfPtr2->img_,
    //                 kp_vect2_in_, matches_in_, img_out);
    
    // std::stringstream ss;
    // ss << "/home/manthan/ws_vins/covins_ws/results/imgs_17PT/"
    //    << KfPtr1->id_.second << KfPtr2->id_.second << "_" << KfPtr1->id_.first
    //    << "_" << KfPtr2->id_.first << "_" << matches_in_.size() << "_" << s
    //    << ".jpg";
    
    // cv::imwrite(ss.str(), img_out);

     }

// Solve the 2d-2d problem
bool RelNonCentralPosSolver::computePose(const KeyframePtr KfPtr1,
                               const KeyframePtr KfPtr2,
                               Matches &ImgMatches, const double threshold,
                               Eigen::Matrix4d &Tc1c2, std::vector<int> &inlierInd, Vector3Vector &lms) {

// Find the Macthing keypoint indices (For Debugging)
const size_t num_matches = ImgMatches.size();
time_utils::Timer timer;

std::ofstream file_5PT("/home/manthan/ws/covins_ws/src/covins/covins_backend/"
                       "output/results_5PT.csv",
                       std::ios::app);

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


  
  // opengv::transformation_t optimized_pose;
  // sacProb.sac_model_->optimizeModelCoefficients(sacProb.inliers_, sacProb.model_coefficients_, optimized_pose);

  // std::cout << "sacProb.inliers_.size(): " << sacProb.inliers_.size()
  //           << std::endl;
  // std::cout << "Ransac needed " << sacProb.iterations_ << " iterations and ";
  // std::cout << std::endl;
  // std::cout << "the number of inliers is: " << sacProb.inliers_.size();
  // std::cout << std::endl << std::endl;

  //Inlier keypoints
  inlierInd = sacProb.inliers_;

  for (auto ind : inlierInd)
  {
    opengv::point_t point = opengv::triangulation::triangulate(adapter, ind);
    // std::cout << point << std::endl;
    lms.push_back(point);
  }

  kp_vect1_in_.clear();
  kp_vect2_in_.clear();
  matches_in_.clear();

  // if (sacProb.inliers_.size() > mMinInliers) {
  // this->plotMatches(KfPtr1, KfPtr2, ImgMatches, inlierInd);
  // }
  std::cout << "Inliers: "<< sacProb.inliers_.size() << " Iters: "<< sacProb.iterations_ << std::endl;

  // fprintf(stderr, "INFO: 5 PT Ransac took: %lu us\n", timer.measure());
  file_5PT << timer.measure() << std::endl;

  if (sacProb.inliers_.size() < mMinInliers || sacProb.iterations_ >=
  sacProb.max_iterations_) {
    return false;
  }

  // if (sacProb.inliers_.size() < mMinInliers) {
  //   return false;
  // }  
  
    Tc1c2 = TransformType::Identity();
    Tc1c2.block<3, 4>(0, 0) = sacProb.model_coefficients_;

    // Eigen::Quaterniond Rotquat(Tc1c2.block<3, 3>(0, 0));

    // std::ofstream myfile(
    //     "/home/manthan/ws_vins/covins_ws/results/results_tf.csv",
    //     std::ios::app);
    
    // Write to File
    // Query KF Timestamp (x1e9), Query KF ID,  QKF Agent ID,
    // Candidate KF Timestamp (x1e9),Candidate KF ID, CKF Agent ID, 
    // x, y, z, qx, qy, qz, qw, Number of Inliers

    // myfile << std::setprecision(19) << (KfPtr1->timestamp_) * 1e9 << "," << KfPtr1->id_.first << ","
    //        << KfPtr1->id_.second << "," << std::setprecision(19) << (KfPtr2->timestamp_) * 1e9 << ","
    //        << KfPtr2->id_.first << "," << KfPtr2->id_.second << ","
    //        << Tc1c2(0, 3) << "," << Tc1c2(1, 3) << "," << Tc1c2(2, 3) << ","
    //        << Rotquat.x() << "," << Rotquat.y() << "," << Rotquat.z() << ","
    //        << Rotquat.w() << "," << sacProb.inliers_.size() << std::endl;
    return true;
}


} //ns ends