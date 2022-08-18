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
/**
* The methods of the PlaceRecognition class partially re-use code of ORB-SLAM2
*/

#include "covins_backend/placerec_be.hpp"
// C++
#include <iostream>
#include <mutex>
#include <eigen3/Eigen/Core>

// COVINS
#include "covins_backend/keyframe_be.hpp"
#include "covins_backend/kf_database.hpp"
#include "covins_backend/landmark_be.hpp"
#include "covins_backend/map_be.hpp"
#include "covins_backend/optimization_be.hpp"
#include "covins_backend/Se3Solver.h"
#include "covins_backend/RelPosSolver.hpp"
#include "covins_backend/RelNonCentralPosSolver.hpp"

// Thirdparty
#include "matcher/MatchingAlgorithm.h"
#include "matcher/ImageMatchingAlgorithm.h"
#include "matcher/LandmarkMatchingAlgorithm.h"

#include <covins/covins_base/utils_base.hpp>
#include <covins/covins_base/timer_utils.hpp>

#include <random>
namespace covins {

PlaceRecognition::PlaceRecognition(ManagerPtr man, bool perform_pgo)
    : mapmanager_(man),
      perform_pgo_(perform_pgo),
      voc_(mapmanager_->GetVoc()),
      mnCovisibilityConsistencyTh(covins_params::placerec::cov_consistency_thres)
{
  //...
  // Load GT Data
  for(int i=0;i<5;++i) {
        std::string datasetDir;
        if(i % 5 == 0) datasetDir = covins_params::sys::data_path0;
        else if(i % 5 == 1) datasetDir = covins_params::sys::data_path1;
        else if(i % 5 == 2) datasetDir = covins_params::sys::data_path2;
        else if(i % 5 == 3) datasetDir = covins_params::sys::data_path3;
        else if(i % 5 == 4) datasetDir = covins_params::sys::data_path4;
        else {
            std::cout << COUTERROR << std::endl;
            exit(-1);
        }
        std::cout << __func__ << "datasetDir: " << datasetDir << std::endl;

        std::ifstream groundtruth_file(datasetDir + "/groundtruth.csv");

        GroundtruthData groundtruth_data;

        std::string line;
        std::getline(groundtruth_file, line);  // skip first line
        while (std::getline(groundtruth_file, line)) {
            std::stringstream stream(line);
            std::string s;
            std::vector<double> groundtruth_msg;
            while (std::getline(stream, s, ',')) {
                groundtruth_msg.push_back(std::stod(s));
            }
            groundtruth_data.push_back(groundtruth_msg);
        }

        if(groundtruth_data.empty()){
            std::cout << "No GT data at " << datasetDir + "/groundtruth.csv" << std::endl;
        } else
            std::cout << "Groundtruth points: " << groundtruth_data.size() << std::endl;

        std::sort(groundtruth_data.begin(), groundtruth_data.end(),
                [](const std::vector<double>& a, const std::vector<double>& b) {
                    return a[0] < b[0];
        });

        gt_[i] = groundtruth_data;
    }
}

auto PlaceRecognition::CheckBuffer()->bool {
    std::unique_lock<std::mutex> lock(mtx_in_);
    return (!buffer_kfs_in_.empty());
}

auto PlaceRecognition::ComputeSE3() -> bool {

    const size_t nInitialCandidates = mvpEnoughConsistentCandidates.size();

    std::cout << "----> Query " << kf_query_ << " : nInitialCandidates: " << nInitialCandidates << std::endl;
    vector<bool> vbDiscarded;
    vbDiscarded.resize(nInitialCandidates);
    int nCandidates = 0; // candidates with enough matches
    Eigen::Matrix4d Tc1c2; // Relative TF between the frames
    Eigen::Matrix<double, 6, 6> cov_loop; // Covariance Matrix for the Loop
    bool foundRelTransform;
    time_utils::Timer timer;

    for (size_t i = 0; i < nInitialCandidates; i++) {
        KeyframePtr pKF = mvpEnoughConsistentCandidates[i];

        pKF->SetNotErase();

        if (pKF->IsInvalid()) {
          vbDiscarded[i] = true;
          continue;
        }

        timer.measure();

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
        Matches img_matches;


        matcher->knnMatch(kf_query_->descriptors_add_, pKF->descriptors_add_,
                   matches_vect, 2);

        for (size_t i = 0; i < matches_vect.size(); ++i) {
            cv::DMatch curr_m = matches_vect[i][0];
            cv::DMatch curr_n = matches_vect[i][1];

            // Distance threshold + Ratio Test
            if (curr_m.distance <= covins_params::features::img_match_thres) {
              if (curr_m.distance < 0.8 * curr_n.distance) {
                matches.push_back(curr_m);
                Match temp_match(curr_m.queryIdx, curr_m.trainIdx, curr_m.distance);
                img_matches.push_back(temp_match);
            }
            }
        }

        fprintf(stderr, "INFO: Image Matching took: %lu us\n", timer.measure());

        // if (img_matches.size() > 20 ){                 
        
        // cv::Mat img_out;

        // std::vector<cv::KeyPoint> kp_vect1_in;
        // std::vector<cv::KeyPoint> kp_vect2_in;

        // for (auto i : kf_query_->keypoints_undistorted_add_) {
        // cv::KeyPoint temp_kp1(float(i.x()), float(i.y()), 1.0);
        // kp_vect1_in.push_back(temp_kp1);     
        // }

        // for (auto i : pKF->keypoints_undistorted_add_) {
        // cv::KeyPoint temp_kp1(float(i.x()), float(i.y()), 1.0);
        // kp_vect2_in.push_back(temp_kp1);
        // }

        // cv::drawMatches(kf_query_->img_, kp_vect1_in, pKF->img_, kp_vect2_in,
        //                 matches, img_out);
        
        // std::stringstream ss;
        // ss << "/home/manthan/ws_vins/covins_ws/results/imgs_time/"
        // << kf_query_->id_.second << pKF->id_.second << "_" << kf_query_->id_.first
        // << "_" << pKF->id_.first << "_" << matches.size() << ".jpg";
        // cv::imwrite(ss.str(), img_out);
        // }

        // Matches img_matches = matchingAlgorithmImage->getMatches();
        int nmatches = img_matches.size();

        // fprintf(stderr, "INFO: Image Matching took: %lu us\n",
        //         timer.measure());

        std::cout << "------> num_img matches: " << nmatches << std::endl;

        if(kf_query_->id_.second == pKF->id_.second && nmatches < covins_params::placerec::matches_thres) {
          vbDiscarded[i] = true;
          continue;
        } else if(nmatches < covins_params::placerec::matches_thres_merge) {
            vbDiscarded[i] = true;
            continue;
        }
        nCandidates++;
    }

    // fprintf(stderr, "INFO: Resetting timer: %lu us\n", timer.measure());
    
    bool bMatch = false;
    for (size_t i = 0; i < nInitialCandidates; ++i) {
        if (vbDiscarded[i]) {
            continue;
        }

        KeyframePtr pKFi = mvpEnoughConsistentCandidates[i];

        // Setup the Rel Pose Estimation Problem
        Tc1c2 = Eigen::Matrix4d::Identity();

        mloops.clear();
        LoopVector loop_vect;

        RelNonCentralPosSolver rel_pose_solver;
        foundRelTransform = rel_pose_solver.computeNonCentralRelPose(
            kf_query_, pKFi, covins_params::placerec::rel_pose::error_thres,
            Tc1c2, cov_loop, loop_vect);

        mloops = loop_vect;

        // fprintf(stderr, "INFO: 17 PT Alogirthm took: %lu us\n", timer.measure());
 
        if (!foundRelTransform) {
          vbDiscarded[i] = true;
          continue;
        }

        // GT Checking
        if (covins_params::placerec::use_gt) {
            // std::cout << "GT Checking" << std::endl;
            // std::cout << kf_query_->id_.first << " | " ;
            // std::cout << kf_query_->id_.second << std::endl;

            // std::cout << "Query KF" << kf_query_->id_;
            TransformType T_w_squery_gt = this->GetPoseTwsGT(kf_query_);

            // std::cout << "Candidate KF" ;
            // std::cout << pKFi->id_.first << " | " ;
            // std::cout << pKFi->id_.second << std::endl;
            // TransformType T_w_cquery_gt = T_w_squery_gt *
            // kf_query_->GetStateExtrinsics();

            TransformType T_w_smatch_gt = this->GetPoseTwsGT(pKFi);
            TransformType T_smatch_squery_gt = T_w_smatch_gt.inverse() * T_w_squery_gt;
            // TransformType T_w_cmatch_gt = T_smatch_squery_gt * pKF->GetStateExtrinsics();
            // TransformType T_12_gt = T_w_cquery_gt.inverse() * T_w_cmatch_gt;
            
            TransformType Twc2 = pKFi->GetPoseTwc();
            TransformType Tsw2 = pKFi->GetPoseTsw();
            TransformType Twc1corr = Twc2 * Tc1c2.inverse();
            TransformType Tws1 = (Twc1corr * kf_query_->GetStateExtrinsics().inverse());
            TransformType T_smatch_squery = Tsw2 * Tws1;
            
            std::cout << "++++++++++" << std::endl;
            std::cout << "T_smatch_squery_ : \n" << T_smatch_squery << std::endl;
            std::cout << "----------" << std::endl;
            std::cout << "T_smatch_squery_gt : \n" << T_smatch_squery_gt << std::endl;
            // std::cout << "----------" << std::endl;
            // std::cout << "Error Transformation : \n" << T_smatch_squery_ << std::endl;
            TransformType T_err = T_smatch_squery * T_smatch_squery_gt.inverse();
            Eigen::AngleAxisd aa_err(T_err.block<3,3>(0,0));
            double angle = abs(aa_err.angle());
            double angle_err_deg = angle * 180.0 / M_PI;
            double trans_err = T_err.block<3,1>(0,3).norm();
            // std::cout << "Angle error [deg]: " << angle_err_deg << std::endl;
            // std::cout << "Trans error [m]  : " << trans_err << std::endl;
            mT_smatch_squery_gt = T_smatch_squery_gt;

            // Add Distrubance according to Translational Covariance
            // std::random_device rd{};
            // std::mt19937 gen{rd()};
            // std::normal_distribution<> d{0, 1};
            
            // float randomNumber = d(gen);

            // std::cout << "Random Number"<< randomNumber << std::endl;

            // mT_smatch_squery_gt(0, 3) += randomNumber * sqrt(cov_loop(3, 3));
            // mT_smatch_squery_gt(1, 3) += randomNumber * sqrt(cov_loop(4, 4));
            // mT_smatch_squery_gt(2, 3) += randomNumber * sqrt(cov_loop(5, 5));

            // mT_smatch_squery_gt.block<3,3>(0,0) = T_smatch_squery.block<3,3>(0,0);
            
            // std::cout << "----------" << std::endl;
            // std::cout << "T_smatch_squery_Disturbed : \n" << mT_smatch_squery_gt << std::endl;
            
            // Calculate GT Yaw
            auto yaw_match =  Utils::R2ypr(T_w_smatch_gt.block<3,3>(0,0)).x();
            TransformType corrected_Tws_query = T_w_smatch_gt * T_smatch_squery_gt;
            auto yaw_query = Utils::R2ypr(corrected_Tws_query.block<3,3>(0,0)).x();
            mrelative_yaw_gt = Utils::normalizeAngle(yaw_query - yaw_match);
        }

        bMatch = true;
        Eigen::Matrix4d T12 = Tc1c2;
        mcov_mat = cov_loop;
        const Eigen::Matrix4d Twc2 = pKFi->GetPoseTwc();
        Eigen::Matrix4d Twc1corr = Twc2 * T12.inverse();
        mTsw = (Twc1corr*kf_query_->GetStateExtrinsics().inverse()).inverse();
        mTcw = Twc1corr.inverse();
        kf_match_ = pKFi;

        TransformType T_smatch_squery = kf_match_->GetPoseTsw() * mTsw.inverse();
        TransformType T_w_s_cand = kf_match_->GetPoseTws();
        auto yaw_match =  Utils::R2ypr(T_w_s_cand.block<3,3>(0,0)).x();
        TransformType corrected_Tws_query = T_w_s_cand * T_smatch_squery;
        auto yaw_query = Utils::R2ypr(corrected_Tws_query.block<3, 3>(0, 0)).x();
        mrelative_yaw = Utils::normalizeAngle(yaw_query - yaw_match);

        std::cout << "Norm: " << T_smatch_squery.block<3,1>(0,3).norm() << "Yaw" << mrelative_yaw << std::endl;
        if (abs(mrelative_yaw) > 50.0 || T_smatch_squery.block<3,1>(0,3).norm() > 2.0)
          bMatch = false;

        if (bMatch) {
            break;
        } else {
            vbDiscarded[i] = true;
            continue;
        }
    }

    if (!bMatch) {
        for(size_t i=0; i<nInitialCandidates; i++) {
            mvpEnoughConsistentCandidates[i]->SetErase();
        }
        kf_query_->SetErase();
        return false;
    }

    return true;

}

auto PlaceRecognition::ConnectLoop(KeyframePtr kf_query, KeyframePtr kf_match, TransformType T_smatch_squery, PoseMap &corrected_poses, MapPtr map)->void {

    TransformType T_w_squery = kf_query->GetPoseTws();
    TransformType T_w_smatch = kf_match->GetPoseTws();
    TransformType T_w_sqcorr = T_w_smatch*T_smatch_squery;

    for(auto kfi : mvpCurrentConnectedKFs) {
        if(kfi == kf_query) continue;
        TransformType T_wq_si = kfi->GetPoseTws();
        TransformType T_sq_si = T_w_squery.inverse() * T_wq_si;
        TransformType T_w_sicorr = T_w_sqcorr * T_sq_si;
        corrected_poses[kfi->id_] = T_w_sicorr;
    }

    corrected_poses[kf_query->id_] = T_w_sqcorr;
}

auto PlaceRecognition::GetPoseTwsGT(KeyframePtr kf)->TransformType {

    double timestamp = kf->timestamp_;
    int idx_gt = 0;
    // std::cout << timestamp << std::endl;
    GroundtruthData groundtruth_data = gt_[kf->id_.second%5];
    // std::cout << "GT data recovered" << std::endl;

    // std::cout << groundtruth_data.back()[0] << std::endl;

    if (groundtruth_data.back()[0] * 1e-9 < timestamp) {
      idx_gt = groundtruth_data.size() - 1;
    } else {
        // get groundtruth with closeset timestamp
    while (groundtruth_data[idx_gt][0] * 1e-9 < timestamp)
        ++idx_gt;
    if (timestamp - groundtruth_data[idx_gt - 1][0] * 1e-9 < groundtruth_data[idx_gt][0] * 1e-9 - timestamp)
        idx_gt -= 1;
    }
    
    // std::cout << "IDX_GT" << idx_gt << std::endl;
    Eigen::Vector4d tws_gt(groundtruth_data[idx_gt][1], groundtruth_data[idx_gt][2], groundtruth_data[idx_gt][3], 1);
    Eigen::Quaterniond q_gt(groundtruth_data[idx_gt][4], groundtruth_data[idx_gt][5], groundtruth_data[idx_gt][6], groundtruth_data[idx_gt][7]);
    Eigen::Matrix3d Rws_gt = q_gt.normalized().toRotationMatrix();
    Eigen::Matrix4d Tws_gt = Eigen::Matrix4d::Identity();;
    Tws_gt.block<3, 3>(0, 0) = Rws_gt;
    Tws_gt.block<4, 1>(0, 3) = tws_gt;

    return Tws_gt;
}

auto PlaceRecognition::CorrectLoop()->bool {
    cout << "\033[1;32m+++ PLACE RECOGNITION FOUND +++\033[0m" << endl;

    last_loops_[kf_query_->id_.second] = kf_query_->id_.first;
    TransformType T_smatch_squery = TransformType::Identity();
    
    if (covins_params::placerec::use_gt) {
        T_smatch_squery = mT_smatch_squery_gt;
        mrelative_yaw = mrelative_yaw_gt;
    } else {
        T_smatch_squery = kf_match_->GetPoseTsw() * mTsw.inverse();
    }
    
    int check_num_map;
    MapPtr map_query = mapmanager_->CheckoutMapExclusiveOrWait(kf_query_->id_.second,check_num_map);

    for(auto lc : map_query->GetLoopConstraints()) {
        bool existing_match = false;
        if(lc.kf1 == kf_match_ && lc.kf2 == kf_query_) existing_match = true;
        if(lc.kf2 == kf_match_ && lc.kf1 == kf_query_) existing_match = true;
        if(!existing_match) continue;
        std::cout << "!!! Loop Constraint already exisiting -- skip !!!" << std::endl;
        kf_query_->SetErase();
        kf_match_->SetErase();
        mapmanager_->ReturnMap(kf_query_->id_.second,check_num_map);
        return false;
    }

    // Ensure current keyframe is updated
    map_query->UpdateCovisibilityConnections(kf_query_->id_);

    mvpCurrentConnectedKFs.clear();
    mvpCurrentConnectedKFs.push_back(kf_query_);

    PoseMap corrected_poses;
    this->ConnectLoop(kf_query_,kf_match_,T_smatch_squery,corrected_poses,map_query);

    if (map_query->GetKeyframe(kf_match_->id_)) {
      if (covins_params::placerec::use_LBA) {
        for (auto lc : mloops)
            map_query->AddLoopConstraint(lc);
      } else {
        LoopConstraint lc(kf_match_, kf_query_, T_smatch_squery, mrelative_yaw,
                          mcov_mat, mcov_mat);
        map_query->AddLoopConstraint(lc);
    }
        
        
        if(perform_pgo_)
        {
            KeyframeVector current_connections_query = kf_query_->GetConnectedKeyframesByWeight(0);
            
            for(auto kfi : current_connections_query) {
                map_query->UpdateCovisibilityConnections(kfi->id_);
            }

            Optimization::PoseGraphOptimization(map_query, corrected_poses);
            map_query->WriteKFsToFile("_aft_PGO");
            map_query->WriteKFsToFileAllAg();
        } else {
            std::cout << COUTNOTICE << "!!! PGO deativated !!!" << std::endl;
        }
    } else {
        std::cout << "\033[1;32m+++ FUSION FOUND +++\033[0m" << std::endl;
        MergeInformation merge;
        merge.kf_query = kf_query_;
        merge.kf_match = kf_match_;
        merge.T_smatch_squery = T_smatch_squery;
        merge.relative_yaw_smatch_squery = mrelative_yaw;
        merge.cov_mat = mcov_mat;
        mapmanager_->RegisterMerge(merge);
    }

    mapmanager_->ReturnMap(kf_query_->id_.second,check_num_map);

    return true;
}

auto PlaceRecognition::DetectLoop()->bool {
    {
        std::unique_lock<std::mutex> lock(mtx_in_);
        kf_query_ = buffer_kfs_in_.front();
        buffer_kfs_in_.pop_front();
        kf_query_->SetNotErase();
    }

    if(kf_query_->id_.first % 20 == 0) {
    }

    if(kf_query_->id_.first < covins_params::placerec::start_after_kf) {
        kf_query_->SetErase();
        return false;
    }

    if(last_loops_.count(kf_query_->id_.second)) {
        if((kf_query_->id_.first - last_loops_[kf_query_->id_.second]) < covins_params::placerec::consecutive_loop_dist) {
            kf_query_->SetErase();
            return false;
        }
    }

    // Compute reference BoW similarity score
    // This is the lowest score to a connected keyframe in the covisibility graph
    // We will impose loop candidates to have a higher similarity than this
    const auto vpConnectedKeyFrames =
        kf_query_->GetConnectedNeighborKeyframes();
        
    const DBoW2::BowVector &CurrentBowVec = kf_query_->bow_vec_;
    float minScore = 1;
    // std::cout << "Number of neighbors: " << vpConnectedKeyFrames.size() << std::endl;
    for (size_t i = 0; i < vpConnectedKeyFrames.size(); i++) {
            KeyframePtr pKF = vpConnectedKeyFrames[i];
            if(pKF->IsInvalid()) continue;

            const DBoW2::BowVector &BowVec = pKF->bow_vec_;
            float score = voc_->score(CurrentBowVec, BowVec);
        if(score<minScore) {
            minScore = score;
        }
    }

    // Query the database imposing the minimum score
    auto database = mapmanager_->GetDatabase();
    KeyframeVector vpCandidateKFs = database->DetectCandidates(kf_query_, minScore*0.7);

    // If there are no loop candidates, just add new keyframe and return false
    if (vpCandidateKFs.empty()) {
        mvConsistentGroups.clear(); //Danger: Why deleting the found consistent groups in this case?
        kf_query_->SetErase();
        return false;
    }

    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a keyframe
    // We must detect a consistent loop in several consecutive keyframes to accept it
    mvpEnoughConsistentCandidates.clear();

    vecConsistentGroup vCurrentConsistentGroups; //pair <set<KF*>,int> --> int counts consistent groups found for this group
    vector<bool> vbConsistentGroup(mvConsistentGroups.size(),false);
    //mvConsistentGroups stores the last found consistent groups.

    for (size_t i = 0; i < vpCandidateKFs.size(); ++i) {
        KeyframePtr pCandidateKF = vpCandidateKFs[i];
        //      setKF spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
        auto candidate_connections = pCandidateKF->GetConnectedNeighborKeyframes();
        KeyframeSet spCandidateGroup(candidate_connections.begin(),candidate_connections.end());
        spCandidateGroup.insert(pCandidateKF);
        //group with candidate and connected KFs

        bool bEnoughConsistent = false;
        bool bConsistentForSomeGroup = false;
        for(size_t iG = 0; iG < mvConsistentGroups.size(); ++iG) {
            auto sPreviousGroup = mvConsistentGroups[iG].first;

            bool bConsistent = false;
            for(KeyframeSet::iterator sit = spCandidateGroup.begin(); sit != spCandidateGroup.end(); ++sit) {
                if (sPreviousGroup.count(*sit)) {
                    //KF found that is contained in candidate's group and comparison group
                    bConsistent=true;
                    bConsistentForSomeGroup=true;
                    break;
                }
            }
            if (bConsistent) {
                int nPreviousConsistency = mvConsistentGroups[iG].second;
                int nCurrentConsistency = nPreviousConsistency + 1;
                if(!vbConsistentGroup[iG]) {
                    ConsistentGroup cg = make_pair(spCandidateGroup,nCurrentConsistency);
                    vCurrentConsistentGroups.push_back(cg);
                    vbConsistentGroup[iG]=true; //this avoid to include the same group more than once
                }
                if(nCurrentConsistency >= mnCovisibilityConsistencyTh && !bEnoughConsistent) {
                    mvpEnoughConsistentCandidates.push_back(pCandidateKF);
                    bEnoughConsistent=true; //this avoid to insert the same candidate more than once
                }
            }
        }

        // If the group is not consistent with any previous group insert with consistency counter set to zero
        if (!bConsistentForSomeGroup) {
            ConsistentGroup cg = make_pair(spCandidateGroup,0); //For "ConsistentGroup" the "int" is initialized with 0
            vCurrentConsistentGroups.push_back(cg);
        }
    }

    // Update Covisibility Consistent Groups
    mvConsistentGroups = vCurrentConsistentGroups;

    if (mvpEnoughConsistentCandidates.empty()) {
        kf_query_->SetErase();
        return false;
    } else {
        return true;
    }
    kf_query_->SetErase();
    return false;
}

auto PlaceRecognition::FuseLandmark(LandmarkPtr lm_target, LandmarkPtr lm_tofuse, MapPtr map)->void {
    // should only call this when sure that LMs cannot be modified by other thread currently
    if(!lm_target) {
        return;
    }
    if(!lm_tofuse) {
        return;
    }
    if(lm_target->id_ == lm_tofuse->id_) {
        return;
    }
    auto observations_tofuse = lm_tofuse->GetObservations();
    size_t non_moved_obs = 0;
    for(auto i : observations_tofuse) {
        KeyframePtr kf = i.first;
        int feat_id_tofuse = i.second;
        int feat_id_target = lm_target->GetFeatureIndex(kf);
        if(feat_id_target < 0) {
            kf->EraseLandmark(feat_id_tofuse);
            kf->AddLandmark(lm_target,feat_id_tofuse);
            lm_target->AddObservation(kf,feat_id_tofuse);
            lm_tofuse->EraseObservation(kf); //we need this, otherwise the landmark will erase some connections from the KF once it is removed from the map
            lm_tofuse->tofuse_lm_ = true;
        } else if(feat_id_target == feat_id_tofuse) {
            continue;
        } else {
            non_moved_obs++;
            continue;
        }
    }
    lm_target->ComputeDescriptor();
    if(non_moved_obs < 2) {
        map->EraseLandmark(lm_tofuse); // TODO: in case of map fusion, "lm_tofuse" might not be in "map" and therefore the erasing operation might fail. At the moment, this is later ironed out by Map::Clean()
    }
    else
        lm_tofuse->ComputeDescriptor();
}

auto PlaceRecognition::InsertKeyframe(KeyframePtr kf)->void {
    std::unique_lock<std::mutex> lock(mtx_in_);
    buffer_kfs_in_.push_back(kf);
}

auto PlaceRecognition::Run()->void {

    int num_runs = 0;
    int num_detected = 0;

    while(1){
        if (CheckBuffer()) {
            num_runs++;
            bool detected = DetectLoop();
            if(detected) {
                num_detected++;
                bool found_se3 = ComputeSE3();
                if(found_se3) {
                    this->CorrectLoop();
                }
            }
            mapmanager_->AddToDatabase(kf_query_);
        }

        if(this->ShallFinish()){
            std::cout << "PlaceRec " << ": close" << std::endl;
            break;
        }

        usleep(1000);
    }

    std::unique_lock<std::mutex> lock(mtx_finish_);
    is_finished_ = true;
}


} //end ns
