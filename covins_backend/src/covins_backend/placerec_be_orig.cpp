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

// Thirdparty
#include "matcher/MatchingAlgorithm.h"
#include "matcher/ImageMatchingAlgorithm.h"
#include "matcher/LandmarkMatchingAlgorithm.h"

namespace covins {

PlaceRecognition::PlaceRecognition(ManagerPtr man, bool perform_pgo)
    : mapmanager_(man),
      perform_pgo_(perform_pgo),
      voc_(mapmanager_->GetVoc()),
      mnCovisibilityConsistencyTh(covins_params::placerec::cov_consistency_thres)
{
    //...
}

auto PlaceRecognition::CheckBuffer()->bool {
    std::unique_lock<std::mutex> lock(mtx_in_);
    return (!buffer_kfs_in_.empty());
}

auto PlaceRecognition::ComputeSE3()->bool {
    const size_t nInitialCandidates = mvpEnoughConsistentCandidates.size();

    // We compute first ORB matches for each candidate
    // If enough matches are found, we setup a Sim3Solver
    FeatureMatcher matcher(0.75,true);

    vecVecMP vvpMapPointMatches;
    vvpMapPointMatches.resize(nInitialCandidates);
    vector<bool> vbDiscarded;
    vbDiscarded.resize(nInitialCandidates);
    int nCandidates=0; //candidates with enough matches

    for (size_t i = 0; i < nInitialCandidates; i++) {
        KeyframePtr pKF = mvpEnoughConsistentCandidates[i];

        pKF->SetNotErase();

        if (pKF->IsInvalid()) {
          vbDiscarded[i] = true;
          continue;
        }

        // Setup the threaded BF matcher
        std::shared_ptr<LandmarkMatchingAlgorithm> matchingAlgorithm
                (new LandmarkMatchingAlgorithm(50.0)); // default: 50.0
        std::unique_ptr<estd2::DenseMatcher> matcherThreaded (new estd2::DenseMatcher(8));
        matchingAlgorithm->setFrames(kf_query_, pKF);
        matcherThreaded->match<LandmarkMatchingAlgorithm>(*matchingAlgorithm);
        Matches matchesThreaded = matchingAlgorithm->getMatches();
        int nmatches = matchesThreaded.size();

        if(kf_query_->id_.second == pKF->id_.second && nmatches < covins_params::placerec::matches_thres) {
          vbDiscarded[i] = true;
          continue;
        } else if(nmatches < covins_params::placerec::matches_thres_merge) {
            vbDiscarded[i] = true;
            continue;
        }

        // Extract the matches and format it to mapPointMatches
        vvpMapPointMatches[i] = LandmarkVector(kf_query_->keypoints_distorted_.size(), static_cast<LandmarkPtr>(NULL));
        for (Matches::iterator itr = matchesThreaded.begin(); itr !=matchesThreaded.end(); ++itr) {
            const size_t idxA = (*itr).idxA;
            const size_t idxB = (*itr).idxB;
            if (kf_query_->GetLandmark(idxA) && pKF->GetLandmark(idxB)) {
                if (!kf_query_->GetLandmark(idxA)->IsInvalid() && !pKF->GetLandmark(idxB)->IsInvalid()) {
                    vvpMapPointMatches[i][idxA] = pKF->GetLandmark(idxB);
                }
            }
        }
        nCandidates++;
    }

    bool bMatch = false;
    for (size_t i = 0; i < nInitialCandidates; ++i) {
        if (vbDiscarded[i]) {
            continue;
        }

        KeyframePtr pKFi = mvpEnoughConsistentCandidates[i];

        // Setup the RANSAC problem
        Eigen::Matrix4d Twc1;
        Se3Solver se3solver(covins_params::placerec::ransac::min_inliers,
                            covins_params::placerec::ransac::probability,
                            covins_params::placerec::ransac::max_iterations);
        int numMatches = 0;
        for (LandmarkVector::iterator itr = vvpMapPointMatches[i].begin(); itr != vvpMapPointMatches[i].end(); ++itr) {
            if ((*itr)) {
                ++numMatches;
            }
        }

        bool foundTransform = se3solver.projectiveAlignment(kf_query_, vvpMapPointMatches[i], covins_params::placerec::ransac::class_threshold, Twc1); //def: 25

        if (!foundTransform) {
          vbDiscarded[i] = true;
          continue;
        }

        // We have a potential Match --> search additional correspondences
        const Eigen::Matrix4d Twc2 = pKFi->GetPoseTwc();
        Eigen::Matrix4d T12 = Twc1.inverse()*Twc2;

        matcher.SearchBySE3(kf_query_, pKFi, vvpMapPointMatches[i], T12, covins_params::matcher::search_radius_SE3);

        size_t numInitialMatches = 0;
        for (LandmarkVector::iterator itr = vvpMapPointMatches[i].begin(); itr != vvpMapPointMatches[i].end(); ++itr) {
            if ((*itr)) {
                ++numInitialMatches;
            }
        }

        const int numInliersOpt = Optimization::OptimizeRelativePose(kf_query_, pKFi, vvpMapPointMatches[i], T12, 4.0f);

        if (numInliersOpt < covins_params::placerec::inliers_thres) {
              vbDiscarded[i] = true;
              continue;
        } else {
            bMatch = true;
            Eigen::Matrix4d Twc1corr = Twc2 * T12.inverse();
            mTsw = (Twc1corr*kf_query_->GetStateExtrinsics().inverse()).inverse();
            mTcw = Twc1corr.inverse();
            kf_match_ = pKFi;
            mvpCurrentMatchedPoints = vvpMapPointMatches[i];
            break;
        }
    }

    if (!bMatch) {
        for(size_t i=0; i<nInitialCandidates; i++) {
            mvpEnoughConsistentCandidates[i]->SetErase();
        }
        kf_query_->SetErase();
        return false;
    }

    // Retrieve MapPoints seen in Loop Keyframe and neighbors
    KeyframeVector vpLoopConnectedKFs = kf_match_->GetConnectedKeyframesByWeight(0);
    vpLoopConnectedKFs.push_back(kf_match_);
    mvpLoopMapPoints.clear();
    for (KeyframeVector::iterator vit = vpLoopConnectedKFs.begin(); vit != vpLoopConnectedKFs.end(); ++vit) {
        KeyframePtr pKF = *vit;
        LandmarkVector vpMapPoints = pKF->GetLandmarks();
        for (size_t i = 0; i < vpMapPoints.size(); ++i) {
        LandmarkPtr pMP = vpMapPoints[i];
            if (pMP) {
                if(!pMP->IsInvalid() && pMP->loop_point_for_kf_!=kf_query_->id_) {
                    mvpLoopMapPoints.push_back(pMP);
                    pMP->loop_point_for_kf_ = kf_query_->id_;
                }
            }
        }
    }

    // Find more matches projecting with the computed Sim3
    matcher.SearchByProjection(kf_query_, kf_query_->GetStateExtrinsics().inverse()*mTsw,
                                   mvpLoopMapPoints, mvpCurrentMatchedPoints,covins_params::matcher::search_radius_proj);

    // If enough matches accept Loop
    int nTotalMatches = 0;
    for (size_t i = 0; i < mvpCurrentMatchedPoints.size(); ++i) {
        if(mvpCurrentMatchedPoints[i]) {
            nTotalMatches++;
        }
    }

    if (nTotalMatches >= covins_params::placerec::total_matches_thres) {
        for (size_t i = 0; i < nInitialCandidates; i++) {
            if (mvpEnoughConsistentCandidates[i]!=kf_match_) {
                mvpEnoughConsistentCandidates[i]->SetErase();
            }
        }
        return true;
    } else {
        for (size_t i = 0; i < nInitialCandidates; i++) {
            mvpEnoughConsistentCandidates[i]->SetErase();
        }
        kf_query_->SetErase();

        return false;
    }
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

    if(1) {
        //check whether matches are ok
        std::set<idpair> lms_query, lms_match;
        for (size_t i = 0; i < mvpCurrentMatchedPoints.size(); ++i) {
            auto lm_m = mvpCurrentMatchedPoints[i];
            if(!lm_m) continue;
            if(!lms_match.insert(lm_m->id_).second) {
            }
            if (lm_m->GetFeatureIndex(kf_query) != -1) {
                continue;
            }
            if(kf_query->GetLandmarkIndex(lm_m) != -1) {
                continue;
            }
            auto lm_q = kf_query->GetLandmark(i);
            if(!lm_q) {
                continue;
            }
            if(!lms_query.insert(lm_q->id_).second) {
            }
            if(lm_q->GetFeatureIndex(kf_match) != -1) {
                continue;
            }
            if(kf_match->GetLandmarkIndex(lm_q) != -1) {
                continue;
            }
        }
    }

    // Update matched map points and replace if duplicated
    for (size_t i = 0; i < mvpCurrentMatchedPoints.size(); ++i) {
        if (mvpCurrentMatchedPoints[i]) {
            if (mvpCurrentMatchedPoints[i]->GetFeatureIndex(kf_query_) != -1) {
                continue; //don't add MPs to mpCurrentKF that already exists there
            }

            LandmarkPtr pLoopMP = mvpCurrentMatchedPoints[i];
            LandmarkPtr pCurMP = kf_query->GetLandmark(i);
            if(pCurMP) {
                FuseLandmark(pLoopMP,pCurMP,map);
            } else {
                kf_query->AddLandmark(pLoopMP,i);
                pLoopMP->AddObservation(kf_query,i);
                pLoopMP->ComputeDescriptor();
            }
        }
    }

    corrected_poses[kf_query->id_] = T_w_sqcorr;
}

auto PlaceRecognition::CorrectLoop()->bool {
    cout << "\033[1;32m+++ PLACE RECOGNITION FOUND +++\033[0m" << endl;

    last_loops_[kf_query_->id_.second] = kf_query_->id_.first;

    TransformType T_smatch_squery = kf_match_->GetPoseTsw() * mTsw.inverse();

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

    if(map_query->GetKeyframe(kf_match_->id_)){
        LoopConstraint lc(kf_match_,kf_query_,T_smatch_squery);
        map_query->AddLoopConstraint(lc);
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
    const auto vpConnectedKeyFrames = kf_query_->GetConnectedKeyframesByWeight(0);
    const DBoW2::BowVector &CurrentBowVec = kf_query_->bow_vec_;
    float minScore = 1;
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
    KeyframeVector vpCandidateKFs = database->DetectCandidates(kf_query_, minScore*0.8);

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
        auto candidate_connections = pCandidateKF->GetConnectedKeyframesByWeight(0);
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
