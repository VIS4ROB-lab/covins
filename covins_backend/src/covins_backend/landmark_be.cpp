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

#include "covins_backend/landmark_be.hpp"

// C++
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>

// COVINS
#include "covins_backend/keyframe_be.hpp"
#include "covins_backend/map_be.hpp"

namespace covins {

Landmark::Landmark(MsgLandmark msg, MapPtr map)
    : LandmarkBase(msg.id)
{
    if(msg.save_to_file) {
        pos_w_ = msg.pos_w;
        msg_ = msg;
    } else {
        this->UpdatePosFromMsg(msg,map);
    }
}

auto Landmark::ComputeDescriptor()->void {
    std::unique_lock<std::mutex> lock(mtx_obs_);

    std::vector<cv::Mat> descriptor_candidates;
    if(observations_.empty()) {
        return;
    }
    descriptor_candidates.reserve(observations_.size());
    for(auto i : observations_) {
        KeyframePtr kf = i.first;
        int feat_idx = i.second;
        if(kf->IsInvalid()){
            continue;
        }
        descriptor_candidates.push_back(kf->descriptors_.row(feat_idx));
    }
    if(descriptor_candidates.empty()) {
        return;
    }
    // Compute mutual distances
    const int num_desc = descriptor_candidates.size();
    precision_t descriptor_distances[num_desc][num_desc];
    for(int i=0;i<num_desc;++i) {
        descriptor_distances[i][i] = 0;
        for(int j=i+1;j<num_desc;++j) {
            const precision_t dist_ij = (precision_t)cv::norm(descriptor_candidates[i],descriptor_candidates[j],cv::NORM_HAMMING);
            descriptor_distances[i][j] = dist_ij;
            descriptor_distances[j][i] = dist_ij;
        }
    }
    // Take the descriptor with least median distance to the rest
    precision_t best_median = INT_MAX;
    int best_idx = -1;
    for(int i=0;i<num_desc;++i) {
        std::vector<precision_t> distances(descriptor_distances[i],descriptor_distances[i]+num_desc);
        std::sort(distances.begin(),distances.end());
        const precision_t median = distances[0.5 * (num_desc-1)];
        if(median < best_median) {
            best_median = median;
            best_idx = i;
        }
    }
    descriptor_ = descriptor_candidates[best_idx];
}

auto Landmark::ConvertToMsg(MsgLandmark &msg, KeyframePtr kf_ref, bool is_update)->void {
    NOT_IMPLEMENTED
    exit(-1);
}

auto Landmark::ConvertToMsgFileExport(MsgLandmark &msg)->void {
    std::unique_lock<std::mutex> loc_obs(mtx_obs_);
    std::unique_lock<std::mutex> lock_pos(mtx_pos_);

    msg.save_to_file = true;

    msg.id = id_;
    if(!reference_kf_) {
        std::cout << COUTFATAL << "LM " << id_ << ": no ref-KF" << std::endl;
        std::cout << "#obs: " << observations_.size() << std::endl;
        std::cout << "valid: " << (int)!invalid_ << std::endl;
        std::cout << "tofuse LM: " << (int)tofuse_lm_ << std::endl;
        exit(-1);
    }
    msg.id_reference = reference_kf_->id_;

    msg.pos_w = pos_w_;

    for(KfObservations::const_iterator mit=observations_.begin();mit!=observations_.end();++mit){
        if(mit->first != nullptr){
            msg.observations.insert(std::make_pair(mit->first->id_,mit->second));
        }
    }
}

auto Landmark::EstablishConnections(MsgLandmark msg, MapPtr map)->void {
    //Observations
    for(std::map<idpair,int>::iterator mit = msg.observations.begin(); mit!=msg.observations.end();++mit){
        size_t feat_id = mit->second;
        idpair kf_id = mit->first;
        // the reference KF should exist, otherwise the Constuctor would have thrown an exception
        bool expect_kf_null = kf_id.first > reference_kf_->id_.first ? true : false;
        KeyframePtr kf = map->GetKeyframe(kf_id,expect_kf_null);

        if(kf){
            if(observations_.count(kf)){
                if(observations_[kf] == feat_id) {
                    //all good, just received the info a seconf time
                }
                continue;
            }

            LandmarkPtr lm_check = kf->GetLandmark(feat_id);
            if(lm_check){
                kf->EraseLandmark(lm_check,feat_id);
                lm_check->EraseObservation(kf);
            }
            observations_[kf]=feat_id;
            kf->AddLandmark(shared_from_this(),feat_id);
        }
    }

    if(!observations_.empty()) {
        reference_kf_ = observations_.begin()->first;
    }
}

auto Landmark::IsOptimized()->bool {
    std::unique_lock<std::mutex> lock_pos(mtx_pos_);
    return optimized_;
}

auto Landmark::SetInvalid()->bool {
    if(invalid_) {
        return false;
    }

    std::unique_lock<std::mutex> lock(mtx_obs_);
    std::unique_lock<std::mutex> lock_inv(mtx_invalid_);
    for(auto mit : observations_) {
        KeyframePtr kf = mit.first;
        int feat_id = mit.second;
        kf->EraseLandmark(shared_from_this(),feat_id);
    }
    observations_.clear();
    reference_kf_ = nullptr;
    invalid_ = true;

    return true;
}

auto Landmark::SetOptimized()->void {
    std::unique_lock<std::mutex> lock_pos(mtx_pos_);
    optimized_ = true;
}

auto Landmark::UpdateNormal()->void {
    std::unique_lock<std::mutex> loc_obs(mtx_obs_);
    std::unique_lock<std::mutex> lock_pos(mtx_pos_);
    if(!reference_kf_){
        std::cout << COUTERROR << "LM " << id_ << ": no ref-KF" << std::endl;
        exit(-1);
    }
    if(observations_.empty()) {
        std::cout << COUTERROR << "no obervations" << std::endl;
        exit(-1);
    }

    Vector3Type normal = Eigen::Vector3d::Zero();
    int n=0;
    for(auto i : observations_) {
        KeyframePtr kf = i.first;
        if(kf->IsInvalid()) {
            continue;
        }
        Eigen::Vector3d Owi = kf->GetPoseTwc().block<3,1>(0,3);
        Eigen::Vector3d normali = pos_w_ - Owi;
        normal = normal + normali/normali.norm();
        n++;
    }
    normal_ = normal/n;

    // Min/Max distance
    Eigen::Vector3d PC = pos_w_ - reference_kf_->GetPoseTwc().block<3,1>(0,3);
    const precision_t dist = PC.norm();

    const int level = (int)reference_kf_->keypoints_aors_[observations_[reference_kf_]](1);
    const precision_t levelScaleFactor =  std::pow(covins_params::features::scale_factor,level);

    max_distance_ = dist*levelScaleFactor;
    min_distance_ = max_distance_/std::pow(covins_params::features::scale_factor,covins_params::features::num_octaves-1);
}

auto Landmark::UpdatePosFromMsg(MsgLandmark &msg, MapPtr map)->void {

    if(this->IsOptimized()) return;

    KeyframePtr kf_ref = map->GetKeyframe(msg.id_reference);
    if(!kf_ref){
        std::cout << COUTFATAL << "Cannot find Ref-KF" << msg.id_reference << " for LM " << id_ << std::endl;
        exit(-1);
    }
    reference_kf_ = kf_ref;
    TransformType T_w_sref = kf_ref->GetPoseTws();
    Matrix3Type R_w_sref = T_w_sref.block<3,3>(0,0);
    Vector3Type t_w_sref = T_w_sref.block<3,1>(0,3);
    Vector3Type pos_w = R_w_sref * msg.pos_ref + t_w_sref;
    this->SetWorldPos(pos_w);
}

} //end ns
