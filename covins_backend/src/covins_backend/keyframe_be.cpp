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

#include "covins_backend/keyframe_be.hpp"

// C++
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>

// COVINS
#include <covins/covins_base/utils_base.hpp>
#include "covins_backend/landmark_be.hpp"
#include "covins_backend/map_be.hpp"

// Thirdparty
#include <robopt_open/imu-error/preintegration-base.h>

namespace covins {

Keyframe::Keyframe(MsgKeyframe msg, MapPtr map, VocabularyPtr voc)
    : KeyframeBase(msg.id,msg.timestamp,msg.calibration,
                   msg.img_dim_x_min,msg.img_dim_y_min,msg.img_dim_x_max,msg.img_dim_y_max)
{
    keypoints_aors_ = msg.keypoints_aors;
    keypoints_distorted_ = msg.keypoints_distorted;
    if(keypoints_distorted_.empty()) {
        std::cout << COUTERROR << "keypoints_distorted_.empty()" << std::endl;
        std::cout << COUTNOTICE << "Note: if you want to work only on undistorted KP, you can fill 'keypoints_distorted_' with the undistorted KPs and set the distortion coefficients to 0.0" << std::endl;
        exit(-1);
    }
    if(keypoints_aors_.size() != keypoints_distorted_.size()) {
        std::cout << COUTERROR << "keypoints given: " << std::endl;
        std::cout << "keypoints_distorted_.size(): " << keypoints_distorted_.size() << std::endl;
        std::cout << "keypoints_aors_.size(): " << keypoints_aors_.size() << std::endl;
        exit(-1);
    }
    keypoints_undistorted_ = msg.keypoints_undistorted;
    if(keypoints_distorted_.size() != keypoints_undistorted_.size()) {
        if(!keypoints_undistorted_.empty()) {
            std::cout << COUTERROR << "keypoints given: " << std::endl;
            std::cout << "keypoints_distorted_.size(): " << keypoints_distorted_.size() << std::endl;
            std::cout << "keypoints_undistorted_.size(): " << keypoints_undistorted_.size() << std::endl;
            exit(-1);
        } else {
            //Undistort
            keypoints_undistorted_.reserve(keypoints_distorted_.size());
            for(size_t idx_kp=0;idx_kp<keypoints_distorted_.size();++idx_kp) {
                Eigen::Vector2d kp_eigen = Utils::FromKeypointType(keypoints_distorted_[idx_kp]);
                Vector3Type p3D;
                camera_->backProject3(kp_eigen,&p3D);
                const Eigen::Vector3d p3_un = calibration_.K*p3D;
                if(p3_un(2) != 1.0) {
                    std::cout << COUTERROR << "p3_un: " << p3_un.transpose() << std::endl;
                    exit(-1);
                }
                TypeDefs::KeypointType kp_as_kptype = Utils::ToKeypointType(kp_eigen);
                keypoints_undistorted_.push_back(kp_as_kptype);
            }
        }
    }
    descriptors_ = msg.descriptors.clone();
    landmarks_.resize(keypoints_aors_.size(),nullptr);

    T_s_c_ = msg.T_s_c;

    bias_accel_ = msg.bias_accel;
    bias_gyro_ = msg.bias_gyro;
    lin_acc_ = msg.lin_acc;
    ang_vel_ = msg.ang_vel;

    img_ = msg.img;

    // Add additional Keypoints and Features
    keypoints_aors_add_ = msg.keypoints_aors_add;
    keypoints_distorted_add_ = msg.keypoints_distorted_add;

    if(keypoints_distorted_add_.empty()) {
        std::cout << COUTERROR << "keypoints_distorted_.empty()" << std::endl;
        std::cout << COUTNOTICE << "Note: if you want to work only on undistorted KP, you can fill 'keypoints_distorted_' with the undistorted KPs and set the distortion coefficients to 0.0" << std::endl;
        exit(-1);
    }
    if(keypoints_aors_add_.size() != keypoints_distorted_add_.size()) {
        std::cout << COUTERROR << "keypoints given: " << std::endl;
        std::cout << "keypoints_distorted_.size(): " << keypoints_distorted_add_.size() << std::endl;
        std::cout << "keypoints_aors_.size(): " << keypoints_aors_add_.size() << std::endl;
        exit(-1);
    }

    keypoints_undistorted_add_ = msg.keypoints_undistorted_add;
    if(keypoints_distorted_add_.size() != keypoints_undistorted_add_.size()) {
        if(!keypoints_undistorted_add_.empty()) {
            std::cout << COUTERROR << "keypoints given: " << std::endl;
            std::cout << "keypoints_distorted_.size(): " << keypoints_distorted_add_.size() << std::endl;
            std::cout << "keypoints_undistorted_.size(): " << keypoints_undistorted_add_.size() << std::endl;
            exit(-1);
        } else {
            //Undistort
            keypoints_undistorted_add_.reserve(keypoints_distorted_add_.size());
            for(size_t idx_kp=0;idx_kp<keypoints_distorted_add_.size();++idx_kp) {
                Eigen::Vector2d kp_eigen = Utils::FromKeypointType(keypoints_distorted_add_[idx_kp]);
                Vector3Type p3D;
                camera_->backProject3(kp_eigen,&p3D);
                const Eigen::Vector3d p3_un = calibration_.K*p3D;
                if(p3_un(2) != 1.0) {
                    std::cout << COUTERROR << "p3_un: " << p3_un.transpose() << std::endl;
                    exit(-1);
                }
                TypeDefs::KeypointType kp_as_kptype = Utils::ToKeypointType(p3_un.block<1,2>(0,0));
                keypoints_undistorted_add_.push_back(kp_as_kptype);
            }
        }
    }

    descriptors_add_ = msg.descriptors_add.clone();

    ///////////////
    this->AssignFeaturesToGrid();

    if(msg.save_to_file) {
        this->SetPoseTws(msg.T_w_s);
        msg_ = msg;
    } else {
        this->UpdatePoseFromMsg(msg,map);
    }

    // BoW
    {
        if(!bow_vec_.empty() || !feat_vec_.empty()){
            cout << COUTERROR << " !mBowVec.empty() || !mFeatVec.empty()" << endl;
            exit(-1);
        }
        vector<cv::Mat> current_desc = Utils::ToDescriptorVector(descriptors_add_);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        voc->transform(current_desc,bow_vec_,feat_vec_,4);
    }

    // PreIntegration
    if(msg.preintegration.dt.empty()) {
        preintegrated_imu_.reset(new robopt::imu::PreintegrationBase(Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),bias_accel_,bias_gyro_,calibration_.sigma_a_c,calibration_.sigma_g_c,calibration_.sigma_aw_c,calibration_.sigma_gw_c,calibration_.g));
        if(id_.first > 0) std::cout << COUTERROR << "KF " << id_ << ": no preintegration data!" << std::endl;
    } else {
        for(size_t idx=0;idx<msg.preintegration.dt.size();++idx) {
            const double dt = msg.preintegration.dt[idx];
            const Eigen::Vector3d linear_acceleration = Eigen::Vector3d(msg.preintegration.lin_acc_x[idx],msg.preintegration.lin_acc_y[idx],msg.preintegration.lin_acc_z[idx]);
            const Eigen::Vector3d angular_velocity = Eigen::Vector3d(msg.preintegration.ang_vel_x[idx],msg.preintegration.ang_vel_y[idx],msg.preintegration.ang_vel_z[idx]);
            if(idx == 0) {
                preintegrated_imu_.reset(new robopt::imu::PreintegrationBase(msg.lin_acc_init,msg.ang_vel_init,bias_accel_,bias_gyro_,calibration_.sigma_a_c,calibration_.sigma_g_c,calibration_.sigma_aw_c,calibration_.sigma_gw_c,calibration_.g));
            }

            if(dt == 0.0) {
                std::cout << COUTWARN << "KF " << id_ << ": dt: " << dt << " -- skip measurement" << std::endl;
                continue;
            }

            preintegrated_imu_->push_back(dt,linear_acceleration,angular_velocity);
            lin_acc_ = linear_acceleration;
            ang_vel_ = angular_velocity;
        }
    }

    // Bearing vectors
    bearings_.resize(keypoints_undistorted_.size());
    double cx = calibration_.intrinsics[2];
    double cy = calibration_.intrinsics[3];
    double invfx = 1.0/calibration_.intrinsics[0];
    double invfy = 1.0/calibration_.intrinsics[1];
    for(size_t idx=0;idx<keypoints_undistorted_.size();++idx) {
        Vector3Type tmpBearing((keypoints_undistorted_[idx](0)-cx)*invfx, (keypoints_undistorted_[idx](1)-cy)*invfy, 1.0);
        bearings_[idx] = tmpBearing.normalized();
    }

    // Additional Bearing Vectors
    bearings_add_.resize(keypoints_undistorted_add_.size());
    for(size_t idx=0;idx<keypoints_undistorted_add_.size();++idx) {
        Vector3Type tmpBearing((keypoints_undistorted_add_[idx](0)-cx)*invfx, (keypoints_undistorted_add_[idx](1)-cy)*invfy, 1.0);
        bearings_add_[idx] = tmpBearing.normalized();
    }
}

auto Keyframe::ComputeRedundancyValue()->double {
    // calculates a redundancy value based on [Schmuck and Chli, 3DV'19]
    double red_sum = 0;
    double n_lms = 0;

    for(const auto& lm : landmarks_) {
        if(!lm) continue;
        if(lm->IsInvalid()) {
            continue;
        }
        size_t n_obs = static_cast<double>(lm->GetObservations().size());
        if(n_obs < 2) {
            continue;
        }
        double value = 0.0;
        if(n_obs == 2) value = 0.0;
        else if(n_obs == 3) value = 0.4;
        else if(n_obs == 4) value = 0.7;
        else if(n_obs == 5) value = 0.9;
        else value = 1.0;

        red_sum += value;
        n_lms += 1.0;
    }
    double red_val = red_sum / n_lms;

    latest_red_val_ = red_val;
    return red_val;
}

auto Keyframe::CompStamp(KeyframePtr kf1, KeyframePtr kf2)->bool {
    return  kf1->timestamp_ > kf2->timestamp_;
}

auto Keyframe::ConvertToMsg(MsgKeyframe &msg, KeyframePtr kf_ref, bool is_update)->void {
    std::unique_lock<std::mutex> lock_conn(mtx_connections_);
    std::unique_lock<std::mutex> lock_feat(mtx_features_);
    std::unique_lock<std::mutex> lock_pose(mtx_pose_);

    if(kf_ref && kf_ref->id_ == id_) {
        //This will cause a deadlock when calling kf_ref->GetPoseTws();
        std::cout << COUTERROR << "kf_ref && kf_ref->id_ == id_" << std::endl;
        exit(-1);
    }

    msg.is_update_msg = is_update;

    msg.id = id_;
    msg.timestamp = timestamp_;

    if(!kf_ref)  {
        std::cout << COUTFATAL << "!kf_ref!" << std::endl;
        exit(-1);
    }

    if(kf_ref->id_.second != id_.second) {
        std::cout << COUTFATAL << "kf_ref and THIS do not belong to the same agent!" << std::endl;
        exit(-1);
    }

    if(kf_ref->id_.first != 0) {
        std::cout << COUTFATAL << "kf_ref is not the origin frame of the KF!" << std::endl;
        exit(-1);
    }

    TransformType T_w_sref = kf_ref->GetPoseTws();

    msg.id_reference = kf_ref->id_;
    msg.T_sref_s = T_w_sref.inverse() * T_w_s_;
}

auto Keyframe::ConvertToMsgFileExport(MsgKeyframe &msg)->void {
    std::unique_lock<std::mutex> lock_conn(mtx_connections_);
    std::unique_lock<std::mutex> lock_feat(mtx_features_);
    std::unique_lock<std::mutex> lock_pose(mtx_pose_);

    msg.save_to_file = true;

    msg.id = id_;
    msg.timestamp = timestamp_;
    msg.calibration = calibration_;
    msg.img_dim_x_min = img_dim_x_min_;
    msg.img_dim_y_min = img_dim_y_min_;
    msg.img_dim_x_max = img_dim_x_max_;
    msg.img_dim_y_max = img_dim_y_max_;

    msg.keypoints_aors = keypoints_aors_;
    msg.keypoints_distorted = keypoints_distorted_;
    msg.keypoints_undistorted = keypoints_undistorted_;
    msg.descriptors = descriptors_.clone();

    msg.T_s_c = T_s_c_;
    msg.T_w_s = T_w_s_;
    msg.velocity = velocity_;
    msg.bias_accel = bias_accel_;
    msg.bias_gyro = bias_gyro_;

    if(predecessor_) {
        predecessor_->GetLinAccAngVel(msg.lin_acc_init,msg.ang_vel_init);
    } else {
        if(id_.first != 0) std::cout << COUTWARN << "KF " << id_ << ": no predecessor - cannot set init acc/gyr" << std::endl;
    }

    ConvertPreintegrationToMsg(msg.preintegration);

    msg.img = img_;

    if(predecessor_) msg.id_predecessor = predecessor_->id_;
    if(successor_) msg.id_successor = successor_->id_;

    for (size_t indx = 0; indx < landmarks_.size(); indx++) {
        if(landmarks_[indx] != nullptr)
            msg.landmarks.insert(std::make_pair(indx, landmarks_[indx]->id_));
    }
}

auto Keyframe::EstablishConnections(MsgKeyframe msg, MapPtr map)->void {
    //Temporal neighborhood
    bool expect_pred_null = msg.id_predecessor == defpair ? true : false;
    KeyframePtr pred = map->GetKeyframe(msg.id_predecessor,expect_pred_null);
    KeyframePtr succ = map->GetKeyframe(msg.id_successor,true);

    if(pred) {
        predecessor_ = pred;
        pred->SetSuccessor(shared_from_this());
    }

    if(succ) {
        successor_ = succ;
        succ->SetPredecessor(shared_from_this());
    }

    //Landmarks
    for(std::map<int, idpair>::iterator mit = msg.landmarks.begin(); mit!=msg.landmarks.end();++mit){
        size_t feat_id = mit->first;
        idpair lm_id = mit->second;
        LandmarkPtr lm = map->GetLandmark(lm_id);

        if(lm){
            if(!lm->GetReferenceKeyframe()) {
                continue;
            }
            landmarks_[feat_id] = lm;
            lm->AddObservation(shared_from_this(),feat_id);
        } else {
            //if MP not in Map, we ignore it. Might be deleted, or comes in later and is then added to this KF
        }
    }
}

auto Keyframe::FusePreintegration(PreintegrationPtr preint)->void {
    Vector3Type acc_init;
    Vector3Type gyr_init;
    if(!predecessor_) {
        std::cout << COUTERROR << "no predecessor" << std::endl;
        exit(-1);
    }
    predecessor_->GetLinAccAngVel(acc_init,gyr_init);
    PreintegrationPtr pi_fused{new robopt::imu::PreintegrationBase(acc_init,gyr_init,bias_accel_,bias_gyro_,calibration_.sigma_a_c,calibration_.sigma_g_c,calibration_.sigma_aw_c,calibration_.sigma_gw_c,calibration_.g)};

    const int n0 = preint->getNumMeasurements();
    for(int idx=0;idx<n0;++idx) {
        Vector3Type acc,gyr;
        preint->getReadingsByIndex(idx,&acc,&gyr);
        precision_t dt = preint->getTimeDiffByIndex(idx);
        pi_fused->push_back(dt,acc,gyr);
    }

    const int n1 = preintegrated_imu_->getNumMeasurements();
    for(int idx=0;idx<n1;++idx) {
        Vector3Type acc,gyr;
        preintegrated_imu_->getReadingsByIndex(idx,&acc,&gyr);
        precision_t dt = preintegrated_imu_->getTimeDiffByIndex(idx);
        pi_fused->push_back(dt,acc,gyr);
    }

    preintegrated_imu_ = pi_fused;
}

auto Keyframe::GetTimeSpanPredSucc(bool bIgnoreMutex)->precision_t {

    if(!bIgnoreMutex) {
        unique_lock<mutex> lockCon(mtx_connections_);
    }

    if(!predecessor_) {
        if(id_.first != 0)
            std::cout << COUTWARN << "KF " << id_ << ": no predecessor" << std::endl;
        return -1.0;
    }

    if(!successor_) {
        return -1.0;
    }

    double dDelta_t = successor_->timestamp_ - predecessor_->timestamp_;

    return dDelta_t;
}

auto Keyframe::IsPoseOptimized()->bool {
    std::unique_lock<std::mutex> lock_pos(mtx_pose_);
    return pose_optimized_;
}

auto Keyframe::IsVelBiasOptimized()->bool {
    std::unique_lock<std::mutex> lock_pos(mtx_pose_);
    return vel_bias_optimized_;
}

auto Keyframe::kf_less::operator ()(const KeyframePtr a, const KeyframePtr b) const ->bool
{
    if(a->id_.second < b->id_.second)
        return true;
    else if(a->id_.second > b->id_.second)
        return false;
    else {
        return a->id_.first < b->id_.first;
    }
}

auto Keyframe::RemapLandmark(LandmarkPtr lm, const size_t feat_id_now, const size_t feat_id_new)->void {
    std::unique_lock<std::mutex> lock_feat(mtx_features_);

    auto lm_new = landmarks_[feat_id_new];

    landmarks_[feat_id_now] = nullptr;
    landmarks_[feat_id_new] = lm;

    lm->EraseObservation(shared_from_this());
    lm->AddObservation(shared_from_this(),feat_id_new);
    if(lm_new) lm_new->EraseObservation(shared_from_this());
}

auto Keyframe::SetErase()->void {
    std::unique_lock<std::mutex> lock_conn(mtx_connections_);
    not_erase_ = false;
}

auto Keyframe::SetInvalid()->bool {
    if(invalid_) {
        return false;
    }

    std::unique_lock<std::mutex> lock_conn(mtx_connections_);
    std::unique_lock<std::mutex> lock_feat(mtx_features_);
    std::unique_lock<std::mutex> lock_inv(mtx_invalid_);
    if(id_.first == 0 || !predecessor_ || !successor_ || not_erase_) {
        return false;
    }

    for(auto lm : landmarks_) {
        if(!lm)
            continue;
        lm->EraseObservation(shared_from_this());
    }

    for(auto con : connected_kfs_) {
        con->EraseConnectedKeyframe(shared_from_this());
    }

    predecessor_->SetSuccessor(successor_);
    successor_->SetPredecessor(predecessor_);
    successor_->FusePreintegration(preintegrated_imu_);

    landmarks_.clear();
    connected_kfs_.clear();
    connections_weights_.clear();
    predecessor_ = nullptr;
    successor_ = nullptr;

    invalid_ = true;

    return true;
}

auto Keyframe::SetNotErase()->void {
    std::unique_lock<std::mutex> lock_conn(mtx_connections_);
    not_erase_ = true;
}

auto Keyframe::SetPoseOptimized()->void {
    std::unique_lock<std::mutex> lock_pos(mtx_pose_);
    pose_optimized_ = true;
}

auto Keyframe::SetVelBiasOptimized()->void {
    std::unique_lock<std::mutex> lock_pos(mtx_pose_);
    vel_bias_optimized_ = true;
}

auto Keyframe::sort_by_redval(const KeyframePtr a, const KeyframePtr b)->bool {
    if(a->latest_red_val_ > b->latest_red_val_) return true;
    else return false;
}

auto Keyframe::UpdateCovisibilityConnections()->void {
    KeyframeIntMap candidate_kfs;
    LandmarkVector landmarks;
    {
        std::unique_lock<std::mutex> lock(mtx_features_);
        landmarks = landmarks_;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(LandmarkVector::iterator vit = landmarks.begin(); vit != landmarks.end(); ++vit) {
        LandmarkPtr lm = *vit;
        if(!lm) continue;
        if(lm->IsInvalid()) continue;
        Landmark::KfObservations obs = lm->GetObservations();
        for(Landmark::KfObservations::iterator mit = obs.begin(); mit != obs.end(); ++mit) {
            if(mit->first->id_ == this->id_) continue;
            candidate_kfs[mit->first]++;
        }
    }

    if(candidate_kfs.empty()){
        return;
    }

    // Add all KFs above covisibility threshold
    {
        std::unique_lock<std::mutex> lock(mtx_connections_);

        int th = covins_params::sys::covis_thres;
        VectorIntKfPair kfs_with_weight;

        connected_kfs_.clear();
        connections_weights_.clear();
        connected_kfs_.reserve(candidate_kfs.size());
        connections_weights_.reserve(candidate_kfs.size());

        for(KeyframeIntMap::iterator mit = candidate_kfs.begin(); mit != candidate_kfs.end(); mit++) {
            int w = mit->second;
            KeyframePtr neigh = mit->first;
            if(w >= th) {
                connected_kfs_.push_back(neigh);
                connections_weights_.push_back(w);
                neigh->AddConnectedKeyframe(shared_from_this(),w);
            }
        }

        this->SortConnectedKeyframes(false);
    }
}

auto Keyframe::UpdatePoseFromMsg(MsgKeyframe &msg, MapPtr map)->void {

    if(!this->IsPoseOptimized()) {
        if(id_.first == 0){
            //first KF - global pose
            this->SetPoseTws(msg.T_sref_s);
        } else {
            KeyframePtr kf_ref = map->GetKeyframe(msg.id_reference);
            if(!kf_ref){
                std::cout << COUTFATAL << "Cannot find Ref-KF" << msg.id_reference << " for KF " << id_ << std::endl;
                exit(-1);
            }
            TransformType T_w_sref = kf_ref->GetPoseTws();
            TransformType Tws = T_w_sref * msg.T_sref_s;
            this->SetPoseTws(Tws);
        }
    }

    if(!this->IsVelBiasOptimized()){
        Eigen::Vector3d v_in_s = msg.velocity;
        Eigen::Vector3d v_in_w = T_w_s_.block<3,3>(0,0) * v_in_s;
        velocity_ = v_in_w;

        bias_accel_ = msg.bias_accel;
        bias_gyro_ = msg.bias_gyro;
    }
}

auto Keyframe::GetDescriptorAddCV(size_t ind)->cv::Mat {
    return descriptors_add_.row(ind).clone();
}

const unsigned char* Keyframe::GetDescriptorAdd(size_t ind) {
  return descriptors_add_.data + descriptors_add_.cols*ind;
}

} //end ns
