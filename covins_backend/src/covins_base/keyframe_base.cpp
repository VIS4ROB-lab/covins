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

#include "covins_base/keyframe_base.hpp"

// COVINS
#include <covins/covins_base/msgs/msg_keyframe.hpp>
#include <covins/covins_base/utils_base.hpp>

// Thirdparty
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera-unified-projection.h>
#include <aslam/cameras/distortion-radtan.h>
#include <aslam/cameras/distortion-equidistant.h>

namespace covins {

KeyframeBase::KeyframeBase(idpair id, double timestamp, VICalibration calib,
                           int img_dim_x_min, int img_dim_y_min, int img_dim_x_max, int img_dim_y_max)
    : id_(id),timestamp_(timestamp),
      calibration_(calib),
      img_dim_x_min_(img_dim_x_min),img_dim_y_min_(img_dim_y_min),
      img_dim_x_max_(img_dim_x_max),img_dim_y_max_(img_dim_y_max)
{
    T_s_c_ = calibration_.T_SC;

    if(calibration_.intrinsics[0] == 0.0){
        std::cout << "Calibration incorrect?: intrinsics: " << calibration_.intrinsics.transpose() << std::endl;
        exit(-1);
    }
    if(calibration_.g < 9.0) {
        std::cout << COUTERROR << "KF" << id_  <<": calibration_.g: " << calibration_.g << " -- check whether calibration is correct!" << std::endl;
        calibration_.show();
        exit(-1);
    }

    aslam::Distortion::UniquePtr distortion_model;
    switch(calibration_.dist_model)
    {
        case(eDistortionModel::RADTAN):
            distortion_model.reset(new aslam::RadTanDistortion(calibration_.dist_coeffs));
            break;
        case(eDistortionModel::EQUI):
            distortion_model.reset(new aslam::EquidistantDistortion(calibration_.dist_coeffs));
            break;
        default:
            std::cout << COUTFATAL << "Distortion model '" << calibration_.dist_model << "' not supported by frame -- kill" << std::endl;
            exit(-1);
    }

    switch(calibration_.cam_model)
    {
        case(eCamModel::PINHOLE):
            camera_.reset(new aslam::PinholeCamera(calibration_.intrinsics,calibration_.img_dims[0],calibration_.img_dims[1],distortion_model));
            break;
        case(eCamModel::OMNI):
            camera_.reset(new aslam::UnifiedProjectionCamera(calibration_.intrinsics,calibration_.img_dims[0],calibration_.img_dims[1],distortion_model));
            break;
        default:
            std::cout << COUTFATAL << "Cam model '" << calibration_.cam_model << "' not supported by frame -- kill" << std::endl;
            exit(-1);
    }
}

KeyframeBase::KeyframeBase(idpair id, double timestamp, VICalibration calib, PreintegrationPtr preintegration,
                           int img_dim_x_min, int img_dim_y_min, int img_dim_x_max, int img_dim_y_max)
    : KeyframeBase(id,timestamp,calib,img_dim_x_min,img_dim_y_min,img_dim_x_max,img_dim_y_max)
{
    preintegrated_imu_ = preintegration;
}

auto KeyframeBase::AddConnectedKeyframe(KeyframePtr kf, int weight)->void {
    {
        std::unique_lock<std::mutex> lock(mtx_connections_);

        KeyframeVector::iterator vit = std::find(connected_kfs_.begin(),connected_kfs_.end(),kf);
        if(vit == connected_kfs_.end()) {
            connected_kfs_.push_back(kf);
            connections_weights_.push_back(weight);
        } else {
            int idx = vit - connected_kfs_.begin();
            if(connections_weights_[idx] != weight)
                connections_weights_[idx] = weight;
            else
                return;
        }
    }

    this->SortConnectedKeyframes();
}

auto KeyframeBase::AddLandmark(LandmarkPtr landmark, int index)->bool {
    std::unique_lock<std::mutex> lock(mtx_features_);
    LandmarkVector::iterator vit = std::find(landmarks_.begin(),landmarks_.end(),landmark);
    if(vit != landmarks_.end()) {
        return false;
    }
    landmarks_[index] = landmark;
    return true;
}

auto KeyframeBase::AssignFeaturesToGrid()->void {
    CHECK_NE(0,keypoints_distorted_.size()) << "Need to have keypoints detected!";
    const size_t num_keypoints = keypoints_distorted_.size();
    int num_reserve = 0.5f * num_keypoints / (FRAME_GRID_COLS * FRAME_GRID_ROWS);
    for (size_t i = 0; i < FRAME_GRID_COLS; ++i) {
        for (size_t j = 0; j < FRAME_GRID_ROWS; ++j) {
            keypoint_grid_[i][j].reserve(num_reserve);
        }
    }

    const uint32_t img_height = camera_->imageHeight();
    const uint32_t img_width = camera_->imageWidth();
    grid_width_inv_ = static_cast<double>(FRAME_GRID_COLS) / static_cast<double>(img_width);
    grid_height_inv_ = static_cast<double>(FRAME_GRID_ROWS) / static_cast<double>(img_height);
    for (size_t i = 0; i < num_keypoints; ++i) {
        size_t pos_x = std::round(keypoints_distorted_[i](0) * grid_width_inv_);
        size_t pos_y = std::round(keypoints_distorted_[i](1) * grid_height_inv_);
        keypoint_grid_[pos_x][pos_y].push_back(i);
    }

    assigned_to_grid_ = true;
}

auto KeyframeBase::ConvertPreintegrationToMsg(PreintegrationData &data)->void {
    if(!preintegrated_imu_) return;

    const int n = preintegrated_imu_->getNumMeasurements();
    preintegrated_imu_->getReadingsByIndex(0, &data.acc, &data.gyr);
    preintegrated_imu_->getLinearizedBias(&data.lin_bias_accel, &data.lin_bias_gyro);

    Eigen::Vector3d acc;
    Eigen::Vector3d gyr;

    data.dt.resize(n);
    data.lin_acc_x.resize(n);
    data.lin_acc_y.resize(n);
    data.lin_acc_z.resize(n);
    data.ang_vel_x.resize(n);
    data.ang_vel_y.resize(n);
    data.ang_vel_z.resize(n);

    for(int idx=0;idx<n;++idx) {
        data.dt[idx] = preintegrated_imu_->getTimeDiffByIndex(idx);
        preintegrated_imu_->getReadingsByIndex(idx, &acc, &gyr);
        data.lin_acc_x[idx] = acc[0];
        data.lin_acc_y[idx] = acc[1];
        data.lin_acc_z[idx] = acc[2];
        data.ang_vel_x[idx] = gyr[0];
        data.ang_vel_y[idx] = gyr[1];
        data.ang_vel_z[idx] = gyr[2];
    }
}

auto KeyframeBase::EraseConnectedKeyframe(KeyframePtr kf)->void {
    std::unique_lock<std::mutex> lock(mtx_connections_);

    auto vit = std::find(connected_kfs_.begin(),connected_kfs_.end(),kf);
    if(vit == connected_kfs_.end()) return;
    else {
        int idx = vit - connected_kfs_.begin();
        connected_kfs_.erase(vit);
        connections_weights_.erase(connections_weights_.begin()+idx);
    }
}

auto KeyframeBase::EraseLandmark(size_t index)->void {
    std::unique_lock<std::mutex> lock(mtx_features_);
    if(index >= landmarks_.size()) {
        return;
    }
    landmarks_[index] = nullptr;
}

auto KeyframeBase::EraseLandmark(LandmarkPtr lm)->void {
    std::unique_lock<std::mutex> lock(mtx_features_);
    LandmarkVector::iterator vit = std::find(landmarks_.begin(),landmarks_.end(),lm);
    if(vit != landmarks_.end())
        landmarks_.erase(vit);

    vit = std::find(landmarks_.begin(),landmarks_.end(),lm);
}

auto KeyframeBase::EraseLandmark(LandmarkPtr lm, size_t index)->void {
    std::unique_lock<std::mutex> lock(mtx_features_);
    if(index >= landmarks_.size()) {
        return;
    }
    if(landmarks_[index] != lm) {
        return;
    }

    landmarks_[index] = nullptr;
}

auto KeyframeBase::GetConnectedKeyframesByWeight(int weight)->KeyframeVector {
    std::unique_lock<std::mutex> lock(mtx_connections_);

    if(connected_kfs_.empty()) {
        return KeyframeVector();
    }

    std::vector<int>::iterator vit = upper_bound(connections_weights_.begin(),connections_weights_.end(),weight,std::greater<int>());
    if(vit == connections_weights_.begin()) {
        return KeyframeVector();
    } else {
        int n = vit - connections_weights_.begin();
        return KeyframeVector(connected_kfs_.begin(), connected_kfs_.begin() + n);
    }
}

auto KeyframeBase::GetConnectedNeighborKeyframes()->KeyframeVector {
    std::unique_lock<std::mutex> lock(mtx_connections_);

    if(connected_n_kfs_.empty()) {
        return KeyframeVector();
    } else {
      return connected_n_kfs_;
    }
}

auto KeyframeBase::GetConnectionWeight(KeyframePtr kf)->int {
    std::unique_lock<std::mutex> lock(mtx_connections_);

    KeyframeVector::iterator vit = std::find(connected_kfs_.begin(),connected_kfs_.end(),kf);
    if(vit == connected_kfs_.end()) {
        return 0;
    } else {
        int index = vit - connected_kfs_.begin();
        return connections_weights_[index];
    }
}

auto KeyframeBase::GetDescriptor(size_t ind)->const unsigned char* {
    return descriptors_.data + descriptors_.cols*ind;
}

auto KeyframeBase::GetDescriptorCV(size_t ind)->cv::Mat {
    return descriptors_.row(ind).clone();
}

auto KeyframeBase::GetFeaturesInArea(const precision_t x, const precision_t y, const precision_t r)->std::vector<size_t> {
    const TypeDefs::KeypointType target = TypeDefs::KeypointType(x,y);
    return this->GetFeaturesInArea(target,r);
}

auto KeyframeBase::GetFeaturesInArea(const TypeDefs::KeypointType target, const precision_t radius)->std::vector<size_t> {
    CHECK_GT(radius, 0.0);

    std::vector<size_t> candidate_indices;
    candidate_indices.reserve(keypoints_distorted_.size());

    if (!assigned_to_grid_) {
        LOG_FIRST_N(WARNING,1 ) << "Brute force proximity search is used, quite inefficient.";

        for (size_t index = 0; index < keypoints_distorted_.size(); ++index) {
            const double distance = (keypoints_distorted_[index] - target).norm();
            if (distance <= radius) {
                candidate_indices.push_back(index);
            }
        }
    } else {
        const int min_cell_x = std::max(0,  int(std::floor((target.x() - radius) * grid_width_inv_)));
        if (min_cell_x >= FRAME_GRID_COLS) {
            return candidate_indices;
        }
        const int max_cell_x = std::min(int(FRAME_GRID_COLS - 1), int(std::ceil((target.x() + radius) * grid_width_inv_)));
        if (max_cell_x < 0) {
            return  candidate_indices;
        }
        const int min_cell_y= std::max(
        0,  int(std::floor((target.y() - radius) * grid_height_inv_)));
        if (min_cell_y >= FRAME_GRID_ROWS) {
            return candidate_indices;
        }
        const int max_cell_y = std::min(int(FRAME_GRID_ROWS - 1), int(std::ceil((target.y() + radius) * grid_height_inv_)));
        if (max_cell_y < 0) {
            return  candidate_indices;
        }

        for (int ix = min_cell_x; ix <= max_cell_x; ++ix) {
            for(int iy = min_cell_y; iy <= max_cell_y; ++iy) {
                const std::vector<size_t> cells = keypoint_grid_[ix][iy];
                if(cells.empty()) continue;

                for(size_t j = 0; j < cells.size(); ++j) {
                    const TypeDefs::KeypointType& keypoint = keypoints_distorted_[cells[j]];
                    const double distance = (keypoint - target).norm();
                    if (distance <= radius) {
                        candidate_indices.push_back(cells[j]);
                    }
                }
            }
        }
    }

    return candidate_indices;
}

auto KeyframeBase::GetLandmark(int index)->LandmarkPtr {
    std::unique_lock<std::mutex> lock(mtx_features_);
    return landmarks_[index];
}

auto KeyframeBase::GetLandmarkIndex(LandmarkPtr lm)->int {
    std::unique_lock<std::mutex> lock(mtx_features_);
    LandmarkVector::iterator vit = std::find(landmarks_.begin(),landmarks_.end(),lm);
    if(vit != landmarks_.end()) return (vit - landmarks_.begin());
    else return -1;
}

auto KeyframeBase::GetLandmarks()->LandmarkVector {
    std::unique_lock<std::mutex> lock(mtx_features_);
    return landmarks_;
}

auto KeyframeBase::GetLinAccAngVel(Vector3Type &lin_acc, Vector3Type &ang_vel)->void {
    std::unique_lock<std::mutex> lock(mtx_pose_);
    lin_acc = lin_acc_;
    ang_vel = ang_vel_;
}

auto KeyframeBase::GetPoseTcw()->TransformType {
    std::unique_lock<std::mutex> lock(mtx_pose_);
    return T_c_w_;
}

auto KeyframeBase::GetPoseTsw()->TransformType {
    std::unique_lock<std::mutex> lock(mtx_pose_);
    return T_s_w_;
}

auto KeyframeBase::GetPoseTwc()->TransformType {
    std::unique_lock<std::mutex> lock(mtx_pose_);
    return T_w_c_;
}

auto KeyframeBase::GetPoseTws()->TransformType {
    std::unique_lock<std::mutex> lock(mtx_pose_);
    return T_w_s_;
}

auto KeyframeBase::GetPredecessor()->KeyframePtr {
    std::unique_lock<std::mutex> lock(mtx_connections_);
    return predecessor_;
}

auto KeyframeBase::GetStateBias(Vector3Type &ba, Vector3Type &bg)->void {
    std::unique_lock<std::mutex> lock(mtx_pose_);
    ba = bias_accel_;
    bg = bias_gyro_;
}

auto KeyframeBase::GetStateExtrinsics()->TransformType {
    std::unique_lock<std::mutex> lock(mtx_pose_);
    return T_s_c_;
}

auto KeyframeBase::GetStateVelocity()->Vector3Type {
    std::unique_lock<std::mutex> lock(mtx_pose_);
    return velocity_;
}

auto KeyframeBase::GetSuccessor()->KeyframePtr {
    std::unique_lock<std::mutex> lock(mtx_connections_);
    return successor_;
}

auto KeyframeBase::GetValidLandmarks()->LandmarkVector {
    std::unique_lock<std::mutex> lock(mtx_features_);
    LandmarkVector valid_lms;
    valid_lms.reserve(landmarks_.size());
    for(auto i : landmarks_) {
        if(i) valid_lms.push_back(i);
    }
    return valid_lms;
}

auto KeyframeBase::IsInvalid()->bool {
    std::unique_lock<std::mutex> lock_inv(mtx_invalid_);
    return invalid_;
}

auto KeyframeBase::IsInImage(const precision_t x, const precision_t y)->bool {
    return (x>=img_dim_x_min_ && x<img_dim_x_max_ && y>=img_dim_y_min_ && y<img_dim_y_max_);
}

auto KeyframeBase::SetLinAccAngVel(Vector3Type lin_acc, Vector3Type ang_vel)->void {
    std::unique_lock<std::mutex> lock(mtx_pose_);
    lin_acc_ = lin_acc;
    ang_vel_ = ang_vel;
}

auto KeyframeBase::SetPoseTws(TransformType Tws, bool lock_mtx)->void {
    if(lock_mtx) mtx_pose_.lock();
    T_w_s_ = Tws;
    T_s_w_ = T_w_s_.inverse();
    T_w_c_ = T_w_s_*T_s_c_;
    T_c_w_ = T_w_c_.inverse();
    if(lock_mtx) mtx_pose_.unlock();
}

auto KeyframeBase::SetPredecessor(KeyframePtr kf)->void {
    std::unique_lock<std::mutex> lock(mtx_connections_);
    predecessor_ = kf;
}

auto KeyframeBase::SetStateBias(Vector3Type ba, Vector3Type bg)->void {
    std::unique_lock<std::mutex> lock(mtx_pose_);
    bias_accel_ = ba;
    bias_gyro_ = bg;
}

auto KeyframeBase::SetStateVelocity(Vector3Type vel)->void {
    std::unique_lock<std::mutex> lock(mtx_pose_);
    velocity_ = vel;
}

auto KeyframeBase::SetSuccessor(KeyframePtr kf)->void {
    std::unique_lock<std::mutex> lock(mtx_connections_);
    successor_ = kf;
}

auto KeyframeBase::SortConnectedKeyframes(bool lock_mtx)->void {
    if(lock_mtx) mtx_pose_.lock();
    VectorIntKfPair kfs_with_weight;
    KeyframeSet check_kfs;
    for(size_t idx=0;idx<connected_kfs_.size();++idx){
        kfs_with_weight.push_back(std::make_pair(connections_weights_[idx],connected_kfs_[idx]));
        if(check_kfs.count(connected_kfs_[idx]))  {
            continue;
        }
        else
            check_kfs.insert(connected_kfs_[idx]);
    }

    sort(kfs_with_weight.begin(),kfs_with_weight.end(),std::greater<IntKfPair>());

    connected_kfs_.resize(kfs_with_weight.size(),nullptr);
    connections_weights_.resize(kfs_with_weight.size(),0);
    for(size_t idx=0;idx<kfs_with_weight.size();++idx){
        IntKfPair p = kfs_with_weight[idx];
        connected_kfs_[idx] = p.second;
        connections_weights_[idx] = p.first;
    }
    if(lock_mtx) mtx_pose_.unlock();
}

auto KeyframeBase::UpdateCeresFromState(precision_t *ceres_pose, precision_t *ceres_velocity_and_bias, precision_t *ceres_extrinsics)->void {
    std::unique_lock<std::mutex> lock(mtx_pose_);

    // Update the pose
    Matrix3Type Rws = T_w_s_.block<3,3>(0,0);
    QuaternionType qws(Rws);

    ceres_pose[0]= qws.x();
    ceres_pose[1]= qws.y();
    ceres_pose[2]= qws.z();
    ceres_pose[3]= qws.w();
    ceres_pose[4]= T_w_s_(0,3);
    ceres_pose[5]= T_w_s_(1,3);
    ceres_pose[6]= T_w_s_(2,3);

    // Update the Extrinsics
    Matrix3Type Rsc = T_s_c_.block<3,3>(0,0);
    QuaternionType qsc(Rsc);
    ceres_extrinsics[0]= qsc.x();
    ceres_extrinsics[1]= qsc.y();
    ceres_extrinsics[2]= qsc.z();
    ceres_extrinsics[3]= qsc.w();
    ceres_extrinsics[4]= T_s_c_(0,3);
    ceres_extrinsics[5]= T_s_c_(1,3);
    ceres_extrinsics[6]= T_s_c_(2,3);

    // Update velocity and bias
    ceres_velocity_and_bias[0] = velocity_[0];
    ceres_velocity_and_bias[1] = velocity_[1];
    ceres_velocity_and_bias[2] = velocity_[2];
    ceres_velocity_and_bias[3] = bias_accel_[0];
    ceres_velocity_and_bias[4] = bias_accel_[1];
    ceres_velocity_and_bias[5] = bias_accel_[2];
    ceres_velocity_and_bias[6] = bias_gyro_[0];
    ceres_velocity_and_bias[7] = bias_gyro_[1];
    ceres_velocity_and_bias[8] = bias_gyro_[2];
}

auto KeyframeBase::UpdateFromCeres(const precision_t *ceres_pose, const precision_t *ceres_velocity_and_bias, const precision_t *ceres_extrinsics)->void {
    std::unique_lock<std::mutex> lock(mtx_pose_);

    // Convert the pose to a matrix
    TransformType Tws = Utils::Ceres2Transform(ceres_pose);
    TransformType Tsc = Utils::Ceres2Transform(ceres_extrinsics);
    T_s_c_ = Tsc;
    this->SetPoseTws(Tws,false);

    // Update the bias and velocity
    velocity_ = Vector3Type(ceres_velocity_and_bias[0], ceres_velocity_and_bias[1], ceres_velocity_and_bias[2]);
    bias_accel_ = Vector3Type(ceres_velocity_and_bias[3], ceres_velocity_and_bias[4], ceres_velocity_and_bias[5]);
    bias_gyro_ = Vector3Type(ceres_velocity_and_bias[6], ceres_velocity_and_bias[7], ceres_velocity_and_bias[8]);
}

} //end ns
