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

#include "covins_base/landmark_base.hpp"

namespace covins {

LandmarkBase::LandmarkBase(idpair id)
    : id_(id)
{
    //...
}

LandmarkBase::LandmarkBase(idpair id, Vector3Type pos_w, KeyframePtr kf_ref)
    : id_(id),pos_w_(pos_w),reference_kf_(kf_ref)
{
    //...
}

auto LandmarkBase::AddObservation(KeyframePtr kf, size_t index, bool suppress_ref_check)->bool {
    std::unique_lock<std::mutex> lock(mtx_obs_);
    if(!kf) {
        std::cout << COUTWARN << "Nullptr passed" << std::endl;
        return false;
    }
    if(observations_.count(kf)){
        return false;
    }
    observations_[kf] = index;

    descriptor_ok_ = false;

    return true;
}

auto LandmarkBase::GetDescriptor()->cv::Mat {
    std::unique_lock<std::mutex> lock(mtx_obs_);
    return descriptor_;
}

auto LandmarkBase::GetFeatureIndex(KeyframePtr kf)->int {
    std::unique_lock<std::mutex> lock(mtx_obs_);
    KfObservations::iterator mit = observations_.find(kf);
    if(mit != observations_.end()) return mit->second;
    else return -1;
}

auto LandmarkBase::GetMaxDistanceInvariance()->precision_t {
    std::unique_lock<std::mutex> lock(mtx_pos_);
    return 1.2*max_distance_;
}

auto LandmarkBase::GetMinDistanceInvariance()->precision_t {
    std::unique_lock<std::mutex> lock(mtx_pos_);
    return 0.8*min_distance_;
}


auto LandmarkBase::GetNormal()->Vector3Type {
    std::unique_lock<std::mutex> lock(mtx_pos_);
    return normal_;
}

auto LandmarkBase::GetObservations()->KfObservations {
    std::unique_lock<std::mutex> lock(mtx_obs_);
    return observations_;
}

auto LandmarkBase::GetReferenceKeyframe()->KeyframePtr {
    std::unique_lock<std::mutex> lock(mtx_obs_);
    return reference_kf_;
}

auto LandmarkBase::GetWorldPos()->Vector3Type {
    std::unique_lock<std::mutex> lock(mtx_pos_);
    return pos_w_;
}

auto LandmarkBase::EraseObservation(KeyframePtr kf)->void {
    std::unique_lock<std::mutex> lock(mtx_obs_);
    KfObservations::iterator mit = observations_.find(kf);
    if(mit != observations_.end())
        observations_.erase(mit);

    if(reference_kf_ = kf) {
        reference_kf_ = nullptr;
        if(!observations_.empty()) {
            reference_kf_ = observations_.begin()->first;
        }
    }

    descriptor_ok_ = false;
}

auto LandmarkBase::IsInvalid()->bool {
    std::unique_lock<std::mutex> lock_inv(mtx_invalid_);
    return invalid_;
}

auto LandmarkBase::PredictScale(const float &currentDist, KeyframePtr kf)->int {
    precision_t ratio;
    {
        std::unique_lock<std::mutex> lock(mtx_pos_);
        ratio = max_distance_/currentDist;
    }
    int nScale = std::ceil(std::log(ratio)/std::log(covins_params::features::scale_factor));
    if(nScale<0)
        nScale = 0;
    else if(nScale>=covins_params::features::num_octaves)
        nScale = covins_params::features::num_octaves-1;

    return nScale;
}

auto LandmarkBase::SetReferenceKeyframe(KeyframePtr kf)->void {
    std::unique_lock<std::mutex> lock(mtx_obs_);
    reference_kf_ = kf;
}

auto LandmarkBase::SetWorldPos(Vector3Type pos_w)->void {
    std::unique_lock<std::mutex> lock(mtx_pos_);
    pos_w_ = pos_w;
}

} //end ns
