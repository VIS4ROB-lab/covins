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

#include "covins_base/map_base.hpp"

namespace covins {

MapBase::MapBase(size_t id)
    : id_map_(id)
{
    if(id_map_ > 1000) {
        std::cout << COUTFATAL << "Map initialized with ID " << id_map_ << std::endl;
        exit(-1);
    }
}

auto MapBase::Clear()->void {
    keyframes_.clear();
    landmarks_.clear();
    max_id_kf_ = 0;
    max_id_lm_ = 0;
}

auto MapBase::GetKeyframe(idpair idp, bool expect_null)->KeyframePtr {
    std::unique_lock<std::mutex> lock(mtx_map_);
    KeyframeMap::iterator mit = keyframes_.find(idp);
    if(mit != keyframes_.end()) return mit->second;
    else {
        return nullptr;
    }
}

auto MapBase::GetKeyframes()->KeyframeMap {
    std::unique_lock<std::mutex> lock(mtx_map_);
    return keyframes_;
}

auto MapBase::GetKeyframesErased()->KeyframeMap {
    std::unique_lock<std::mutex> lock(mtx_map_);
    return keyframes_erased_;
}

auto MapBase::GetKeyframesVec()->KeyframeVector {
    std::unique_lock<std::mutex> lock(mtx_map_);
    KeyframeVector kfs;
    for(KeyframeMap::iterator mit = keyframes_.begin();mit!=keyframes_.end();++mit)
        kfs.push_back(mit->second);
    return kfs;
}

auto MapBase::GetLandmark(idpair idp)->LandmarkPtr {
    std::unique_lock<std::mutex> lock(mtx_map_);
    LandmarkMap::iterator mit = landmarks_.find(idp);
    if(mit != landmarks_.end()) return mit->second;
    else {
        return nullptr;
    }
}

auto MapBase::GetLandmarks()->LandmarkMap {
    std::unique_lock<std::mutex> lock(mtx_map_);
    return landmarks_;
}

auto MapBase::GetLandmarksVec()->LandmarkVector {
    std::unique_lock<std::mutex> lock(mtx_map_);
    LandmarkVector lms;
    for(LandmarkMap::iterator mit = landmarks_.begin();mit!=landmarks_.end();++mit)
        lms.push_back(mit->second);
    return lms;
}

auto MapBase::GetMaxKfId()->size_t {
    std::unique_lock<std::mutex> lock(mtx_map_);
    return max_id_kf_;
}

auto MapBase::GetMaxLmId()->size_t {
    std::unique_lock<std::mutex> lock(mtx_map_);
    return max_id_lm_;
}

} //end ns
