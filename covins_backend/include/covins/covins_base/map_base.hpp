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

#pragma once

// C++
#include <mutex>
#include <eigen3/Eigen/Core>

// COVINS
#include <covins/covins_base/config_comm.hpp>
#include <covins/covins_base/config_backend.hpp>
#include <covins/covins_base/typedefs_base.hpp>

namespace covins {

class Keyframe;
class Landmark;

class MapBase {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using precision_t                   = TypeDefs::precision_t;
    using idpair                        = TypeDefs::idpair;

    using TransformType                 = TypeDefs::TransformType;

    using KeyframePtr                   = TypeDefs::KeyframePtr;
    using LandmarkPtr                   = TypeDefs::LandmarkPtr;

    using KeyframeMap                   = TypeDefs::KeyframeMap;
    using LandmarkMap                   = TypeDefs::LandmarkMap;
    using KeyframeVector                = TypeDefs::KeyframeVector;
    using LandmarkVector                = TypeDefs::LandmarkVector;
    using LandmarkSet                   = TypeDefs::LandmarkSet;

public:
    MapBase(size_t id);

    // Getter
    virtual auto GetKeyframe(idpair idp, bool expect_null = false)                      ->KeyframePtr;
    virtual auto GetKeyframe(size_t kf_id, size_t client_id,
                             bool expect_null = false)                                  ->KeyframePtr {
        return GetKeyframe(std::make_pair(kf_id,client_id), expect_null);
    }
    virtual auto GetKeyframes()                                                         ->KeyframeMap;
    virtual auto GetKeyframesVec()                                                      ->KeyframeVector;
    virtual auto GetKeyframesErased()                                                   ->KeyframeMap;
    virtual auto GetLandmark(idpair idp)                                                ->LandmarkPtr;
    virtual auto GetLandmark(size_t lm_id, size_t client_id)                            ->LandmarkPtr {
        return GetLandmark(std::make_pair(lm_id,client_id));
    }
    virtual auto GetLandmarks()                                                         ->LandmarkMap;
    virtual auto GetLandmarksVec()                                                      ->LandmarkVector;

    virtual auto GetMaxKfId()                                                           ->size_t;
    virtual auto GetMaxLmId()                                                           ->size_t;

    // Add / Erase data
    virtual auto AddKeyframe(KeyframePtr kf)                                            ->void      = 0;
    virtual auto AddLandmark(LandmarkPtr lm)                                            ->void      = 0;
    virtual auto EraseKeyframe(KeyframePtr kf, bool mtx_lock = true)                    ->bool      = 0;
    virtual auto EraseLandmark(LandmarkPtr lm, bool mtx_lock = true)                    ->bool      = 0;

    // Clear Map
    virtual auto Clear()                                                                ->void;

    // Sync
    virtual auto LockMapUpdate()                                                        ->void {
        mtx_update_.lock();
    }
    virtual auto UnLockMapUpdate()                                                      ->void {
        mtx_update_.unlock();
    }

    // Identifier
    size_t                      id_map_                                                 = std::numeric_limits<size_t>::max();
    std::set<size_t>            associated_clients_;

protected:
    // Data
    KeyframeMap                 keyframes_;
    LandmarkMap                 landmarks_;

    KeyframeMap                 keyframes_erased_;

    size_t                      max_id_kf_                                              = 0;
    size_t                      max_id_lm_                                              = 0;

    // Sync
    std::mutex                  mtx_map_;
    std::mutex                  mtx_update_;
};

} //end ns
