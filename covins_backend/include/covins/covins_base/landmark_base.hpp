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
#include <memory>
#include <mutex>
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>

// COVINS
#include <covins/covins_base/config_comm.hpp>
#include <covins/covins_base/config_backend.hpp>
#include <covins/covins_base/typedefs_base.hpp>

namespace covins {

class Keyframe;
class Landmark;

struct MsgLandmark;

class LandmarkBase {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using precision_t                   = TypeDefs::precision_t;
    using idpair                        = TypeDefs::idpair;
    using Vector3Type                   = TypeDefs::Vector3Type;
    using Matrix3Type                   = TypeDefs::Matrix3Type;
    using TransformType                 = TypeDefs::TransformType;

    using KeyframePtr                   = TypeDefs::KeyframePtr;
    using LandmarkPtr                   = TypeDefs::LandmarkPtr;
    using KfObservations                = TypeDefs::KfObservations;

    static auto CompStamp(KeyframePtr kf1, KeyframePtr kf2)                             ->bool;     //greater - newest stamp at beginning of container

public:
    LandmarkBase(idpair id);
    LandmarkBase(idpair id,Vector3Type pos_w, KeyframePtr kf_ref);

    // Msg interfaces
    virtual auto ConvertToMsg(MsgLandmark &msg,
                             KeyframePtr kf_ref, bool is_update)                        ->void      = 0;

    // Interfaces
    virtual auto GetWorldPos()                                                          ->Vector3Type;
    virtual auto SetWorldPos(Vector3Type pos_w)                                         ->void;
    virtual auto GetObservations()                                                      ->KfObservations;
    virtual auto IsInvalid()                                                            ->bool;
    virtual auto GetReferenceKeyframe()                                                 ->KeyframePtr;
    virtual auto SetReferenceKeyframe(KeyframePtr kf)                                   ->void;
    virtual auto GetFeatureIndex(KeyframePtr kf)                                        ->int;
    virtual auto GetNormal()                                                            ->Vector3Type;
    virtual auto GetDescriptor()                                                        ->cv::Mat;

    virtual auto AddObservation(KeyframePtr kf, size_t index,
                                bool suppress_ref_check = false)                        ->bool;
    virtual auto EraseObservation(KeyframePtr kf)                                       ->void;

    virtual auto PredictScale(const float &currentDist, KeyframePtr kf)                 ->int;
    virtual auto GetMinDistanceInvariance()                                             ->precision_t;
    virtual auto GetMaxDistanceInvariance()                                             ->precision_t;

    // Identifier
    idpair                      id_;

    // These vars should not be acessed in parallel (TODO: anyway, guard them later)
    bool                        is_loaded_                                              = false;
    bool                        is_gba_optimized_                                       = false;

protected:
    // Position
    Vector3Type                 pos_w_;

    // Best descriptor
    cv::Mat                     descriptor_;
    bool                        descriptor_ok_                                          = false;

    // Mean viewing direction
    Vector3Type                 normal_                                                 = Vector3Type::Zero();

    // Scale invariance distances
    precision_t                 min_distance_                                           = 0;
    precision_t                 max_distance_                                           = 0;

    // Neighborhood
    KfObservations              observations_;
    KeyframePtr                 reference_kf_;

    // Infrastructure
    bool                        invalid_                                                = false;

    // Mutexes
    std::mutex                  mtx_obs_;
    std::mutex                  mtx_pos_;
    std::mutex                  mtx_invalid_;
};

} //end ns
