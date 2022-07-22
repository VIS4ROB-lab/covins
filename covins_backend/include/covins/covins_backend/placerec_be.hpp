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
#include <random>
#include <set>
#include <list>
#include <vector>

#include <eigen3/Eigen/Core>

// COVINS
#include <covins/covins_base/typedefs_base.hpp>
#include <covins/covins_base/config_comm.hpp>
#include <covins/covins_base/config_backend.hpp>
#include "covins_base/vocabulary.h"
#include "covins_base/placerec_base.hpp"

namespace covins {

class Keyframe;
class Landmark;
class Map;
class MapManager;

class PlaceRecognition : public PlacerecBase {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using precision_t                   = TypeDefs::precision_t;
    using idpair                        = TypeDefs::idpair;

    using TransformType                 = TypeDefs::TransformType;
    using Matrix3Type                   = TypeDefs::Matrix3Type;
    using Vector3Type                   = TypeDefs::Vector3Type;

    using KeyframePtr                   = TypeDefs::KeyframePtr;
    using LandmarkPtr                   = TypeDefs::LandmarkPtr;
    using MapPtr                        = TypeDefs::MapPtr;
    using ManagerPtr                    = TypeDefs::ManagerPtr;
    using VocabularyPtr                 = CovinsVocabulary::VocabularyPtr;

    using KeyframeBufferType            = TypeDefs::KeyframeList;
    using KeyframeVector                = TypeDefs::KeyframeVector;
    using KeyframeSet                   = TypeDefs::KeyframeSet;
    using LandmarkVector                = TypeDefs::LandmarkVector;
    using LandmarkSet                   = TypeDefs::LandmarkSet;
    using PoseMap                       = TypeDefs::PoseMap;

    using RdevPtr                       = std::shared_ptr<std::random_device>;
    using MersennePtr                   = std::shared_ptr<std::mt19937>;
    using DistPtr                       = std::shared_ptr<std::normal_distribution<double>>;

    using ConsistentGroup               = std::pair<KeyframeSet,int>;
    using vecConsistentGroup            = std::vector<ConsistentGroup, Eigen::aligned_allocator<ConsistentGroup>>;
    using vecVecMP                      = std::vector<LandmarkVector>;

    using GroundtruthData               = std::vector<std::vector<double>>;

public:
    PlaceRecognition(ManagerPtr man, bool perform_pgo = true);

    // Main
    virtual auto Run()                                                                  ->void override;

    // Interfaces
    virtual auto InsertKeyframe(KeyframePtr kf)                                         ->void override;
    virtual auto CheckBufferExt()                                                       ->bool override {
        return CheckBuffer();}

    // Synchronization
    auto SetFinish()                                                                    ->void override {
        std::unique_lock<std::mutex> lock(mtx_finish_); finish_ = true;}
    auto ShallFinish()                                                                  ->bool override {
        std::unique_lock<std::mutex> lock(mtx_finish_); return finish_;}
    virtual auto IsFinished()                                                           ->bool override{
        std::unique_lock<std::mutex> lock(mtx_finish_); return is_finished_;}

protected:
    virtual auto CheckBuffer()                                                          ->bool;
    virtual auto DetectLoop()                                                           ->bool;
    virtual auto ComputeSE3()                                                           ->bool;
    virtual auto CorrectLoop()                                                          ->bool;
    virtual auto ConnectLoop(KeyframePtr kf_query, KeyframePtr kf_match,
                     TransformType T_smatch_squery,
                     PoseMap &corrected_poses, MapPtr map)                              ->void;

    // Loop Fusion
    virtual auto FuseLandmark(LandmarkPtr lm_target,
                             LandmarkPtr lm_tofuse, MapPtr map)                         ->void;

    // Infrastructure
    ManagerPtr                  mapmanager_;
    bool                        perform_pgo_                                            = true;
    VocabularyPtr               voc_;

    map<size_t,size_t>          last_loops_;

    // Data
    KeyframeBufferType          buffer_kfs_in_;

    KeyframePtr                 kf_query_;
    KeyframePtr                 kf_match_;
    
    precision_t                 mnCovisibilityConsistencyTh;
    vecConsistentGroup          mvConsistentGroups;
    KeyframeVector              mvpEnoughConsistentCandidates;
    KeyframeVector              mvpCurrentConnectedKFs;
    LandmarkVector              mvpCurrentMatchedPoints;
    LandmarkVector              mvpLoopMapPoints;
    TransformType               mTcw;
    TransformType               mTsw;
    TypeDefs::Matrix6Type       mcov_mat;
    double                      mrelative_yaw = -1.0;
    double                      mrelative_yaw_gt = -1.0;
    
    // Sync
    std::mutex                  mtx_in_;
    std::mutex                  mtx_finish_;

    bool                        finish_                                                 = false;
    bool is_finished_ = false;

    // GT Data
    auto GetPoseTwsGT(KeyframePtr kf)                                   ->TransformType;
    std::map<int, GroundtruthData> gt_;
    TransformType           mT_smatch_squery_gt;
};

} //end ns
