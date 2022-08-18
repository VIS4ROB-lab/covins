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
#include <map>
#include <vector>
#include <eigen3/Eigen/Core>

// COVINS
#include "covins_base/keyframe_base.hpp"
#include "covins_base/vocabulary.h"
#include <covins/covins_base/msgs/msg_keyframe.hpp>

namespace covins {

class Map;

class Keyframe : public KeyframeBase, public std::enable_shared_from_this<Keyframe> {
public:
    using MapPtr                        = TypeDefs::MapPtr;
    using VocabularyPtr                 = CovinsVocabulary::VocabularyPtr;
    using KeyframeIntMap                = TypeDefs::KeyframeIntMap;

    struct kf_less{
        auto operator() (const KeyframePtr a, const KeyframePtr b) const                ->bool;
    };

    static auto sort_by_redval(const KeyframePtr a, const KeyframePtr b)                ->bool;

    static auto CompStamp(KeyframePtr kf1, KeyframePtr kf2)                             ->bool;     //greater - newest stamp at beginning of container

public:
    Keyframe(MsgKeyframe msg, MapPtr map, VocabularyPtr voc);

    auto EstablishConnections(MsgKeyframe msg, MapPtr map)                              ->void;     //Establishes connections to KFs/Landmarks - cannot call shared_from_this() from constructor!

    auto EstablishNeighbors(MsgKeyframe msg, MapPtr map)                                ->void;    //Establishes extra neighbor connections for placerecognition
    // Interfaces
    virtual auto RemapLandmark(LandmarkPtr lm,
                               const size_t feat_id_now,
                               const size_t feat_id_new)                                ->void;

    virtual auto SetPoseOptimized()                                                     ->void;
    virtual auto IsPoseOptimized()                                                      ->bool;
    virtual auto SetVelBiasOptimized()                                                  ->void;
    virtual auto IsVelBiasOptimized()                                                   ->bool;
    virtual auto FusePreintegration(PreintegrationPtr preint)                           ->void;

    virtual auto SetErase()                                                             ->void;
    virtual auto SetNotErase()                                                          ->void;

    virtual auto ComputeRedundancyValue()                                               ->double;

    virtual auto GetTimeSpanPredSucc(bool bIgnoreMutex = false)                                 ->precision_t;

    // Msg interfaces
    virtual auto ConvertToMsg(MsgKeyframe &msg,
                             KeyframePtr kf_ref, bool is_update)                        ->void;
    virtual auto ConvertToMsgFileExport(MsgKeyframe &msg)                               ->void;
    virtual auto UpdatePoseFromMsg(MsgKeyframe &msg, MapPtr map)                        ->void;

    // BoW
    DBoW2::BowVector            bow_vec_;
    DBoW2::FeatureVector        feat_vec_;

    // Loop Detection
    int                         loop_words_                                             = 0;
    idpair                      loop_query_                                             = defpair;
    double                      loop_score_                                             = 0.0;
    bool                        is_loop_kf_                                             = false;

    // Image handling
    cv::Mat                     img_;

    // Map Save/Load
    MsgKeyframe                 msg_;

    // Redundancy detection
    double latest_red_val_ = 0.0;

    // Bearing vectors
    std::vector<Vector3Type> bearings_;

    AorsVector                  keypoints_aors_add_;        //Angle,Octave,Response,Size
    KeypointVector              keypoints_distorted_add_;
    KeypointVector              keypoints_undistorted_add_;
    cv::Mat                     descriptors_add_;
    std::vector<Vector3Type>    bearings_add_;

    auto GetDescriptorAddCV(size_t ind) -> cv::Mat;
    const unsigned char *GetDescriptorAdd(size_t ind);

protected:

    // Interfaces
    friend class Map;
    virtual auto SetInvalid()                                                           ->bool;     // This function should only be called by the map

    // Covisibility Graph Functions
    virtual auto UpdateCovisibilityConnections()                                        ->void;     // This function should only be called by the map

    // Connected Neighbor FUnctions
    virtual auto UpdateNeighborConnections()                                            ->void;
    
    // Infrastructure
    bool                        pose_optimized_                                         = false;    // Indicates that this LM was part of an optimization process (important for landmark culling)
    bool                        vel_bias_optimized_                                     = false;
    bool                        not_erase_                                              = false;
};

inline std::ostream& operator<<(std::ostream& out, const Keyframe::KeyframePtr kf) {
    return out << "KF " << kf->id_.first << "|" << kf->id_.second;
}

} //end ns
