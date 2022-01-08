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

// COVINS
#include <covins/covins_base/msgs/msg_landmark.hpp>
#include "covins_base/landmark_base.hpp"

// Thirdpary
#include <robopt_open/common/definitions.h>

namespace covins {

class Map;

class Landmark : public LandmarkBase, public std::enable_shared_from_this<Landmark> {
public:
    using MapPtr                        = TypeDefs::MapPtr;

public:
    Landmark(MsgLandmark msg, MapPtr map);
    auto EstablishConnections(MsgLandmark msg, MapPtr map)                              ->void;     // Establishes connections to KFs/Landmarks - cannot call shared_from_this() from constructor!

    // Interfaces
    virtual auto UpdateNormal()                                                         ->void;
    virtual auto SetOptimized()                                                         ->void;
    virtual auto IsOptimized()                                                          ->bool;

    // Landmark updates
    virtual auto ComputeDescriptor()                                                    ->void;

    // Msg interfaces
    virtual auto ConvertToMsg(MsgLandmark &msg,
                             KeyframePtr kf_ref, bool is_update)                        ->void;
    virtual auto ConvertToMsgFileExport(MsgLandmark &msg)                               ->void;
    virtual auto UpdatePosFromMsg(MsgLandmark &msg, MapPtr map)                         ->void;

    // Loop Detection
    idpair                      loop_point_for_kf_                                      = defpair;

    // Ceres Variable Access
    precision_t                 ceres_pos_[robopt::defs::pose::kPositionBlockSize];

    // Map Save/Load
    MsgLandmark                 msg_;

    // debug
    bool                        tofuse_lm_                                              = false;

protected:

    // Interfaces
    friend class Map;
    virtual auto SetInvalid()                                                           ->bool;     // This function should only be called by the map

    bool                        optimized_                                              = false;    // Indicates that this LM was part of an optimization process (important for landmark culling)
};

inline std::ostream& operator<<(std::ostream& out, const Landmark::LandmarkPtr lm) {
    return out << "LM " << lm->id_.first << "|" << lm->id_.second;
}

} //end ns
