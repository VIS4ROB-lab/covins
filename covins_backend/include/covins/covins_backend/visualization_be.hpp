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
#include "covins_base/visualization_base.hpp"
#include "covins_backend/keyframe_be.hpp"

// Thirdparty
#include <ros/ros.h>

namespace covins {

class Visualizer : public VisualizerBase, public std::enable_shared_from_this<Visualizer> {
public:
    using KeyframeSetById               = std::set<KeyframePtr,Keyframe::kf_less,Eigen::aligned_allocator<KeyframePtr>>;

public:
    Visualizer(std::string topic_prefix = std::string());

    // Main
    virtual auto Run()                                                                  ->void override;

    // Interfaces
    virtual auto DrawMap(MapPtr map)                                                    ->void;

    // Draw Loaded Map
    auto DrawMapBitByBit(MapPtr map, std::string frame)                                 ->void;

protected:
    // Draw Map
    virtual auto PubCovGraph()                                                          ->void;
    virtual auto PubKeyframesAsFrusta()                                                 ->void;
    virtual auto PubLandmarksAsCloud()                                                  ->void;
    virtual auto PubLoopEdges()                                                         ->void;
    virtual auto PubTrajectories()                                                      ->void;
};

} //end ns
