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
#include <covins/covins_base/communicator_base.hpp>
#include <covins/covins_base/config_comm.hpp>
#include <covins/covins_base/config_backend.hpp>

namespace covins {

class Map;
class MapManager;
class PlacerecBase;
class Visualizer;

class Communicator : public CommunicatorBase, public std::enable_shared_from_this<Communicator> {
public:
    using MapPtr                        = TypeDefs::MapPtr;
    using ManagerPtr                    = TypeDefs::ManagerPtr;
    using PlacerecPtr                   = TypeDefs::PlacerecPtr;
    using VisPtr                        = TypeDefs::VisPtr;

    using KeyframeList                  = TypeDefs::KeyframeList;
    using LandmarkList                  = TypeDefs::LandmarkList;

public:
    Communicator(int client_id, int newfd, ManagerPtr man,
                 VisPtr vis, PlacerecPtr placerec);

    //main function
    virtual auto Run()                                                                  ->void;

protected:
    // Find data to send to client
    virtual auto CollectDataForAgent()                                                  ->void;

    // data handling
    virtual auto ProcessAdditional()                                                    ->void;
    virtual auto ProcessKeyframeMessages()                                              ->void;
    virtual auto ProcessLandmarkMessages()                                              ->void;
    virtual auto ProcessNewKeyframes()                                                  ->void;
    virtual auto ProcessNewLandmarks()                                                  ->void;

    // LM Culling
    auto LandmarkCulling(size_t min_obs, size_t max_gap)                                ->int;
    auto KeyframeCulling(double th_red, int recent_window_size)                         ->void;

    // Infrastructure
    MapPtr                      map_                                                    = nullptr;
    ManagerPtr                  mapmanager_                                             = nullptr;
    VisPtr                      vis_                                                    = nullptr;
    PlacerecPtr                 placerec_                                               = nullptr;

    //data
    idpair                      most_recent_kf_id_                                      = defpair;

    KeyframeList                keyframes_new_;
    LandmarkList                landmarks_new_;

    idpair                      last_processed_kf_msg_                                  = defpair;

    // LM Culling
    LandmarkList                recent_landmarks_;
    KeyframeList                recent_keyframes_;

};

} //end ns
