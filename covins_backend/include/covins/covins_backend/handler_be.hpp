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
#include <thread>

// COVINS
#include <covins/covins_base/config_comm.hpp>
#include <covins/covins_base/config_backend.hpp>

namespace covins {

class Communicator;
class Map;
class MapManager;
class PlacerecBase;
class Visualizer;

class AgentHandler : public std::enable_shared_from_this<AgentHandler> {
public:
    using CommPtr                       = TypeDefs::CommPtr;
    using MapPtr                        = TypeDefs::MapPtr;
    using ManagerPtr                    = TypeDefs::ManagerPtr;
    using PlacerecPtr                   = TypeDefs::PlacerecPtr;
    using VisPtr                        = TypeDefs::VisPtr;
    using ThreadPtr                     = TypeDefs::ThreadPtr;

public:
    AgentHandler(int client_id, int newfd, VisPtr vis, ManagerPtr man);

protected:
    // Infrastructure
    int                         client_id_;
    CommPtr                     comm_;
    ManagerPtr                  mapmanager_;
    PlacerecPtr                 placerec_;
    ThreadPtr                   thread_comm_;
    ThreadPtr                   thread_placerec_;
};

} //end ns
