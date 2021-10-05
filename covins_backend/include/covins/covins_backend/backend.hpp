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
#include <vector>
#include <thread>

// COVINS
#include <covins/covins_base/config_comm.hpp>
#include <covins/covins_base/config_backend.hpp>
#include <covins/covins_base/typedefs_base.hpp>
#include "covins_base/vocabulary.h"

// Thirdparty
#include <ros/ros.h>
#include "covins_backend/ServiceGBA.h"
#include "covins_backend/ServiceSaveMap.h"
#include "covins_backend/ServiceLoadMap.h"
#include "covins_backend/ServicePruneMap.h"

// Socket Programming
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <atomic>

namespace covins {

class AgentHandler;
class Map;
class MapManager;
class Visualizer;

class AgentPackage : public std::enable_shared_from_this<AgentPackage> {
public:
    using MapPtr                        = TypeDefs::MapPtr;
    using ManagerPtr                    = TypeDefs::ManagerPtr;
    using HandlerPtr                    = std::shared_ptr<AgentHandler>;
    using VisPtr                        = TypeDefs::VisPtr;

public:
    AgentPackage(size_t client_id, int newfd, VisPtr vis, ManagerPtr man);

protected:
    HandlerPtr agent_;
};

class CovinsBackend {
public:
    using AgentPtr                      = std::shared_ptr<AgentPackage>;
    using AgentVector                   = std::vector<AgentPtr>;
    using ManagerPtr                    = TypeDefs::ManagerPtr;
    using VisPtr                        = TypeDefs::VisPtr;
    using ThreadPtr                     = TypeDefs::ThreadPtr;
    using VocabularyPtr                 = CovinsVocabulary::VocabularyPtr;

public:
    CovinsBackend();
    auto Run()                                                                          ->void;

    // Service Interfaces
    auto CallbackGBA(covins_backend::ServiceGBA::Request &req,
                     covins_backend::ServiceGBA::Response &res)                         ->bool;

    auto CallbackSaveMap(covins_backend::ServiceSaveMap::Request &req,
                         covins_backend::ServiceSaveMap::Response &res)                 ->bool;

    auto CallbackLoadMap(covins_backend::ServiceLoadMap::Request &req,
                               covins_backend::ServiceLoadMap::Response &res)           ->bool;

    auto CallbackPruneMap(covins_backend::ServicePruneMap::Request &req,
                         covins_backend::ServicePruneMap::Response &res)                ->bool;

protected:
    auto AddAgent()                                                                     ->void;
    auto AcceptAgent()                                                                  ->void;
    auto ConnectSocket()                                                                ->void;
    auto LoadVocabulary()                                                               ->void;

    auto add_counter()                                                                  ->void;
    auto sub_counter()                                                                  ->void;
    auto disp_counter()                                                                 ->void;
    auto get_counter()                                                                  ->int;

    // Infrastructure
    AgentVector                 agents_;
    ManagerPtr                  mapmanager_;
    VisPtr                      vis_;
    VocabularyPtr               voc_;

    ThreadPtr                   thread_mapmanager_;
    ThreadPtr                   thread_vis_;

    int                         agent_next_id_                                          = 0;

    ros::NodeHandle             nh_;
    ros::ServiceServer          service_gba_;
    ros::ServiceServer          service_savemap_;
    ros::ServiceServer          service_loadmap_;
    ros::ServiceServer          service_prune_;

    fd_set                      master_;
    fd_set                      read_fds_;
    int                         listener_, newfd_;

    // Device Counter
    std::atomic<int>            counter_, overall_counter_;

    // Sync
    std::mutex                  mtx_num_agents_;
};

} //end ns
