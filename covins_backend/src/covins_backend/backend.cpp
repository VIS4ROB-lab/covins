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

#include "covins_backend/backend.hpp"

// COVINS
#include <covins/covins_backend/communicator_be.hpp>
#include "covins_backend/handler_be.hpp"
#include "covins_backend/map_be.hpp"
#include "covins_backend/optimization_be.hpp"
#include "covins_backend/visualization_be.hpp"
#include "covins_backend/placerec_be.hpp"
#include "covins_backend/placerec_gen_be.hpp"

// Profiling
#include <valgrind/callgrind.h>

namespace covins {

AgentPackage::AgentPackage(size_t client_id, int newfd, VisPtr vis, ManagerPtr man) {
    agent_.reset(new AgentHandler(client_id,newfd,vis,man));
}

CovinsBackend::CovinsBackend() {

    //+++++ Parameters +++++
    covins_params::ShowParamsComm();
    covins_params::ShowParamsBackend();
    if(!(covins_params::sys::trajectory_format == "TUM" || covins_params::sys::trajectory_format == "EUROC")) {
        std::cout << COUTFATAL << "trajectory_format '" << covins_params::sys::trajectory_format << "' not in { EUROC | TUM }" << std::endl;
        exit(-1);
    }

    //+++++ Load Vocabulary +++++
    this->LoadVocabulary();

    //+++++ Create MapManager +++++
    mapmanager_.reset(new MapManager(voc_));
    thread_mapmanager_.reset(new std::thread(&MapManager::Run,mapmanager_));
    thread_mapmanager_->detach(); // Thread will be cleaned up when exiting main()

    service_gba_ = nh_.advertiseService("covins_gba",&CovinsBackend::CallbackGBA, this);
    service_savemap_ = nh_.advertiseService("covins_savemap",&CovinsBackend::CallbackSaveMap, this);
    service_loadmap_ = nh_.advertiseService("covins_loadmap",&CovinsBackend::CallbackLoadMap, this);
    service_prune_ = nh_.advertiseService("covins_prunemap",&CovinsBackend::CallbackPruneMap, this);

    //+++++ Create Viewer +++++
    if(covins_params::vis::active){
        vis_.reset(new Visualizer("_be"));
        thread_vis_.reset(new std::thread(&Visualizer::Run,vis_));
        thread_vis_->detach(); // Thread will be cleaned up when exiting main()
    }
}

auto CovinsBackend::AcceptAgent()->void {
    socklen_t addrlen;
    struct sockaddr_storage remoteaddr; // client address
    char remoteIP[INET6_ADDRSTRLEN];
    struct timeval tv;
    int temp;

    if (listen(listener_, 10) == -1) {
        perror("listen");
        exit(3);
    }

    // add the listener to the master set
    FD_SET(listener_, &master_);

    // main loop
    for(;;) {
        read_fds_ = master_; // copy it
        tv.tv_sec = 5;
        tv.tv_usec = 0;


        if ((temp = select(listener_ + 1, &read_fds_, NULL, NULL, &tv)) == -1) {
            perror("select");
            exit(4);
        }

        if (FD_ISSET(listener_, &read_fds_)) {
            addrlen = sizeof remoteaddr;
            if ((newfd_ = accept(listener_, (struct sockaddr *) &remoteaddr, &addrlen)) == -1) {
                perror("accept");
            } else {
                printf("selectserver: new connection from %s on socket %d\n", inet_ntop(remoteaddr.ss_family, CommunicatorBase::GetInAddr((struct sockaddr *) &remoteaddr), remoteIP, INET6_ADDRSTRLEN), newfd_);
            }

            //Creating new threads for every agent
            AgentPtr agent{new AgentPackage(agent_next_id_++,newfd_,vis_,mapmanager_)};
            agents_.push_back(agent);
        }
        usleep(100);
    }
}

auto CovinsBackend::AddAgent()->void {
    //connect
    this->ConnectSocket();

    //listen and accept
    this->AcceptAgent();
}

auto CovinsBackend::add_counter()->void {
    counter_++;
    overall_counter_++;
}

auto CovinsBackend::CallbackGBA(covins_backend::ServiceGBA::Request &req, covins_backend::ServiceGBA::Response &res)->bool {
    int map_id = req.map_id;
    int action = req.action;
    std::cout << "Service: Request GBA for Map-ID " << map_id << std::endl;
    std::cout << "--> Get map" << std::endl;
    int check_num_map;
    MapManager::MapPtr map = mapmanager_->CheckoutMapExclusiveOrWait(map_id,check_num_map);
    std::cout << "----> Done" << std::endl;
    if(action < 100) std::cout << "--> Start GBA" << std::endl;
    if(action == 100) std::cout << "--> Start PGO" << std::endl;
    if(action == 0) {
        std::cout << COUTPURPLE(COVINS GBA) << ": Visual-Inertial" << std::endl;
        std::cout << "Outlier Rejection:            NO" << std::endl;
        Optimization::GlobalBundleAdjustment(map,covins_params::opt::gba_iteration_limit,-1.0,false,false,false);
    } else if (action == 1) {
        std::cout << COUTPURPLE(COVINS GBA) << ": Visual-Inertial" << std::endl;
        std::cout << "Outlier Rejection:            YES" << std::endl;
        Optimization::GlobalBundleAdjustment(map,covins_params::opt::gba_iteration_limit,-1.0,false,true,false);
    } else if (action == 4) {
        std::cout << COUTPURPLE(COVINS GBA) << ": Visual" << std::endl;
        std::cout << "Outlier Rejection:            NO" << std::endl;
        Optimization::GlobalBundleAdjustment(map,covins_params::opt::gba_iteration_limit,-1.0,true,false,false);
    } else if (action == 5) {
        std::cout << COUTPURPLE(COVINS GBA) << ": Visual" << std::endl;
        std::cout << "Outlier Rejection:            YES" << std::endl;
        Optimization::GlobalBundleAdjustment(map,covins_params::opt::gba_iteration_limit,-1.0,true,true,false);
    } else if (action == 100) {
        Optimization::PoseMap corrected_poses;
        Optimization::PoseGraphOptimization(map,corrected_poses);
    } else {
        std::cout << COUTERROR << "action " << action << " is not supported" << std::endl;
    }
    std::cout << "----> Done" << std::endl;
    map->WriteKFsToFile();
    if(covins_params::vis::active) {
        usleep(100000);
        std::cout << "--> Update Covisbility Connections" << std::endl;
        Keyframe::KeyframeVector all_kfs = map->GetKeyframesVec();
        for(const auto& kf : all_kfs)
            map->UpdateCovisibilityConnections(kf->id_);
        std::cout << "--> Display Map" << std::endl;
        vis_->DrawMap(map);
        std::cout << "----> Done" << std::endl;
    }
    std::cout << "--> Return map" << std::endl;
    mapmanager_->ReturnMap(map_id,check_num_map);
    std::cout << "----> Done" << std::endl;
    return true;
}

auto CovinsBackend::CallbackLoadMap(covins_backend::ServiceLoadMap::Request &req, covins_backend::ServiceLoadMap::Response &res)->bool {
    std::cout << "Service: Load Map" << std::endl;
    int action = req.action;
    bool perform_placerec = false;
    bool perform_pgo = false;
    if(action == 0) {
        std::cout << COUTPURPLE(Map Load) << std::endl;
    } else if (action == 1) {
        std::cout << COUTPURPLE(Map Load) << ": PlaceRec; NO PGO" << std::endl;
        perform_placerec = true;
        perform_pgo = false;
    } else if (action == 2) {
        std::cout << COUTPURPLE(Map Load) << ": PlaceRec + PGO" << std::endl;
        perform_placerec = true;
        perform_pgo = true;
    } else {
        std::cout << COUTERROR << "action " << action << " is not supported" << std::endl;
        return false;
    }

    if(agent_next_id_ > 0) {
        // The OS version of COVINS is only tested for the case of loading a map at the beginning of the mission, before the first agent is registered
        std::cout << COUTERROR << "Already " << agent_next_id_ << " agent(s) registered -- map need to be loaded before registering the first agent" << std::endl;
        return false;
    }

    std::stringstream filepath;
    if(agent_next_id_ == 0) {
        filepath << covins_params::sys::map_path0;
    } else if(agent_next_id_ == 1) {
        filepath << covins_params::sys::map_path1;
    } else if(agent_next_id_ == 2) {
        filepath << covins_params::sys::map_path2;
    } else if(agent_next_id_ == 3) {
        filepath << covins_params::sys::map_path3;
    } else if(agent_next_id_ == 4) {
        filepath << covins_params::sys::map_path4;
    } else if(agent_next_id_ == 5) {
        filepath << covins_params::sys::map_path5;
    } else if(agent_next_id_ == 6) {
        filepath << covins_params::sys::map_path6;
    } else if(agent_next_id_ == 7) {
        filepath << covins_params::sys::map_path7;
    } else if(agent_next_id_ == 8) {
        filepath << covins_params::sys::map_path8;
    } else if(agent_next_id_ == 9) {
        filepath << covins_params::sys::map_path9;
    } else if(agent_next_id_ == 10) {
        filepath << covins_params::sys::map_path10;
    } else if(agent_next_id_ == 11) {
        filepath << covins_params::sys::map_path11;
    } else {
        std::cout << COUTFATAL << "can only load " << agent_next_id_ << " maps..." << std::endl;
        exit(-1);
    }
    std::cout << "----> Load map from: " << filepath.str() << std::endl;
    AgentPackage::MapPtr map(new Map(agent_next_id_));
    map->LoadFromFile(filepath.str(),voc_);
    agent_next_id_ += map->associated_clients_.size();
    std::cout << "--> Register to Map Manager" << std::endl;
    mapmanager_->RegisterMap(map,!perform_placerec);
    std::cout << "----> Done" << std::endl;

    if(covins_params::vis::active) {
        usleep(100000);
        std::cout << "--> Display Map" << std::endl;
        vis_->DrawMap(map);
        std::cout << "----> Done" << std::endl;
    }
    if(perform_placerec) {
        std::cout << "--> Place Recognition" << std::endl;
        std::cout << "----> Spawn PlaceRec Thread" << std::endl;

        AgentHandler::PlacerecPtr placerec;
        ThreadPtr thread_placerec;

        // Other types of place recognition system could be integrated and activated using the placerec::type parameter
        if(covins_params::placerec::type == "COVINS") {
            placerec.reset(new PlaceRecognition(mapmanager_,perform_pgo));
            thread_placerec.reset(new std::thread(&PlacerecBase::Run,placerec));
        } else if (covins_params::placerec::type == "COVINS_G") {
            placerec.reset(new PlaceRecognitionG(mapmanager_,perform_pgo));
            thread_placerec.reset(new std::thread(&PlacerecBase::Run,placerec));
        }
        else {
            std::cout << COUTFATAL << "Place Recognition System Type \"" << covins_params::placerec::type << "\" not valid" << std::endl;
            exit(-1);
        }

        std::cout << "----> Run PlaceRec" << std::endl;
        auto kfs = map->GetKeyframesVec();
        std::sort(kfs.begin(),kfs.end(),Keyframe::CompStamp);
        std::reverse(kfs.begin(),kfs.end());
        for(auto kf : kfs) {
            placerec->InsertKeyframe(kf);
            usleep(1000);
        }
        while(placerec->CheckBufferExt()) {
            usleep(10000);
        }
        std::cout << "----> Close PlaceRec" << std::endl;
        placerec->SetFinish();
        std::cout << "------> Join Thread" << std::endl;
        thread_placerec->join();
        std::cout << "----> Done" << std::endl;
    }
    if(covins_params::vis::active) {
        usleep(100000);
        std::cout << "--> Display Map" << std::endl;
        std::cout << "----> Get Map" << std::endl;
        int check_num_map;
        auto map_processed = mapmanager_->CheckoutMapOrWait(map->id_map_,check_num_map);
        std::cout << "----> Show Map" << std::endl;
        vis_->DrawMap(map_processed);
        std::cout << "----> Write KFs" << std::endl;
        map->WriteKFsToFile();
        std::cout << "----> Return Map" << std::endl;
        mapmanager_->ReturnMap(map->id_map_,check_num_map);
        std::cout << "----> Done" << std::endl;
    }

    return true;
}

auto CovinsBackend::CallbackPruneMap(covins_backend::ServicePruneMap::Request &req, covins_backend::ServicePruneMap::Response &res)->bool {
    int map_id = req.map_id;
    int max_num_kfs = req.max_num_kfs;
    std::cout << "Service: Request Map Pruning for Map-ID " << map_id << std::endl;
    std::cout << "--> Get map" << std::endl;
    int check_num_map;
    MapManager::MapPtr map = mapmanager_->CheckoutMapExclusiveOrWait(map_id,check_num_map);
    std::cout << "----> Done" << std::endl;
    if(max_num_kfs == 0) {
        std::cout << COUTPURPLE(Redundancy Removal) << ": Prune by Threshold" << std::endl;
        std::cout << "Redundancy Threshold:         " << covins_params::mapping::kf_culling_th_red << std::endl;
        size_t rem_kfs = map->RemoveRedundantData(mapmanager_,covins_params::mapping::kf_culling_th_red);
        std::cout << "--> Removed " << rem_kfs << " KFs" << std::endl;
    } else if(max_num_kfs > 0) {
        std::cout << COUTPURPLE(Redundancy Removal) << ": Max Num KFs" << std::endl;
        std::cout << "Target KF count:              " << max_num_kfs << std::endl;
        size_t rem_kfs = map->RemoveRedundantData(mapmanager_,covins_params::mapping::kf_culling_th_red,max_num_kfs);
        std::cout << "--> Removed " << rem_kfs << " KFs" << std::endl;
    } else {
        std::cout << COUTERROR << "value " << max_num_kfs << " is not supported" << std::endl;
        return false;
    }
    std::cout << "----> Done" << std::endl;
    std::cout << "KFs in map: " << map->GetKeyframes().size() << std::endl;
    if(covins_params::vis::active) {
        usleep(100000);
        std::cout << "--> Display Map" << std::endl;
        vis_->DrawMap(map);
        std::cout << "----> Done" << std::endl;
    }
    std::cout << "--> Return map" << std::endl;
    mapmanager_->ReturnMap(map_id,check_num_map);
    std::cout << "----> Done" << std::endl;
    return true;
}

auto CovinsBackend::CallbackSaveMap(covins_backend::ServiceSaveMap::Request &req, covins_backend::ServiceSaveMap::Response &res)->bool {
    int map_id = req.map_id;
    std::cout << "Service: Save Map for Map-ID " << map_id << std::endl;
    std::stringstream filepath;
    filepath << covins_params::sys::output_dir << "map_data/";
    std::cout << "----> saving map to: " << filepath.str() << std::endl;
    std::cout << "--> Get map" << std::endl;
    int check_num_map;
    MapManager::MapPtr map = mapmanager_->CheckoutMapExclusiveOrWait(map_id,check_num_map);
    std::cout << "----> Done" << std::endl;
    if(!map) {
        std::cout << COUTERROR << "No map found with ID " << map_id << std::endl;
        return false;
    }
    map->SaveToFile(filepath.str());
    std::cout << "--> Return map" << std::endl;
    mapmanager_->ReturnMap(map_id,check_num_map);
    std::cout << "----> Done" << std::endl;
    return true;
}

auto CovinsBackend::ConnectSocket()->void {
    struct addrinfo hints, *ai, *p;
    int rv;
    int yes = 1;

    FD_ZERO(&master_);    // clear the master and temp sets
    FD_ZERO(&read_fds_);

    // get us a socket and bind it
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;
    if ((rv = getaddrinfo(NULL, covins_params::sys::port.c_str(), &hints, &ai)) != 0) {
        fprintf(stderr, "selectserver: %s\n", gai_strerror(rv));
        exit(1);
    }

    for (p = ai; p != NULL; p = p->ai_next) {
        listener_ = socket(p->ai_family, p->ai_socktype, p->ai_protocol);
        if (listener_ < 0) {
            continue;
        }

        // lose the pesky "address already in use" error message
        setsockopt(listener_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));

        if (bind(listener_, p->ai_addr, p->ai_addrlen) < 0) {
            //close(listener);
            continue;
        }

        break;
    }

    // if we got here, it means we didn't get bound
    if (p == NULL) {
        fprintf(stderr, "selectserver: failed to bind\n");
        exit(2);
    }

    freeaddrinfo(ai); // all done with this
}

auto CovinsBackend::disp_counter()->void {
    std::cout << "Counter: " << counter_ << " / " << overall_counter_ << std::endl;
}

auto CovinsBackend::get_counter()->int {
    return counter_;
}

auto CovinsBackend::LoadVocabulary()->void {
    bool vocload = false;
    if(covins_params::features::type == "ORB" || covins_params::features::type == "SIFT")
    {
        std::cout << endl << "Loading ORB Vocabulary. This could take a while..." << std::endl;
        voc_.reset(new CovinsVocabulary::Vocabulary());
        vocload = voc_->loadFromTextFile(covins_params::sys::voc_orb_dir);
    } else {
            std::cout << COUTFATAL << "Feature type given: " << covins_params::features::type << std::endl;
            std::cout << "Supported feature types: 'ORB' " << std::endl;
            exit(-1);
    }
    if(!vocload)
    {
        std::cerr << "Wrong path to vocabulary. " << std::endl;
        exit(-1);
    }
    std::cout << "Vocabulary loaded!" << std::endl << std::endl;
    CALLGRIND_START_INSTRUMENTATION;
}

auto CovinsBackend::Run()->void {
    //+++++ Add the 1st agent +++++
    this->AddAgent();
}

auto CovinsBackend::sub_counter()->void {
    if(counter_==0)
        std::cout << "ERROR: No Device is connected (sub_device() not possible)." << std::endl;
    else
        counter_--;
}

} //end ns
