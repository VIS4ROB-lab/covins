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

#include "covins_backend/handler_be.hpp"

// COVINS
#include "covins_backend/communicator_be.hpp"
#include "covins_backend/map_be.hpp"
#include "covins_backend/placerec_be.hpp"
#include "covins_backend/placerec_gen_be.hpp"

namespace covins {

AgentHandler::AgentHandler(int client_id, int newfd, VisPtr vis, ManagerPtr man)
    : client_id_(client_id),
      mapmanager_(man)
{
    mapmanager_->InitializeMap(client_id);

    // Other types of place recognition system could be integrated and activated using the placerec::type parameter
    if(covins_params::placerec::type == "COVINS") {
        placerec_.reset(new PlaceRecognition(mapmanager_,covins_params::opt::perform_pgo));
    } else if (covins_params::placerec::type == "COVINS_G") {
        placerec_.reset(new PlaceRecognitionG(mapmanager_,covins_params::opt::perform_pgo));
    } else {
        std::cout << COUTFATAL << "Place Recognition System Type \"" << covins_params::placerec::type << "\" not valid" << std::endl;
        exit(-1);
    }

    comm_.reset(new Communicator(client_id_,newfd,man,vis,placerec_));

    thread_placerec_.reset(new std::thread(&PlacerecBase::Run,placerec_));
    thread_placerec_->detach(); // Thread will be cleaned up when exiting main()

    thread_comm_.reset(new std::thread(&Communicator::Run,comm_));
    thread_comm_->detach(); // Thread will be cleaned up when exiting main()
}

} //end ns
