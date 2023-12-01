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

// COVINS
#include "covins_backend/backend.hpp"

// C++
#include <iostream>
#include <csignal>
#include <cstdlib>
#include <ros/ros.h>

// Thirdparty
#include <ros/ros.h>

// Profiling
#include <valgrind/callgrind.h>

auto SignalHandler(int signum)->void {
   std::cout << "Interrupt signal (" << signum << ") received." << std::endl;
   exit(signum);
}

int main(int argc, char* argv[]) {
    std::cout << "+++ COVINS Back-End +++" << std::endl;

    if(argc != 1){
        std::cout << "Error: " << argc-1 << " arguments - 0 required" << std::endl;
        return 0;
    }

    ros::init(argc, argv, "COVINS_BackEnd");

    signal(SIGINT, SignalHandler);

    std::shared_ptr<covins::CovinsBackend> backend(new covins::CovinsBackend());
    covins::CovinsBackend::ThreadPtr main_thread(new std::thread(&covins::CovinsBackend::Run,backend));
    main_thread->detach(); // Thread will be cleaned up when exiting main()

    ros::spin();
    CALLGRIND_STOP_INSTRUMENTATION;
    return 0;
}
