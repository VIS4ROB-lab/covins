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

#include "covins_base/config_comm.hpp"

namespace covins_params {

auto GetPort()->std::string {
    return covins_params::sys::port;
}

auto GetServerIP()->std::string {
    return covins_params::sys::server_ip;
}

auto ShowParamsComm()->void
{
    std::cout << "++++++++++ System ++++++++++" << std::endl;
    std::cout << "server_ip: " << covins_params::sys::server_ip << std::endl;
    std::cout << "port: " << covins_params::sys::port << std::endl;
    std::cout << std::endl;
    std::cout << "++++++++++ Communication ++++++++++" << std::endl;
    std::cout << "send_updates: " << (int)covins_params::comm::send_updates << std::endl;
    std::cout << "data_to_client: " << (int)covins_params::comm::data_to_client << std::endl;
    std::cout << "start_sending_after_kf: " << covins_params::comm::start_sending_after_kf << std::endl;
    std::cout << "kf_buffer_withold: " << covins_params::comm::kf_buffer_withold << std::endl;
    std::cout << "max_sent_kfs_per_iteration: " << covins_params::comm::max_sent_kfs_per_iteration << std::endl;
    std::cout << "update_window_size: " << covins_params::comm::update_window_size << std::endl;
    std::cout << "to_agent_freq: " << covins_params::comm::to_agent_freq << std::endl;
    std::cout << "++++++++++ ORB-SLAM3 ++++++++++" << std::endl;
    std::cout << "activate_visualization: " << (int)covins_params::orb::activate_visualization << std::endl;
    std::cout << "imu_stamp_max_diff: " << covins_params::orb::imu_stamp_max_diff << std::endl;
    std::cout << std::endl;
}

} //end ns
