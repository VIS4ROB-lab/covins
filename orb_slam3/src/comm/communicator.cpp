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
#include "comm/communicator.hpp"
#include <covins/covins_base/utils_base.hpp>

// ORB-SLAM3
#include "Atlas.h"

namespace ORB_SLAM3 {

Communicator::Communicator(std::string server_ip, std::string port, Atlas* map)
    : CommunicatorBase()
{
    covins_params::ShowParamsComm();

    map_ = map;

    std::cout << "--> Connect to server" << std::endl;
    newfd_ = ConnectToServer(server_ip.c_str(),port);
    if(newfd_ == 2){
        std::cout << COUTFATAL << ": Could no establish connection - exit" << std::endl;
        exit(-1);
    }
    std::cout << "newfd_: " << newfd_ << std::endl;
    std::cout << "--> Connected" << std::endl;
}

auto Communicator::ProcessAdditional()->void {

}

auto Communicator::ProcessKfBuffer()->void {
    std::unique_lock<std::mutex>(mtx_kf_queue_);
    int cnt = 0;

    while(!kf_out_buffer_.empty()) {
        auto kfi = kf_out_buffer_.front();
        kf_out_buffer_.pop_front();
        if(kfi->sent_once_ && !covins_params::comm::send_updates) continue;
        if(kfi->sent_once_ && kfi->mnId == 0) continue;
        covins::data_bundle map_chunk;
        covins::MsgKeyframe msg_kf;
        kfi->ConvertToMsg(msg_kf,kfi->mPrevKF,kfi->sent_once_,client_id_);
        kfi->sent_once_ = true;
        map_chunk.keyframes.push_back(msg_kf);
        if(!kfi->sent_once_) cnt++;
        auto kfi_lms = kfi->GetMapPointMatches();
        for(auto lmi : kfi_lms){
            if(!lmi) continue;
            if(lmi->sent_once_ && !covins_params::comm::send_updates) continue;
            covins::MsgLandmark msg_lm;
            lmi->ConvertToMsg(msg_lm,kfi,lmi->sent_once_,client_id_);
            lmi->sent_once_ = true;
            map_chunk.landmarks.push_back(msg_lm);
        }
        this->PassDataBundle(map_chunk);
        if(cnt >= covins_params::comm::max_sent_kfs_per_iteration) break;
    }
}

auto Communicator::ProcessKeyframeMessages()->void {
    std::unique_lock<std::mutex> lock(mtx_in_);
    while(!buffer_keyframes_in_.empty()) {
        covins::MsgKeyframe msg = buffer_keyframes_in_.front();
        buffer_keyframes_in_.pop_front();
        std::cout << COUTRED("Received KF from server -- define usage") << std::endl;
        // Define here what should be done with the received KF
    }
}

auto Communicator::ProcessLandmarkMessages()->void {

}

auto Communicator::ProcessNewKeyframes()->void {

}

auto Communicator::ProcessNewLandmarks()->void {

}

auto Communicator::Run()->void {
    std::thread thread_recv(&Communicator::RecvMsg, this);

    while(true)
    {
        this->ProcessKfBuffer();
        this->ProcessBufferOut();
        this->ProcessBufferIn();
        if(this->TryLock()){
            this->ProcessKeyframeMessages();
            this->ProcessLandmarkMessages();
            this->ProcessNewKeyframes();
            this->ProcessNewLandmarks();
            this->ProcessAdditional();
            this->UnLock();
        }

        if(this->ShallFinish()){
            std::unique_lock<std::mutex>(mtx_kf_queue_);
            if(!kf_out_buffer_.empty()) {
                std::cout << "Comm:: waiting for kf_out_buffer_" << std::endl;
            } else {
                std::cout << "Comm " << client_id_ << ": close" << std::endl;
                break;
            }
        }
        usleep(1000);
    }

    std::cout << "Comm " << client_id_ << ": wait for recv thread" << std::endl;
    thread_recv.join();
    std::cout << "Comm " << client_id_ << ": leave" << std::endl;
}

} //end ns
