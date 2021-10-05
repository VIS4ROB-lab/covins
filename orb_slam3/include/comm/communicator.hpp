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

#define NO_LOOP_FINDER
#define NO_RELOC

namespace ORB_SLAM3 {

//class Map;
class Atlas;
class KeyFrame;

class Communicator : public covins::CommunicatorBase, public std::enable_shared_from_this<Communicator> {
public:

    struct cmp_by_id{
        bool operator() (const std::pair<size_t,KeyFrame*> a, const std::pair<size_t,KeyFrame*> b){
            if(a.first < b.first) return true;
            else return false;
    }};

public:
    Communicator(std::string server_ip, std::string port, Atlas*);

    // main function
    virtual auto Run()                                                                  ->void;

    virtual auto PassKfToComm(KeyFrame* kf)                                             ->void {
        std::unique_lock<std::mutex>(mtx_kf_queue_);
        kf_out_buffer_.push_back(kf);
    }

protected:

    // data handling
    virtual auto ProcessAdditional()                                                    ->void;
    virtual auto ProcessKeyframeMessages()                                              ->void;
    virtual auto ProcessLandmarkMessages()                                              ->void;
    virtual auto ProcessNewKeyframes()                                                  ->void;
    virtual auto ProcessNewLandmarks()                                                  ->void;

    virtual auto ProcessKfBuffer()                                                      ->void;

    // Infrastructure
    Atlas*                  map_                                                                = nullptr;  // the map is not necessary to send data to the server. However, we keep a ptr to it to facilitate implementing potetnial interaction

    bool sending_init_ = false;
    std::list<KeyFrame*> kf_out_buffer_;
    std::mutex              mtx_kf_queue_;
};

} //end ns
