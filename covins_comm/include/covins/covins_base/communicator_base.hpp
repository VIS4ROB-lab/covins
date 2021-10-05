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
#include <unistd.h>
#include <mutex>
#include <thread>
#include <list>
#include <netinet/in.h>
#include <eigen3/Eigen/Core>

// COVINS
#include "typedefs_base.hpp"
#include "config_comm.hpp"
#include "msgs/msg_keyframe.hpp"
#include "msgs/msg_landmark.hpp"
#define ContainerSize 10

namespace covins {

struct MsgKeyframe;
struct MsgLandmark;

struct data_bundle {
public:

    struct compare_less{bool operator() (const data_bundle &a, const data_bundle &b) const;};

    TypeDefs::KeyframeMsgList   keyframes;
    TypeDefs::LandmarkMsgList   landmarks;
};

struct message_container {
public:
    std::stringstream           ser_msg;
    TypeDefs::MsgTypeVector     msg_info;
};

class CommunicatorBase {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using precision_t                   = TypeDefs::precision_t;
    using idpair                        = TypeDefs::idpair;

    using MsgInfoType                   = TypeDefs::MsgTypeVector;
    using TransformType                 = TypeDefs::TransformType;

    using KeyframePtr                   = TypeDefs::KeyframePtr;
    using LandmarkPtr                   = TypeDefs::LandmarkPtr;

    using DataBundleBufferType          = std::list<data_bundle>;
    using KeyframeBufferType            = TypeDefs::KeyframeMsgList;
    using LandmarkBufferType            = TypeDefs::LandmarkMsgList;

public:
    CommunicatorBase();
    CommunicatorBase(int client_id, int newfd);

    // Main
    virtual auto Run()                                                                  ->void      = 0;

    // Interfaces
    virtual auto PassDataBundle(data_bundle &msg)                                       ->void;
    virtual auto Lock()                                                                 ->void;
    virtual auto UnLock()                                                               ->void;
    virtual auto TryLock()                                                              ->bool;
    virtual auto GetClientId()                                                          ->int;

    // Message handling
    virtual auto Serialize(MsgKeyframe &msg)                                            ->void;
    virtual auto Serialize(MsgLandmark &msg)                                            ->void;

    // Message passing
    static auto GetInAddr(struct sockaddr *sa)                                          ->void*;    // get sockaddr, IPv4 or IPv6:
    virtual auto ConnectToServer(const char *node, std::string port)                    ->int;

    // Synchronization
    virtual auto SetFinish()                                                            ->void {
        std::unique_lock<std::mutex> lock(mtx_finish_); finish_ = true;}
    virtual auto ShallFinish()                                                          ->bool {
        std::unique_lock<std::mutex> lock(mtx_finish_); return finish_;}

protected:

    // Message passing
    virtual auto SendAll(MsgInfoType &msg_send)                                         ->int;
    virtual auto SendAll(std::stringstream &msg)                                        ->int;
    virtual auto RecvAll(unsigned int sz, std::vector<char> &buffer)                    ->int;
    virtual auto RecvAll(unsigned int sz, MsgInfoType &buffer)                          ->int;
    virtual auto RecvMsg()                                                              ->void;
    virtual auto WriteToBuffer()                                                        ->void;
    virtual auto CheckBufferAndPop()                                                    ->bool;     //Attention: Removes data from buffer, do not use if for mere empty/non-empty checks
    virtual auto packi32(std::vector<unsigned char> &buf, MsgInfoType &msg)             ->void;
    virtual auto unpacki32(std::vector<unsigned char> &buf, MsgInfoType &msg)           ->void;
    virtual auto SendMsgContainer(message_container &msg)                               ->void;

    // Data handling
    virtual auto ProcessBufferOut()                                                     ->void;
    virtual auto ProcessBufferIn()                                                      ->void;
    virtual auto ProcessKeyframeMessages()                                              ->void      = 0;
    virtual auto ProcessLandmarkMessages()                                              ->void      = 0;

    // Infrastructure
    int                         client_id_                                              = -1;

    // Data
    std::list<std::vector<char>>        buffer_recv_data_;
    std::list<std::vector<uint32_t>>    buffer_recv_info_;

    DataBundleBufferType        buffer_data_out_;
    KeyframeBufferType          buffer_keyframes_out_;
    LandmarkBufferType          buffer_landmarks_out_;

    KeyframeBufferType          buffer_keyframes_in_;
    LandmarkBufferType          buffer_landmarks_in_;

    // Sync
    std::mutex                  mtx_comm_;
    std::mutex                  mtx_finish_;
    std::mutex                  mtx_recv_buffer_;
    std::mutex                  mtx_out_;
    std::mutex                  mtx_in_;

    bool                        finish_                                                 = false;

    //message passing
    std::vector<char>           recv_buf_;
    int                         package_size_send_;
    int                         newfd_;
    std::stringstream           send_ser_;
    std::vector<char>           send_buf_;                                                                  // copy shared buffer to send thread for deserialization
    MsgInfoType                 msg_type_buf_                                           = MsgInfoType(5);   // msg size, SentOnce, id.first, id.second
    MsgInfoType                 msg_type_container_;                                                        // msg size, SentOnce, id.first, id.second
    MsgInfoType                 msg_type_deserialize_                                   = MsgInfoType(5);   // msg size, SentOnce, id.first, id.second
};

} //end ns
