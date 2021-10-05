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

#include "covins_base/communicator_base.hpp"

// Socket Programming
#include <atomic>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

namespace covins {

CommunicatorBase::CommunicatorBase() {
    //...
}

CommunicatorBase::CommunicatorBase(int client_id, int newfd)
    : client_id_(client_id), newfd_(newfd)
{
    //...
}

auto CommunicatorBase::CheckBufferAndPop()->bool {
    send_buf_.clear();
    mtx_recv_buffer_.lock();

    if (!buffer_recv_data_.empty()) {
        if (send_buf_.size() < buffer_recv_data_.front().size())
            send_buf_.resize(buffer_recv_data_.front().size());
        send_buf_ = buffer_recv_data_.front();
        buffer_recv_data_.pop_front();
        msg_type_deserialize_ = buffer_recv_info_.front();
        buffer_recv_info_.pop_front();
        mtx_recv_buffer_.unlock();
        return true;
    } else {
        mtx_recv_buffer_.unlock();
        return false;
    }
}

auto CommunicatorBase::ConnectToServer(const char *node, std::string port)->int {
    int sockfd, rv;
    struct addrinfo hints, *servinfo, *p;
    char s[INET6_ADDRSTRLEN];

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    if ((rv = getaddrinfo(node, port.c_str(), &hints, &servinfo)) != 0) {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
        return 1;
    }

    // loop through all the results and connect to the first we can
    for (p = servinfo; p != NULL; p = p->ai_next) {
        if ((sockfd = socket(p->ai_family, p->ai_socktype,
                             p->ai_protocol)) == -1) {
            perror("client: socket");
            continue;
        }

        if (connect(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
            close(sockfd);
            perror("client: connect");
            continue;
        }

        break;
    }

    if (p == NULL) {
        fprintf(stderr, "client: failed to connect\n");
        return 2;
    }

    inet_ntop(p->ai_family, GetInAddr((struct sockaddr *) p->ai_addr), s, sizeof s);

//    printf("client: connecting to %s\n", s);

    freeaddrinfo(servinfo); // all done with this structure

    return sockfd;
}

auto CommunicatorBase::GetClientId()->int {
    // we do not use a mutex here - client_id_ is set at the beginning, and never changed.
    return client_id_;
}

auto CommunicatorBase::GetInAddr(sockaddr *sa)->void* {
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in *) sa)->sin_addr);
    }

    return &(((struct sockaddr_in6 *) sa)->sin6_addr);
}

auto CommunicatorBase::Lock()->void {
    mtx_comm_.lock();
}

auto CommunicatorBase::packi32(std::vector<unsigned char> &buf, MsgInfoType &msg)->void {

    if(buf.size() != msg.size()*sizeof(msg[0]))
        buf.resize(msg.size() * sizeof(msg[0]));

    for(size_t i = 0; i < msg.size(); i++){
        buf[i*4] = msg[i]>>24;
        buf[i*4+1] = msg[i]>>16;
        buf[i*4+2] = msg[i]>>8;
        buf[i*4+3] = msg[i];
    }
}

auto CommunicatorBase::PassDataBundle(data_bundle &msg)->void {
    std::unique_lock<std::mutex> lock(mtx_out_);
    buffer_data_out_.push_back(msg);
}

auto CommunicatorBase::ProcessBufferIn()->void {
    std::unique_lock<std::mutex> lock(mtx_in_);
    while(CheckBufferAndPop()){
        std::stringstream temp;
        temp.write(send_buf_.data(), std::streamsize(send_buf_.size()));
        cereal::BinaryInputArchive iarchive(temp);

        if(msg_type_deserialize_[1] == 0) { // new data msg
            if(msg_type_deserialize_[4] == 0) { // Keyframe
                MsgKeyframe msg(msg_type_deserialize_);
                iarchive(msg);
                buffer_keyframes_in_.push_back(msg);
            } else if(msg_type_deserialize_[4] == 1) { // Landmark
                MsgLandmark msg(msg_type_deserialize_);
                iarchive(msg);
                buffer_landmarks_in_.push_back(msg);
            } else {
                std::cout << COUTERROR << "msg_type_deserialize[4]: " << msg_type_deserialize_[4] << std::endl;
                exit(-1);
            }
            send_buf_.clear();
        } else if(msg_type_deserialize_[1] == 1) { // update msg
                if(msg_type_deserialize_[4] == 0) { // Keyframe
                    MsgKeyframe msg(msg_type_deserialize_);
                    iarchive(msg);
                    buffer_keyframes_in_.push_back(msg);
                } else if(msg_type_deserialize_[4] == 1) { // Landmark
                    MsgLandmark msg(msg_type_deserialize_);
                    iarchive(msg);
                    buffer_landmarks_in_.push_back(msg);
                } else {
                    std::cout << COUTERROR << "msg_type_deserialize[4]: " << msg_type_deserialize_[4] << std::endl;
                    exit(-1);
                }
//            }
            send_buf_.clear();
        } else {
            std::cout << COUTERROR << "msg_type_deserialize[1]: " << msg_type_deserialize_[1] << std::endl;
            exit(-1);
        }
    }
}

auto CommunicatorBase::ProcessBufferOut()->void {
    std::unique_lock<std::mutex> lock(mtx_out_);
    while(!buffer_data_out_.empty()){
        data_bundle db = buffer_data_out_.front();
        buffer_data_out_.pop_front();

        for(int i=0;db.keyframes.size();++i){
            message_container kf_out_container;
            MsgKeyframe msg = db.keyframes.front();
            db.keyframes.pop_front();
            Serialize(msg);
            kf_out_container.ser_msg << send_ser_.str();
            kf_out_container.msg_info.insert(kf_out_container.msg_info.end(), msg.msg_type.begin(), msg.msg_type.end());
            while(kf_out_container.msg_info.size() != ContainerSize*5)
                kf_out_container.msg_info.push_back(0);
            SendMsgContainer(kf_out_container);
        }

        while(!db.landmarks.empty()){
            message_container mp_out_container;
            for(int i = 0; i<ContainerSize; i++){
                if(!db.landmarks.empty()){
                    MsgLandmark msg = db.landmarks.front();
                    db.landmarks.pop_front();
                    Serialize(msg);
                    mp_out_container.ser_msg << send_ser_.str();
                    mp_out_container.msg_info.insert(mp_out_container.msg_info.end(), msg.msg_type.begin(), msg.msg_type.end());
                }
            }
            while(mp_out_container.msg_info.size() != ContainerSize*5)
                mp_out_container.msg_info.push_back(0);
            SendMsgContainer(mp_out_container);
        }
    }
}

auto CommunicatorBase::RecvAll(unsigned int sz, std::vector<char> &buffer)->int {
    size_t tot_bytes = 0;
    ssize_t n_bytes;

    if (buffer.size() != sz/sizeof(char))
        buffer.resize(sz/sizeof(char));

    while (tot_bytes < sz) {
        n_bytes = recv(newfd_, &buffer[tot_bytes], (sz - tot_bytes), 0);
        if (n_bytes <= 0) {
            if (n_bytes == 0) {
                // connection closed
                printf("------- selectserver: socket %d hung up. -------\n", newfd_);
            } else {
                perror("recv");
            }
            close(newfd_);
            return -1;
        }
        tot_bytes += n_bytes;
    }
    return 0;
}

auto CommunicatorBase::RecvAll(unsigned int sz, MsgInfoType &buffer)->int {
    std::vector<unsigned char> buf_stream(sz);

    size_t tot_bytes = 0;
    size_t bytes_left = sz;
    ssize_t n_bytes;

    while(tot_bytes < buf_stream.size()) {
        n_bytes = recv(newfd_, &buf_stream[tot_bytes], bytes_left, 0);
        if (n_bytes <= 0) {
            if (n_bytes == 0) {
                // connection closed
                printf("------- selectserver: socket %d hung up. -------\n", newfd_);
            } else {
                perror("recv");
            }
            close(newfd_);
            return -1;
        }

        tot_bytes += n_bytes;
        bytes_left -= n_bytes;
    }

    unpacki32(buf_stream, buffer);
    return 0;
}

auto CommunicatorBase::RecvMsg()->void {
    int total_msg;
    msg_type_container_.resize(ContainerSize*5);

    while(true) {
        //receive msg_type_
        if(RecvAll(msg_type_container_.size()*sizeof(msg_type_container_[0]), msg_type_container_) == -1) {
            printf("----> Exit Recv Thread\n");
            SetFinish();
            break;
        }

        if(msg_type_container_[0] == 1){
            client_id_ = msg_type_container_[1];
            std::cout << "--> Set Client ID: " << client_id_ << std::endl;
            msg_type_container_[0] = 0;
        }

        //calculate total message size
        total_msg = 0;
        for(int i = 0; i < 10; i ++)
          total_msg += msg_type_container_[i*5];

        //receive message
        if(RecvAll(total_msg, recv_buf_) == -1) {
            printf("----> Exit Recv Thread\n");
            SetFinish();
            break;
        }

        //write msg_type_container and msg to buffer
        WriteToBuffer();

        if(this->ShallFinish()){
            printf("----> Exit Recv Thread\n");
            SetFinish();
            break;
        }
    }
}

auto CommunicatorBase::SendAll(MsgInfoType &msg_send)->int {
    std::vector<unsigned char> buf;
    packi32(buf, msg_send);

    size_t tot_bytes = 0;
    size_t bytes_left = buf.size();
    ssize_t n_bytes;

    while(tot_bytes < buf.size()) {
        n_bytes = send(newfd_, &buf[tot_bytes], bytes_left, 0);
        if (n_bytes == -1) {
            return -1;
        }
        tot_bytes += n_bytes;
        bytes_left -= n_bytes;
    }
    return tot_bytes;
}

auto CommunicatorBase::SendAll(std::stringstream &msg)->int {
    size_t tot_bytes = 0;
    size_t bytes_left = msg.str().length();
    ssize_t n_bytes;

    std::vector<char> msg_send;
    if (msg_send.size() < bytes_left)
        msg_send.resize(bytes_left);
    msg.read(msg_send.data(), std::streamsize(msg_send.size()));

    while(tot_bytes < msg_send.size()) {
        n_bytes = send(newfd_, &msg_send[tot_bytes], bytes_left, 0);
        if (n_bytes == -1) {
            return -1;
        }
        tot_bytes += n_bytes;
        bytes_left -= n_bytes;
    }
    return tot_bytes;
}

auto CommunicatorBase::SendMsgContainer(message_container &msg)->void {
    const size_t bytes_sent_info = SendAll(msg.msg_info);
    const size_t bytes_sent_msg = SendAll(msg.ser_msg);
}

auto CommunicatorBase::Serialize(MsgKeyframe &msg)->void {
    //clear stringstream
    send_ser_.str("");
    send_ser_.clear();

    cereal::BinaryOutputArchive oarchive(send_ser_);
    oarchive(msg);

    package_size_send_ = (int)send_ser_.str().length();
    msg.SetMsgType(package_size_send_);
}

auto CommunicatorBase::Serialize(MsgLandmark &msg)->void {
    //clear stringstream
    send_ser_.str("");
    send_ser_.clear();

    cereal::BinaryOutputArchive oarchive(send_ser_);
    oarchive(msg);

    package_size_send_ = (int)send_ser_.str().length();
    msg.SetMsgType(package_size_send_);
}

auto CommunicatorBase::TryLock()->bool {
    return mtx_comm_.try_lock();
}

auto CommunicatorBase::UnLock()->void {
    mtx_comm_.unlock();
}

auto CommunicatorBase::unpacki32(std::vector<unsigned char> &buf, MsgInfoType &msg)->void {

    if(msg.size()*sizeof(msg[0]) != buf.size())
        msg.resize(buf.size()/sizeof(msg[0]));

    for(size_t i = 0; i < buf.size()/sizeof(msg[0]); i++)
        msg[i] = ((uint32_t)buf[i*4]<<24)|((uint32_t)buf[i*4+1]<<16)|((uint32_t)buf[i*4+2]<<8)|(uint32_t)buf[i*4+3];

}

auto CommunicatorBase::WriteToBuffer()->void {
    MsgInfoType tmp_msg_type_buf(5);

    mtx_recv_buffer_.lock();
    for(int i = 0; i<ContainerSize; i++) {
        if(msg_type_container_[i*5] == 0)
            break;
        for(int j = 0; j<5; j++){
            tmp_msg_type_buf[j] = msg_type_container_[5*i+j];
        }

        buffer_recv_info_.push_back(tmp_msg_type_buf);
    }

    int current_position= 0;
    for(int i = 0; i<ContainerSize; i++) {

        if(msg_type_container_[i*5]==0)
            break;

        std::vector<char> tmp_recv_buf(recv_buf_.begin() + current_position, recv_buf_.begin() + current_position + msg_type_container_[i*5]);
        buffer_recv_data_.push_back(tmp_recv_buf);
        current_position += msg_type_container_[i*5];
    }

    mtx_recv_buffer_.unlock();
}

} //end ns
