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

#include "covins_backend/communicator_be.hpp"

// COVINS
#include "covins_backend/keyframe_be.hpp"
#include "covins_backend/landmark_be.hpp"
#include "covins_backend/map_be.hpp"
#include "covins_backend/placerec_be.hpp"
#include "covins_backend/visualization_be.hpp"

namespace covins {

Communicator::Communicator(int client_id, int newfd, ManagerPtr man, VisPtr vis, PlacerecPtr placerec)
    : CommunicatorBase(client_id,newfd),
      mapmanager_(man),
      vis_(vis),
      placerec_(placerec)
{
    //Send client ID
    std::cout << "Pass new ID " << client_id_ << " to client" << std::endl;
    message_container out_container;
    out_container.msg_info.push_back(1);
    out_container.msg_info.push_back(client_id_);
    while(out_container.msg_info.size() != ContainerSize*5)
        out_container.msg_info.push_back(0);
    SendMsgContainer(out_container);
}

auto Communicator::CollectDataForAgent()->void {
    if(most_recent_kf_id_ == defpair) return;
    KeyframePtr kf_newest = map_->GetKeyframe(most_recent_kf_id_);
    if(!kf_newest) {
        std::cout << COUTWARN << "cannot find KF " << most_recent_kf_id_ << std::endl;
        return;
    }
    KeyframePtr kf0 = map_->GetKeyframe(0,client_id_);
    if(!kf_newest) {
        std::cout << COUTWARN << "cannot find KF 0" << std::endl;
        return;
    }
    if(kf_newest == kf0) return;
    data_bundle map_chunk;
    MsgKeyframe msg_kf;
    kf_newest->ConvertToMsg(msg_kf,kf0,true);
    map_chunk.keyframes.push_back(msg_kf);
    this->PassDataBundle(map_chunk);
}

auto Communicator::LandmarkCulling(size_t min_obs, size_t max_gap)->int {
    if(recent_landmarks_.empty()) return 0;

    int removed_lms = 0;

    while (recent_landmarks_.front()) {
        auto lm = recent_landmarks_.front();

        if(lm->IsInvalid()) {
            recent_landmarks_.pop_front();
            continue;
        }

        if(lm->GetReferenceKeyframe() && ((most_recent_kf_id_.first - lm->GetReferenceKeyframe()->id_.first) <= max_gap)) break;

        if(!lm->GetReferenceKeyframe() && !lm->GetObservations().empty()) {
            std::cout << COUTERROR << lm << ": no KF ref -- #obs: " << lm->GetObservations().size() << std::endl;
        }
        if(lm->GetObservations().size() < min_obs) {
            map_->EraseLandmark(lm);
            removed_lms++;
        }

        recent_landmarks_.pop_front();
    }

    return removed_lms;
}


auto Communicator::ProcessAdditional()->void {
    if(covins_params::mapping::activate_lm_culling) {
        this->LandmarkCulling(2,5);
    }
}

auto Communicator::ProcessKeyframeMessages()->void {
    std::unique_lock<std::mutex> lock(mtx_in_);
    auto voc = mapmanager_->GetVoc();
    size_t cnt = 0;
    while(!buffer_keyframes_in_.empty() && cnt < 1) {
        MsgKeyframe msg = buffer_keyframes_in_.front();
        buffer_keyframes_in_.pop_front();
        if(msg.is_update_msg){
            if(!covins_params::comm::send_updates) continue;
            else {
                auto kf = map_->GetKeyframe(msg.id,false);
                if(kf && kf->id_.first == 0) continue;
                if(kf) kf->UpdatePoseFromMsg(msg,map_);
            }
        } else {
            KeyframePtr kf;
            kf = map_->GetKeyframe(msg.id,true);
            if(!kf) {
                if(msg.keypoints_distorted.empty()) {
                    std::cout << COUTERROR << "keypoints_distorted_.empty()" << std::endl;
                    std::cout << COUTNOTICE << "Note: if you want to work only on undistorted KP, you can fill 'keypoints_distorted_' with the undistorted KPs and set the distortion coefficients to 0.0" << std::endl;
                    buffer_recv_data_.clear();
                    buffer_recv_info_.clear();
                    send_buf_.clear();
                    buffer_keyframes_in_.clear();
                    buffer_landmarks_in_.clear();
                    continue;
                }
                kf.reset(new Keyframe(msg,map_,voc));
                kf->EstablishConnections(msg, map_);
                kf->EstablishNeighbors(msg,map_);
                map_->AddKeyframe(kf);                      // Add it to the map already here, because otherwise the next KF in 'buffer_keyframes_in_' (which is most likely the successor) will not find this KF
                keyframes_new_.push_back(kf);
                recent_keyframes_.push_back(kf);
                last_processed_kf_msg_ = kf->id_;
                cnt++;
            } else {
                std::cout << COUTWARN << "Received full msgs for existing KF " << kf << std::endl;
                kf->UpdatePoseFromMsg(msg,map_);
            }
        }
    }
}

auto Communicator::ProcessLandmarkMessages()->void {
    std::unique_lock<std::mutex> lock(mtx_in_);
    while(!buffer_landmarks_in_.empty()) {
        MsgLandmark msg = buffer_landmarks_in_.front();
        if(msg.id_reference.first > last_processed_kf_msg_.first) break;
        buffer_landmarks_in_.pop_front();
        if(msg.is_update_msg){
            if(!covins_params::comm::send_updates) continue;
            else {
                auto lm = map_->GetLandmark(msg.id);
                if(lm) lm->UpdatePosFromMsg(msg,map_);
            }
            continue;
        } else {
            LandmarkPtr lm;
            lm = map_->GetLandmark(msg.id);
            if(!lm) {
                lm.reset(new Landmark(msg,map_));
                lm->EstablishConnections(msg,map_);
                map_->AddLandmark(lm);
                landmarks_new_.push_back(lm);
                recent_landmarks_.push_back(lm);
            } else {
                lm->EstablishConnections(msg,map_);
                lm->UpdatePosFromMsg(msg,map_);
            }
        }
    }
}

auto Communicator::ProcessNewKeyframes()->void {
    std::unique_lock<std::mutex> lock(mtx_in_);
    while(!keyframes_new_.empty()) {
        KeyframePtr kf = keyframes_new_.front();
        keyframes_new_.pop_front();

        if(covins_params::placerec::active)
            placerec_->InsertKeyframe(kf);
        map_->UpdateCovisibilityConnections(kf->id_);
        Keyframe::LandmarkVector landmarks = kf->GetLandmarks();
        for(size_t idx=0;idx<landmarks.size();++idx) {
            LandmarkPtr i = landmarks[idx];
            if(!i) {
                continue;
            }
            i->ComputeDescriptor();
            i->UpdateNormal();
        }

        if(static_cast<int>(kf->id_.second) == client_id_) {
            if(most_recent_kf_id_ == defpair) most_recent_kf_id_ = kf->id_;
            else most_recent_kf_id_.first = std::max(most_recent_kf_id_.first,kf->id_.first);
        }
    }
}

auto Communicator::ProcessNewLandmarks()->void {
    std::unique_lock<std::mutex> lock(mtx_in_);
    while(!landmarks_new_.empty()) {
        LandmarkPtr lm = landmarks_new_.front();
        landmarks_new_.pop_front();
    }
}

auto Communicator::Run()->void {
    std::thread thread_recv(&Communicator::RecvMsg, this);
    thread_recv.detach();

    auto last = std::chrono::steady_clock::now();
    double wait_time = 1.0/covins_params::comm::to_agent_freq;

    while(true)
    {
        int check_num_map;
        map_ = mapmanager_->CheckoutMapOrWait(client_id_,check_num_map);

        if(covins_params::comm::data_to_client) {
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<double> diff = now-last;
            if(diff.count() > wait_time) {
                this->CollectDataForAgent();
                last = now;
            }
            this->ProcessBufferOut();
        }

        this->ProcessBufferIn();
        if(this->TryLock()){
            this->ProcessKeyframeMessages();
            this->ProcessLandmarkMessages();
            this->ProcessNewKeyframes();
            this->ProcessNewLandmarks();
            this->ProcessAdditional();
            this->UnLock();
        }
        vis_->DrawMap(map_);

        mapmanager_->ReturnMap(client_id_,check_num_map);
        map_ = nullptr; //make sure the MapManager is used correctly - this will cause SEGFAULT if accessed

        if(this->ShallFinish()){
            std::cout << "Comm " << client_id_ << ": close" << std::endl;
            break;
        }
        usleep(1000);
    }

    std::unique_lock<std::mutex> lock(mtx_finish_);
    is_finished_ = true;
}

} //end ns
