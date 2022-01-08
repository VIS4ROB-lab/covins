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

#include "covins_backend/map_be.hpp"

// C++
#include <dirent.h>

// COVINS
#include "covins_backend/keyframe_be.hpp"
#include "covins_backend/landmark_be.hpp"
#include "covins_backend/kf_database.hpp"

namespace covins {

MapInstance::MapInstance(int id) {
    map.reset(new Map(id));
}

MapInstance::MapInstance(MapInstancePtr map_target, MapInstancePtr map_tofuse, TransformType T_wmatch_wtofuse)
    : usage_cnt(-1)
{
    map.reset(new Map(map_target->map,map_tofuse->map,T_wmatch_wtofuse));
}

MapInstance::MapInstance(MapPtr external_map) {
    map = external_map;
}

MapManager::MapManager(VocabularyPtr voc)
    : voc_(voc)
{
    // Other types of place recognition system could be integrated and activated using the placerec::type parameter
    if(covins_params::placerec::type == "COVINS") {
        database_.reset(new KeyframeDatabase(voc_));
    } else {
        std::cout << COUTFATAL << "Place Recognition System Type \"" << covins_params::placerec::type << "\" not valid" << std::endl;
        exit(-1);
    }

    if(!voc_) {
        std::cout << COUTFATAL << "invalid vocabulary ptr" << std::endl;
        exit(-1);
    }
}

auto MapManager::AddToDatabase(KeyframePtr kf)->void {
    std::unique_lock<std::mutex> lock(mtx_access_);
    database_->AddKeyframe(kf);
}

auto MapManager::CheckMergeBuffer()->bool {
    std::unique_lock<std::mutex> lock(mtx_access_);
    return !(buffer_merge_.empty());
}

auto MapManager::CheckoutMap(int map_id, int &check_num)->MapPtr {
    std::unique_lock<std::mutex> lock(mtx_access_);

    MapContainer::iterator mit = maps_.find(map_id);
    if(mit == maps_.end()){
        std::cout << COUTERROR << "no existing map with Map-ID " << map_id << std::endl;
        return nullptr;
    }

    MapInstancePtr map = mit->second;

    if(map->usage_cnt == -1){
        return nullptr;
    } else if(map->block_checkout) {
        return nullptr;
    } else {
        int check = rand();
        map->usage_cnt++;
        map->check_nums.insert(check);
        check_num = check;

        return map->map;
    }
}

auto MapManager::CheckoutMapExclusive(int map_id, int &check_num)->MapPtr {
    std::unique_lock<std::mutex> lock(mtx_access_);

    MapContainer::iterator mit = maps_.find(map_id);
    if(mit == maps_.end()){
        std::cout << COUTFATAL << "no existing map with Map-ID " << map_id << std::endl;
        return nullptr;
    }

    MapInstancePtr map = mit->second;

    if(map->usage_cnt != 0){
        return nullptr;
    } else {
        int check = rand();
        map->usage_cnt = -1;
        map->check_nums.insert(check);
        map->block_checkout = false;
        check_num = check;

        return map->map;
    }
}

auto MapManager::CheckoutMapExclusiveOrWait(int map_id, int &check_num)->MapPtr {

    {
        MapContainer::iterator mit = maps_.find(map_id);
        if(mit == maps_.end()){
            std::cout << COUTFATAL << "no existing map with Map-ID " << map_id << std::endl;
            return nullptr;
        }
    }

    MapPtr map = this->CheckoutMapExclusive(map_id,check_num);
    if(!map) {
        while(!this->SetCheckoutBlock(map_id,true))
            usleep(100);
    }

    while(!map){
        map = this->CheckoutMapExclusive(map_id,check_num);
        usleep(100);
    }

    return map;
}

auto MapManager::CheckoutMapOrWait(int map_id, int &check_num)->MapPtr {

    {
        MapContainer::iterator mit = maps_.find(map_id);
        if(mit == maps_.end()){
            std::cout << COUTFATAL << "no existing map with Map-ID " << map_id << std::endl;
            return nullptr;
        }
    }

    MapPtr map = this->CheckoutMap(map_id,check_num);
    while(!map){
        map = this->CheckoutMap(map_id,check_num);
        usleep(100);
    }
    return map;
}

auto MapManager::EraseFromDatabase(KeyframePtr kf)->void {
    std::unique_lock<std::mutex> lock(mtx_access_);
    database_->EraseKeyframe(kf);
}

auto MapManager::GetDatabase()->DatabasePtr {
    std::unique_lock<std::mutex> lock(mtx_access_);
    return database_;
}

auto MapManager::InitializeMap(int map_id)->void {
    std::unique_lock<std::mutex> lock(mtx_access_);

    MapContainer::iterator mit = maps_.find(map_id);
    if(mit != maps_.end()){
        std::cout << COUTFATAL << "Existing map with Map-ID " << map_id << std::endl;
        exit(-1);
    }

    MapInstancePtr map(new MapInstance(map_id));
    maps_[map_id] = map;
}

auto MapManager::PerformMerge()->void {

    std::cout << "+++ Perform Merge +++" << std::endl;

    KeyframePtr kf_query;
    KeyframePtr kf_match;
    TransformType T_smatch_squery;

    std::cout << "--> Fetch merge data" << std::endl;
    {
        std::unique_lock<std::mutex> lock(mtx_access_);
        MergeInformation merge = buffer_merge_.front();
        buffer_merge_.pop_front();
        kf_query = merge.kf_query;
        kf_match = merge.kf_match;
        T_smatch_squery = merge.T_smatch_squery;
    }
    std::cout << "--> Process merge data" << std::endl;

    int check_query, check_match;

    this->CheckoutMapExclusiveOrWait(kf_query->id_.second,check_query);
    MapInstancePtr map_query = maps_[kf_query->id_.second];

    this->CheckoutMapExclusiveOrWait(kf_match->id_.second,check_match);
    MapInstancePtr map_match = maps_[kf_match->id_.second];


    std::cout << "----> Merge maps" << std::endl;

    TransformType T_w_squery = kf_query->GetPoseTws();
    TransformType T_w_smatch = kf_match->GetPoseTws();
    TransformType T_wmatch_wquery = T_w_smatch * T_smatch_squery * T_w_squery.inverse();

    MapInstancePtr map_merged(new MapInstance(map_match,map_query,T_wmatch_wquery));

    LoopConstraint lc(kf_match,kf_query,T_smatch_squery);
    map_merged->map->AddLoopConstraint(lc);

//    std::cout << "Map Match: Agents|KFs|LMs :" << map_match->map->associated_clients_.size() << "|" << map_match->map->GetKeyframes().size() << "|" << map_match->map->GetLandmarks().size() << std::endl;
//    std::cout << "Map Query: Agents|KFs|LMs :" << map_query->map->associated_clients_.size() << "|" << map_query->map->GetKeyframes().size() << "|" << map_query->map->GetLandmarks().size() << std::endl;
    std::cout << "Merged Map: Agents|KFs|LMs: " << map_merged->map->associated_clients_.size() << "|" << map_merged->map->GetKeyframes().size() << "|" << map_merged->map->GetLandmarks().size() << std::endl;

    for(std::set<size_t>::iterator sit = map_merged->map->associated_clients_.begin();sit != map_merged->map->associated_clients_.end();++sit) {
        maps_[*sit] = map_merged;
    }

    map_merged->usage_cnt = 0;

    std::cout << "\033[1;32m+++ MAPS MERGED +++\033[0m" << std::endl;
}

auto MapManager::RegisterMap(MapPtr external_map, bool enter_kfs_in_database)->bool {

    if(enter_kfs_in_database) {
        auto keyframes = external_map->GetKeyframesVec();
        for(const auto& kf : keyframes) {
            this->AddToDatabase(kf);
        }
    } else {
        // skip this because we do a PR test
        std::cout << COUTNOTICE << "!!! KFs not entered in database !!!" << std::endl;
    }

    MapInstancePtr map_inst(new MapInstance(external_map));
    for(std::set<size_t>::iterator sit = map_inst->map->associated_clients_.begin();sit != map_inst->map->associated_clients_.end();++sit) {
        std::cout << "----> Add maps_[" << *sit << "]" << std::endl;
        maps_[*sit] = map_inst;
    }
    return true;
}

auto MapManager::RegisterMerge(MergeInformation merge_data)->void {
    std::unique_lock<std::mutex> lock(mtx_access_);
    buffer_merge_.push_back(merge_data);
}

auto MapManager::ReturnMap(int map_id, int &check_num)->void {
    std::unique_lock<std::mutex> lock(mtx_access_);

    MapContainer::iterator mit = maps_.find(map_id);
    if(mit == maps_.end()){
        std::cout << COUTFATAL << "no existing map with Map-ID " << map_id << std::endl;
        exit(-1);
    }

    MapInstancePtr map = mit->second;

    if(!map->check_nums.count(check_num)){
        std::cout << COUTFATAL << "check_num error" << std::endl;
        exit(-1);
    }

    if(map->usage_cnt == -1)
        map->usage_cnt = 0;
    else
        map->usage_cnt--;

    map->check_nums.erase(check_num);
}

auto MapManager::Run()->void {
    while(1) {
        if(this->CheckMergeBuffer()) {
            this->PerformMerge();
        }

        usleep(5000);
    }
}

auto MapManager::SetCheckoutBlock(int map_id, bool val)->bool {
    std::unique_lock<std::mutex> lock(mtx_access_);

    MapContainer::iterator mit = maps_.find(map_id);
    if(mit == maps_.end()){
        std::cout << COUTFATAL << "no existing map with Map-ID " << map_id << std::endl;
        exit(-1);
    }

    MapInstancePtr map = mit->second;

    if(val && map->block_checkout) {
        return false;
    } else {
        map->block_checkout = val;
        return true;
    }
}

//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------

Map::Map(size_t id)
    : MapBase(id)
{
    associated_clients_.insert(id);
}

Map::Map(MapPtr map_target, MapPtr map_tofuse, TransformType T_wtarget_wtofuse)
    : MapBase(map_target->id_map_)
{
    // data map_target
    std::set<size_t> associated_clients_target = map_target->associated_clients_;
    KeyframeMap keyframes_target = map_target->GetKeyframes();
    LandmarkMap landmarks_target = map_target->GetLandmarks();
    size_t tmax_id_kf_target = map_target->GetMaxKfId();
    size_t max_id_lm_target = map_target->GetMaxLmId();
    LoopVector loops_target = map_target->GetLoopConstraints();

    // data map_tofuse
    std::set<size_t> associated_clients_tofuse = map_tofuse->associated_clients_;
    KeyframeMap keyframes_tofuse = map_tofuse->GetKeyframes();
    LandmarkMap landmarks_tofuse = map_tofuse->GetLandmarks();
    size_t tmax_id_kf_tofuse = map_tofuse->GetMaxKfId();
    size_t max_id_lm_tofuse = map_tofuse->GetMaxLmId();
    LoopVector loops_tofuse = map_tofuse->GetLoopConstraints();

    //fill new map
    associated_clients_.insert(associated_clients_target.begin(),associated_clients_target.end());
    associated_clients_.insert(associated_clients_tofuse.begin(),associated_clients_tofuse.end());
    keyframes_.insert(keyframes_target.begin(),keyframes_target.end());
    keyframes_.insert(keyframes_tofuse.begin(),keyframes_tofuse.end());
    landmarks_.insert(landmarks_target.begin(),landmarks_target.end());
    landmarks_.insert(landmarks_tofuse.begin(),landmarks_tofuse.end());
    max_id_kf_ = std::max(tmax_id_kf_target,tmax_id_kf_tofuse);
    max_id_lm_ = std::max(max_id_lm_target,max_id_lm_tofuse);
    loop_constraints_.insert(loop_constraints_.end(),loops_target.begin(),loops_target.end());
    loop_constraints_.insert(loop_constraints_.end(),loops_tofuse.begin(),loops_tofuse.end());

    // Transform poses of map_tofuse
    for(KeyframeMap::iterator mit = keyframes_tofuse.begin();mit != keyframes_tofuse.end();++mit) {
        KeyframePtr kf = mit->second;
        TransformType T_w_s_befcorrection = kf->GetPoseTws();
        TransformType T_w_s_corrected = T_wtarget_wtofuse * T_w_s_befcorrection;
        kf->SetPoseTws(T_w_s_corrected);
        kf->velocity_ = T_wtarget_wtofuse.block<3,3>(0,0) * kf->velocity_;
    }
    Matrix3Type R_wmatch_wtofuse = T_wtarget_wtofuse.block<3,3>(0,0);
    Vector3Type t_wmatch_wtofuse = T_wtarget_wtofuse.block<3,1>(0,3);
    for(LandmarkMap::iterator mit = landmarks_tofuse.begin();mit != landmarks_tofuse.end();++mit) {
        LandmarkPtr lm = mit->second;
        Vector3Type pos_w_befcorrection = lm->GetWorldPos();
        Vector3Type pos_w_corrected = R_wmatch_wtofuse * pos_w_befcorrection + t_wmatch_wtofuse;
        lm->SetWorldPos(pos_w_corrected);
    }
}

auto Map::AddKeyframe(KeyframePtr kf)->void {
    this->AddKeyframe(kf,false);
}

auto Map::AddKeyframe(covins::MapBase::KeyframePtr kf, bool suppress_output)->void {
    std::unique_lock<std::mutex> lock(mtx_map_);
    keyframes_[kf->id_] = kf;
    max_id_kf_ = std::max(max_id_kf_,kf->id_.first);
    if(!suppress_output && !(keyframes_.size() % 50)) {
        std::cout << "Map " << this->id_map_  << " : " << keyframes_.size() << " KFs | " << landmarks_.size() << " LMs" << std::endl;
        this->WriteKFsToFile();
    }
}

auto Map::AddLandmark(covins::MapBase::LandmarkPtr lm)->void {
    std::unique_lock<std::mutex> lock(mtx_map_);
    landmarks_[lm->id_] = lm;
    max_id_lm_ = std::max(max_id_lm_,lm->id_.first);
}

auto Map::AddLoopConstraint(LoopConstraint lc)->void {
    std::unique_lock<std::mutex> lock(mtx_map_);
    loop_constraints_.push_back(lc);
    lc.kf1->is_loop_kf_ = true;
    lc.kf2->is_loop_kf_ = true;
}

auto Map::ApplyLoopCorrection(KeyframePtr kf_query, KeyframePtr kf_match, TransformType T_smatch_squery)->void {
    std::unique_lock<std::mutex> lock(mtx_map_);

    TransformType T_w_squery_befcorr = kf_match->GetPoseTws();
    TransformType T_w_smatch = kf_match->GetPoseTws();

    TransformType T_w_squery_corrected = T_w_smatch * T_smatch_squery;

    kf_query->SetPoseTws(T_w_squery_corrected);
    KeyframePtr succ = kf_query->GetSuccessor();
    while(succ){
        TransformType T_w_ssucc_befcorr = succ->GetPoseTws();
        TransformType T_squery_ssucc = T_w_squery_befcorr * T_w_ssucc_befcorr.inverse();
        TransformType T_w_ssucc_corrected = T_w_squery_corrected * T_squery_ssucc;
        succ->SetPoseTws(T_w_ssucc_corrected);
        succ = succ->GetSuccessor();
    }

    LoopConstraint lc(kf_match,kf_query,T_smatch_squery);
    loop_constraints_.push_back(lc);
}

auto Map::ConvertToMsgFileExport(MsgMap &msg)->void {
    std::unique_lock<std::mutex> lock(mtx_map_);
    msg.id_map = id_map_;
    msg.keyframes1.reserve(loop_constraints_.size());
    msg.keyframes2.reserve(loop_constraints_.size());
    msg.transforms12.reserve(loop_constraints_.size());
    for(const auto& i : loop_constraints_) {
        msg.keyframes1.push_back(i.kf1->id_);
        msg.keyframes2.push_back(i.kf2->id_);
        msg.transforms12.push_back(i.T_s1_s2);
    }
}

auto Map::Clean()->void {
    std::unique_lock<std::mutex> lock(mtx_map_);
    std::cout << "+++ Clean Map +++" << std::endl;
    std::cout << "--> Remove Landmarks" << std::endl;
    this->RemoveLandmarkOutliers();
    std::cout << "+++ DONE +++" << std::endl;
}

auto Map::EraseKeyframe(KeyframePtr kf, bool mtx_lock)->bool {
    if(mtx_lock) mtx_map_.lock();
    bool success = false;
    KeyframeMap::iterator mit = keyframes_.find(kf->id_);
    if(mit != keyframes_.end()) {
        bool removed = kf->SetInvalid();
        if(!removed) {
            std::cout << COUTWARN << "Map " << this->id_map_ << ": could not remove " << kf << std::endl;
        } else{
            keyframes_.erase(mit);
            std::cout << "Map " << this->id_map_ << " : erased " << kf << std::endl;
            keyframes_erased_[kf->id_] = kf;
            success = true;
        }
    }
    else std::cout << COUTWARN << "Map " << this->id_map_ << ": trying to erase KF from map that does not exist" << std::endl;
    if(mtx_lock) mtx_map_.unlock();

    return success;
}

auto Map::EraseKeyframeWithDatabase(KeyframePtr kf, bool mtx_lock, MapManager::DatabasePtr database)->bool {
    bool success = this->EraseKeyframe(kf,mtx_lock);
    if(!database) std::cout << COUTNOTICE << "need to erase KF from database!" << std::endl;
    else database->EraseKeyframe(kf);

    return success;
}

auto Map::EraseLandmark(LandmarkPtr lm, bool mtx_lock)->bool {
    if(mtx_lock) mtx_map_.lock();
    bool success = false;
    LandmarkMap::iterator mit = landmarks_.find(lm->id_);
    if(mit != landmarks_.end()) {
        bool removed = lm->SetInvalid();
        if(!removed) {
            std::cout << COUTWARN << "Map " << this->id_map_ << ": could not remove " << lm << std::endl;
        } else {
            landmarks_.erase(mit);
            success = true;
        }
    }
    if(mtx_lock) mtx_map_.unlock();

    return success;
}

auto Map::GetLoopConstraints()->LoopVector {
    std::unique_lock<std::mutex> lock(mtx_map_);
    return loop_constraints_;
}

auto Map::LoadFromFile(const std::string &path_name, VocabularyPtr voc)->void {
    std::cout << "+++ Load Map from File +++" << std::endl;

    if(!voc && covins_params::placerec::type != "VINS") {
        std::cout << COUTFATAL << "invalid vocabulary ptr" << std::endl;
        exit(-1);
    }

    std::vector<std::string> filenames_kf, filenames_mp;
    KeyframeVector keyframes;
    LandmarkVector landmarks;

    // quickfix char*/string compatibility [TODO] cleaner solution
    std::string kf_tmp = "/keyframes/";
    std::string mp_tmp = "/mappoints/";
    char cstr0[path_name.size()+kf_tmp.size()+1];
    char cstr1[path_name.size()+mp_tmp.size()+1];
    strcpy(cstr0, (path_name+kf_tmp).c_str());
    strcpy(cstr1, (path_name+mp_tmp).c_str());

    // getting all filenames for the keyframes
    struct dirent *entry;
    DIR *dir = opendir(cstr0);

    if (dir == nullptr) {
        std::cout << "Directory is empty." << std::endl;
        return;
    }

    while ((entry = readdir(dir)) != nullptr) {
        if ( !strcmp(entry->d_name, ".") || !strcmp(entry->d_name, "..") ) {
            continue;
        } else {
            std::stringstream path;
            path << path_name+"/keyframes/";
            path << entry->d_name;
            filenames_kf.push_back(path.str());
        }
    }
    closedir(dir);

    // getting all filenames for the mappoints
    struct dirent *entry2;
    DIR *dir2 = opendir(cstr1);
    if (dir2 == nullptr) {
        std::cout << "Directory is empty." << std::endl;
        return;
    }

     while ((entry2 = readdir(dir2)) != nullptr) {
        if ( !strcmp(entry2->d_name, ".") || !strcmp(entry2->d_name, "..") ) {
            continue;
        } else {
            std::stringstream path;
            path << path_name+"/mappoints/";
            path << entry2->d_name;
            filenames_mp.push_back(path.str());
        }
    }
    closedir(dir);

    Eigen::Matrix4d T_ow = Eigen::Matrix4d::Identity();

    std::cout << "--> Loading Keyframes" << std::endl;
    for(unsigned long int i = 0; i < filenames_kf.size(); i++) {
        if(i % 50 == 0) std::cout << "----> Loaded " << i << " of " << filenames_kf.size() << " KFs" << std::endl;
        MsgKeyframe msg(true);
        std::stringstream buf;
        std::ifstream fs;
        fs.open(filenames_kf[i]);
        if(fs.is_open()) {
            buf << fs.rdbuf();
            cereal::BinaryInputArchive iarchive(buf);
            iarchive(msg);
            KeyframePtr kf(new Keyframe(msg,nullptr,voc));
            kf->is_loaded_ = true;
            keyframes.push_back(kf);
            if(kf->id_.first == 0) {
                T_ow.block<3,3>(0,0) = Eigen::Matrix3d::Zero();
                T_ow(0,1) = 1.0;
                T_ow(1,0) = -1.0;
                T_ow(2,2) = 1.0;
                T_ow.block<3,1>(0,3) = -1.0 * T_ow.block<3,3>(0,0) * kf->GetPoseTws().block<3,1>(0,3);
            }
            fs.close();
        } else {
            std::cout << filenames_kf[i] << std::endl;
            exit(-1);
        }
    }

    std::cout << "--> Loading Landmarks" << std::endl;
    for(unsigned long int i = 0; i < filenames_mp.size(); i++) {
        if(i % 1000 == 0) std::cout << "----> Loaded " << i << " of " << filenames_mp.size() << " LMs" << std::endl;
        MsgLandmark msg(true);
        std::stringstream buf;
        std::ifstream fs;
        fs.open(filenames_mp[i]);
        if(fs.is_open()) {
            buf << fs.rdbuf();
            cereal::BinaryInputArchive iarchive(buf);
            iarchive(msg);
            LandmarkPtr lm(new Landmark(msg,nullptr));
            lm->is_loaded_ = true;
            landmarks.push_back(lm);
            fs.close();
        } else {
            std::cout << COUTERROR << "Cannot read file: " << filenames_mp[i] << std::endl;
            exit(-1);
        }
    }

    std::cout << "--> Loading Map Data" << std::endl;
    MsgMap msg_map;
    {
        std::stringstream buf;
        std::ifstream fs;
        fs.open(path_name+"/mapdata"+".txt");
        if(fs.is_open()) {
            buf << fs.rdbuf();
            cereal::BinaryInputArchive iarchive(buf);
            iarchive(msg_map);
            fs.close();
        } else {
            std::cout << COUTFATAL << "cannot load map data" << std::endl;
            exit(-1);
        }
    }

    std::cout << "Map consists of " << keyframes.size() << " keyframes" << std::endl;
    std::cout << "Map consists of " << landmarks.size() << " landmarks" << std::endl;

    std::cout << "--> Building Connections" << std::endl;

    for(const auto& kf : keyframes) {
        this->AddKeyframe(kf,true);
        if(!this->associated_clients_.count(kf->id_.second))
            associated_clients_.insert(kf->id_.second);
    }

    std::cout << "----> Landmarks" << std::endl;
    for(const auto& lm : landmarks) {
        for(auto mit = lm->msg_.observations.begin(); mit!=lm->msg_.observations.end();++mit){
            size_t feat_id = mit->second;
            idpair kf_id = mit->first;
            KeyframePtr kf = this->GetKeyframe(kf_id);
            if(!kf){
                continue;
            }
            lm->AddObservation(kf,feat_id,true);
        }
        lm->SetReferenceKeyframe(this->GetKeyframe(lm->msg_.id_reference));
        lm->ComputeDescriptor();
        lm->UpdateNormal();
        this->AddLandmark(lm);
    }

    std::cout << "----> Keyframes" << std::endl;
    for(const auto& kf : keyframes) {
        kf->SetPredecessor(this->GetKeyframe(kf->msg_.id_predecessor));
        kf->SetSuccessor(this->GetKeyframe(kf->msg_.id_successor));
        for(auto mit = kf->msg_.landmarks.begin(); mit!=kf->msg_.landmarks.end();++mit){
            size_t feat_id = mit->first;
            idpair lm_id = mit->second;
            LandmarkPtr lm = this->GetLandmark(lm_id);
            kf->AddLandmark(lm,feat_id);
        }
        kf->UpdateCovisibilityConnections();
    }

    std::cout << "----> Map Data" << std::endl;
    for(size_t i=0;i<msg_map.keyframes1.size();++i) {
        KeyframePtr kf1 = this->GetKeyframe(msg_map.keyframes1[i]);
        KeyframePtr kf2 = this->GetKeyframe(msg_map.keyframes2[i]);
//        std::cout << "------> Add Loop Constraint: " << std::endl;
//        std::cout << "kf1: "  << kf1 << std::endl;
//        std::cout << "kf2: "  << kf2 << std::endl;
//        std::cout << "T12: \n"  << msg_map.transforms12[i] << std::endl;
        LoopConstraint lc(kf1,kf2,msg_map.transforms12[i]);
        loop_constraints_.push_back(lc);
    }

    std::cout << "+++ DONE +++" << std::endl;
}

auto Map::RemoveLandmarkOutliers()->int {
    // assumes mtx is locked by calling method
    int removed_lms = 0;
    int removed_lms_map = 0;
    int removed_lms_kfs = 0;

    std::list<LandmarkPtr> lms_to_remove;

    for(LandmarkMap::iterator mit = landmarks_.begin(); mit!=landmarks_.end();) {
        LandmarkPtr lm = mit->second;
        if(!lm) {
            std::cout << COUTWARN << "Map " << this->id_map_ << ": nullptr LM in map" << std::endl;
            mit = landmarks_.erase(mit);
        } else {
            if(lm->GetObservations().size() < 2) {
                lms_to_remove.push_back(lm);
            }
            mit++;
        }
    }

    for(const auto& lm : lms_to_remove) {
        this->EraseLandmark(lm,false);
        removed_lms++;
        removed_lms_map++;
    }

    for(const auto& i : keyframes_) {
        KeyframePtr kf = i.second;
        auto kf_lms = kf->GetLandmarks();
        for(const auto& lm : kf_lms) {
            if(!lm) continue;
            if(lm->GetObservations().size() < 2) {
                this->EraseLandmark(lm,false);
                removed_lms++;
                removed_lms_kfs++;
            }
        }
    }

    std::cout << "----> Done: Removed " << removed_lms << " Landmarks" << std::endl;
    std::cout << "------> from map checks: " << removed_lms_map << std::endl;
    std::cout << "------> from kf checks:  " << removed_lms_kfs << std::endl;

    return removed_lms;
}

auto Map::RemoveRedundantData(ManagerPtr mapmanager, double th_red, size_t max_kfs)->size_t {

    this->Clean();

    std::unique_lock<std::mutex> lock(mtx_map_); // Clean() also needs mtx_map_
    size_t removed_kfs = 0;

    KeyframeVector kfs;
    for(const auto& p : keyframes_) {
        KeyframePtr kf = p.second;
        if(kf->id_.first == 0) continue;
        if(!kf->GetPredecessor()) continue;
        if(!kf->GetSuccessor()) continue;
        kfs.push_back(kf);
    }

    auto database = mapmanager->GetDatabase();

    if(max_kfs == std::numeric_limits<size_t>::max()) {
        // remove all KFs with red_val >= th_red
        while(!kfs.empty()) {
            this->CalRedVals(kfs);
            if(kfs.empty()) {
                std::cout << COUTWARN << "no KFs to erase" << std::endl;
                break;
            }
            if(kfs[0]->latest_red_val_ >= th_red) {
                bool can_be_removed = true;
                if(kfs[0]->GetTimeSpanPredSucc() >= covins_params::mapping::kf_culling_max_time_dist) can_be_removed = false;
                if(kfs[0]->is_loop_kf_) can_be_removed = false;
                if(can_be_removed) {
                    this->EraseKeyframeWithDatabase(kfs[0],false,database);
                    removed_kfs++;
                }
                kfs.erase(kfs.begin());
            } else {
                break;
            }
        }
    } else {
        // remove KFs until there are only max_kfs left
        while(keyframes_.size() > max_kfs) {
            this->CalRedVals(kfs);
            if(kfs.empty()) {
                std::cout << COUTWARN << "no KFs to erase" << std::endl;
                break;
            }
            bool can_be_removed = true;
            if(kfs[0]->GetTimeSpanPredSucc() >= covins_params::mapping::kf_culling_max_time_dist) can_be_removed = false;
            if(kfs[0]->is_loop_kf_) can_be_removed = false;
            if(can_be_removed) {
                this->EraseKeyframeWithDatabase(kfs[0],false,database);
                removed_kfs++;
            }
            kfs.erase(kfs.begin());
        }
    }

    return removed_kfs;
}

auto Map::CalRedVals(KeyframeVector &kfs)->void {
    for(const auto& kf : kfs) {
        kf->ComputeRedundancyValue();
    }
    std::sort(kfs.begin(),kfs.end(),Keyframe::sort_by_redval);
}

auto Map::SaveToFile(const std::string &path_name)->void {
//    std::unique_lock<std::mutex> lock(mtx_map_); -- do not lock, calls map interfaces (uses mutexes there)
    std::cout << "+++ Save Map to File +++" << std::endl;

    {
        // check folder (only checks whether a folder "keyframes/" or "mappoints/" exists
        std::string kf_tmp = "/keyframes/";
        std::string mp_tmp = "/mappoints/";
        char cstr0[path_name.size()+kf_tmp.size()+1];
        char cstr1[path_name.size()+mp_tmp.size()+1];
        strcpy(cstr0, (path_name+kf_tmp).c_str());
        strcpy(cstr1, (path_name+mp_tmp).c_str());

        std::vector<std::string> filenames_kf, filenames_mp;

        // Check for KF files
        DIR *dir = opendir(cstr0);
        if (dir != nullptr) {
            std::cout << COUTERROR << "Write directory is not empty: 'keyframes/' found." << std::endl;
            return;
        }

        // Check for LM files
        DIR *dir2 = opendir(cstr1);
        if (dir2 != nullptr) {
            std::cout << COUTERROR << "Write directory is not empty: 'mappoints/' found." << std::endl;
            return;
        };
    }

    this->Clean();

    std::string kf_tmp = "/keyframes";
    std::string mp_tmp = "/mappoints";

    char cstr0[path_name.size()+1];
    char cstr[path_name.size() + kf_tmp.size()+1];
    char cstr2[path_name.size() + mp_tmp.size()+1];
    strcpy(cstr0,path_name.c_str());
    strcpy(cstr, (path_name+kf_tmp).c_str());
    strcpy(cstr2, (path_name+mp_tmp).c_str());
    std::cout << cstr0 << std::endl;
    std::cout << mkdir(cstr0,  0777);
    std::cout << mkdir(cstr,  0777);
    std::cout << mkdir(cstr2,  0777);
    std::cout << std::endl;

    std::cout << "--> Writing Keyframes to file" << std::endl;
    auto keyframes = this->GetKeyframesVec();
    for(unsigned long int i = 0; i < keyframes.size(); i++) {
        std::ofstream fs;
        fs.open(path_name+"/keyframes/keyframes"+std::to_string(i)+".txt");
        if(fs.is_open()) {
            KeyframePtr kfi = keyframes[i];
            MsgKeyframe msg;
            kfi->ConvertToMsgFileExport(msg);

            std::stringstream kf_ss;
            cereal::BinaryOutputArchive oarchive(kf_ss);
            oarchive(msg);

            fs << kf_ss.str();
            fs.close();
        }
    }

    std::cout << "--> Writing Landmarks to file" << std::endl;
    auto landmarks = this->GetLandmarksVec();
    for(unsigned long int i = 0; i < landmarks.size(); i++) {
        LandmarkPtr lmi = landmarks[i];
        if(lmi->GetObservations().size() < 2) {
            continue;
        }
        if(!lmi->GetReferenceKeyframe()) {
            continue;
        }
        std::ofstream fs;
        fs.open(path_name+"/mappoints/mappoints"+std::to_string(i)+".txt");
        if(fs.is_open()) {
            MsgLandmark msg;
            lmi->ConvertToMsgFileExport(msg);

            std::stringstream mp_ss;
            cereal::BinaryOutputArchive oarchive(mp_ss);
            oarchive(msg);

            fs << mp_ss.str();
            fs.close();
        }
    }

    std::cout << "--> Writing Map Data to file" << std::endl;
    {
        std::ofstream fs;
        fs.open(path_name+"/mapdata"+".txt");
        if(fs.is_open()) {
            MsgMap msg;
            this->ConvertToMsgFileExport(msg);

            std::stringstream map_ss;
            cereal::BinaryOutputArchive oarchive(map_ss);
            oarchive(msg);

            fs << map_ss.str();
            fs.close();
        }
    }

    std::cout << "+++ DONE +++" << std::endl;
}

auto Map::UpdateCovisibilityConnections(idpair kf_id)->void {
    // Why this complicated access to update covisibility connections?
    // If UpdateCovisibilityConnections() is called for two neighboring KFs simultaneously, it will cause a deadlock since
    // they are accessing  1) their own connection (locking their own connection mutex)
    // and 2) the connections of the other KF, e.g. to add a connection, which requires to lock the connection mutex of the other KF
    // Therefore, we need a way to ensure that this method is only called for one KF at the same time.

    std::unique_lock<std::mutex> lock(mtx_update_connections_);
    if(kf_id != defpair) {
        KeyframePtr kf = this->GetKeyframe(kf_id);
        if(kf) kf->UpdateCovisibilityConnections();
    } else {
        std::unique_lock<std::mutex> lock(mtx_map_);
        for(const auto& mit : keyframes_) {
            KeyframePtr kf = mit.second;
            kf->UpdateCovisibilityConnections();
        }
    }
}

auto Map::WriteKFsToFile(std::string suffix)->void{
    for(std::set<size_t>::iterator sit = associated_clients_.begin();sit!=associated_clients_.end();++sit){
        int client_id = *sit;
        if(covins_params::sys::trajectory_format == "EUROC") {
            std::stringstream ss;
            ss << covins_params::sys::output_dir << "KF_" << client_id << suffix << "_feuroc" << ".csv";
            this->WriteStateToCsv(ss.str(),client_id);
        } else if(covins_params::sys::trajectory_format == "TUM") {
            std::stringstream ss;
            ss << covins_params::sys::output_dir << "KF_" << client_id << suffix << "_ftum" << ".csv";
            this->WriteStateToCsvTUM(ss.str(),client_id);
        } else {
            std::cout << COUTFATAL << "trajectory_format '" << covins_params::sys::trajectory_format << "' not in { EUROC | TUM }" << std::endl;
            exit(-1);
        }
    }
}

auto Map::WriteStateToCsv(const std::string& filename, const size_t client_id)->void {
    KeyframeVector found_kfs;
    found_kfs.reserve(keyframes_.size());
    // Get all frames from the required client
    for (KeyframeMap::const_iterator mit = keyframes_.begin(); mit != keyframes_.end(); ++mit) {
        KeyframePtr kf = mit->second;
        idpair curr_id = mit->first;
        if (curr_id.second == client_id) {
            found_kfs.push_back(kf);
        }
    }

    if(found_kfs.empty()) //do not overwrite files from other maps with empty files
    return;

    // Sort the keyframes by timestamp
    std::sort(found_kfs.begin(), found_kfs.end(), Keyframe::CompStamp);

    // Write out the keyframe data
    std::ofstream keyframes_file;
    keyframes_file.open(filename, std::ios::out | std::ios::trunc);
    if (keyframes_file.is_open()) {
        for (KeyframeVector::const_iterator vit = found_kfs.begin(); vit != found_kfs.end(); ++vit) {
            KeyframePtr kf = (*vit);
            const double stamp = kf->timestamp_;
            Eigen::Vector3d bias_accel, bias_gyro;
            kf->GetStateBias(bias_accel, bias_gyro);
            Eigen::Vector3d vel = kf->GetStateVelocity();
            const Eigen::Matrix4d Tws = kf->GetPoseTws();
            const Eigen::Quaterniond q(Tws.block<3,3>(0,0));

            keyframes_file << std::setprecision(25) << stamp * 1e9f << ",";
            keyframes_file << Tws(0,3) << "," << Tws(1,3) << "," << Tws(2,3) << ",";
            keyframes_file << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << ",";
            keyframes_file << vel[0] << "," << vel[1] << "," << vel[2] << ",";
            keyframes_file << bias_gyro[0] << "," << bias_gyro[1] << "," << bias_gyro[2] << ",";
            keyframes_file << bias_accel[0] << "," << bias_accel[1] << "," << bias_accel[2] << std::endl;
        }
        keyframes_file.close();
    }
    else
        std::cout << COUTERROR << ": Unable to open file: " << filename << std::endl;
}

auto Map::WriteStateToCsvTUM(const std::string& filename, const size_t client_id)->void {
    // TUM format for EVO: stamp tx ty tz qx qy qz qw - separated by spaces

    KeyframeVector found_kfs;
    found_kfs.reserve(keyframes_.size());
    // Get all frames from the required client
    for (KeyframeMap::const_iterator mit = keyframes_.begin(); mit != keyframes_.end(); ++mit) {
        KeyframePtr kf = mit->second;
        idpair curr_id = mit->first;
        if (curr_id.second == client_id) {
            found_kfs.push_back(kf);
        }
    }

    if(found_kfs.empty()) //do not overwrite files from other maps with empty files
    return;

    // Sort the keyframes by timestamp
    std::sort(found_kfs.begin(), found_kfs.end(), Keyframe::CompStamp);

    // Write out the keyframe data
    std::ofstream keyframes_file;
    keyframes_file.open(filename, std::ios::out | std::ios::trunc);
    if (keyframes_file.is_open()) {
        for (KeyframeVector::const_iterator vit = found_kfs.begin(); vit != found_kfs.end(); ++vit) {
            KeyframePtr kf = (*vit);
            const double stamp = kf->timestamp_;
            const Eigen::Matrix4d Tws = kf->GetPoseTws();
            const Eigen::Quaterniond q(Tws.block<3,3>(0,0));

            keyframes_file << std::setprecision(25) << stamp << " ";
            keyframes_file << Tws(0,3) << " " << Tws(1,3) << " " << Tws(2,3) << " ";
            keyframes_file << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        }
        keyframes_file.close();
    }
    else
        std::cout << COUTERROR << ": Unable to open file: " << filename << std::endl;
}

} //end ns
