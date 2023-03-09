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
*
* Modified by Manthan Patel, 2022 for COVINS-G Release
*/

#pragma once

// COVINS
#include "covins_base/map_base.hpp"
#include "covins_base/vocabulary.h"
#include "covins_base/kf_database_base.hpp"
#include "covins_base/placerec_base.hpp"

namespace covins {

class KeyframeDatabase;
class Map;

struct MapInstance {
    using TransformType                 = TypeDefs::TransformType;
    using MapPtr                        = TypeDefs::MapPtr;
    using MapInstancePtr                = std::shared_ptr<MapInstance>;

    MapInstance()                                                                       = delete;
    MapInstance(int id);
    MapInstance(MapInstancePtr map_target, MapInstancePtr map_tofuse, TransformType T_wmatch_wtofuse);
    MapInstance(MapPtr external_map);

    MapPtr                      map;
    int                         usage_cnt                                               = 0;
    std::set<int>               check_nums;
    bool                        block_checkout                                          = false;
};

struct MergeInformation {
    using TransformType                 = TypeDefs::TransformType;
    using KeyframePtr                   = TypeDefs::KeyframePtr;

    KeyframePtr                 kf_query;
    KeyframePtr                 kf_match;
    TransformType               T_smatch_squery                                         = TransformType::Zero();
    TypeDefs::Matrix6Type       cov_mat                                                 = TypeDefs::Matrix6Type::Identity();
}; 

class MapManager {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using idpair                        = TypeDefs::idpair;
    using Matrix3Type                   = TypeDefs::Matrix3Type;
    using TransformType                 = TypeDefs::TransformType;

    using KeyframePtr                   = TypeDefs::KeyframePtr;
    using MapPtr                        = TypeDefs::MapPtr;
    using MapInstancePtr                = std::shared_ptr<MapInstance>;

    using MapContainer                  = std::map<int,MapInstancePtr>;
    using MergeBuffer                   = std::list<MergeInformation, Eigen::aligned_allocator<MergeInformation>>;

    using DatabasePtr                   = std::shared_ptr<KeyframeDatabaseBase>;
    using VocabularyPtr                 = CovinsVocabulary::VocabularyPtr;

public:
    MapManager(VocabularyPtr voc);

    // Main
    auto Run()->void;

    // Interfaces
    auto CheckoutMap(int map_id, int& check_num)                                        ->MapPtr;
    auto CheckoutMapExclusive(int map_id, int& check_num)                               ->MapPtr;
    auto CheckoutMapExclusiveOrWait(int map_id, int& check_num)                         ->MapPtr;
    auto CheckoutMapOrWait(int map_id, int& check_num)                                  ->MapPtr;
    auto InitializeMap(int map_id)                                                      ->void;
    auto ReturnMap(int map_id, int& check_num)                                          ->void;
    auto SetCheckoutBlock(int map_id, bool val)                                         ->bool;
    auto RegisterMap(MapPtr external_map, bool enter_kfs_in_database)                   ->bool;

    auto RegisterMerge(MergeInformation merge_data)                                     ->void;

    auto GetVoc()                                                                       ->VocabularyPtr {   // will never change - no need to be guarded by mutex
        return voc_;
    }

    auto AddToDatabase(KeyframePtr kf)                                                  ->void;
    auto GetDatabase()                                                                  ->DatabasePtr;
    auto EraseFromDatabase(KeyframePtr kf)                                              ->void;

protected:

    auto CheckMergeBuffer()                                                             ->bool;
    auto PerformMerge()                                                                 ->void;

    // Data
    MapContainer                maps_;
    MergeBuffer                 buffer_merge_;
    DatabasePtr                 database_;
    VocabularyPtr               voc_;

    // Sync
    std::mutex                  mtx_access_;
};

//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------

struct MsgMap {
    size_t id_map;
    std::vector<TypeDefs::idpair> keyframes1;
    std::vector<TypeDefs::idpair> keyframes2;
    std::vector<TypeDefs::TransformType> transforms12;
    std::vector<TypeDefs::Matrix6Type> cov;

    template<class Archive> auto serialize( Archive & archive )                         ->void {
        archive(id_map,keyframes1,keyframes2,transforms12,cov);
    }
};

class Map : public MapBase, public std::enable_shared_from_this<Map> {
public:
    using idpair                        = TypeDefs::idpair;
    using Vector3Type                   = TypeDefs::Vector3Type;
    using Matrix3Type                   = TypeDefs::Matrix3Type;
    using TransformType                 = TypeDefs::TransformType;

    using MapPtr                        = TypeDefs::MapPtr;
    using ManagerPtr                    = TypeDefs::ManagerPtr;
    using VocabularyPtr                 = CovinsVocabulary::VocabularyPtr;

    using KeyframePairVector            = TypeDefs::KeyframePairVector;
    using LoopVector                    = TypeDefs::LoopVector;

public:
    Map()                                                                               = delete;
    Map(size_t id);
    Map(MapPtr map_target, MapPtr map_tofuse, TransformType T_wtarget_wtofuse);

    // Add / Erase data
    virtual auto AddKeyframe(KeyframePtr kf, bool suppress_output)                      ->void;
    virtual auto AddKeyframe(KeyframePtr kf)                                            ->void override;
    virtual auto AddLandmark(LandmarkPtr lm)                                            ->void override;
    virtual auto EraseKeyframe(KeyframePtr kf, bool mtx_lock = true)                    ->bool override;
    virtual auto EraseKeyframeWithDatabase(KeyframePtr kf, bool mtx_lock = true,
                                           MapManager::DatabasePtr database = nullptr)  ->bool;
    virtual auto EraseLandmark(LandmarkPtr lm, bool mtx_lock = true)                    ->bool override;

    virtual auto Clean()                                                                ->void;

    virtual auto RemoveRedundantData(ManagerPtr mapmanager, double th_red,
                                     size_t max_kfs
                                        = std::numeric_limits<size_t>::max())           ->size_t;
    virtual auto CalRedVals(KeyframeVector& kfs)                                        ->void;

    // Covisiblity Graph Functions
    virtual auto UpdateCovisibilityConnections(idpair kf_id = defpair)                  ->void;

    // Loop Correction
    virtual auto ApplyLoopCorrection(KeyframePtr kf_query,
                                     KeyframePtr kf_match,
                                     TransformType T_smatch_squery)                     ->void;
    virtual auto AddLoopConstraint(LoopConstraint lc)                                   ->void;
    virtual auto GetLoopConstraints()                                                   ->LoopVector;

    // Save/Load Data
    virtual auto SaveToFile(std::string const &path_name)                               ->void;
    virtual auto LoadFromFile(std::string const &path_name,
                              VocabularyPtr voc)                                        ->void;
    virtual auto ConvertToMsgFileExport(MsgMap &msg)                                    ->void;

    // Write-Out
    auto WriteKFsToFile(std::string suffix = std::string())                             ->void;

    auto WriteKFsToFileAllAg(std::string prefix = std::string())                        ->void;

protected:
    // Outlier Removal
    virtual auto RemoveLandmarkOutliers()                                               ->int;

    // Write-Out
    auto WriteStateToCsv(const std::string& filename,
                         const size_t client_id, const bool truncate = true)            ->void;
    auto WriteStateToCsvTUM(const std::string& filename,
                         const size_t client_id, const bool truncate = true)            ->void;

    // Loop Correction
    LoopVector                  loop_constraints_;

    // Sync
    std::mutex                  mtx_update_connections_;
};

} //end ns
