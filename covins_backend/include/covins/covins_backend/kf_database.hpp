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
#include <memory>
#include <mutex>
#include <list>
#include <set>
#include <utility>
#include <vector>
#include <eigen3/Eigen/Core>

// COVINS
#include <covins/covins_base/config_comm.hpp>
#include <covins/covins_base/config_backend.hpp>
#include <covins/covins_base/typedefs_base.hpp>
#include "covins_base/vocabulary.h"
#include "covins_base/kf_database_base.hpp"

namespace covins {

class Keyframe;

class KeyframeDatabase : public KeyframeDatabaseBase {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using VocabularyPtr                 = CovinsVocabulary::VocabularyPtr;
    using KeyframeList                  = TypeDefs::KeyframeList;
    using KeyframeSet                   = TypeDefs::KeyframeSet;

    using InvertedIndex                 = std::vector<KeyframeList>;
    using listFloatKfPair               = std::list<std::pair<float, KeyframePtr>, Eigen::aligned_allocator<std::pair<float,KeyframePtr>>>;

public:
    KeyframeDatabase(const VocabularyPtr voc);

    // Interfacing
    virtual auto AddKeyframe(KeyframePtr kf)                                            ->void override;
    virtual auto EraseKeyframe(KeyframePtr kf)                                          ->void override;

    // Loop Detection
    virtual auto DetectCandidates(KeyframePtr kf, precision_t min_score)                ->KeyframeVector override;

private:
    // Associated vocabulary
    const VocabularyPtr         voc_;

    // Inverted file
    InvertedIndex               inverted_file_index_;

    // Mutex
    std::mutex                  mtx_;
};

} // end ns
