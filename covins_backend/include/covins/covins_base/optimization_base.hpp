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
#include <eigen3/Eigen/Core>

// COVINS
#include <covins/covins_base/config_comm.hpp>
#include <covins/covins_base/config_backend.hpp>
#include <covins/covins_base/typedefs_base.hpp>

namespace covins {

class Frame;
class Keyframe;
class Landmark;
class Map;

class OptimizationBase {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using precision_t                   = TypeDefs::precision_t;
    using idpair                        = TypeDefs::idpair;

    using Vector2Type                   = TypeDefs::Vector2Type;
    using Vector3Type                   = TypeDefs::Vector3Type;
    using Vector4Type                   = TypeDefs::Vector4Type;
    using KeypointType                  = TypeDefs::Vector2Type;
    using QuaternionType                = TypeDefs::QuaternionType;
    using Matrix3Type                   = TypeDefs::Matrix3Type;
    using TransformType                 = TypeDefs::TransformType;

    using KeyframePtr                   = TypeDefs::KeyframePtr;
    using LandmarkPtr                   = TypeDefs::LandmarkPtr;
    using MapPtr                        = TypeDefs::MapPtr;

    using KeyframeList                  = TypeDefs::KeyframeList;
    using LandmarkList                  = TypeDefs::LandmarkList;
    using KeyframeVector                = TypeDefs::KeyframeVector;
    using LandmarkVector                = TypeDefs::LandmarkVector;
    using KeyframeSet                   = TypeDefs::KeyframeSet;
    using PoseMap                       = TypeDefs::PoseMap;
    using Vector3Vector                 = TypeDefs::Vector3Vector;
    using KfObservations                = TypeDefs::KfObservations;
    
    struct KeypointIdentifier {
        KeyframePtr keyframe;
        LandmarkPtr landmark;
        size_t keypoint_id;
        KeypointIdentifier(KeyframePtr kf, LandmarkPtr lm, size_t kp_id):
            keyframe(kf), landmark(lm), keypoint_id(kp_id) {}
    };
};

} //end ns
