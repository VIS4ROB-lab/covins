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

// C++
#include <eigen3/Eigen/Eigen>

// COVINS
#include "covins_base/optimization_base.hpp"

namespace covins {

class Optimization : public OptimizationBase {
public:
    Optimization()                                                                      = delete;

    static auto GlobalBundleAdjustment(MapPtr map,
                                       int interations_limit,
                                       double time_limit,
                                       bool visual_only = false,
                                       bool outlier_removal = true,
                                       bool estimate_bias = false)                      ->void;    
    
    static auto OptimizeRelativePose(KeyframePtr kf1, KeyframePtr kf2,
                                     LandmarkVector &matches1,
                                     TransformType& T12,
                                     const precision_t th2)                             ->int;

    static auto PoseGraphOptimization(MapPtr map,
                                      PoseMap corrected_poses)                          ->void;
        
};

} //end ns
