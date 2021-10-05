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
#include <opencv2/opencv.hpp>

// COVINS
#include <covins/covins_base/typedefs_base.hpp>
#include <covins/covins_base/config_comm.hpp>
#include <covins/covins_base/config_backend.hpp>

namespace covins {

class Keyframe;
class Landmark;

class FeatureMatcher {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using precision_t                   = TypeDefs::precision_t;
    using KeyframePtr                   = TypeDefs::KeyframePtr;
    using LandmarkPtr                   = TypeDefs::LandmarkPtr;
    using LandmarkVector                = TypeDefs::LandmarkVector;
    using LandmarkSet                   = TypeDefs::LandmarkSet;

public:
    FeatureMatcher(precision_t nnratio=0.6, bool check_orientation=true);

    // Computes the Hamming distance between two ORB descriptors
    static auto DescriptorDistanceHamming(const cv::Mat &a, const cv::Mat &b)           ->int;

    // Project MapPoints into KeyFrame using a given SE3 and search for duplicated MapPoints.
    auto Fuse(KeyframePtr pKF, Eigen::Matrix4d Tcw,
              const LandmarkVector &vpPoints, precision_t th,
              LandmarkVector &vpReplacePoint)                                           ->int;

    // Project MapPoints using a Similarity Transformation and search matches.
    auto SearchByProjection(KeyframePtr pKF, Eigen::Matrix4d Tcw,
                            const LandmarkVector &vpPoints,
                            LandmarkVector &vpMatched, precision_t th)                  ->int;

    // Search matches between MapPoints in KF1 and KF2 transforming by a SE3.
    auto SearchBySE3(KeyframePtr pKF1, KeyframePtr pKF2,
                     LandmarkVector& matches12,
                     const Eigen::Matrix4d T12, const precision_t th = 3.0)             ->int;

private:
    precision_t                 nnratio_;
    bool                        check_orientation_;

    const int                   desc_matching_th_low_                                   = covins_params::matcher::desc_matching_th_low;
    const int                   desc_matching_th_high_                                  = covins_params::matcher::desc_matching_th_high;
};

} //end ns
