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
#include <mutex>
#include <eigen3/Eigen/Core>

// COVINS
#include <covins/covins_base/config_comm.hpp>
#include <covins/covins_base/config_backend.hpp>
#include <covins/covins_base/typedefs_base.hpp>

// Thirdparty
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>


#define MUL 2.0 //factor by that the KF at current agent position is scaled

namespace covins {

class Keyframe;
class Landmark;
class Map;

class VisualizerBase {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using precision_t                   = TypeDefs::precision_t;
    using idpair                        = TypeDefs::idpair;

    using TransformType                 = TypeDefs::TransformType;

    using KeyframePtr                   = TypeDefs::KeyframePtr;
    using LandmarkPtr                   = TypeDefs::LandmarkPtr;
    using MapPtr                        = TypeDefs::MapPtr;

    using KeyframeMap                   = TypeDefs::KeyframeMap;
    using LandmarkMap                   = TypeDefs::LandmarkMap;
    using KeyframeVector                = TypeDefs::KeyframeVector;
    using LandmarkVector                = TypeDefs::LandmarkVector;
    using KeyframePairVector            = TypeDefs::KeyframePairVector;
    using LoopVector                    = TypeDefs::LoopVector;

    struct VisBundle
    {
        KeyframeMap             keyframes;
        KeyframeMap             keyframes_erased;
        LandmarkMap             landmarks;
        size_t                  id_map;
        std::set<size_t>        associated_clients;
        LoopVector              loops;
        std::set<idpair>        most_recent_kfs;
        std::string             frame;
    };

public:
    VisualizerBase(std::string topic_prefix = std::string());

    // Main
    virtual auto Run()                                                                  ->void      = 0;

    // Reset
    virtual auto RequestReset()                                                         ->void;

    // Auxiliary Functions
    static auto CreatePoint3D(Eigen::Vector3d &p3D, size_t client_id)                   ->pcl::PointXYZRGB;
    static auto MakeColorMsg(float fR,float fG, float fB)                               ->std_msgs::ColorRGBA;

protected:

    virtual auto CheckVisData()                                                         ->bool;
    virtual auto ResetIfRequested()                                                     ->void;

    // Infrastructure
    ros::NodeHandle             nh_                                                     = ros::NodeHandle();
    ros::Publisher              pub_marker_;
    ros::Publisher              pub_cloud_;
    std::string                 topic_prefix_                                           = std::string();

    // Data
    std::map<size_t,VisBundle>  vis_data_;
    VisBundle                   curr_bundle_;

    // Reset
    bool                        reset_                                                  = false;

    // Sync
    std::mutex                  mtx_draw_;
    std::mutex                  mtx_reset_;
};

} //end ns
