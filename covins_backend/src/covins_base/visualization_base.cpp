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

#include "covins_base/visualization_base.hpp"

namespace covins {

VisualizerBase::VisualizerBase(std::string topic_prefix)
    : topic_prefix_(topic_prefix)
{
    std::string marker_topic = "covins_markers";
    std::string cloud_topic = "covins_cloud";
    marker_topic += topic_prefix;
    cloud_topic += topic_prefix;
    pub_marker_ = nh_.advertise<visualization_msgs::Marker>(marker_topic,10);
    pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_topic,10);
}

auto VisualizerBase::CheckVisData()->bool
{
    std::unique_lock<std::mutex> lock(mtx_draw_);
    return !vis_data_.empty();
}

auto VisualizerBase::CreatePoint3D(Eigen::Vector3d &p3D, size_t client_id)->pcl::PointXYZRGB {
    pcl::PointXYZRGB p;
    p.x = covins_params::vis::scalefactor*p3D(0);
    p.y = covins_params::vis::scalefactor*p3D(1);
    p.z = covins_params::vis::scalefactor*p3D(2);

    if(client_id < 12) {
        p.r = static_cast<uint8_t>(covins_params::colors::col_vec[client_id].mu8R);
        p.g = static_cast<uint8_t>(covins_params::colors::col_vec[client_id].mu8G);
        p.b = static_cast<uint8_t>(covins_params::colors::col_vec[client_id].mu8B);
    } else {
        p.r = 127;
        p.g = 127;
        p.b = 127;
    }

    return p;
}

auto VisualizerBase::MakeColorMsg(float fR,float fG, float fB)->std_msgs::ColorRGBA {
    std_msgs::ColorRGBA msg;
    msg.r = fR;
    msg.g = fG;
    msg.b = fB;
    msg.a = 1.0;
    return msg;
}

auto VisualizerBase::RequestReset()->void {
    {
        std::unique_lock<std::mutex> lock(mtx_reset_);
        reset_ = true;
    }

    while(1){
        {
            std::unique_lock<std::mutex> lock(mtx_reset_);
            if(!reset_)
                break;
        }
        usleep(1000);
    }
}

auto VisualizerBase::ResetIfRequested()->void
{
    std::unique_lock<std::mutex> lock(mtx_reset_);

    if(reset_) {
        std::unique_lock<std::mutex> lock(mtx_draw_);
        vis_data_.clear();
        reset_=false;
    }
}

} //end ns
