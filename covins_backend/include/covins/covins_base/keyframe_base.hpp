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
#include <vector>
#include <set>
#include <mutex>
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>

// COVINS
#include <covins/covins_base/config_comm.hpp>
#include <covins/covins_base/config_backend.hpp>
#include <covins/covins_base/typedefs_base.hpp>

// Thirdparty
#include <aslam/cameras/camera.h>
#include <robopt_open/common/definitions.h>
#include <robopt_open/imu-error/preintegration-base.h>

namespace covins {

class Keyframe;
class Landmark;
struct MsgKeyframe;
struct PreintegrationData;

class KeyframeBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using precision_t                   = TypeDefs::precision_t;
    using idpair                        = TypeDefs::idpair;

    using Vector3Type                   = TypeDefs::Vector3Type;
    using Vector4Type                   = TypeDefs::Vector4Type;
    using QuaternionType                = TypeDefs::QuaternionType;
    using TransformType                 = TypeDefs::TransformType;
    using Matrix3Type                   = TypeDefs::Matrix3Type;
    using KeypointType                  = TypeDefs::KeypointType;
    using AorsType                      = TypeDefs::AorsType;
    using KeypointVector                = TypeDefs::KeypointVector;
    using AorsVector                    = TypeDefs::AorsVector;

    using KeyframePtr                   = TypeDefs::KeyframePtr;
    using LandmarkPtr                   = TypeDefs::LandmarkPtr;
    using PreintegrationPtr             = std::shared_ptr<robopt::imu::PreintegrationBase>;
    using KeyframeVector                = TypeDefs::KeyframeVector;
    using KeyframeSet                   = TypeDefs::KeyframeSet;
    using LandmarkVector                = TypeDefs::LandmarkVector;
    using IntKfPair                     = TypeDefs::IntKfPair;
    using VectorIntKfPair               = TypeDefs::VectorIntKfPair;

    static auto CompStampBase(std::shared_ptr<KeyframeBase> kf1,
                              std::shared_ptr<KeyframeBase> kf2)                        ->bool {
        return  kf1->timestamp_ > kf2->timestamp_;
    }

public:
    KeyframeBase(idpair id, double timestamp, VICalibration calib,
                 int img_dim_x_min, int img_dim_y_min,
                 int img_dim_x_max,int img_dim_y_max);
    KeyframeBase(idpair id, double timestamp, VICalibration calib,
                 PreintegrationPtr preintegration,
                 int img_dim_x_min, int img_dim_y_min,
                 int img_dim_x_max,int img_dim_y_max);

    // Msg interfaces
    virtual auto ConvertToMsg(MsgKeyframe &msg,
                             KeyframePtr kf_ref, bool is_update)                        ->void      = 0;
    auto ConvertPreintegrationToMsg(PreintegrationData &data)                           ->void;

    // Interfaces
    virtual auto GetPredecessor()                                                       ->KeyframePtr;
    virtual auto GetSuccessor()                                                         ->KeyframePtr;
    virtual auto GetPoseTcw()                                                           ->TransformType;
    virtual auto GetPoseTwc()                                                           ->TransformType;
    virtual auto GetPoseTws()                                                           ->TransformType;
    virtual auto GetPoseTsw()                                                           ->TransformType;
    virtual auto GetPoseTws_vio()                                                       ->TransformType;
    virtual auto GetPoseTsw_vio()                                                       ->TransformType;
    virtual auto GetStateBias(Vector3Type& ba,Vector3Type& bg)                          ->void;
    virtual auto GetStateVelocity()                                                     ->Vector3Type;
    virtual auto GetStateExtrinsics()                                                   ->TransformType;
    virtual auto GetLinAccAngVel(Vector3Type& lin_acc,
                                 Vector3Type& ang_vel)                                  ->void;

    virtual auto IsInvalid()                                                            ->bool;
    virtual auto IsInImage(const precision_t x, const precision_t y)                    ->bool;

    virtual auto SetPredecessor(KeyframePtr kf)                                         ->void;
    virtual auto SetSuccessor(KeyframePtr kf)                                           ->void;
    virtual auto SetPoseTws(TransformType Tws, bool lock_mtx = true)                    ->void;
    virtual auto SetPoseTws_vio(TransformType Tws, bool lock_mtx = true)                ->void;
    virtual auto SetStateBias(Vector3Type ba, Vector3Type bg)                           ->void;
    virtual auto SetStateVelocity(Vector3Type vel)                                      ->void;
    virtual auto SetLinAccAngVel(Vector3Type lin_acc,
                                 Vector3Type ang_vel)                                   ->void;

    virtual auto GetLandmark(int index)                                                 ->LandmarkPtr;
    virtual auto GetLandmarkIndex(LandmarkPtr lm)                                       ->int;
    virtual auto GetLandmarks()                                                         ->LandmarkVector;
    virtual auto GetValidLandmarks()                                                    ->LandmarkVector;
    virtual auto AddLandmark(LandmarkPtr landmark, int index)                           ->bool;
    virtual auto EraseLandmark(size_t index)                                            ->void;
    virtual auto EraseLandmark(LandmarkPtr lm)                                          ->void;
    virtual auto EraseLandmark(LandmarkPtr lm, size_t index)                            ->void;

    virtual auto GetDescriptorCV(size_t ind)                                            ->cv::Mat;
    virtual auto GetDescriptor(size_t ind)                                              ->const unsigned char*;

    virtual auto GetFeaturesInArea(const precision_t x,
                                   const precision_t y,
                                   const precision_t r)                                 ->std::vector<size_t>;

    virtual auto GetFeaturesInArea(const TypeDefs::KeypointType target,
                                   const precision_t radius)                            ->std::vector<size_t>;

    // Covisiblity Graph Functions
    virtual auto AddConnectedKeyframe(KeyframePtr kf, int weight)                       ->void;
    virtual auto GetConnectedKeyframesByWeight(int weight)                              ->KeyframeVector;
    virtual auto GetConnectedNeighborKeyframes()                                        ->KeyframeVector;
    virtual auto EraseConnectedKeyframe(KeyframePtr kf)                                 ->void;
    virtual auto GetConnectionWeight(KeyframePtr kf)                                    ->int;

    // Update the state data from the ceres state
    virtual auto UpdateFromCeres(const precision_t* ceres_pose,
                                 const precision_t* ceres_velocity_and_bias,
                                 const precision_t* ceres_extrinsics)                   ->void;

    virtual auto UpdateCeresFromState(precision_t* ceres_pose,
                                      precision_t* ceres_velocity_and_bias,
                                      precision_t* ceres_extrinsics)                    ->void;

    // Identifier
    idpair                      id_;
    double                      timestamp_;

    // Calibration & image size
    VICalibration               calibration_;
    aslam::Camera::Ptr          camera_;
    int                         img_dim_x_min_;
    int                         img_dim_y_min_;
    int                         img_dim_x_max_;
    int                         img_dim_y_max_;

    // IMU Preintegration
    PreintegrationPtr           preintegrated_imu_;

    // KeyPoints and descriptors (all associated by an index)
    KeypointVector              keypoints_distorted_;
    KeypointVector              keypoints_undistorted_;
    AorsVector                  keypoints_aors_;                                                            //Angle,Octave,Response,Size
    cv::Mat                     descriptors_;

    // Ceres Variable Access (Pose as Tws)
    precision_t                 ceres_pose_[robopt::defs::pose::kPoseBlockSize];                            //qx,qy,qz,qw,X,Y,Z
    precision_t                 ceres_pose_local_[robopt::defs::pose::kPoseBlockSize];                            //qx,qy,qz,qw,X,Y,Z
    precision_t                 ceres_velocity_and_bias_[robopt::defs::pose::kSpeedBiasBlockSize];
    precision_t                 ceres_extrinsics_[robopt::defs::pose::kPoseBlockSize];

    // GT
    TransformType               T_w_s_gt                                                = TransformType::Identity();

    // These vars should not be acessed in parallel (anyway, guard them later;
    bool                        is_loaded_                                              = false;
    bool                        is_gba_optimized_                                       = false;

protected:
    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    virtual auto AssignFeaturesToGrid()                                                 ->void;
    double                      grid_width_inv_;
    double                      grid_height_inv_;
    bool                        assigned_to_grid_ = false;
    std::vector<size_t>         keypoint_grid_[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Covisiblity Graph Functions
    virtual auto SortConnectedKeyframes(bool lock_mtx = true)                           ->void;

    // SE3 Pose, Bias, Velocity
    TransformType               T_s_c_                                                  = TransformType::Identity();    // Tranformation IMU-Cam

    TransformType               T_w_s_                                                  = TransformType::Identity();
    TransformType               T_w_s_vio_                                              = TransformType::Identity();
    TransformType               T_s_w_vio_                                              = TransformType::Identity();
    TransformType               T_s_w_                                                  = TransformType::Identity();
    TransformType               T_c_w_                                                  = TransformType::Identity();
    TransformType               T_w_c_                                                  = TransformType::Identity();

    Vector3Type                 velocity_                                               = Vector3Type::Zero();          // in world frame
    Vector3Type                 bias_accel_                                             = Vector3Type::Zero();
    Vector3Type                 bias_gyro_                                              = Vector3Type::Zero();
    Vector3Type                 lin_acc_                                                = Vector3Type::Zero();          // could be avoided to send: is the last IMU measurment
    Vector3Type                 ang_vel_                                                = Vector3Type::Zero();          // could be avoided to send: is the last IMU measurment

    // Neighborhood
    LandmarkVector              landmarks_;
    KeyframePtr                 predecessor_;
    KeyframePtr                 successor_;

    // Covisiblity Graph
    KeyframeVector              connected_kfs_;
    KeyframeVector              connected_n_kfs_;
    std::vector<int>            connections_weights_;

    // Infrastructure
    bool                        invalid_                                                = false;

    // Mutexes
    std::mutex                  mtx_connections_;
    std::mutex                  mtx_features_;
    std::mutex                  mtx_pose_;
    std::mutex                  mtx_invalid_;
};

} //end ns
