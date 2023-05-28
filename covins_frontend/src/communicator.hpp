#pragma once

// C++
#include <unistd.h>
#include <mutex>
#include <thread>
#include <netinet/in.h>
#include <eigen3/Eigen/Eigen>

// CoVINS
#include <covins/covins_base/communicator_base.hpp>

#define ContainerSize 10

namespace covins {

class Communicator : public covins::CommunicatorBase {
public:
public:
    Communicator(std::string server_ip, std::string port);

    // main function
    virtual auto Run()                                                  ->void;

    // Interfaces
    virtual auto GetOdomFrame()                                         ->TransformType;
    virtual auto SetOdomFrame(TransformType T_world_odom)               ->void;

protected:

    // data handling
    virtual auto ProcessAdditional()                                    ->void;
    virtual auto ProcessKeyframeMessages()                              ->void;
    virtual auto ProcessLandmarkMessages()                              ->void;
    virtual auto ProcessNewKeyframes()                                  ->void;
    virtual auto ProcessNewLandmarks()                                  ->void;

    // Data
    TransformType           T_odom_correction_                                                  = TransformType::Identity();    // Tws_no_drift = T_odom_correction * Tws_est_with_drift

    //Sync
    std::mutex              mtx_odom_;
};

} //end ns
