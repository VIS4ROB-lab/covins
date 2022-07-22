
#include "frontend_wrapper.hpp"
#include <ros/ros.h>


int main(int argc, char* argv[]) {

    ros::init(argc, argv, "frontend_node");
    ROS_INFO("\nStarting Frontend ROS Wrapper node with name %s\n",
             ros::this_node::getName().c_str());

    covins::FrontendWrapper frontend;
    frontend.run();
    return 0;
}
