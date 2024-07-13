/*
    * vrs_failsafe_controller.hpp
    * A simple failsafe controller for the vortex ring state in a quad tiltrotor
*/

#ifndef VRS_FAILSAFE_CONTROLLER_VRS_FAILSAFE_CONTROLLER_HPP
#define VRS_FAILSAFE_CONTROLLER_VRS_FAILSAFE_CONTROLLER_HPP

#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>

class VrsFailsafeController
{
public:

    void UpdateNode(void);
    VrsFailsafeController(ros::NodeHandle& nh);
    ~VrsFailsafeController() = default;

private:
    void LocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void ThrottleSetpointCallback(const std_msgs::Float32::ConstPtr& msg);
    //void FailsafeCallback(const std_msgs::Bool::ConstPtr& msg);
    void PubThrust(float thrust);
    void PubLocalPosition(float x, float y);

    ros::Publisher localPositionPub_;
    ros::Publisher thrustPub_;
    ros::Subscriber localPositionSub_;
    ros::Subscriber throttleSetpointSub_;
    ros::Subscriber failsafeSub_;

    geometry_msgs::PoseStamped lastLocalPosition_;
    float throttleSetpoint_;
};

#endif
