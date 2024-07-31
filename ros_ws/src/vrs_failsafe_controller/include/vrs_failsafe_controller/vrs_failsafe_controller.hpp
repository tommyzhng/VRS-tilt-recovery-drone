/*
    * vrs_failsafe_controller.hpp
    * A simple failsafe controller testbed for the vortex ring state in a quad tiltrotor
*/

#ifndef VRS_FAILSAFE_CONTROLLER_VRS_FAILSAFE_CONTROLLER_HPP
#define VRS_FAILSAFE_CONTROLLER_VRS_FAILSAFE_CONTROLLER_HPP

#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Imu.h"
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <Eigen/Dense>

class VrsFailsafeController
{
public:

    void UpdateNode(void);
    VrsFailsafeController(ros::NodeHandle& nh);
    ~VrsFailsafeController() = default;

private:
    // subscriber callbacks
    void LocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void LocalVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void AccelCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void AttitudeCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void TargetLocalPositionCallback(const mavros_msgs::PositionTarget::ConstPtr& msg);
    void TargetAttitudeCallback(const mavros_msgs::AttitudeTarget::ConstPtr& msg);
    void StateMachineCallback(const std_msgs::String::ConstPtr& msg);
    void GUISetpointPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void GUIDropVelCallback(const std_msgs::Float32::ConstPtr& msg);
    // publisher funcs
    void PubThrust(float thrust);
    void PubServo(float angle);
    void PubFreeFall();
    void PubDropVel(float vel);
    void PubPositionSetpoint(float x, float y, float z, float yaw);
    // controller funcs
    void ThrottleController();                                  
    void ServoController();
    // estimation func
    void EstimateVRS();
    ros::Publisher positionSetpointPub_;
    ros::Publisher thrustPub_;
    ros::Publisher servoPub_;
    ros::Subscriber positionSub_;
    ros::Subscriber velocitySub_;
    ros::Subscriber accelSub_;
    ros::Subscriber attitudeSub_;
    ros::Subscriber targetPositionSub_;
    ros::Subscriber targetAttitudeSub_;
    ros::Subscriber guiStateMachine_;
    ros::Subscriber guiPositionSetpoint_;
    ros::Subscriber guiDropVelSetpoint_;
    
    // subscriber data holders
    Eigen::Vector3f curLocalPosition_{0, 0, 0};
    Eigen::Vector3f curLocalVelocity_{0, 0, 0};
    Eigen::Vector3f curAccel_{0, 0, 0};
    Eigen::Quaternionf curAttitude_{1, 0, 0, 0};
    
    Eigen::Vector3f targetLocalPosition_{0, 0, 0};
    Eigen::Vector3f targetLocalVelocity_{0, 0, 0};
    Eigen::Vector3f targetAccel_{0, 0, 0};
    Eigen::Quaternionf targetAttitude_{1, 0, 0, 0};

    Eigen::Vector3f setpointPosition_{0, 0, 0};
    float setpointDropVel_{0};
    float setpointYaw_{0};
    std::string curState_;

    // failsafe vars
    float throttle_{0};
    float servoAngle_{0};
    float lastServoAngle_{0};
    float throttleSetpoint_;
    
    const float mass_ = 1.0;
    const float propRadius = 0.127;
    const float vh = sqrt((mass_ * 9.81 / 4)/2*1.225*propRadius);

    const float maxSafeVelocity = 0.28 * vh;    // m/s
    const float maxYawError = 6.5;      // degrees
    const float maxRollPitchError = 5;  // degrees
};

#endif

