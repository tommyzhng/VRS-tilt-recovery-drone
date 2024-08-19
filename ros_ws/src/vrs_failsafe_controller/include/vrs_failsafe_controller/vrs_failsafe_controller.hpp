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
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ActuatorControl.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/CommandLong.h>
#include <std_msgs/Int32.h>

class VrsFailsafeController
{
public:
    void UpdateNode(void);
    VrsFailsafeController(ros::NodeHandle& nh);
    ~VrsFailsafeController() = default;

private:
    // subscriber callbacks
    void LocalPositionCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void LocalVelocityCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void AccelCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void AttitudeCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void TargetLocalPositionCallback(const mavros_msgs::PositionTarget::ConstPtr& msg);
    void TargetAttitudeCallback(const mavros_msgs::AttitudeTarget::ConstPtr& msg);
    void GUIStateMachineCallback(const std_msgs::String::ConstPtr& msg);
    void GUISetpointPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void GUIDropVelCallback(const std_msgs::Float32::ConstPtr& msg);
    void GUIServoSetpointCallback(const std_msgs::Float32::ConstPtr& msg);
    // publisher funcs
    void PubThrust(float thrust);
    void PubServo(float tilt);
    void PubFreeFall();
    void PubDropVel(float vel);
    void PubPositionSetpoint(float x, float y, float z, float yaw);
    void PubStateMachineLoopback();
    // controller funcs
    int ConvertTiltToPWM(float tilt, float servoMin, float servoMax);
    void CalculateTargetError();
    void ThrottleController();                                  
    void ServoController();
    void EstimateVRS();
    Eigen::Vector3f ToEulerAngles(const Eigen::Quaternionf& q);
    
    ros::Subscriber odomSub_;
    ros::Subscriber velocitySub_;
    ros::Subscriber accelSub_;
    ros::Subscriber attitudeSub_;
    ros::Subscriber targetPositionSub_;
    ros::Subscriber targetAttitudeSub_;
    ros::Subscriber guiStateMachineSub_;
    ros::Subscriber guiPositionSetpointSub_;
    ros::Subscriber guiDropVelSetpointSub_;
    ros::Subscriber guiServoSetpointSub_;

    ros::Publisher positionSetpointPub_;
    ros::Publisher thrustPub_;
    ros::Publisher servoPub_;
    ros::Publisher stateMachineLoopbackPub_;
    ros::Publisher servo1Pub;
    ros::Publisher servo2Pub;
    ros::ServiceClient servo_cmd_srv_;
    ros::Publisher attitudeErrorPub_;

    // subscriber data holders
    Eigen::Vector3d curLocalPosition_{0, 0, 0};
    Eigen::Vector3d curLocalVelocity_{0, 0, 0};
    Eigen::Vector3d curAccel_{0, 0, 0};
    Eigen::Quaternionf curAttitude_{1, 0, 0, 0};
    
    Eigen::Vector3d targetLocalPosition_{0, 0, 0};
    Eigen::Vector3d targetLocalVelocity_{0, 0, 0};
    Eigen::Vector3d targetAccel_{0, 0, 0};
    Eigen::Quaternionf targetAttitude_{1, 0, 0, 0};

    Eigen::Vector3f errorAttitudeEuler_{0,0,0};
    float lastStabilityError_{0};

    Eigen::Vector3d setpointPosition_{0, 0, 0};
    float setpointDropVel_{0};
    float setpointYaw_{0};
    std::string curState_{"posSetpoint"};

    // servo trim
    float servo1Min_;
    float servo1Max_;
    float servo2Min_;
    float servo2Max_;

    // failsafe vars
    float servoSetpoint_{0};
    float guiServoSetpoint_{0};
    float lastServoSetpoint_{0};
    float throttleSetpoint_{0};

    float p_velError_{0};
    float i_velErrorSum_{0};
    float lastVelError_{0};

    double servoKp;
    double servoKi;
    double servoKd;
    
    const float mass_ = 1.5;
    const float propRadius = 0.0762; // 3 inch propeller
    const float vh = sqrt((mass_ * 9.81/4)/(2*1.225*(M_PI*pow(propRadius, 2))));  // m/s

    // const float mass_ = 1.9;
    // const float propRadius = 0.127; // 3 inch propeller
    // const float vh = sqrt((mass_ * 9.81/4)/(2*1.225*(M_PI*pow(propRadius, 2))));  // m/s

    const float maxSafeVelocity = 0.28 * vh;    // m/s
};

#endif

