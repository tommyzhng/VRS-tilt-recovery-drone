#include "vrs_failsafe_controller.hpp"

VrsFailsafeController::VrsFailsafeController(ros::NodeHandle& nh)
{
    localPositionSub_ = nh.subscribe("/mavros/local_position/pose", 1, &VrsFailsafeController::LocalPositionCallback, this);
    throttleSetpointSub_ = nh.subscribe("/vrs_failsafe/throttle_setpoint", 1, &VrsFailsafeController::ThrottleSetpointCallback, this);
    //failsafeSub_ = nh.subscribe("/vrs_failsafe_controller/failsafe", 1, &VrsFailsafeController::FailsafeCallback, this);
    localPositionPub_ = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    thrustPub_ = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
}

// subscriber callbacks
void VrsFailsafeController::LocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    lastLocalPosition_ = *msg;
}

void VrsFailsafeController::ThrottleSetpointCallback(const std_msgs::Float32::ConstPtr& msg)
{
    throttleSetpoint_ = msg->data;
}

// publisher functions
void VrsFailsafeController::PubThrust(float thrust)
{
    mavros_msgs::AttitudeTarget thrustMsg;
    thrustMsg.header.stamp = ros::Time::now();
    thrustMsg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE | mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE | mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE | mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    thrustMsg.thrust = thrust;
    thrustPub_.publish(thrustMsg);
}

void VrsFailsafeController::PubLocalPosition(float x, float y)
{
    mavros_msgs::PositionTarget localPositionMsg;
    localPositionMsg.header.stamp = ros::Time::now();
    localPositionMsg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    // typemask ignores everything except for the position
    mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ | mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_YAW | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    localPositionMsg.position.x = x;
    localPositionMsg.position.y = y;
    localPositionPub_.publish(localPositionMsg);
}

void VrsFailsafeController::UpdateNode(void)
{
    PubThrust(throttleSetpoint_);
    PubLocalPosition(0, 0);
}