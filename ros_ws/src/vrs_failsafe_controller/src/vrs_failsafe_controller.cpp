#include "vrs_failsafe_controller.hpp"

VrsFailsafeController::VrsFailsafeController(ros::NodeHandle& nh)
{
    positionSub_ = nh.subscribe("/mavros/local_position/pose", 1, &VrsFailsafeController::LocalPositionCallback, this);
    velocitySub_ = nh.subscribe("/mavros/local_position/velocity", 1, &VrsFailsafeController::LocalVelocityCallback, this);
    accelSub_ = nh.subscribe("/mavros/imu/data", 1, &VrsFailsafeController::AccelCallback, this);
    guiStateMachine_ = nh.subscribe("/vrs_failsafe/state", 1, &VrsFailsafeController::StateMachineCallback, this);
    guiPositionTarget_ = nh.subscribe("/vrs_failsafe/setpoint_position", 1, &VrsFailsafeController::GUISetpointPositionCallback, this);

    positionTargetPub_ = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    thrustPub_ = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
    servoPub_ = nh.advertise<std_msgs::Float32>("/vrs_failsafe/servo_setpoint", 1);
}

// subscriber callbacks
void VrsFailsafeController::LocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    curLocalPosition_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
}
void VrsFailsafeController::LocalVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    curLocalVelocity_ << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;
}
void VrsFailsafeController::AccelCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    curAccel_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
}
void VrsFailsafeController::GUISetpointPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    setpointPosition_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    setpointYaw_ = msg->pose.orientation.z;
}
void VrsFailsafeController::StateMachineCallback(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data == "vrsFailsafe"){
        return;
    }
    curState_ = msg->data;
}
// publisher functions
void VrsFailsafeController::PubThrust(float thrust)
{
    mavros_msgs::AttitudeTarget thrustMsg;
    thrustMsg.header.stamp = ros::Time::now();
    thrustMsg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE | mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE | mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    thrustMsg.thrust = thrust;
    thrustPub_.publish(thrustMsg);
}

void VrsFailsafeController::PubFreeFall()
{
    mavros_msgs::PositionTarget freefallMsg;
    freefallMsg.header.stamp = ros::Time::now();
    freefallMsg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    // typemask ignores everything except for the position and accel z
    freefallMsg.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_VZ | mavros_msgs::PositionTarget::IGNORE_YAW | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    freefallMsg.position.x = curLocalPosition_[0];
    freefallMsg.position.y = curLocalPosition_[1];
    // freefall
    freefallMsg.acceleration_or_force.z = -9.81;
    positionTargetPub_.publish(freefallMsg);
}

void VrsFailsafeController::PubPositionSetpoint(float x, float y, float z, float yaw)
{
    mavros_msgs::PositionTarget localPositionMsg;
    localPositionMsg.header.stamp = ros::Time::now();
    localPositionMsg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    // typemask ignores everything except for the position
    localPositionMsg.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_VZ | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    localPositionMsg.position.x = x;
    localPositionMsg.position.y = y;
    localPositionMsg.position.z = z;
    localPositionMsg.yaw = yaw * M_PI / 180.0;
    positionTargetPub_.publish(localPositionMsg);
}

void VrsFailsafeController::UpdateNode(void)
{
    if (curState_ == "vrsFailsafe")
    {
        ThrottleController();
        PubThrust(throttleSetpoint_);
    }
    else if (curState_ == "freefall")
    {
        PubFreeFall();
    }
    else if (curState_ == "posSetpoint")
    {
        PubPositionSetpoint(setpointPosition_[0], setpointPosition_[1], setpointPosition_[2], setpointYaw_);
    }
}
    
    