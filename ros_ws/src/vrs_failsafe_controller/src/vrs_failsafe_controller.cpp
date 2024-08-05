#include "vrs_failsafe_controller.hpp"

VrsFailsafeController::VrsFailsafeController(ros::NodeHandle& nh)
{
    positionSub_ = nh.subscribe("/mavros/local_position/pose", 1, &VrsFailsafeController::LocalPositionCallback, this);
    velocitySub_ = nh.subscribe("/mavros/local_position/odom", 1, &VrsFailsafeController::LocalVelocityCallback, this);
    accelSub_ = nh.subscribe("/mavros/imu/data", 1, &VrsFailsafeController::AccelCallback, this);
    attitudeSub_ = nh.subscribe("/mavros/imu/data", 1, &VrsFailsafeController::AttitudeCallback, this);
    targetPositionSub_ = nh.subscribe("/mavros/setpoint_raw/target_local", 1, &VrsFailsafeController::TargetLocalPositionCallback, this);
    targetAttitudeSub_ = nh.subscribe("/mavros/setpoint_raw/target_attitude", 1, &VrsFailsafeController::TargetAttitudeCallback, this);
   
    guiStateMachineSub_ = nh.subscribe("/vrs_failsafe/state", 1, &VrsFailsafeController::GUIStateMachineCallback, this);
    guiPositionSetpointSub_ = nh.subscribe("/vrs_failsafe/setpoint_position", 1, &VrsFailsafeController::GUISetpointPositionCallback, this);
    guiDropVelSetpointSub_ = nh.subscribe("/vrs_failsafe/setpoint_drop_vel", 1, &VrsFailsafeController::GUIDropVelCallback, this);
    guiServoSetpointSub_ = nh.subscribe("/vrs_failsafe/gui_servo_setpoint", 1, &VrsFailsafeController::GUIServoSetpointCallback, this);

    positionSetpointPub_ = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    thrustPub_ = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
    servoPub_ = nh.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control", 1);
    stateMachineLoopbackPub_ = nh.advertise<std_msgs::String>("/vrs_failsafe/state_machine_loopback", 1);

    
}

// subscriber callbacks
void VrsFailsafeController::LocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    curLocalPosition_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
}
void VrsFailsafeController::LocalVelocityCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    curLocalVelocity_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
}
void VrsFailsafeController::AccelCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    curAccel_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
}
void VrsFailsafeController::AttitudeCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    curAttitude_ = Eigen::Quaternionf(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
}
void VrsFailsafeController::TargetLocalPositionCallback(const mavros_msgs::PositionTarget::ConstPtr& msg)
{
    targetLocalPosition_ << msg->position.x, msg->position.y, msg->position.z;
}
void VrsFailsafeController::TargetAttitudeCallback(const mavros_msgs::AttitudeTarget::ConstPtr& msg)
{
    targetAttitude_ = Eigen::Quaternionf(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
}
void VrsFailsafeController::GUISetpointPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    setpointPosition_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    setpointYaw_ = msg->pose.orientation.z;
}
void VrsFailsafeController::GUIDropVelCallback(const std_msgs::Float32::ConstPtr& msg)
{
    setpointDropVel_ = msg->data;
}
void VrsFailsafeController::GUIServoSetpointCallback(const std_msgs::Float32::ConstPtr& msg)
{
    guiServoSetpoint_ = msg->data;
}
void VrsFailsafeController::GUIStateMachineCallback(const std_msgs::String::ConstPtr& msg)
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
    thrustMsg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE | mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE | mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    thrustMsg.orientation.w = 1;
    thrustMsg.orientation.x = 0;
    thrustMsg.orientation.y = 0;
    thrustMsg.orientation.z = 0;
    thrustMsg.thrust = thrust;
    thrustPub_.publish(thrustMsg);
}

void VrsFailsafeController::PubServo(float tilt)
{
    // normalize tilt between -1 and 1 from -45 and 45
    float tiltNorm = tilt / 45;
    mavros_msgs::ActuatorControl servoMsg;
    servoMsg.header.stamp = ros::Time::now();
    servoMsg.group_mix = 0;
    servoMsg.controls[5] = tiltNorm;
    servoMsg.controls[6] = tiltNorm;
    servoPub_.publish(servoMsg);
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
    positionSetpointPub_.publish(freefallMsg);
}

void VrsFailsafeController::PubDropVel(float vel)
{
    mavros_msgs::PositionTarget dropVelMsg;
    dropVelMsg.header.stamp = ros::Time::now();
    dropVelMsg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    // typemask ignores everything except for the velocity
    dropVelMsg.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_YAW | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    dropVelMsg.position.x = curLocalPosition_[0];
    dropVelMsg.position.y = curLocalPosition_[1];
    dropVelMsg.velocity.z = vel;
    positionSetpointPub_.publish(dropVelMsg);
}

void VrsFailsafeController::PubPositionSetpoint(float x, float y, float z, float yaw)
{
    mavros_msgs::PositionTarget localPositionMsg;
    localPositionMsg.header.stamp = ros::Time::now();
    localPositionMsg.header.frame_id = "home";
    localPositionMsg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    // typemask ignores everything except for the position
    localPositionMsg.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX + mavros_msgs::PositionTarget::IGNORE_AFY + mavros_msgs::PositionTarget::IGNORE_AFZ + mavros_msgs::PositionTarget::IGNORE_VZ + mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    //localPositionMsg.type_mask = 4088;
    localPositionMsg.position.x = x;
    localPositionMsg.position.y = y;
    localPositionMsg.position.z = z;
    localPositionMsg.yaw = yaw * M_PI / 180.0;
    positionSetpointPub_.publish(localPositionMsg);    
}

void VrsFailsafeController::PubStateMachineLoopback()
{
    std_msgs::String loopbackMsg;
    loopbackMsg.data = curState_;
    stateMachineLoopbackPub_.publish(loopbackMsg);
}

void VrsFailsafeController::UpdateNode(void)
{
    EstimateVRS();
    PubStateMachineLoopback();
    if (curState_ == "vrsFailsafe") {
        ThrottleController();
        ServoController();
        ROS_INFO("Throttle: %f, Servo: %f", throttleSetpoint_, servoSetpoint_);
    } else if (curState_ == "freefall") {
        PubFreeFall();
    } else if (curState_ == "dropVelSetpoint") {
        PubDropVel(setpointDropVel_);
    } else if (curState_ == "posSetpoint") {
        PubPositionSetpoint(setpointPosition_[0], setpointPosition_[1], setpointPosition_[2], setpointYaw_);
        PubServo(guiServoSetpoint_);
    } else {
        return;
    }
}
    
    