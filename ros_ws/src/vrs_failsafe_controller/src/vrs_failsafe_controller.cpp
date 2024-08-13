#include "vrs_failsafe_controller.hpp"

VrsFailsafeController::VrsFailsafeController(ros::NodeHandle& nh)
{
    odomSub_ = nh.subscribe("/mavros/local_position/odom", 1, &VrsFailsafeController::LocalPositionCallback, this);
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
    servoPub_ = nh.advertise<mavros_msgs::OverrideRCIn>("/mavross/actuator_control", 1);
    servo1Pub = nh.advertise<std_msgs::Int32>("/rcio/tilt1", 1);
    servo2Pub = nh.advertise<std_msgs::Int32>("/rcio/tilt2", 1);

    stateMachineLoopbackPub_ = nh.advertise<std_msgs::String>("/vrs_failsafe/state_machine_loopback", 1);


    // get params from params.json
    // "servo_pid": {
        // "ff": 0.001,
        // "kp": 0.1,
        // "ki": 0.0,
        // "kd": 0.0002
    // }
    nh.param("servo_pid/kp", servoKp, 0.0);
    nh.param("servo_pid/ki", servoKi, 0.0);
    nh.param("servo_pid/kd", servoKd, 0.0);
    nh.param("servo_trim/min1", servo1Min_, 1000.0f);
    nh.param("servo_trim/max1", servo1Max_, 2000.0f);
    nh.param("servo_trim/min2", servo2Min_, 1000.0f);
    nh.param("servo_trim/max2", servo2Max_, 2000.0f);

    ROS_INFO("VRS Failsafe Controller Initialized");
    ROS_INFO("Kp: %f, Ki: %f, Kd: %f", servoKp, servoKi, servoKd);
}

// subscriber callbacks
void VrsFailsafeController::LocalPositionCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    curLocalPosition_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
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
    float tiltNorm = -tilt/45;
    mavros_msgs::ActuatorControl servoMsg;
    servoMsg.header.stamp = ros::Time::now();
    servoMsg.group_mix = 0;
    servoMsg.controls[0] = tiltNorm;
    servoPub_.publish(servoMsg);

    // saturate output to -1 and 1
    if(tiltNorm < -1) {tiltNorm = -1;}
    else if (tiltNorm > 1) {tiltNorm = 1;}
    
    // publish to the servo node
    std_msgs::Int32 tiltMsg1;
    tiltMsg1.data = ConvertTiltToPWM(tiltNorm, servo1Min_, servo1Max_);
    servo1Pub.publish(tiltMsg1);
    std_msgs::Int32 tiltMsg2;
    tiltMsg2.data = ConvertTiltToPWM(tiltNorm, servo2Min_, servo2Max_);
    servo2Pub.publish(tiltMsg2);
}

int VrsFailsafeController::ConvertTiltToPWM(float tilt, float servoMin, float servoMax)
{
    // Ensure the input is within the range of -1 to +1
    tilt = std::max(-1.0f, std::min(1.0f, tilt));

    // The neutral PWM value (0 tilt)
    int neutralPWM = 1500;

    // Calculate the range above and below the neutral PWM
    float positiveRange = servoMax - neutralPWM;
    float negativeRange = neutralPWM - servoMin;

    // Map the tilt to the appropriate PWM value
    int pwmValue;
    if (tilt > 0) {
        pwmValue = static_cast<int>(tilt * positiveRange + neutralPWM);
    } else {
        pwmValue = static_cast<int>(tilt * negativeRange + neutralPWM);
    }

    return pwmValue;
}

void VrsFailsafeController::PubFreeFall()
{
    // mavros_msgs::PositionTarget freefallMsg;
    // freefallMsg.header.stamp = ros::Time::now();
    // freefallMsg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    // // typemask ignores everything except for the position and accel z
    // freefallMsg.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_VZ | mavros_msgs::PositionTarget::IGNORE_YAW | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    // freefallMsg.position.x = curLocalPosition_[0];
    // freefallMsg.position.y = curLocalPosition_[1];
    // // freefall
    // freefallMsg.acceleration_or_force.z = -9.81;
    // positionSetpointPub_.publish(freefallMsg);
        // Sine wave parameters
    double frequency = 1.0 / 3.0;  // Frequency in Hz
    double amplitude = 1.0;        // Amplitude of the sine wave
    double phase_shift = M_PI;     // Phase shift to make the wave range from 0 to -1

    // Current time
    double current_time = ros::Time::now().toSec();
    
    // Calculate the elapsed time
    static double start_time = current_time;  // Initialize start_time on the first call
    double elapsed_time = current_time - start_time;

    // Calculate the sine wave value
    double tilt = -0.5 * (std::sin(2 * M_PI * frequency * elapsed_time + phase_shift) + 1);
    PubServo(tilt);
    ROS_INFO("%f", tilt);

}

void VrsFailsafeController::PubDropVel(float vel)
{
    mavros_msgs::PositionTarget dropVelMsg;
    dropVelMsg.header.stamp = ros::Time::now();
    dropVelMsg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    // typemask ignores everything except for the velocity
    dropVelMsg.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    dropVelMsg.position.x = curLocalPosition_[0];
    dropVelMsg.position.y = curLocalPosition_[1];
    dropVelMsg.velocity.z = vel;
    dropVelMsg.yaw = ToEulerAngles(curAttitude_)[2];
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
    //EstimateVRS();
    PubStateMachineLoopback();
    if (curState_ == "vrsFailsafe") {
        ThrottleController();
        ServoController();
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
    
    