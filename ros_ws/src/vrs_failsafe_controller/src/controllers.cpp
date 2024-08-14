#include "vrs_failsafe_controller.hpp"
Eigen::Vector3f VrsFailsafeController::ToEulerAngles(const Eigen::Quaternionf& q) {
    Eigen::Vector3f angles;    //yaw pitch roll
    const auto x = q.x();
    const auto y = q.y();
    const auto z = q.z();
    const auto w = q.w();

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    angles[0] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        angles[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles[1] = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    angles[2] = std::atan2(siny_cosp, cosy_cosp);
    return angles;
}

/*  calculate error between target published by px4 and current measured by imu */
void VrsFailsafeController::CalculateTargetError()
{
    targetAttitude_.normalize();
    curAttitude_.normalize();
    // store  quaternion error in temp var and convert to euler angles
    Eigen::Quaternionf q_err = targetAttitude_ * curAttitude_.inverse();
    errorAttitudeEuler_ = ToEulerAngles(q_err); 
    errorAttitudeEuler_ = errorAttitudeEuler_ * 180 / M_PI;

    // print the euler
    //ROS_INFO("Error: %f", errorAttitudeEuler_(1));
}

/*  open loop control for majority of recovery, then switch to closed loop to stabilize
    because response is unknown at this stage since there is assumed to be a random sine wave applied to thrust */
void VrsFailsafeController::ThrottleController()
{
    // open loop control
    if (curLocalVelocity_(2) <= - 0.28 * vh) {
        throttleSetpoint_ = 1.0;
        PubThrust(throttleSetpoint_);
        return;
    } 
    ROS_INFO("Switched to closed loop control");
    // // closed loop control in acceleration with pid and tilt angle feedforward
    // p_velError_ = 0 - curLocalVelocity_(2);
    // i_velErrorSum_ += p_velError_;
    // double d_velErrorDer = (p_velError_ - lastVelError_) / ros::Duration(0.01).toSec();;
    // lastVelError_ = p_velError_;

    // float T_u = 0.5 * 0.7;
    // float T_d = 0.125 * 0.7;

    // throttleSetpoint_ = 0.0001 * p_velError_ + 1 * i_velErrorSum_ + 1 * d_velErrorDer;

    // // saturate the throttle setpoint
    // if (throttleSetpoint_ > 1.0) {
    //     throttleSetpoint_ = 1.0;
    // } else if (throttleSetpoint_ < 0.0) {
    //     throttleSetpoint_ = 0.0;
    // }

    setpointDropVel_ = 0;
    curState_ = "dropVelSetpoint";

    //PubThrust(throttleSetpoint_);
}
/*  response should be based on stability of the drone, in terms of roll pitch and yaw
    more instability should result in more servo tilt */
void VrsFailsafeController::ServoController()
{
    // initially set to 45 degree angle to disrupt the cyclic circulation
    // go back to 0 angle with criterion to get maximum thrust in the down direction
    if (curLocalVelocity_(2) > - 0.28*vh) {
        servoSetpoint_ = 0;
        PubServo(servoSetpoint_);
        return;
    } 
    ROS_INFO("Servo: %f Throttle %f", servoSetpoint_, throttleSetpoint_);

    if (curAccel_(2) < 9.81) { // check if vehicle is still accelerating downwards
        servoSetpoint_ = 45;
        PubServo(servoSetpoint_);
        return;
    }

    // closed loop control for tilt 
    //auto velError = curLocalVelocity_(2) - 0;
    // get average of error magnitude
    auto stabilityError = (std::abs(errorAttitudeEuler_(0)) + std::abs(errorAttitudeEuler_(1)) + std::abs(errorAttitudeEuler_(2))) / 3;
    auto stabilityError_d = (stabilityError - lastStabilityError_) / ros::Duration(0.01).toSec();
    auto stabilityOutput = servoKp * stabilityError + servoKd * stabilityError_d;
    //servoSetpoint_ = velError + stabilityOutput;
    servoSetpoint_ = std::max(20.0f, std::min(45.0f, throttleSetpoint_));        // clamp output

    PubServo(servoSetpoint_);
    lastStabilityError_ = stabilityError;
}

void VrsFailsafeController::EstimateVRS()
{   
    //ROS_INFO("Current State: %s", curState_.c_str());
    CalculateTargetError();
    // satisfy some critereons to change curState to vrsDetected
    if (curLocalVelocity_(2) < -0.28 * vh) {  // Xin and Gao criterion
        // print velocityt and 0.28 * vh
        if ((sqrt(pow(curLocalVelocity_(0), 2) + pow(curLocalVelocity_(1), 2))/vh) < 0.7) {  // see if velocity magnitude is less than 0.7 m/s
            if (errorAttitudeEuler_(0) > 5 || errorAttitudeEuler_(1) > 5 || errorAttitudeEuler_(2) > 5) {  // check if roll and pitch are within 5 degrees
                curState_ = "vrsFailsafe";
                //ROS_WARN("VRS Detected");
            }
        }
    }

}