#include "throttle_controller.hpp" 

ThrottleController::ThrottleController(ros::NodeHandle& nh)
{
    // input subscribers
    // outputs
    throttlePub_ = nh.advertise<std_msgs::Float32>("/vrs_failsafe/throttle_setpoint", 1);
    servoPub_ = nh.advertise<std_msgs::Float32>("/vrs_failsafe/servo_setpoint", 1);
}

void ThrottleController::PubThrottleRequest(float throttle)
{
    std_msgs::Float32 throttleMsg;
    throttleMsg.data = 1;
    throttlePub_.publish(throttleMsg);
}

void ThrottleController::PubServoRequest(float angle)
{
    std_msgs::Float32 servoMsg;
    servoMsg.data = angle;
    servoPub_.publish(servoMsg);
}

void ThrottleController::PIDThrottleController()
{

}

void ThrottleController::ServoController()
{
    // write a sine wave type function with a period of 2 seconds and amplitude of 22.5 with offset at 22.5
    servoAngle_ = 22.5 * sin(2 * M_PI * 0.5 * ros::Time::now().toSec()) + 22.5;
}

void ThrottleController::UpdateController(void)
{
    //PIDThrottleController();
    //ServoController();
    PubThrottleRequest(throttle_);
    PubServoRequest(servoAngle_);
}