#include "throttle_controller.hpp" 

ThrottleController::ThrottleController(ros::NodeHandle& nh)
{
    rcThrottleSub_ = nh.subscribe("/mavros/rc/in", 1, &ThrottleController::ThrottleSetpointCallback, this);
    throttlePub_ = nh.advertise<std_msgs::Float32>("/vrs_failsafe/throttle_setpoint", 1);
}

void ThrottleController::ThrottleSetpointCallback(const mavros_msgs::RCIn::ConstPtr& msg)
{
    throttle_ = msg->channels[2];
}

void ThrottleController::PubThrottleRequest(float throttle)
{
    std_msgs::Float32 throttleMsg;
    throttleMsg.data = 1;
    throttlePub_.publish(throttleMsg);
}

void ThrottleController::UpdateThrottleController(void)
{
    PubThrottleRequest(throttle_);
}