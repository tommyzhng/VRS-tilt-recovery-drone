#ifndef THROTTLE_CONTROLLER_HPP_THROTTLE_CONTROLLER_HPP
#define THROTTLE_CONTROLLER_HPP_THROTTLE_CONTROLLER_HPP

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <mavros_msgs/RCIn.h>

class ThrottleController
{
public:
    void UpdateThrottleController(void);
    ThrottleController(ros::NodeHandle& nh);
    ~ThrottleController() = default;
private:
    void ThrottleSetpointCallback(const mavros_msgs::RCIn::ConstPtr& msg);
    void PubThrottleRequest(float throttle);
    ros::Subscriber rcThrottleSub_;
    ros::Publisher throttlePub_;
    float throttle_;
};
#endif