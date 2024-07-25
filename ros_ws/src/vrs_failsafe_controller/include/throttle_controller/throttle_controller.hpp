#ifndef THROTTLE_CONTROLLER_HPP_THROTTLE_CONTROLLER_HPP
#define THROTTLE_CONTROLLER_HPP_THROTTLE_CONTROLLER_HPP

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <mavros_msgs/RCIn.h>

class ThrottleController
{
public:
    void UpdateController(void);
    ThrottleController(ros::NodeHandle& nh);
    ~ThrottleController() = default;
private:
    void ThrottleSetpointCallback(const mavros_msgs::RCIn::ConstPtr& msg);
    void PubThrottleRequest(float throttle);
    void PubServoRequest(float angle);
    void PIDThrottleController();
    void ServoController();
    ros::Subscriber localPositionSub_;
    ros::Publisher throttlePub_;
    ros::Publisher servoPub_;
    float throttle_{0};
    float servoAngle_{0};
    float lastServoAngle_{0};
};
#endif