#include "vrs_failsafe_controller.hpp"
#include "throttle_controller.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vrs_failsafe_controller");
    ros::NodeHandle nh;

    VrsFailsafeController controller(nh);
    ThrottleController throttleController(nh);

    ros::Rate rate(30.0);
    while (ros::ok())
    {
        throttleController.UpdateThrottleController();
        controller.UpdateNode();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}