#include "vrs_failsafe_controller.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vrs_failsafe_controller");
    ros::NodeHandle nh;

    VrsFailsafeController controller(nh);
    
    ros::Rate rate(100.0);
    while (ros::ok())
    {
        controller.UpdateNode();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}