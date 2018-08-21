#include "ares_slam_ros.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ares_slam_node");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    AresSlamRos mapper;

    ros::spin();

    return 0;
}
