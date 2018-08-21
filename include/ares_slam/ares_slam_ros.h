#ifndef ARES_SLAM_ROS_H
#define ARES_SLAM_ROS_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <thread>
#include <mutex>
#include "map_builder.h"

class AresSlamRos
{
public:
    AresSlamRos();
    ~AresSlamRos();

private:
    void scanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg);
    void publishPose(const Eigen::Vector3f& pose, const ros::Time& stamp);
    void publishOccupancyGridMap();
    void publishProbabilityGridMap();
    void publishCorrelativeGrid();
    void publishLoop();
    void publishPath();
    void publishConstraintList();
    void setCorrelativeTranslationTable();
    bool optimizationCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

private:
    // frame id
    std::string base_frame_;
    std::string odom_frame_;
    std::string map_frame_;

    double publish_freq_;

    // flags
    bool got_lidar_tf_;
    bool initialize_;

    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    ros::Publisher map_pub_;
    ros::Publisher correlative_grid_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher path_pub_;
    ros::Publisher constraint_list_pub_;
    ros::Subscriber scan_sub_;
    ros::ServiceServer optimization_srv_;

    tf::StampedTransform lidar_tf_;

    ares_slam::MapBuilder map_builder_;
    std::shared_ptr<std::thread> publish_thread_;

    char* correlative_translation_table_;
};

#endif // ARES_SLAM_ROS_H
