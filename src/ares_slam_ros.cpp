#include "ares_slam_ros.h"

AresSlamRos::AresSlamRos() : got_lidar_tf_(false), initialize_(false)
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string scan_topic, map_topic, odom_topic;
    private_nh.param("scan_topic", scan_topic, std::string("scan"));
    private_nh.param("map_topic", map_topic, std::string("map"));
    private_nh.param("odom_topic", odom_topic, std::string("odom"));

    private_nh.param("base_frame", base_frame_, std::string("base_link"));
    private_nh.param("odom_frame", odom_frame_, std::string("odom"));
    private_nh.param("map_frame", map_frame_, std::string("map"));
    private_nh.param("publish_freq", publish_freq_, 0.25);

    double occupancy_grid_map_resolution;
    double min_update_distance, min_update_orientation;
    int scan_buffer_size;
    double loop_scan_search_distance;
    int loop_match_min_chain_size;
    double loop_closure_min_response;
    double loop_closure_xy_variance_threshold, loop_closure_angle_variance_threshold;
    int optimize_every_n_constrains;

    double loop_closure_xy_search_range, loop_closure_angle_search_range;
    double loop_closure_grid_resolution;
    double loop_closure_coarse_xy_search_resolution, loop_closure_fine_xy_search_range;
    double loop_closure_coarse_angle_search_resolution;
    double loop_closure_fine_angle_search_range, loop_closure_fine_angle_search_resolution;

    bool use_correlative_scan_matcher;

    private_nh.param("occupancy_grid_map_resolution", occupancy_grid_map_resolution, 0.05);
    private_nh.param("min_update_distance", min_update_distance, 0.2);
    private_nh.param("min_update_orientation", min_update_orientation, 10.0);
    private_nh.param("scan_buffer_size", scan_buffer_size, 30);
    private_nh.param("loop_scan_search_distance", loop_scan_search_distance, 10.0);
    private_nh.param("loop_match_min_chain_size", loop_match_min_chain_size, 5);
    private_nh.param("loop_closure_min_response", loop_closure_min_response, 0.65);
    private_nh.param("loop_closure_xy_variance_threshold", loop_closure_xy_variance_threshold, 0.01);
    private_nh.param("loop_closure_angle_variance_threshold", loop_closure_angle_variance_threshold, 0.05);
    private_nh.param("optimize_every_n_constrains", optimize_every_n_constrains, 20);

    private_nh.param("loop_closure_xy_search_range", loop_closure_xy_search_range, 5.0);
    private_nh.param("loop_closure_angle_search_range", loop_closure_angle_search_range, 20.0);
    private_nh.param("loop_closure_grid_resolution", loop_closure_grid_resolution, 0.05);
    private_nh.param("loop_closure_coarse_xy_search_resolution", loop_closure_coarse_xy_search_resolution, 0.1);
    private_nh.param("loop_closure_fine_xy_search_range", loop_closure_fine_xy_search_range, 0.1);
    private_nh.param("loop_closure_coarse_angle_search_resolution", loop_closure_coarse_angle_search_resolution, 2.0);
    private_nh.param("loop_closure_fine_angle_search_range", loop_closure_fine_angle_search_range, 2.0);
    private_nh.param("loop_closure_fine_angle_search_resolution", loop_closure_fine_angle_search_resolution, 0.2);

    map_builder_.setOccupancyGridMapResolution(occupancy_grid_map_resolution);
    map_builder_.setMinUpdateDistance(min_update_distance);
    map_builder_.setMinUpdateOrientation(ares_slam::degToRad(min_update_orientation));
    map_builder_.setScanBufferSize(scan_buffer_size);
    map_builder_.setLoopScanSearchDistance(loop_scan_search_distance);
    map_builder_.setLoopMatchMinChainSize(loop_match_min_chain_size);
    map_builder_.setLoopClosureMinResponse(loop_closure_min_response);
    map_builder_.setLoopClosureXYVarianceThreshold(loop_closure_xy_variance_threshold);
    map_builder_.setLoopClosureAngleVarianceThreshold(loop_closure_angle_variance_threshold);
    map_builder_.setOptimizeEveryNConstraints(optimize_every_n_constrains);

    map_builder_.setLoopClosureXYSearchRange(loop_closure_xy_search_range);
    map_builder_.setLoopClosureAngleSearchRange(ares_slam::degToRad(loop_closure_angle_search_range));
    map_builder_.setLoopClosureGridResolution(loop_closure_grid_resolution);
    map_builder_.setLoopClosureCoarseXYSearchResolution(loop_closure_coarse_xy_search_resolution);
    map_builder_.setLoopClosureFineXYSearchRange(loop_closure_fine_xy_search_range);
    map_builder_.setLoopClosureCoarseAngleSearchResolution(ares_slam::degToRad(loop_closure_coarse_angle_search_resolution));
    map_builder_.setLoopClosureFineAngleSearchRange(ares_slam::degToRad(loop_closure_fine_angle_search_range));
    map_builder_.setLoopClosureFineAngleSearchResolution(ares_slam::degToRad(loop_closure_fine_angle_search_resolution));

    private_nh.param("use_correlative_scan_matcher", use_correlative_scan_matcher, false);
    if(use_correlative_scan_matcher) {
        double csm_xy_search_range;
        double csm_angle_search_range;
        double csm_grid_resolution;
        double csm_xy_search_resolution;
        double csm_angle_search_resolution;

        private_nh.param("csm_xy_search_range", csm_xy_search_range, 0.2);
        private_nh.param("csm_angle_search_range", csm_angle_search_range, 20.0);
        private_nh.param("csm_grid_resolution", csm_grid_resolution, 0.01);
        private_nh.param("csm_xy_search_resolution", csm_xy_search_resolution, 0.02);
        private_nh.param("csm_angle_search_resolution", csm_angle_search_resolution, 0.5);

        map_builder_.useCorrelativeScanMatcher(true);
        map_builder_.setCSMXYSearchRange(csm_xy_search_range);
        map_builder_.setCSMAngleSearchRange(ares_slam::degToRad(csm_angle_search_range));
        map_builder_.setCSMGridResolution(csm_grid_resolution);
        map_builder_.setCSMXYSearchResolution(csm_xy_search_resolution);
        map_builder_.setCSMAngleSearchResolution(ares_slam::degToRad(csm_angle_search_resolution));
    }

    map_builder_.initialize();

    map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    correlative_grid_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("correlative_grid", 1, true);
    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("robot_pose", 1, true);
    path_pub_ = nh.advertise<nav_msgs::Path>("path", 1, true);
    constraint_list_pub_ = nh.advertise<visualization_msgs::MarkerArray>("constraint_list", 1, true);
    scan_sub_ = nh.subscribe(scan_topic, 10, &AresSlamRos::scanCallback, this);

    publish_thread_.reset(new std::thread(std::bind(&AresSlamRos::publishLoop, this)));

    optimization_srv_ = nh.advertiseService("optimization", &AresSlamRos::optimizationCallback, this);

    setCorrelativeTranslationTable();
}

AresSlamRos::~AresSlamRos()
{
    if(correlative_translation_table_) {
        delete[] correlative_translation_table_;
    }
}

void AresSlamRos::setCorrelativeTranslationTable()
{
    correlative_translation_table_ = new char[256];

    for (int i = 0; i < 256; i++) {
        correlative_translation_table_[i] = char(i * 100 / 255);
    }
}

bool AresSlamRos::optimizationCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    map_builder_.doPoseAdjustment();
    return true;
}

void AresSlamRos::scanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg)
{
    if(!got_lidar_tf_) {
        try {
            tf_listener_.waitForTransform(base_frame_, scan_msg->header.frame_id, ros::Time(0), ros::Duration(0.5));
            tf_listener_.lookupTransform(base_frame_, scan_msg->header.frame_id, ros::Time(0), lidar_tf_);
            got_lidar_tf_ = true;
        }
        catch (tf::TransformException e) {
            ROS_ERROR("%s", e.what());
            return;
        }
    }

    tf::StampedTransform odom_to_base;
    try {
        tf_listener_.waitForTransform(odom_frame_, base_frame_, scan_msg->header.stamp, ros::Duration(0.5));
        tf_listener_.lookupTransform(odom_frame_, base_frame_, scan_msg->header.stamp, odom_to_base);
    }
    catch(tf::TransformException e) {
        ROS_ERROR("Transform failed during publishing of map_odom transform: %s",e.what());
        return;
    }

    Eigen::Vector3f odom_pose;
    odom_pose[0] = odom_to_base.getOrigin().x();
    odom_pose[1] = odom_to_base.getOrigin().y();
    odom_pose[2] = tf::getYaw(odom_to_base.getRotation());
    map_builder_.addOdom(odom_pose);

    PointCloud scan_points;

    for(int i = 0; i < scan_msg->ranges.size(); ++i) {
        float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
        if(scan_msg->ranges[i] > scan_msg->range_min && scan_msg->ranges[i] < scan_msg->range_max) {
            tf::Vector3 scan_point(lidar_tf_ * tf::Vector3(scan_msg->ranges[i] * cos(angle), scan_msg->ranges[i] * sin(angle), 0.0));
            scan_points.emplace_back(scan_point.x(), scan_point.y());
        }
    }

    std::shared_ptr<ares_slam::LaserScan> laser_scan(new ares_slam::LaserScan(scan_points));
    map_builder_.addLaserScan(laser_scan);

    Eigen::Vector3f pose = laser_scan->getPose();

    if(!initialize_) {
        initialize_ = true;
    }

    publishPose(pose, scan_msg->header.stamp);
    tf::Transform map_to_base(tf::createQuaternionFromYaw(pose[2]), tf::Vector3(pose[0], pose[1], 0.0f));
    tf::Transform map_to_odom = tf::Transform(map_to_base * odom_to_base.inverse());
    tf_broadcaster_.sendTransform(tf::StampedTransform(map_to_odom, scan_msg->header.stamp, map_frame_, odom_frame_));
}

void AresSlamRos::publishPath()
{
    std::vector<Eigen::Vector3f> path = map_builder_.getPath();
    nav_msgs::Path path_msg;

    path_msg.header.frame_id = map_frame_;
    path_msg.header.stamp = ros::Time::now();

    for(const Eigen::Vector3f& pose : path) {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = map_frame_;
        pose_msg.pose.position.x = pose[0];
        pose_msg.pose.position.y = pose[1];
        pose_msg.pose.position.z = 0;

        tf::Quaternion q;
        q.setRPY(0.0, 0.0, pose[2]);
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();
        path_msg.poses.push_back(pose_msg);
    }

    path_pub_.publish(path_msg);
}

void AresSlamRos::publishPose(const Eigen::Vector3f& pose, const ros::Time& stamp)
{
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose.position.x = pose[0];
    pose_msg.pose.position.y = pose[1];
    pose_msg.pose.position.z = 0;

    tf::Quaternion q;
    q.setRPY(0.0, 0.0, pose[2]);
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();
    pose_pub_.publish(pose_msg);
}

void AresSlamRos::publishOccupancyGridMap()
{
    std::shared_ptr<ares_slam::OccupancyGridMap> map = map_builder_.getOccupancyGridMap();

    nav_msgs::OccupancyGrid map_msg;
    Eigen::Vector2f origin = map->getOrigin();
    map_msg.header.stamp = ros::Time::now();
    map_msg.info.origin.position.x = origin.x();
    map_msg.info.origin.position.y = origin.y();
    map_msg.info.origin.orientation.x = 0;
    map_msg.info.origin.orientation.y = 0;
    map_msg.info.origin.orientation.z = 0;
    map_msg.info.origin.orientation.w = 1;
    map_msg.info.resolution = map->getResolution();
    map_msg.info.width = map->getSizeX();
    map_msg.info.height = map->getSizeY();
    map_msg.data.resize(map_msg.info.width * map_msg.info.height, -1);

    for(int i = 0; i < map_msg.data.size(); ++i) {
        if(map->isFree(i)) {
            map_msg.data[i] = 0;
        }
        else if(map->isOccupied(i)) {
            map_msg.data[i] = 100;
        }
    }

    map_pub_.publish(map_msg);
}

void AresSlamRos::publishProbabilityGridMap()
{
    std::shared_ptr<ares_slam::ProbabilityGridMap> map = map_builder_.getProbabilityGridMap();

    nav_msgs::OccupancyGrid map_msg;
    Eigen::Vector2f origin = map->getOrigin();
    map_msg.header.stamp = ros::Time::now();
    map_msg.info.origin.position.x = origin.x();
    map_msg.info.origin.position.y = origin.y();
    map_msg.info.origin.orientation.x = 0;
    map_msg.info.origin.orientation.y = 0;
    map_msg.info.origin.orientation.z = 0;
    map_msg.info.origin.orientation.w = 1;
    map_msg.info.resolution = map->getResolution();
    map_msg.info.width = map->getSizeX();
    map_msg.info.height = map->getSizeY();
    map_msg.data.resize(map_msg.info.width * map_msg.info.height, -1);

    for(int i = 0; i < map_msg.data.size(); ++i) {
        int value = map->getGridValue(i);
        if(value == ares_slam::LogOdds_Unknown) {
            map_msg.data[i] = -1;
        }
        else {
            map_msg.data[i] = map->getGridValue(i);
        }
    }

    map_pub_.publish(map_msg);
}

void AresSlamRos::publishCorrelativeGrid()
{  
    std::shared_ptr<ares_slam::CorrelativeGrid> correlative_grid = map_builder_.getCorrelativeGrid();

    if(correlative_grid->getSize() == 0) {
        return;
    }

    nav_msgs::OccupancyGrid map_msg;
    Eigen::Vector2f origin = correlative_grid->getOrigin();
    map_msg.header.stamp = ros::Time::now();
    map_msg.info.origin.position.x = origin.x();
    map_msg.info.origin.position.y = origin.y();
    map_msg.info.origin.orientation.x = 0;
    map_msg.info.origin.orientation.y = 0;
    map_msg.info.origin.orientation.z = 0;
    map_msg.info.origin.orientation.w = 1;
    map_msg.info.resolution = correlative_grid->getResolution();
    map_msg.info.width = correlative_grid->getSizeX();
    map_msg.info.height = correlative_grid->getSizeY();
    map_msg.data.resize(map_msg.info.width * map_msg.info.height, 0);

    for(int i = 0; i < map_msg.data.size(); ++i) {
        map_msg.data[i] = correlative_translation_table_[correlative_grid->getGridValue(i)];
    }

    correlative_grid_pub_.publish(map_msg);
}

void AresSlamRos::publishConstraintList()
{
    std::vector<Eigen::Vector2f> graph_nodes;
    std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> graph_edges;
    map_builder_.getGraph(graph_nodes, graph_edges);

    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.5;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0);

    visualization_msgs::Marker edge;
    edge.header.frame_id = map_frame_;
    edge.header.stamp = ros::Time::now();
    edge.action = visualization_msgs::Marker::ADD;
    edge.id = 0;
    edge.type = visualization_msgs::Marker::LINE_STRIP;
    edge.scale.x = 0.05;
    edge.scale.y = 0.05;
    edge.scale.z = 0.05;
    edge.color.a = 1.0;
    edge.color.r = 0.0;
    edge.color.g = 1.0;
    edge.color.b = 0.0;

    int id = 0;
    for (int i = 0; i < graph_nodes.size(); ++i) {
        marker.id = id;
        marker.pose.position.x = graph_nodes[i](0);
        marker.pose.position.y = graph_nodes[i](1);
        marker_array.markers.push_back(visualization_msgs::Marker(marker));
        id++;
    }

    for (int i = 0; i < graph_edges.size(); ++i) {
        edge.points.clear();
        geometry_msgs::Point p;
        p.x = graph_edges[i].first(0);
        p.y = graph_edges[i].first(1);
        edge.points.push_back(p);
        p.x = graph_edges[i].second(0);
        p.y = graph_edges[i].second(1);
        edge.points.push_back(p);
        edge.id = id;
        marker_array.markers.push_back(visualization_msgs::Marker(edge));
        id++;
    }

    constraint_list_pub_.publish(marker_array);
}

void AresSlamRos::publishLoop()
{
    ros::Rate rate(publish_freq_);

    while (ros::ok()) {
        if(initialize_) {
            publishConstraintList();
            publishProbabilityGridMap();
        }
        rate.sleep();
    }
}

