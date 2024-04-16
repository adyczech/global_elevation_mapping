#include "global_elevation_mapping/global_elevation_mapping.hpp"

namespace global_elevation_mapping{

GlobalElevationMapping::GlobalElevationMapping(ros::NodeHandle& nh)
{
    nh_ = nh;
    read_parameters();
    setup_subsribers();
    setup_publishers();
    initialize();

    ROS_INFO("GlobalElevationMapping started");
}

GlobalElevationMapping::~GlobalElevationMapping()
{
    ROS_INFO("GlobalElevationMapping destructor");
}

void GlobalElevationMapping::read_parameters(void)
{    
    nh_.param("global_frame", global_frame_, std::string("map"));
    nh_.param("robot_frame", robot_frame_, std::string("base_link"));
    nh_.param("update_frequency", update_frequency_, 1.0);
    nh_.param("publish_frequency", publish_frequency_, 1.0);
    nh_.param("resolution", resolution_, 0.5);
    nh_.param("width", width_, 5.0);
    nh_.param("height", height_, 5.0);
    nh_.param("layers", map_layers_, std::vector<std::string>({"elevation"}));

    ROS_INFO("GlobalElevationMapping with width: %f, height:%f, resolution: %f",
        width_,
        height_,
        resolution_
    );
}

void GlobalElevationMapping::setup_subsribers(void)
{
    std::string input_grid_map_topic;
    nh_.param("input_grid_map_topic", input_grid_map_topic, std::string("/elevation_map_raw"));

    input_grid_map_sub_ = nh_.subscribe<grid_map_msgs::GridMap>(
        input_grid_map_topic,
        10,
        std::bind(&GlobalElevationMapping::input_grid_map_callback, this, std::placeholders::_1)
    );
}

void GlobalElevationMapping::setup_publishers(void)
{
    std::string output_grid_map_topic;
    nh_.param("output_grid_map_topic", output_grid_map_topic, std::string("/elevation_map"));

    output_grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>(
        output_grid_map_topic,
        10
    );
}

void GlobalElevationMapping::initialize(void)
{
    map_.setFrameId(global_frame_);
    map_.setGeometry(
        grid_map::Length(width_, height_),
        resolution_
    );

    for(auto layer : map_layers_){
        map_.add(layer);
    }

    publish_map_timer_ = nh_.createTimer(
        ros::Duration(1.0/publish_frequency_),
        std::bind(&GlobalElevationMapping::publish_map_callback, this)
    );
}

void GlobalElevationMapping::input_grid_map_callback(const grid_map_msgs::GridMap::ConstPtr& msg)
{
    ROS_INFO("Local map received");
    
    grid_map::GridMap local_map;
    grid_map::GridMapRosConverter::fromMessage(*msg, local_map);

    map_.addDataFrom(local_map, true, false, false, map_layers_);
}

void GlobalElevationMapping::publish_map_callback(void)
{    
    grid_map_msgs::GridMap output_msg;
    grid_map::GridMapRosConverter::toMessage(map_, output_msg);

    output_grid_map_pub_.publish(output_msg);
    ROS_INFO("Global map published"); 
}

} // namespace global_elevation_mapping