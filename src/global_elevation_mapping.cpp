#include "global_elevation_mapping/global_elevation_mapping.hpp"

namespace global_elevation_mapping{

GlobalElevationMapping::GlobalElevationMapping():
    Node("global_elevation_mapping")
{
    read_parameters();
    setup_subsribers();
    setup_publishers();
    initialize();


    RCLCPP_INFO(this->get_logger(), "GlobalElevationMapping started");
}

GlobalElevationMapping::~GlobalElevationMapping()
{
    RCLCPP_INFO(this->get_logger(), "GlobalElevationMapping destructor");
}

void GlobalElevationMapping::read_parameters(void)
{    
    declare_parameter("global_frame", "map");
    declare_parameter("robot_frame", "base_link");
    declare_parameter("input_grid_map_topic", "/elevation_map_raw");
    declare_parameter("output_grid_map_topic", "/global_elevation_map");
    declare_parameter("update_frequency", 1.0);
    declare_parameter("publish_frequency", 1.0);
    declare_parameter("resolution", 0.5);
    declare_parameter("width", 5.0);
    declare_parameter("height", 5.0);
    declare_parameter("layers", std::vector<std::string>()); //TODO: check how to declare array

    RCLCPP_INFO(this->get_logger(), "GlobalElevationMapping with width: %f, height:%f and resolution: %f",
        get_parameter("width").as_double(),
        get_parameter("height").as_double(),
        get_parameter("resolution").as_double()
    );
}

void GlobalElevationMapping::setup_subsribers(void)
{
    input_grid_map_sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
        get_parameter("input_grid_map_topic").as_string(),
        10,
        std::bind(&GlobalElevationMapping::input_grid_map_callback, this, std::placeholders::_1)
    );
}

void GlobalElevationMapping::setup_publishers(void)
{
    output_grid_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        get_parameter("output_grid_map_topic").as_string(),
        10
    );
}

void GlobalElevationMapping::initialize(void)
{
    map_.setFrameId(get_parameter("global_frame").as_string());
    map_.setGeometry(
        grid_map::Length(get_parameter("width").as_double(), get_parameter("height").as_double()),
        get_parameter("resolution").as_double()
    );
    map_.add("elevation"); //TODO: initialize all layers from parameters

    publish_map_timer_ = rclcpp::create_timer(
        this,
        this->get_clock(),
        std::chrono::milliseconds(static_cast<int>(1000.0/get_parameter("publish_frequency").as_double())),
        std::bind(&GlobalElevationMapping::publish_map_callback, this)
    );
}

void GlobalElevationMapping::input_grid_map_callback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Local map received");
    
    grid_map::GridMap local_map;
    grid_map::GridMapRosConverter::fromMessage(*msg, local_map);

    map_.addDataFrom(local_map, true, false, false, {"elevation"});
}

void GlobalElevationMapping::publish_map_callback(void)
{    
    RCLCPP_INFO(this->get_logger(), "Global map published1"); 
    grid_map_msgs::msg::GridMap::SharedPtr output_msg = std::make_shared<grid_map_msgs::msg::GridMap>();
    output_msg = grid_map::GridMapRosConverter::toMessage(map_);

    output_grid_map_pub_->publish(*output_msg);
    RCLCPP_INFO(this->get_logger(), "Global map published"); 
}

} // namespace global_elevation_mapping