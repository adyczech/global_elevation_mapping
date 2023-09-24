# pragma once

#include "rclcpp/rclcpp.hpp"

#include "grid_map_ros/grid_map_ros.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"

namespace global_elevation_mapping{

class GlobalElevationMapping : public rclcpp::Node
{
  public:
    explicit GlobalElevationMapping();
      
    virtual ~GlobalElevationMapping();    
    
    void read_parameters(void);

    void setup_subsribers(void);

    void setup_publishers(void);   
    
    void initialize(void);

    void input_grid_map_callback(const grid_map_msgs::msg::GridMap::SharedPtr msg);

    void publish_map_callback(void);

  private:
    std::string global_frame_;
    std::string robot_frame_;

    double update_frequency_;
    double publish_frequency_;

    double resolution_;
    double width_;
    double height_;
    std::vector<std::string> layers_;

    grid_map::GridMap map_;

    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr input_grid_map_sub_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr output_grid_map_pub_;

    rclcpp::TimerBase::SharedPtr publish_map_timer_;
};


} // namespace global_elevation_mapping