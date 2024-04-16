# pragma once

#include <ros/ros.h>

#include "grid_map_ros/grid_map_ros.hpp"
// #include "grid_map_msgs/grid_map.h"

namespace global_elevation_mapping{

class GlobalElevationMapping
{
  public:
    explicit GlobalElevationMapping(ros::NodeHandle& nh);
      
    virtual ~GlobalElevationMapping();    
    
    void read_parameters(void);

    void setup_subsribers(void);

    void setup_publishers(void);   
    
    void initialize(void);

    void input_grid_map_callback(const grid_map_msgs::GridMap::ConstPtr& msg);

    void publish_map_callback(void);

  private:
    ros::NodeHandle nh_;

    std::string global_frame_;
    std::string robot_frame_;

    double update_frequency_;
    double publish_frequency_;

    double resolution_;
    double width_;
    double height_;
    std::vector<std::string> layers_;

    grid_map::GridMap map_;
    std::vector<std::string> map_layers_;

    ros::Subscriber input_grid_map_sub_;
    ros::Publisher output_grid_map_pub_;

    ros::Timer publish_map_timer_;
};


} // namespace global_elevation_mapping