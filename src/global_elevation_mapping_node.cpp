#include <cstdio>

#include <ros/ros.h>
#include "global_elevation_mapping/global_elevation_mapping.hpp"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "global_elevation_mapping");
  ros::NodeHandle nh("~");
  auto node = std::make_shared<global_elevation_mapping::GlobalElevationMapping>(nh);
  ros::spin();
  ros::waitForShutdown();
  return 0;
}
