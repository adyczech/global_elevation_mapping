#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "global_elevation_mapping/global_elevation_mapping.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<global_elevation_mapping::GlobalElevationMapping>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
