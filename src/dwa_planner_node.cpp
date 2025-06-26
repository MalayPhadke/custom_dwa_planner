#include "custom_dwa_planner/dwa_planner.hpp"

// Launcher DWA Planner Node
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto planner = std::make_shared<DWAPlanner>(rclcpp::NodeOptions());
  planner->process();
  rclcpp::shutdown();
  return 0;
}
