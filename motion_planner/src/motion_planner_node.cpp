#include <ros/ros.h>
#include "motion_planner.hpp"
#include <memory>

int main(int argc, char **argv) {
  ros::init(argc, argv, "motion_planning_node");
  ros::NodeHandle nh;
  // ros::console::levels::Debug
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  auto planner = std::make_unique<planning::MotionPlanner>(nh);
  planner->Launch();
  return 0;
}