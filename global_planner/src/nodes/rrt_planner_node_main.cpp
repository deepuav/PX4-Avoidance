#include "global_planner/rrt_planner_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "rrt_planner_node");

  ros::NodeHandle nh("~");
  ros::NodeHandle nh_private("");

  rrt_planner::RRTPlannerNode rrt_planner_node(nh, nh_private);

  ros::spin();
  return 0;
}