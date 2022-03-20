#ifndef GLOBAL_PLANNER_GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_GLOBAL_PLANNER_H

#include <math.h>     // abs
#include <algorithm>  // std::reverse
#include <limits>     // numeric_limits
#include <queue>      // std::priority_queue
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>  // getYaw createQuaternionMsgFromYaw

#include <octomap/OcTree.h>
#include <octomap/octomap.h>

#include <global_planner/RRTPlannerNodeConfig.h>
#include "global_planner/state.h"
#include "global_planner/common.h"
#include "global_planner/common_ros.h"
#include <global_planner/octomap_ompl_rrt.h>
#include <Eigen/Dense>
 
class RRTPlanner {
 public:
  octomap::OcTree* octree_ = NULL;
 
  // TODO: rename and remove not needed
  std::vector<State> path_back_;
  geometry_msgs::Point curr_pos_;
  double curr_yaw_;
  geometry_msgs::Vector3 curr_vel_;
  State goal_pos_;
  bool going_back_ = true;  // we start by just finding the start position

  // Dynamic reconfigure parameters
  int min_altitude_ = 1;
  int max_altitude_ = 10;
  double default_speed_ = 1.0;  // Default speed of flight.
  double max_speed_ = 3.0;      // Maximum speed of flight.
  int max_iterations_ = 2000;

  bool goal_is_blocked_ = false;
  bool goal_must_be_free_ = true;  // If false, the planner may try to find a path close to the goal
  bool current_state_blocked_ = false;
  bool current_goal_blocked_ = false;
 
  std::string default_node_type_ = "SpeedNode";
  std::string frame_id_ = "world";

  std::vector<State> curr_path_;

  void setIntermediateGoal(geometry_msgs::PoseStamped& intermediate_goal);

  RRTPlanner();
  ~RRTPlanner();

  void setPose(const geometry_msgs::PoseStamped& new_pose);
  void setGoal(const GoalState& goal);
  void setPath(const std::vector<State>& path);
  void setFrame(std::string frame_id);

  void updateFullOctomap(octomap::AbstractOcTree* tree);

  
  bool isOccupied(const State& state);

  geometry_msgs::PoseStamped createPoseMsg(const State& state, double yaw);
  nav_msgs::Path getPathMsg();
  nav_msgs::Path getPathMsg(const std::vector<State>& path);

  bool findPath(std::vector<State>& path);
  double planYaw();
  void checkBounds(const Eigen::Vector3d& in_lower, const Eigen::Vector3d& in_upper, Eigen::Vector3d& out_lower, Eigen::Vector3d& out_upper );
  geometry_msgs::PoseStamped intermediate_goal_;

  bool getGlobalPath();
  void goBack();
  void stop();
  void setRobotRadius(double radius);
  OctomapOmplRrt rrt_planner_;

 private:
  double robot_radius_;
  double octree_resolution_;
};


#endif  // RRT_PLANNER_RRT_PLANNER_H
