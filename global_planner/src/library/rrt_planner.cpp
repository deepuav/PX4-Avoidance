#include "global_planner/rrt_planner.h"

namespace rrt_planner {

RRTPlanner::RRTPlanner() {}
RRTPlanner::~RRTPlanner() {}

// Updates the current pose and keeps track of the path back
void RRTPlanner::setPose(const geometry_msgs::PoseStamped& new_pose) {
  curr_pos_ = new_pose.pose.position;
  curr_yaw_ = tf::getYaw(new_pose.pose.orientation);
  State curr_state = State(curr_pos_);
  if (!going_back_ && path_back_.empty() ) { //|| curr_state != path_back_.back())) {
    // Keep track of where we have been, add current position to path_back_ if
    // it is different from last one
    path_back_.push_back(curr_state);
  }
}

// Sets a new mission goal
void RRTPlanner::setGoal(const GoalState& goal) {
  goal_pos_ = goal;
  going_back_ = false;
  goal_is_blocked_ = false;
}

// Sets path to be the current path
void RRTPlanner::setPath(const std::vector<State>& path) {
  curr_path_ = path;
}

void RRTPlanner::setFrame(std::string frame_id) 
{ 
  frame_id_ = frame_id; 
}

// Returns false iff current path has an obstacle
// Going through the octomap can take more than 50 ms for 100m x 100m explored
// map
void RRTPlanner::updateFullOctomap(octomap::AbstractOcTree* tree) {
  if (octree_) {
    delete octree_;
  }
  octree_ = dynamic_cast<octomap::OcTree*>(tree);
  octree_resolution_ = octree_->getResolution();
}

// TODO
bool RRTPlanner::isOccupied(const State& state) { return true; }

geometry_msgs::PoseStamped RRTPlanner::createPoseMsg(const State& state, double yaw) {
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.frame_id = frame_id_;
  pose_msg.pose.position = state.toPoint();
  pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  return pose_msg;
}

nav_msgs::Path RRTPlanner::getPathMsg() { return getPathMsg(curr_path_); }

nav_msgs::Path RRTPlanner::getPathMsg(const std::vector<State>& path) {
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = frame_id_;

  if (path.size() == 0) {
    return path_msg;
  }

  // Use actual position instead of the center of the state
  double last_yaw = curr_yaw_;

  for (int i = 0; i < path.size() - 1; ++i) {
    State p = path[i];
    double new_yaw = 0.0; // TODO
    // if (new_yaw != last_yaw) {   // only publish corner points
    path_msg.poses.push_back(createPoseMsg(p, new_yaw));
    // }
    last_yaw = new_yaw;
  }
  State last_point = path[path.size() - 1];  // Last point should have the same yaw as the previous point
  path_msg.poses.push_back(createPoseMsg(last_point, last_yaw));
  return path_msg;
}

double RRTPlanner::planYaw()
{
  
  double dx = current_goal_.pose.position.x - curr_pos_.x;
  double dy = current_goal_.pose.position.y - curr_pos_.y;

  return atan2(dy,dx);

}

bool RRTPlanner::findPath(std::vector<State>& path) {

  path.clear();
  std::vector<Eigen::Vector3d>* rrt_path;
  Eigen::Vector3d in_lower, in_upper, out_lower,out_upper;
  
  Eigen::Vector3d local_position; local_position << curr_pos_.x, curr_pos_.y, curr_pos_.z;
 
  in_lower << curr_pos_.x, curr_pos_.y, min_altitude_;
  in_upper << goal_pos_(0), goal_pos_(1), goal_pos_(2) + max_altitude_;

  checkBounds(in_lower,in_upper,out_lower,out_upper);

  rrt_planner_.setMap(octree_);
  rrt_planner_.setBounds(out_lower, out_upper);
  rrt_planner_.setupProblem();
  rrt_planner_.getPath(local_position, goal_pos_, rrt_path);
  yaw_ = planYaw();

  if (rrt_path.empty()) {
    //current_path_.push_back(reference_pos_);
    return false;
  }

  for(auto p : rrt_path)
  {
    State s( p(0), p(1), p(2) );
    path.push_back(s);
  }

  return true;

} 

void RRTPlanner::checkBounds(const Eigen::Vector3d& in_lower, const Eigen::Vector3d& in_upper, Eigen::Vector3d& out_lower, Eigen::Vector3d& out_upper ) {
  for(int i = 0; i < 3; i++) 
  {
    if(in_lower(i) < in_upper(i))
    { 
      out_lower(i) = in_lower(i);
      out_upper(i) = in_upper(i);
    } 
    else
    {
      out_lower(i) = in_upper(i);
      out_upper(i) = in_lower(i);
    }
  }  

}

  // // Start from a position thats a bit ahead [s = curr_pos + (search_time_ *
  // // curr_vel_)]
  // Cell s(addPoints(curr_pos_, scalePoint(curr_vel_, search_time_)));
  // GoalCell t = goal_pos_;
  // // Cell parent_of_s = s.getNeighborFromYaw(curr_yaw_ + M_PI); // The cell
  // // behind the start cell Cell parent_of_s = Cell(curr_pos_);
  // Cell parent_of_s(subtractPoints(curr_pos_, scalePoint(curr_vel_, search_time_)));
  // if (!use_current_yaw_) {
  //   Cell parent_of_s = s;  // Ignore the current yaw
  // }

  // ROS_INFO("Planning a path from %s to %s", s.asString().c_str(), t.asString().c_str());
  // ROS_INFO("curr_pos_: %2.2f,%2.2f,%2.2f\t s: %2.2f,%2.2f,%2.2f", curr_pos_.x, curr_pos_.y, curr_pos_.z, s.xPos(),
  //          s.yPos(), s.zPos());

  // bool found_path = false;
  // double best_path_cost = INFINITY;
  // overestimate_factor_ = max_overestimate_factor_;
  // int iter_left = max_iterations_;
  // int last_iter = 0;

  // // reverseSearch(t); // REVERSE_SEARCH

  // printf("Search              iter_time overest   num_iter  path_cost \n");
  // while (overestimate_factor_ >= min_overestimate_factor_ && iter_left > last_iter) {
  //   std::vector<Cell> new_path;
  //   SearchInfo search_info;
  //   std::string node_type = default_node_type_;
  //   if (overestimate_factor_ > 1.5) {
  //     // Use a cheap search for higher overestimate
  //     // found_new_path = findPathOld(new_path, s, t, parent_of_s, true);  // No
  //     // need to search with smoothness
  //     node_type = "NodeWithoutSmooth";
  //   }

  //   NodePtr start_node = getStartNode(s, parent_of_s, node_type);
  //   search_info = findSmoothPath(this, new_path, start_node, t, iter_left, visitor_);
  //   printSearchInfo(search_info, node_type, overestimate_factor_);

  //   if (search_info.found_path) {
  //     PathInfo path_info = getPathInfo(new_path);
  //     printf("(cost: %2.2f, dist: %2.2f, risk: %2.2f, smooth: %2.2f) \n", path_info.cost, path_info.dist,
  //            path_info.risk, path_info.smoothness);
  //     if (true || path_info.cost <= best_path_cost) {
  //       // TODO: always use the newest path?
  //       best_path_cost = path_info.cost;
  //       path = new_path;
  //       found_path = true;
  //     }
  //   } else {
  //     break;
  //   }
  //   last_iter = search_info.num_iter;
  //   iter_left -= last_iter;
  //   overestimate_factor_ = (overestimate_factor_ - 1.0) / 4.0 + 1.0;
  // }

  // // Last resort, try 2d search at max_altitude_
  // if (!found_path) {
  //   printf("No path found, search in 2D \n");
  //   max_iterations_ = 5000;
  //   found_path = find2DPath(this, path, s, t, parent_of_s, max_altitude_);
  // }

  // return found_path;

  return true;
}

// Returns true iff a path needs to be published, either a new path or a path
// back The path is then stored in this.pathMsg
bool RRTPlanner::getGlobalPath() {
  State s = State(curr_pos_);
  State t = State(goal_pos_);
  current_state_blocked_ = isOccupied(s);
  current_goal_blocked_ = isOccupied(t);

  if (goal_must_be_free_ && current_goal_blocked_) {
    // If goal is occupied, no path is published
    ROS_INFO("Goal position is occupied");
    goal_is_blocked_ = true;
    return false;
  } else if (current_state_blocked_) {
    // If current position is occupied the way back is published
    ROS_INFO("Current position is occupied, going back.");
    // goBack();
    // return true;
    return false;
  } else {
    // Both current position and goal are free, try to find a path
    std::vector<State> path;
    if (!findPath(path)) {
      goal_is_blocked_ = true;
      return false;
    }
    setPath(path);
    return true;
  }
}

// Sets the current path to be the path back until a safe cell is reached
// Then the mission can be tried again or a new mission can be set
void RRTPlanner::goBack() {
  ROS_INFO("  GO BACK ");
  going_back_ = true;
  std::vector<State> new_path = path_back_;
  std::reverse(new_path.begin(), new_path.end());

  // Follow the path back until the risk is low
  for (int i = 1; i < new_path.size() - 1; ++i) {
    if (i > 5 && getRisk(new_path[i]) < 0.5) {
      new_path.resize(i + 1);                        // new_path is the last i+1 positions of path_back_
      path_back_.resize(path_back_.size() - i - 2);  // Remove part of path_back_ that is also in new_path
      break;
    }
  }
  curr_path_ = new_path;
  goal_pos_ = GoalState(new_path[new_path.size() - 1], 1.0);
}

void RRTPlanner::stop() {
  setGoal(GoalState(curr_pos_));
  setPath({curr_pos_});
}

void RRTPlanner::setRobotRadius(double radius) { robot_radius_ = radius; }

}  // namespace rrt_planner
