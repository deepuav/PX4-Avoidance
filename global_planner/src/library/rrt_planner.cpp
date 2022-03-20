#include "global_planner/rrt_planner.h"

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
  
  double dx = intermediate_goal_.pose.position.x - curr_pos_.x;
  double dy = intermediate_goal_.pose.position.y - curr_pos_.y;

  return atan2(dy,dx);

}

bool RRTPlanner::findPath(std::vector<State>& path) {

  path.clear();
  std::vector<Eigen::Vector3d>* rrt_path;
  Eigen::Vector3d in_lower, in_upper, out_lower,out_upper;
  
  Eigen::Vector3d local_position; local_position << curr_pos_.x, curr_pos_.y, curr_pos_.z;
  Eigen::Vector3d goal_position; goal_position << goal_pos_.xPos(), goal_pos_.yPos(), goal_pos_.zPos();
 
  in_lower << curr_pos_.x, curr_pos_.y, min_altitude_;
  in_upper << goal_pos_.xPos(), goal_pos_.yPos(), goal_pos_.zPos() + max_altitude_;

  checkBounds(in_lower,in_upper,out_lower,out_upper);

  rrt_planner_.setMap(octree_);
  rrt_planner_.setBounds(out_lower, out_upper);
  rrt_planner_.setupProblem();
  rrt_planner_.getPath(local_position, goal_position, rrt_path);
  curr_yaw_ = planYaw();

  if (rrt_path->empty()) {
    //current_path_.push_back(reference_pos_);
    return false;
  }

  for(auto p : *rrt_path)
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
  curr_path_ = new_path;
  goal_pos_ = GoalState(new_path[new_path.size() - 1], 1.0);
}

void RRTPlanner::stop() {
  setGoal(GoalState(curr_pos_));
  setPath({curr_pos_});
}

void RRTPlanner::setRobotRadius(double radius) { robot_radius_ = radius; }

void RRTPlanner::setIntermediateGoal(geometry_msgs::PoseStamped& intermediate_goal) {

  intermediate_goal_ = intermediate_goal;

}

