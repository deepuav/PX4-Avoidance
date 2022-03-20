#include "global_planner/rrt_planner_node.h"
#include "global_planner/octomap_ompl_rrt.h"

using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;

RRTPlannerNode::RRTPlannerNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      avoidance_node_(nh, nh_private),
      cmdloop_dt_(0.1),
      plannerloop_dt_(1.0),
      mapupdate_dt_(0.2),
      start_yaw_(0.0) {
  // Set up Dynamic Reconfigure Server
  dynamic_reconfigure::Server<global_planner::RRTPlannerNodeConfig>::CallbackType f;
  f = boost::bind(&RRTPlannerNode::dynamicReconfigureCallback, this, _1, _2);
  server_.setCallback(f);

#ifndef DISABLE_SIMULATION
  world_visualizer_.reset(new avoidance::WorldVisualizer(nh_, ros::this_node::getName()));
#endif

  avoidance_node_.init();
  // Read Ros parameters
  readParams();

  // Subscribers
  octomap_full_sub_ = nh_.subscribe("/octomap_full", 1, &RRTPlannerNode::octomapFullCallback, this);
  ground_truth_sub_ = nh_.subscribe("/mavros/local_position/pose", 1, &RRTPlannerNode::positionCallback, this);
  velocity_sub_ = nh_.subscribe("/mavros/local_position/velocity", 1, &RRTPlannerNode::velocityCallback, this);
  clicked_point_sub_ = nh_.subscribe("/clicked_point", 1, &RRTPlannerNode::clickedPointCallback, this);
  move_base_simple_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &RRTPlannerNode::moveBaseSimpleCallback, this);
  fcu_input_sub_ = nh_.subscribe("/mavros/trajectory/desired", 1, &RRTPlannerNode::fcuInputGoalCallback, this);

  // Publishers
  global_temp_path_pub_ = nh_.advertise<nav_msgs::Path>("/global_temp_path", 10);
  actual_path_pub_ = nh_.advertise<nav_msgs::Path>("/actual_path", 10);
  smooth_path_pub_ = nh_.advertise<nav_msgs::Path>("/smooth_path", 10);
  global_goal_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/global_goal", 10);
  global_temp_goal_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/global_temp_goal", 10);
  explored_cells_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/explored_cells", 10);
  mavros_waypoint_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  mavros_obstacle_free_path_pub_ = nh_.advertise<mavros_msgs::Trajectory>("/mavros/trajectory/generated", 10);
  current_waypoint_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/current_setpoint", 10);
  pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_in", 10);

  actual_path_.header.frame_id = frame_id_;

  ros::TimerOptions cmdlooptimer_options(ros::Duration(cmdloop_dt_),
                                         boost::bind(&RRTPlannerNode::cmdLoopCallback, this, _1), &cmdloop_queue_);
  cmdloop_timer_ = nh_.createTimer(cmdlooptimer_options);

  cmdloop_spinner_.reset(new ros::AsyncSpinner(1, &cmdloop_queue_));
  cmdloop_spinner_->start();

  ros::TimerOptions plannerlooptimer_options(ros::Duration(plannerloop_dt_),
                                             boost::bind(&RRTPlannerNode::plannerLoopCallback, this, _1),
                                             &plannerloop_queue_);
  plannerloop_timer_ = nh_.createTimer(plannerlooptimer_options);

  plannerloop_spinner_.reset(new ros::AsyncSpinner(1, &plannerloop_queue_));
  plannerloop_spinner_->start();

  current_goal_.header.frame_id = frame_id_;
  current_goal_.pose.position = start_pos_;
  current_goal_.pose.orientation = tf::createQuaternionMsgFromYaw(start_yaw_);
  rrt_planner.setIntermediateGoal(current_goal_);
  last_goal_ = current_goal_;

  speed_ = rrt_planner.default_speed_;
  start_time_ = ros::Time::now();
}

RRTPlannerNode::~RRTPlannerNode() {}

// Read Ros parameters
void RRTPlannerNode::readParams() {
  std::vector<std::string> camera_topics;

  nh_.param<double>("start_pos_x", start_pos_.x, 0.5);
  nh_.param<double>("start_pos_y", start_pos_.y, 0.5);
  nh_.param<double>("start_pos_z", start_pos_.z, 3.5);
  nh_.param<std::string>("frame_id", frame_id_, "/local_origin");
  nh_.getParam("pointcloud_topics", camera_topics);
  if (!nh_.hasParam("camera_frame_id")) {
    nh_.setParam("camera_frame_id", "/camera_link");
  } else {
    nh_.getParam("camera_frame_id", camera_frame_id_);
  }

  initializeCameraSubscribers(camera_topics);
  rrt_planner.goal_pos_ = State(start_pos_.x, start_pos_.y, start_pos_.z);
  double robot_radius;
  nh_.param<double>("robot_radius", robot_radius, 0.5);
  rrt_planner.setFrame(frame_id_);
  rrt_planner.setRobotRadius(robot_radius);
}

void RRTPlannerNode::initializeCameraSubscribers(std::vector<std::string>& camera_topics) {
  cameras_.resize(camera_topics.size());

  for (size_t i = 0; i < camera_topics.size(); i++) {
    cameras_[i].pointcloud_sub_ = nh_.subscribe(camera_topics[i], 1, &RRTPlannerNode::depthCameraCallback, this);
  }
}

// Sets a new goal, plans a path to it and publishes some info
void RRTPlannerNode::setNewGoal(const GoalState& goal) {
  rrt_planner.setGoal(goal);
  publishGoal(goal);
}

// Sets the next waypoint to be the current goal
void RRTPlannerNode::popNextGoal() {
  if (!waypoints_.empty()) {
    // Set the first goal in waypoints_ as the new goal
    State new_goal = waypoints_.front();
    waypoints_.erase(waypoints_.begin());
    setNewGoal(new_goal);
  } else if (rrt_planner.goal_is_blocked_) {
    // Goal is blocked but there is no other goal in waypoints_, just stop
    ROS_INFO("  STOP  ");
    rrt_planner.stop();
  }
}

// Plans a new path and publishes it
void RRTPlannerNode::planPath() {

  std::clock_t start_time = std::clock();
  
  if (rrt_planner.octree_) {
    ROS_INFO("OctoMap memory usage: %2.3f MB", rrt_planner.octree_->memoryUsage() / 1000000.0);
  }

  bool found_path = rrt_planner.getGlobalPath();

  if (!found_path) {
    // TODO: popNextGoal(), instead of checking if goal_is_blocked in
    // positionCallback?
    ROS_INFO("Failed to find a path");
  }

  printf("Total time: %2.2f ms \n", (std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));
}

void RRTPlannerNode::dynamicReconfigureCallback(global_planner::RRTPlannerNodeConfig& config, uint32_t level) {
  // rrt_planner
  rrt_planner.min_altitude_ = config.min_altitude_;
  rrt_planner.max_altitude_ = config.max_altitude_;
  rrt_planner.default_speed_ = config.default_speed_;
  rrt_planner.max_speed_ = config.max_speed_;
  rrt_planner.max_iterations_ = config.max_iterations_;

  // rrt_planner_node
  clicked_goal_alt_ = config.clicked_goal_alt_;
  clicked_goal_radius_ = config.clicked_goal_radius_;

}

void RRTPlannerNode::velocityCallback(const geometry_msgs::TwistStamped& msg) {
  rrt_planner.curr_vel_ = msg.twist.linear;
}

// Sets the current position and checks if the current goal has been reached
void RRTPlannerNode::positionCallback(const geometry_msgs::PoseStamped& msg) {
  // Update position
  last_pos_ = msg;
  rrt_planner.setPose(last_pos_);

  // Check if a new goal is needed
  if (num_pos_msg_++ % 10 == 0) {
    // Keep track of and publish the actual travel trajectory
    // ROS_INFO("Travelled path extended");
    last_pos_.header.frame_id = frame_id_;
    actual_path_.poses.push_back(last_pos_);
    actual_path_pub_.publish(actual_path_);
  }

  position_received_ = true;

  // Check if we are close enough to current goal to get the next part of the
  // path
  if (path_.size() > 0 && isCloseToGoal()) {
    // TODO: get yawdiff(yaw1, yaw2)
    double yaw1 = tf::getYaw(current_goal_.pose.orientation);
    double yaw2 = tf::getYaw(last_pos_.pose.orientation);
    double yaw_diff = std::abs(yaw2 - yaw1);
    // Transform yaw_diff to [0, 2*pi]
    yaw_diff -= std::floor(yaw_diff / (2 * M_PI)) * (2 * M_PI);
    double max_yaw_diff = M_PI / 1.0;
    if (yaw_diff < max_yaw_diff || yaw_diff > 2 * M_PI - max_yaw_diff) {
      // If we are facing the right direction, then pop the first point of the
      // path
      last_goal_ = current_goal_;
      current_goal_ = path_[0];
      rrt_planner.setIntermediateGoal(current_goal_);
      path_.erase(path_.begin());
    }
  }
}

void RRTPlannerNode::clickedPointCallback(const geometry_msgs::PointStamped& msg) {
  //plan_ = true;
  geometry_msgs::PoseStamped pose;
  pose.header = msg.header;
  pose.pose.position = msg.point;
  pose.pose.position.z = rrt_planner.curr_pos_.z;
  last_clicked_points.push_back(pose);
}

void RRTPlannerNode::moveBaseSimpleCallback(const geometry_msgs::PoseStamped& msg) {
  //plan_ = true;
  State goal;
  setNewGoal(GoalState(msg.pose.position.x, msg.pose.position.y, clicked_goal_alt_, clicked_goal_radius_));
}

void RRTPlannerNode::fcuInputGoalCallback(const mavros_msgs::Trajectory& msg) {

  //plan_ = true;

  State new_goal(msg.point_2.position.x, msg.point_2.position.y, msg.point_2.position.z);
  if (msg.point_valid[1] == true && ((std::fabs(rrt_planner.goal_pos_.xPos() - new_goal.xPos()) > 0.001) ||
                                     (std::fabs(rrt_planner.goal_pos_.yPos() - new_goal.yPos()) > 0.001))) {
    setNewGoal(new_goal);
  }
}

void RRTPlannerNode::octomapFullCallback(const octomap_msgs::Octomap& msg) {
  std::lock_guard<std::mutex> lock(mutex_);

  ros::Time current = ros::Time::now();
  // Update map at a fixed rate. This is useful on setting replanning rates for the planner.
  if ((current - last_wp_time_).toSec() < mapupdate_dt_) {
    ROS_ERROR("No Map received in last 200 ms");
    return;
  }
  last_wp_time_ = ros::Time::now();

  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(msg);

  rrt_planner.updateFullOctomap(tree);
}

// Go through obstacle points and store them
void RRTPlannerNode::depthCameraCallback(const sensor_msgs::PointCloud2& msg) {
  try {
    // Transform msg from camera frame to world frame
    ros::Time now = ros::Time::now();
    listener_.waitForTransform("/world", "/camera_link", now, ros::Duration(5.0));
    tf::StampedTransform transform;
    listener_.lookupTransform("/world", "/camera_link", now, transform);
    sensor_msgs::PointCloud2 transformed_msg;
    pcl_ros::transformPointCloud("/world", transform, msg, transformed_msg);
    pcl::PointCloud<pcl::PointXYZ> cloud;  // Easier to loop through pcl::PointCloud
    pcl::fromROSMsg(transformed_msg, cloud);

    pointcloud_pub_.publish(msg);
  } catch (tf::TransformException const& ex) {
    ROS_DEBUG("%s", ex.what());
    ROS_WARN("Transformation not available (/world to /camera_link");
  }
}

void RRTPlannerNode::setCurrentPath(const std::vector<geometry_msgs::PoseStamped>& poses) {
  path_.clear();

  if (poses.size() < 2) {
    ROS_INFO("  Received empty path\n");
    return;
  }
  last_goal_ = poses[0];
  current_goal_ = poses[1];
  rrt_planner.setIntermediateGoal(current_goal_);

  for (int i = 2; i < poses.size(); ++i) {
    path_.push_back(poses[i]);
  }
}

void RRTPlannerNode::cmdLoopCallback(const ros::TimerEvent& event) {
  hover_ = false;

  // Check if all information was received
  ros::Time now = ros::Time::now();

  ros::Duration since_last_cloud = now - last_wp_time_;
  ros::Duration since_start = now - start_time_;

  avoidance_node_.checkFailsafe(since_last_cloud, since_start, hover_);
  publishSetpoint();
}

void RRTPlannerNode::plannerLoopCallback(const ros::TimerEvent& event) {
  std::lock_guard<std::mutex> lock(mutex_);
  bool is_in_goal = isCloseToGoal();
  if (is_in_goal || rrt_planner.goal_is_blocked_) {
    popNextGoal();
  }

  planPath();

  // Print and publish info
  if (is_in_goal && !waypoints_.empty()) {
    ROS_INFO("Reached current goal, %d goals left\n\n",(int)waypoints_.size());
  }

  publishPath();
}

// Publish the position of goal
void RRTPlannerNode::publishGoal(const GoalState& goal) {
  geometry_msgs::PointStamped pointMsg;
  pointMsg.header.frame_id = frame_id_;
  pointMsg.point = goal.toPoint();

  // Always publish as temporary to remove any obsolete temporary path
  global_temp_goal_pub_.publish(pointMsg);
  if (!goal.is_temporary_) {
    global_goal_pub_.publish(pointMsg);
  }
}

// Publish the current path
void RRTPlannerNode::publishPath() {
  auto path_msg = rrt_planner.getPathMsg();
  // Always publish as temporary to remove any obsolete temporary path
  global_temp_path_pub_.publish(path_msg);
  setCurrentPath(path_msg.poses);
  //smooth_path_pub_.publish(smoothPath(path_msg));
}

template <typename P1, typename P2>
P1 subtractPoints(const P1& p1, const P2& p2) {
  P1 new_p;
  new_p.x = p1.x - p2.x;
  new_p.y = p1.y - p2.y;
  new_p.z = p1.z - p2.z;
  return new_p;
}

template <typename P>
tf::Vector3 toTfVector3(const P& point) {
  return tf::Vector3(point.x, point.y, point.z);
}

void RRTPlannerNode::publishSetpoint() {
  // Vector pointing from current position to the current goal
  tf::Vector3 vec = toTfVector3(subtractPoints(current_goal_.pose.position, last_pos_.pose.position));

  // If we are less than 1.0 away, then we should stop at the goal
  double new_len = vec.length() < 1.0 ? vec.length() : speed_;
  vec.normalize();
  vec *= new_len;

  auto setpoint = current_goal_;  // The intermediate position sent to Mavros
  setpoint.pose.position.x = last_pos_.pose.position.x + vec.getX();
  setpoint.pose.position.y = last_pos_.pose.position.y + vec.getY();
  setpoint.pose.position.z = last_pos_.pose.position.z + vec.getZ();

  // Publish setpoint for vizualization
  current_waypoint_publisher_.publish(setpoint);

  // Publish setpoint to Mavros
  mavros_waypoint_publisher_.publish(setpoint);
  mavros_msgs::Trajectory obst_free_path = {};
  geometry_msgs::Twist velocity_setpoint{};
  velocity_setpoint.linear.x = NAN;
  velocity_setpoint.linear.y = NAN;
  velocity_setpoint.linear.z = NAN;
  avoidance::transformToTrajectory(obst_free_path, setpoint, velocity_setpoint);
  mavros_obstacle_free_path_pub_.publish(obst_free_path);
}

bool RRTPlannerNode::isCloseToGoal() { 

  double dist = poseDistance(rrt_planner.goal_pos_, rrt_planner.curr_pos_);

  if( dist < 0.5)
    return true;
  else 
    return false; 

}

double RRTPlannerNode::poseDistance(const State& p1, const State& p2) {
  
  double dx = p2.xPos() - p1.xPos();
  double dy = p2.yPos() - p1.yPos();
  double dz = p2.zPos() - p1.zPos();

  Vector3d vec(dx,dy,dz);

  return  vec.norm();

}
//bool RRTPlannerNode::isCloseToGoal() { return distance(current_goal_, last_pos_) < speed_; }
