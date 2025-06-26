
#include <algorithm>
#include <string>
#include <utility>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <limits>
#include <random>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <sstream>
#include <filesystem>


#include "custom_dwa_planner/dwa_planner.hpp"

// --- State Class Implementation ---
DWAPlanner::State::State(void)
: x_(0.0), y_(0.0), yaw_(0.0), velocity_(0.0), yawrate_(0.0) {}

DWAPlanner::State::State(
  const double x, const double y, const double yaw, const double velocity,
  const double yawrate)
: x_(x), y_(y), yaw_(yaw), velocity_(velocity), yawrate_(yawrate)
{
}

// --- Window Class Implementation ---
DWAPlanner::Window::Window(void)
: min_velocity_(0.0), max_velocity_(0.0), min_yawrate_(0.0), max_yawrate_(0.0), logger_(rclcpp::get_logger(
      "dwa_planner_window")) {}

void DWAPlanner::Window::show(void)
{
  RCLCPP_INFO(logger_, "Window:");
  RCLCPP_INFO(logger_, "\tVelocity:");
  RCLCPP_INFO(logger_, "\t\tmax: %f", max_velocity_);
  RCLCPP_INFO(logger_, "\t\tmin: %f", min_velocity_);
  RCLCPP_INFO(logger_, "\tYawrate:");
  RCLCPP_INFO(logger_, "\t\tmax: %f", max_yawrate_);
  RCLCPP_INFO(logger_, "\t\tmin: %f", min_yawrate_);
}

// --- Cost Class Implementation ---
DWAPlanner::Cost::Cost(void)
: obs_cost_(0.0), to_goal_cost_(0.0), speed_cost_(0.0), path_cost_(0.0), total_cost_(0.0), logger_(rclcpp::get_logger(
      "dwa_planner_window"))
{
}

DWAPlanner::Cost::Cost(
  const float obs_cost, const float to_goal_cost, const float speed_cost, const float path_cost,
  const float total_cost)
: obs_cost_(obs_cost), to_goal_cost_(to_goal_cost), speed_cost_(speed_cost), path_cost_(path_cost),
  total_cost_(total_cost), logger_(rclcpp::get_logger("dwa_planner_window"))
{
}

void DWAPlanner::Cost::show(void)
{
  RCLCPP_INFO(logger_, "Cost: %f", total_cost_);
  RCLCPP_INFO(logger_, "\tObs cost: %f", obs_cost_);
  RCLCPP_INFO(logger_, "\tGoal cost: %f", to_goal_cost_);
  RCLCPP_INFO(logger_, "\tSpeed cost: %f", speed_cost_);
  RCLCPP_INFO(logger_, "\tPath cost: %f", path_cost_);
}

void DWAPlanner::Cost::calc_total_cost(void)
{
  total_cost_ = obs_cost_ + to_goal_cost_ + speed_cost_ + path_cost_;
}

// --- DWAPlanner Class Implementation ---

DWAPlanner::DWAPlanner(const rclcpp::NodeOptions & options)
: rclcpp::Node("dwa_planner_node", options),
  odom_updated_(false), local_map_updated_(false), scan_updated_(false), has_reached_(false),
  use_speed_cost_(false), odom_not_subscribe_count_(0), local_map_not_subscribe_count_(0),
  scan_not_subscribe_count_(0), timing_active_(false), run_counter_(0)
{

  RCLCPP_INFO(this->get_logger(), "=== CUSTOM DWA Planner ===");


  load_params();

  print_params();

  // Initialize ROS publishers
  velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  candidate_trajectories_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "candidate_trajectories", 1);
  selected_trajectory_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "selected_trajectory", 1);
  predict_footprints_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "predict_footprints", 1);
  finish_flag_pub_ = this->create_publisher<std_msgs::msg::Bool>("finish_flag", 1);

  // Initialize ROS subscribers
  dist_to_goal_th_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/dist_to_goal_th", 1,
    std::bind(&DWAPlanner::dist_to_goal_th_callback, this, std::placeholders::_1));
  edge_on_global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 1, std::bind(&DWAPlanner::edge_on_global_path_callback, this, std::placeholders::_1));
  footprint_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
    "/footprint", 1, std::bind(&DWAPlanner::footprint_callback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/move_base_simple/goal", 1,
    std::bind(&DWAPlanner::goal_callback, this, std::placeholders::_1));
  local_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/local_map", 1, std::bind(&DWAPlanner::local_map_callback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 1, std::bind(&DWAPlanner::odom_callback, this, std::placeholders::_1));
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 1, std::bind(&DWAPlanner::scan_callback, this, std::placeholders::_1));
  target_velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/target_velocity", 1,
    std::bind(&DWAPlanner::target_velocity_callback, this, std::placeholders::_1));

  // Initialize TF listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if (!use_footprint_) {
    footprint_ = geometry_msgs::msg::PolygonStamped();
  }
  if (!use_path_cost_) {
    edge_points_on_path_ = nav_msgs::msg::Path();
  }
  if (!use_scan_as_input_) {
    scan_updated_ = true;
  } else {
    local_map_updated_ = true;
  }
  
  // Initialize time measurement and debugging
  init_time_measurement();
}

// --- Callback Functions ---
void DWAPlanner::goal_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  goal_msg_ = *msg;
  if (goal_msg_->header.frame_id != global_frame_) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_->lookupTransform(
        global_frame_, goal_msg_->header.frame_id,
        tf2::TimePointZero);


      geometry_msgs::msg::PoseStamped transformed_pose;
      tf2::doTransform(*goal_msg_, transformed_pose, transform_stamped);
      goal_msg_ = transformed_pose;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform goal: %s", ex.what());
      goal_msg_.reset();
      return;
    }
  }
  
  // Start timing when new goal is received
  start_timing();
  
  // Log goal information
  std::stringstream goal_info;
  goal_info << "New goal received: (" 
            << std::fixed << std::setprecision(3) 
            << goal_msg_->pose.position.x << ", " 
            << goal_msg_->pose.position.y << ", " 
            << goal_msg_->pose.position.z << ")";
  log_debug_info(goal_info.str(), "INFO");
}

void DWAPlanner::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  if (use_scan_as_input_) {
    create_obs_list(*msg);
  }
  scan_not_subscribe_count_ = 0;
  scan_updated_ = true;
}

void DWAPlanner::local_map_callback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  if (!use_scan_as_input_) {
    create_obs_list(*msg);
  }
  local_map_not_subscribe_count_ = 0;
  local_map_updated_ = true;
}

void DWAPlanner::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  current_cmd_vel_ = msg->twist.twist;
  odom_not_subscribe_count_ = 0;
  odom_updated_ = true;
}

void DWAPlanner::target_velocity_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  target_velocity_ = std::min(msg->linear.x, max_velocity_);
  RCLCPP_INFO_STREAM_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "target velocity was updated to " << target_velocity_ << " [m/s]");
}

void DWAPlanner::footprint_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  footprint_ = *msg;
  for (auto & point : footprint_->polygon.points) {
    point.x += point.x < 0 ? -footprint_padding_ : footprint_padding_;
    point.y += point.y < 0 ? -footprint_padding_ : footprint_padding_;
  }
}

void DWAPlanner::dist_to_goal_th_callback(const std_msgs::msg::Float64::ConstSharedPtr msg)
{
  dist_to_goal_th_ = msg->data;
  RCLCPP_INFO_STREAM_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "distance to goal threshold was updated to " << dist_to_goal_th_ << " [m]");
}

void DWAPlanner::edge_on_global_path_callback(const nav_msgs::msg::Path::ConstSharedPtr msg)
{
  if (!use_path_cost_) {
    return;
  }
  edge_points_on_path_ = *msg;

  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform(
      robot_frame_, msg->header.frame_id,
      tf2::TimePointZero);

    for (auto & pose : edge_points_on_path_->poses) {
      geometry_msgs::msg::PoseStamped original_pose_stamped;
      original_pose_stamped.header = msg->header;
      original_pose_stamped.pose = pose.pose;

      geometry_msgs::msg::PoseStamped transformed_pose_stamped;
      tf2::doTransform(original_pose_stamped, transformed_pose_stamped, transform_stamped);
      pose.pose = transformed_pose_stamped.pose;
    }

  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not transform path edge points: %s", ex.what());
    edge_points_on_path_.reset();
  }
}

// --- DWA Planning Core Functions ---
std::vector<DWAPlanner::State>
DWAPlanner::dwa_planning(
  const Eigen::Vector3d & goal, std::vector<std::pair<std::vector<State>,
  bool>> & trajectories)
{
  Cost min_cost(0.0, 0.0, 0.0, 0.0, 1e6);
  const Window dynamic_window = calc_dynamic_window();
  std::vector<State> best_traj;
  best_traj.resize(sim_time_samples_);
  std::vector<Cost> costs;
  const size_t costs_size = velocity_samples_ * (yawrate_samples_ + 1);
  costs.reserve(costs_size);
  
  // Log DWA planning start
  std::stringstream planning_start_info;
  planning_start_info << "DWA Planning Start - Goal: (" 
                      << std::fixed << std::setprecision(3)
                      << goal.x() << ", " << goal.y() << ", " << goal.z() << ")";
  log_debug_info(planning_start_info.str(), "INFO");

  const double velocity_resolution =
    std::max(
    (dynamic_window.max_velocity_ - dynamic_window.min_velocity_) /
    (static_cast<double>(velocity_samples_ - 1) + std::numeric_limits<double>::epsilon()),
    DBL_EPSILON);
  const double yawrate_resolution =
    std::max(
    (dynamic_window.max_yawrate_ - dynamic_window.min_yawrate_) /
    (static_cast<double>(yawrate_samples_ - 1) + std::numeric_limits<double>::epsilon()),
    DBL_EPSILON);

  // Goal-directed polar lattice sampling
  int available_traj_count = 0;
  // heading error (goal is already in robot frame)
  const double goal_angle = std::atan2(goal.y(), goal.x());
  // desired yawrate to point at goal within prediction horizon
  const double center_yawrate = std::clamp(goal_angle / predict_time_, dynamic_window.min_yawrate_, dynamic_window.max_yawrate_);
  const double yawrate_start = std::clamp(
    center_yawrate - yawrate_resolution * 0.5 * (yawrate_samples_ - 1),
    dynamic_window.min_yawrate_, dynamic_window.max_yawrate_);

  for (int i = 0; i < velocity_samples_; ++i) {
    const double v = dynamic_window.min_velocity_ + velocity_resolution * i;
    for (int j = 0; j < yawrate_samples_; j++) {
      std::pair<std::vector<State>, bool> traj;
      double y = yawrate_start + yawrate_resolution * j;
      y = std::clamp(y, dynamic_window.min_yawrate_, dynamic_window.max_yawrate_);
      if (v < slow_velocity_th_) {
        y = y > 0 ? std::max(y, min_yawrate_) : std::min(y, -min_yawrate_);
      }
      traj.first = generate_trajectory(v, y);
      const Cost cost = evaluate_trajectory(traj.first, goal);
      costs.push_back(cost);
      if (cost.obs_cost_ == 1e6) {
        traj.second = false;
      } else {
        traj.second = true;
        available_traj_count++;
      }
      
      // Log detailed trajectory information
      std::stringstream traj_info;
      traj_info << "Trajectory " << trajectories.size() << " - v:" << v << " w:" << y
                << " - Costs[obs:" << cost.obs_cost_ << ", goal:" << cost.to_goal_cost_
                << ", speed:" << cost.speed_cost_ << ", path:" << cost.path_cost_
                << ", total:" << cost.total_cost_ << "] - Valid:" << (traj.second ? "YES" : "NO");
      log_debug_info(traj_info.str(), "DEBUG");
      
      trajectories.push_back(traj);
    }

    if (dynamic_window.min_yawrate_ < 0.0 && 0.0 < dynamic_window.max_yawrate_) {
      std::pair<std::vector<State>, bool> traj;
      traj.first = generate_trajectory(v, 0.0);
      const Cost cost = evaluate_trajectory(traj.first, goal);
      costs.push_back(cost);
      if (cost.obs_cost_ == 1e6) {
        traj.second = false;
      } else {
        traj.second = true;
        available_traj_count++;
      }
      
      // Log detailed trajectory information for zero yawrate
      std::stringstream traj_info;
      traj_info << "Trajectory " << trajectories.size() << " - v:" << v << " w:0.0"
                << " - Costs[obs:" << cost.obs_cost_ << ", goal:" << cost.to_goal_cost_
                << ", speed:" << cost.speed_cost_ << ", path:" << cost.path_cost_
                << ", total:" << cost.total_cost_ << "] - Valid:" << (traj.second ? "YES" : "NO");
      log_debug_info(traj_info.str(), "DEBUG");
      
      trajectories.push_back(traj);
    }
  }

  if (available_traj_count == 0) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No available trajectory");
    log_debug_info("CRITICAL: No available trajectories found! Robot may be stuck or facing obstacles.", "ERROR");
    
    // Log obstacle information for debugging
    std::stringstream obs_info;
    obs_info << "Obstacle count: " << obs_list_.poses.size()
             << " - Dynamic window: v[" << dynamic_window.min_velocity_ << ", " << dynamic_window.max_velocity_
             << "] w[" << dynamic_window.min_yawrate_ << ", " << dynamic_window.max_yawrate_ << "]";
    log_debug_info(obs_info.str(), "ERROR");
    
    best_traj = generate_trajectory(0.0, 0.0);
  } else {
    normalize_costs(costs);
    log_debug_info("Cost normalization completed", "DEBUG");
    
    for (int i = 0; i < costs.size(); i++) {
      if (costs[i].obs_cost_ != 1e6) {
        costs[i].to_goal_cost_ *= to_goal_cost_gain_;
        costs[i].obs_cost_ *= obs_cost_gain_;
        costs[i].speed_cost_ *= speed_cost_gain_;
        costs[i].path_cost_ *= path_cost_gain_;
        costs[i].calc_total_cost();
        
        // Log normalized and weighted costs
        std::stringstream cost_info;
        cost_info << "Final Trajectory " << i << " - Weighted Costs[obs:" 
                  << costs[i].obs_cost_ << ", goal:" << costs[i].to_goal_cost_
                  << ", speed:" << costs[i].speed_cost_ << ", path:" << costs[i].path_cost_
                  << ", total:" << costs[i].total_cost_ << "]";
        log_debug_info(cost_info.str(), "DEBUG");
        
        if (costs[i].total_cost_ < min_cost.total_cost_) {
          min_cost = costs[i];
          best_traj = trajectories[i].first;
          
          // Log when a new best trajectory is found
          std::stringstream best_info;
          best_info << "New best trajectory found at index " << i 
                    << " with total cost: " << costs[i].total_cost_;
          log_debug_info(best_info.str(), "DEBUG");
        }
      }
    }
  }

  // Enhanced logging for DWA planning results
  std::stringstream planning_info;
  planning_info << "DWA Planning Results - Selected: (v=" 
                << std::fixed << std::setprecision(3) 
                << best_traj.front().velocity_ << ", w=" 
                << best_traj.front().yawrate_ << ") - Cost: " 
                << min_cost.total_cost_ << " - Available trajectories: " 
                << available_traj_count << "/" << trajectories.size();
  log_debug_info(planning_info.str(), "INFO");
  
  // Log detailed cost breakdown for selected trajectory
  std::stringstream cost_breakdown;
  cost_breakdown << "Selected trajectory cost breakdown - Obs:" << min_cost.obs_cost_
                 << " Goal:" << min_cost.to_goal_cost_ << " Speed:" << min_cost.speed_cost_
                 << " Path:" << min_cost.path_cost_ << " Total:" << min_cost.total_cost_;
  log_debug_info(cost_breakdown.str(), "INFO");
  
  // Log selected trajectory details
  log_trajectory_info(best_traj, "Selected");

  RCLCPP_INFO(this->get_logger(), "===");
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "(v, y) = (" << best_traj.front().velocity_ << ", " << best_traj.front().yawrate_ << ")");
  min_cost.show();
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "num of trajectories available: " << available_traj_count << " of " << trajectories.size());
  RCLCPP_INFO(this->get_logger(), " ");

  return best_traj;
}

// Normalize costs for comparison
void DWAPlanner::normalize_costs(std::vector<DWAPlanner::Cost> & costs)
{
  Cost min_cost(1e6, 1e6, 1e6, 1e6, 1e6), max_cost;

  for (const auto & cost : costs) {
    if (cost.obs_cost_ != 1e6) {
      min_cost.obs_cost_ = std::min(min_cost.obs_cost_, cost.obs_cost_);
      max_cost.obs_cost_ = std::max(max_cost.obs_cost_, cost.obs_cost_);
      min_cost.to_goal_cost_ = std::min(min_cost.to_goal_cost_, cost.to_goal_cost_);
      max_cost.to_goal_cost_ = std::max(max_cost.to_goal_cost_, cost.to_goal_cost_);
      if (use_speed_cost_) {
        min_cost.speed_cost_ = std::min(min_cost.speed_cost_, cost.speed_cost_);
        max_cost.speed_cost_ = std::max(max_cost.speed_cost_, cost.speed_cost_);
      }
      if (use_path_cost_) {
        min_cost.path_cost_ = std::min(min_cost.path_cost_, cost.path_cost_);
        max_cost.path_cost_ = std::max(max_cost.path_cost_, cost.path_cost_);
      }
    }
  }

  for (auto & cost : costs) {
    if (cost.obs_cost_ != 1e6) {
      cost.obs_cost_ = (cost.obs_cost_ - min_cost.obs_cost_) /
        (max_cost.obs_cost_ - min_cost.obs_cost_ + std::numeric_limits<double>::epsilon());
      cost.to_goal_cost_ = (cost.to_goal_cost_ - min_cost.to_goal_cost_) /
        (max_cost.to_goal_cost_ - min_cost.to_goal_cost_ + std::numeric_limits<double>::epsilon());
      if (use_speed_cost_) {
        cost.speed_cost_ =
          (cost.speed_cost_ - min_cost.speed_cost_) /
          (max_cost.speed_cost_ - min_cost.speed_cost_ + std::numeric_limits<double>::epsilon());
      }
      if (use_path_cost_) {
        cost.path_cost_ =
          (cost.path_cost_ - min_cost.path_cost_) /
          (max_cost.path_cost_ - min_cost.path_cost_ + std::numeric_limits<double>::epsilon());
      }
    }
  }
}

// Main loop
void DWAPlanner::process(void)
{
  rclcpp::Rate loop_rate(hz_);
  while (rclcpp::ok()) {
    geometry_msgs::msg::Twist cmd_vel;
    if (can_move()) {
      cmd_vel = calc_cmd_vel();
    }
    velocity_pub_->publish(cmd_vel);
    finish_flag_pub_->publish(has_finished_);
    if (has_finished_.data) {
      loop_rate.sleep();
    }
    rclcpp::sleep_for(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(sleep_time_after_finish_)));


    if (use_scan_as_input_) {
      scan_updated_ = false;
    } else {
      local_map_updated_ = false;
    }
    odom_updated_ = false;
    has_finished_.data = false;

    rclcpp::spin_some(this->get_node_base_interface());
    loop_rate.sleep();
  }
}

// Check if planner can move
bool DWAPlanner::can_move(void)
{

  if (!footprint_.has_value()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), 1000, "Robot Footprint has not been updated");
  }
  if (!goal_msg_.has_value()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), 1000, "Local goal has not been updated");
  }
  if (!edge_points_on_path_.has_value()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), 1000, "Edge on global path has not been updated");
  }
  if (subscribe_count_th_ < odom_not_subscribe_count_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Odom has not been updated");
  }
  if (subscribe_count_th_ < local_map_not_subscribe_count_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), 1000, "Local map has not been updated");
  }
  if (subscribe_count_th_ < scan_not_subscribe_count_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Scan has not been updated");
  }

  if (!odom_updated_) {
    odom_not_subscribe_count_++;
  }
  if (!local_map_updated_) {
    local_map_not_subscribe_count_++;
  }
  if (!scan_updated_) {
    scan_not_subscribe_count_++;
  }


  if (footprint_.has_value() && goal_msg_.has_value() && edge_points_on_path_.has_value() &&
    odom_not_subscribe_count_ <= subscribe_count_th_ &&
    local_map_not_subscribe_count_ <= subscribe_count_th_ &&
    scan_not_subscribe_count_ <= subscribe_count_th_)
  {
    return true;
  } else {
    return false;
  }
}

// Calculate command velocity
geometry_msgs::msg::Twist DWAPlanner::calc_cmd_vel(void)
{
  geometry_msgs::msg::Twist cmd_vel;
  std::pair<std::vector<State>, bool> best_traj;
  std::vector<std::pair<std::vector<State>, bool>> trajectories;
  const size_t trajectories_size = velocity_samples_ * (yawrate_samples_ + 1);
  trajectories.reserve(trajectories_size);

  geometry_msgs::msg::PoseStamped goal_;
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {

    transform_stamped = tf_buffer_->lookupTransform(
      robot_frame_, goal_msg_->header.frame_id,
      tf2::TimePointZero);

    tf2::doTransform(*goal_msg_, goal_, transform_stamped);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not transform goal for calc_cmd_vel: %s", ex.what());
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return cmd_vel;
  }

  tf2::Quaternion tf2_quat;
  tf2::fromMsg(goal_.pose.orientation, tf2_quat);

  tf2::Matrix3x3 m(tf2_quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  const Eigen::Vector3d goal(goal_.pose.position.x, goal_.pose.position.y, yaw);


  const double angle_to_goal = atan2(goal.y(), goal.x());
  if (M_PI / 4.0 < fabs(angle_to_goal)) {
    use_speed_cost_ = true;
  }

  if (dist_to_goal_th_ < goal.segment(0, 2).norm() && !has_reached_) {
    if (can_adjust_robot_direction(goal)) {
      cmd_vel.angular.z = angle_to_goal > 0 ? std::min(angle_to_goal, max_in_place_yawrate_) :
        std::max(angle_to_goal, -max_in_place_yawrate_);
      cmd_vel.angular.z = cmd_vel.angular.z >
        0 ? std::max(cmd_vel.angular.z, min_in_place_yawrate_) :
        std::min(cmd_vel.angular.z, -min_in_place_yawrate_);
      best_traj.first = generate_trajectory(cmd_vel.angular.z, goal);
      trajectories.push_back(best_traj);
    } else {
      best_traj.first = dwa_planning(goal, trajectories);
      cmd_vel.linear.x = best_traj.first.front().velocity_;
      cmd_vel.angular.z = best_traj.first.front().yawrate_;
    }
  } else {
    has_reached_ = true;
    if (turn_direction_th_ < fabs(goal[2])) {
      cmd_vel.angular.z =
        goal[2] > 0 ? std::min(goal[2], max_in_place_yawrate_) : std::max(
        goal[2],
        -max_in_place_yawrate_);
      cmd_vel.angular.z = cmd_vel.angular.z >
        0 ? std::max(cmd_vel.angular.z, min_in_place_yawrate_) :
        std::min(cmd_vel.angular.z, -min_in_place_yawrate_);
    } else {
      has_finished_.data = true;
      has_reached_ = false;
      
      // Stop timing and log when goal is reached
      stop_timing_and_log(goal);
    }
    best_traj.first = generate_trajectory(cmd_vel.linear.x, cmd_vel.angular.z);
    trajectories.push_back(best_traj);


  }


  for (int i = 0; i < trajectories_size; i++) {
    trajectories.push_back(trajectories.front());
  }


  visualize_trajectory(best_traj.first, selected_trajectory_pub_);
  visualize_trajectories(trajectories, candidate_trajectories_pub_);
  visualize_footprints(best_traj.first, predict_footprints_pub_);

  use_speed_cost_ = false;

  return cmd_vel;
}

// Check if robot can adjust direction
bool DWAPlanner::can_adjust_robot_direction(const Eigen::Vector3d & goal)
{
  const double angle_to_goal = atan2(goal.y(), goal.x());
  if (fabs(angle_to_goal) < angle_to_goal_th_) {
    return false;
  }

  const double yawrate = std::min(
    std::max(
      angle_to_goal,
      -max_in_place_yawrate_), max_in_place_yawrate_);
  std::vector<State> traj = generate_trajectory(0.0, yawrate);

  if (!check_collision(traj)) {
    return true;
  } else {
    return false;
  }
}

// Check if trajectory collides with obstacles
bool DWAPlanner::check_collision(const std::vector<State> & traj)
{
  if (!use_footprint_) {
    return false;
  }

  if (obs_list_.poses.empty()) {
    return false;
  }

  // Build spatial hash grid of obstacles
  const double cell_size = 0.5;  // keep in sync with calc_obs_cost
  auto cell_index = [cell_size](double x, double y) {
    return std::make_pair(
      static_cast<int>(std::floor(x / cell_size)),
      static_cast<int>(std::floor(y / cell_size)));
  };
  auto make_key = [](int ix, int iy) {
    return (static_cast<int64_t>(ix) << 32) ^ (static_cast<int64_t>(iy) & 0xffffffff);
  };

  std::unordered_map<int64_t, std::vector<const geometry_msgs::msg::Pose*>> grid;
  grid.reserve(obs_list_.poses.size());
  for (const auto & obs : obs_list_.poses) {
    const auto [ix, iy] = cell_index(obs.position.x, obs.position.y);
    grid[make_key(ix, iy)].push_back(&obs);
  }

  const double search_radius = robot_radius_ + footprint_padding_ + 0.2;  // small margin
  const int neighborhood_radius = static_cast<int>(std::ceil(search_radius / cell_size));

  for (const auto & state : traj) {
    const auto [cx, cy] = cell_index(state.x_, state.y_);
    const geometry_msgs::msg::PolygonStamped footprint = move_footprint(state);
    for (int dx = -neighborhood_radius; dx <= neighborhood_radius; ++dx) {
      for (int dy = -neighborhood_radius; dy <= neighborhood_radius; ++dy) {
        auto it = grid.find(make_key(cx + dx, cy + dy));
        if (it == grid.end()) {
          continue;
        }
        for (const auto * obs_ptr : it->second) {
          const auto & obs = *obs_ptr;
          // quick bounding-circle reject
          if (std::hypot(state.x_ - obs.position.x, state.y_ - obs.position.y) > search_radius) {
            continue;
          }
          if (is_inside_of_robot(obs.position, footprint, state)) {
            return true;
          }
        }
      }
    }
  }

  return false;
}

// Calculate dynamic window
DWAPlanner::Window DWAPlanner::calc_dynamic_window(void)
{
  Window window;
  window.min_velocity_ = std::max(
    (current_cmd_vel_.linear.x - max_deceleration_ * sim_period_),
    min_velocity_);
  window.max_velocity_ = std::min(
    (current_cmd_vel_.linear.x + max_acceleration_ * sim_period_),
    target_velocity_);
  window.min_yawrate_ = std::max(
    (current_cmd_vel_.angular.z - max_d_yawrate_ * sim_period_),
    -max_yawrate_);
  window.max_yawrate_ = std::min(
    (current_cmd_vel_.angular.z + max_d_yawrate_ * sim_period_),
    max_yawrate_);
  return window;
}

// Calculate distance to goal cost
float DWAPlanner::calc_to_goal_cost(const std::vector<State> & traj, const Eigen::Vector3d & goal)
{
  Eigen::Vector3d last_position(traj.back().x_, traj.back().y_, traj.back().yaw_);
  return (last_position.segment(0, 2) - goal.segment(0, 2)).norm();
}

// Calculate obstacle cost
float DWAPlanner::calc_obs_cost(const std::vector<State> & traj)
{
  // Spatial hash grid parameters
  const double cell_size = 0.5;  // [m]  tune if necessary
  const int neighborhood_radius = static_cast<int>(std::ceil(obs_range_ / cell_size));

  // Helper lambdas
  auto cell_index = [cell_size](double x, double y) {
    return std::make_pair(
      static_cast<int>(std::floor(x / cell_size)),
      static_cast<int>(std::floor(y / cell_size)));
  };
  auto make_key = [](int ix, int iy) {
    return (static_cast<int64_t>(ix) << 32) ^ (static_cast<int64_t>(iy) & 0xffffffff);
  };

  // Build spatial grid of obstacles (pointer to avoid copies)
  std::unordered_map<int64_t, std::vector<const geometry_msgs::msg::Pose*>> grid;
  grid.reserve(obs_list_.poses.size());
  for (const auto & obs : obs_list_.poses) {
    const auto [ix, iy] = cell_index(obs.position.x, obs.position.y);
    grid[make_key(ix, iy)].push_back(&obs);
  }

  float min_dist = obs_range_;

  // Iterate trajectory states and query nearby grid cells only
  for (const auto & state : traj) {
    const auto [cx, cy] = cell_index(state.x_, state.y_);
    for (int dx = -neighborhood_radius; dx <= neighborhood_radius; ++dx) {
      for (int dy = -neighborhood_radius; dy <= neighborhood_radius; ++dy) {
        auto it = grid.find(make_key(cx + dx, cy + dy));
        if (it == grid.end()) {
          continue;
        }
        for (const auto * obs_ptr : it->second) {
          const auto & obs = *obs_ptr;
          float dist;
          if (use_footprint_) {
            dist = calc_dist_from_robot(obs.position, state);
          } else {
            dist = std::hypot(state.x_ - obs.position.x, state.y_ - obs.position.y) -
              robot_radius_ - footprint_padding_;
          }
          if (dist < std::numeric_limits<double>::epsilon()) {
            return 1e6;  // Collision
          }
          min_dist = std::min(min_dist, dist);
        }
      }
    }
  }
  return obs_range_ - min_dist;
}

// Calculate speed cost
float DWAPlanner::calc_speed_cost(const std::vector<State> & traj)
{
  if (!use_speed_cost_) {
    return 0.0;
  }
  const Window dynamic_window = calc_dynamic_window();
  return dynamic_window.max_velocity_ - traj.front().velocity_;
}

// Calculate path cost
float DWAPlanner::calc_path_cost(const std::vector<State> & traj)
{
  if (!use_path_cost_) {
    return 0.0;
  } else if (edge_points_on_path_.has_value()) {
    return calc_dist_to_path(traj.back());
  } else {
    return 0.0;
  }
}

// Calculate distance to path
float DWAPlanner::calc_dist_to_path(const State state)
{
  if (!edge_points_on_path_.has_value() || edge_points_on_path_->poses.empty()) {
    RCLCPP_WARN_ONCE(this->get_logger(), "Path for cost calculation is not available.");
    return 0.0;
  }
  geometry_msgs::msg::Point edge_point1 = edge_points_on_path_->poses.front().pose.position;
  geometry_msgs::msg::Point edge_point2 = edge_points_on_path_->poses.back().pose.position;
  const float a = edge_point2.y - edge_point1.y;
  const float b = -(edge_point2.x - edge_point1.x);
  const float c = -a * edge_point1.x - b * edge_point1.y;

  return fabs(a * state.x_ + b * state.y_ + c) /
         (hypot(a, b) + std::numeric_limits<double>::epsilon());
}

// Generate trajectory
std::vector<DWAPlanner::State> DWAPlanner::generate_trajectory(
  const double velocity,
  const double yawrate)
{
  std::vector<State> trajectory;
  trajectory.resize(sim_time_samples_);
  State state;
  for (int i = 0; i < sim_time_samples_; i++) {
    motion(state, velocity, yawrate);
    trajectory[i] = state;
  }
  return trajectory;
}

// Generate trajectory
std::vector<DWAPlanner::State> DWAPlanner::generate_trajectory(
  const double yawrate,
  const Eigen::Vector3d & goal)
{
  const double target_direction = atan2(goal.y(), goal.x()) > 0 ? sim_direction_ : -sim_direction_;

  const double predict_time_for_turn = target_direction /
    (yawrate + std::numeric_limits<double>::epsilon());

  std::vector<State> trajectory;
  trajectory.resize(sim_time_samples_);
  State state;
  for (int i = 0; i < sim_time_samples_; i++) {
    motion(state, 0.0, yawrate);
    trajectory[i] = state;
  }
  return trajectory;
}

// Evaluate trajectory
DWAPlanner::Cost DWAPlanner::evaluate_trajectory(
  const std::vector<State> & trajectory,
  const Eigen::Vector3d & goal)
{
  Cost cost;
  cost.to_goal_cost_ = calc_to_goal_cost(trajectory, goal);
  cost.obs_cost_ = calc_obs_cost(trajectory);
  cost.speed_cost_ = calc_speed_cost(trajectory);
  cost.path_cost_ = calc_path_cost(trajectory);
  cost.calc_total_cost();
  return cost;
}

// Calculate intersection point
geometry_msgs::msg::Point DWAPlanner::calc_intersection(
  const geometry_msgs::msg::Point & obstacle, const State & state,
  geometry_msgs::msg::PolygonStamped footprint)
{
  for (size_t i = 0; i < footprint.polygon.points.size(); ++i) {
    const Eigen::Vector3d vector_A(obstacle.x, obstacle.y, 0.0);
    const Eigen::Vector3d vector_B(state.x_, state.y_, 0.0);
    const Eigen::Vector3d vector_C(footprint.polygon.points[i].x, footprint.polygon.points[i].y,
      0.0);
    Eigen::Vector3d vector_D(0.0, 0.0, 0.0);
    if (i != footprint.polygon.points.size() - 1) {
      vector_D << footprint.polygon.points[i + 1].x, footprint.polygon.points[i + 1].y, 0.0;
    } else {
      vector_D << footprint.polygon.points[0].x, footprint.polygon.points[0].y, 0.0;
    }

    const double deno = (vector_B - vector_A).cross(vector_D - vector_C).z();

    if (std::abs(deno) < std::numeric_limits<double>::epsilon()) {
      continue;
    }
    const double s = (vector_C - vector_A).cross(vector_D - vector_C).z() / deno;
    const double t = (vector_B - vector_A).cross(vector_A - vector_C).z() / deno;

    geometry_msgs::msg::Point point;
    point.x = vector_A.x() + s * (vector_B - vector_A).x();
    point.y = vector_A.y() + s * (vector_B - vector_A).y();


    if (!(s < 0.0 || 1.0 < s || t < 0.0 || 1.0 < t)) {
      return point;
    }
  }

  geometry_msgs::msg::Point point;
  point.x = 1e6;
  point.y = 1e6;
  return point;
}

// Calculate distance from robot
float DWAPlanner::calc_dist_from_robot(
  const geometry_msgs::msg::Point & obstacle,
  const State & state)
{
  const geometry_msgs::msg::PolygonStamped footprint = move_footprint(state);
  if (is_inside_of_robot(obstacle, footprint, state)) {
    return 0.0;
  } else {
    geometry_msgs::msg::Point intersection = calc_intersection(obstacle, state, footprint);
    return hypot((obstacle.x - intersection.x), (obstacle.y - intersection.y));
  }
}

// Move footprint
geometry_msgs::msg::PolygonStamped DWAPlanner::move_footprint(const State & target_pose)
{
  geometry_msgs::msg::PolygonStamped footprint_moved;
  if (use_footprint_ && footprint_.has_value()) {
    footprint_moved = footprint_.value();
  } else {
    const int plot_num = 20;
    for (int i = 0; i < plot_num; i++) {
      geometry_msgs::msg::Point32 point;
      point.x = (robot_radius_ + footprint_padding_) * cos(2 * M_PI * i / plot_num);
      point.y = robot_radius_ * sin(2 * M_PI * i / plot_num);
      footprint_moved.polygon.points.push_back(point);
    }
  }

  footprint_moved.header.stamp = this->get_clock()->now();

  for (auto & point : footprint_moved.polygon.points) {
    Eigen::Vector2f point_in(point.x, point.y);
    Eigen::Rotation2Df rot(target_pose.yaw_);
    const Eigen::Vector2f point_out = rot * point_in;

    point.x = point_out.x() + target_pose.x_;
    point.y = point_out.y() + target_pose.y_;
  }

  return footprint_moved;
}

// Check if obstacle is inside robot footprint
bool DWAPlanner::is_inside_of_robot(
  const geometry_msgs::msg::Point & obstacle, const geometry_msgs::msg::PolygonStamped & footprint,
  const State & state)
{
  geometry_msgs::msg::Point32 state_point;
  state_point.x = state.x_;
  state_point.y = state.y_;

  for (size_t i = 0; i < footprint.polygon.points.size(); ++i) {
    geometry_msgs::msg::Polygon triangle;
    triangle.points.push_back(state_point);
    triangle.points.push_back(footprint.polygon.points[i]);

    if (i != footprint.polygon.points.size() - 1) {
      triangle.points.push_back(footprint.polygon.points[i + 1]);
    } else {
      triangle.points.push_back(footprint.polygon.points[0]);
    }

    if (is_inside_of_triangle(obstacle, triangle)) {
      return true;
    }
  }

  return false;
}

// Check if point is inside triangle
bool DWAPlanner::is_inside_of_triangle(
  const geometry_msgs::msg::Point & target_point,
  const geometry_msgs::msg::Polygon & triangle)
{
  if (triangle.points.size() != 3) {
    RCLCPP_ERROR(this->get_logger(), "Not triangle, points size: %zu", triangle.points.size());

    return false;
  }

  const Eigen::Vector3d vector_A(triangle.points[0].x, triangle.points[0].y, 0.0);
  const Eigen::Vector3d vector_B(triangle.points[1].x, triangle.points[1].y, 0.0);
  const Eigen::Vector3d vector_C(triangle.points[2].x, triangle.points[2].y, 0.0);
  const Eigen::Vector3d vector_P(target_point.x, target_point.y, 0.0);

  const Eigen::Vector3d vector_AB = vector_B - vector_A;
  const Eigen::Vector3d vector_BP = vector_P - vector_B;
  const Eigen::Vector3d cross1 = vector_AB.cross(vector_BP);

  const Eigen::Vector3d vector_BC = vector_C - vector_B;
  const Eigen::Vector3d vector_CP = vector_P - vector_C;
  const Eigen::Vector3d cross2 = vector_BC.cross(vector_CP);

  const Eigen::Vector3d vector_CA = vector_A - vector_C;
  const Eigen::Vector3d vector_AP = vector_P - vector_A;
  const Eigen::Vector3d cross3 = vector_CA.cross(vector_AP);


  if ((cross1.z() >= 0 && cross2.z() >= 0 && cross3.z() >= 0) ||
    (cross1.z() <= 0 && cross2.z() <= 0 && cross3.z() <= 0))
  {
    return true;
  } else {
    return false;
  }
}

// Update state
void DWAPlanner::motion(State & state, const double velocity, const double yawrate)
{
  const double sim_time_step = predict_time_ / static_cast<double>(sim_time_samples_);
  // pre-compute trig of current yaw before update
  const double cos_yaw = std::cos(state.yaw_);
  const double sin_yaw = std::sin(state.yaw_);
  state.x_ += velocity * cos_yaw * sim_time_step;
  state.y_ += velocity * sin_yaw * sim_time_step;
  state.yaw_ += yawrate * sim_time_step;
  state.velocity_ = velocity;
  state.yawrate_ = yawrate;
}

// Create obstacle list using LaserScan with Spatial Grid 
void DWAPlanner::create_obs_list(const sensor_msgs::msg::LaserScan & scan)
{
  obs_list_.poses.clear();

  const int angle_index_step =
    static_cast<int>(std::round(angle_resolution_ / static_cast<double>(scan.angle_increment)));
  if (angle_index_step <= 0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid angle_index_step (<=0)");
    return;
  }

  // Reserve expected capacity to avoid reallocations
  obs_list_.poses.reserve(scan.ranges.size() / angle_index_step + 1);

  const float angle_min = scan.angle_min;
  const float inc = scan.angle_increment;
  const size_t total = scan.ranges.size();

  for (size_t i = 0; i < total; i += angle_index_step) {
    const float r = scan.ranges[i];
    if (r < scan.range_min || r > scan.range_max) {
      continue;
    }
    const float ang = angle_min + static_cast<float>(i) * inc;
    geometry_msgs::msg::Pose pose;
    pose.position.x = r * std::cos(ang);
    pose.position.y = r * std::sin(ang);
    obs_list_.poses.push_back(pose);
  }
}

// Create obstacle list using OccupancyGrid with Bresenham's Line Algorithm
void DWAPlanner::create_obs_list(const nav_msgs::msg::OccupancyGrid & map)
{
  obs_list_.poses.clear();  

  const double origin_x = map.info.origin.position.x;
  const double origin_y = map.info.origin.position.y;
  const double resolution = map.info.resolution;
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);

  // Conservative upper bound on search distance: map diagonal
  const double max_search_dist = std::hypot(width * resolution, height * resolution);

  // Pre-compute unit vectors for each angle ray
  std::vector<std::pair<double, double>> unit_vecs;
  for (float angle = -static_cast<float>(M_PI); angle <= static_cast<float>(M_PI); angle += angle_resolution_) {
    unit_vecs.emplace_back(std::cos(angle), std::sin(angle));
  }
  obs_list_.poses.reserve(unit_vecs.size());

  for (const auto & uv : unit_vecs) {
    const double dx = uv.first;
    const double dy = uv.second;
    for (double dist = 0.0; dist <= max_search_dist; dist += resolution) {
      const double x = dist * dx;
      const double y = dist * dy;
      const int ix = static_cast<int>((x - origin_x) / resolution);
      const int iy = static_cast<int>((y - origin_y) / resolution);
      if (ix < 0 || iy < 0 || ix >= width || iy >= height) {
        break;  // Ray has left map bounds
      }
      if (map.data[ix + iy * width] == 100) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        obs_list_.poses.push_back(pose);
        break;  // First obstacle on this ray
      }
    }
  }
}

// Create marker message for visualization
visualization_msgs::msg::Marker DWAPlanner::create_marker_msg(
  const int id, const double scale, const std_msgs::msg::ColorRGBA color,
  const std::vector<State> & trajectory,
  const geometry_msgs::msg::PolygonStamped & footprint)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = robot_frame_;
  marker.header.stamp = this->get_clock()->now();
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1;
  marker.scale.x = scale;
  marker.color = color;
  marker.color.a = 0.8;
  marker.lifetime = rclcpp::Duration::from_seconds(10/hz_);

  geometry_msgs::msg::Point p;
  if (footprint.polygon.points.empty()) {
    for (const auto & point : trajectory) {
      p.x = point.x_;
      p.y = point.y_;
      marker.points.push_back(p);
    }
  } else {
    for (const auto & point : footprint.polygon.points) {
      p.x = point.x;
      p.y = point.y;
      marker.points.push_back(p);
    }
    p.x = footprint.polygon.points.front().x;
    p.y = footprint.polygon.points.front().y;
    marker.points.push_back(p);
  }

  return marker;
}

// Visualize trajectory
void DWAPlanner::visualize_trajectory(
  const std::vector<State> & trajectory,
  const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & pub)
{
  std_msgs::msg::ColorRGBA color;
  color.r = 1.0;
  visualization_msgs::msg::Marker v_trajectory = create_marker_msg(
    0, v_path_width_, color,
    trajectory);
  pub->publish(v_trajectory);
}

// Visualize trajectories
void DWAPlanner::visualize_trajectories(
  const std::vector<std::pair<std::vector<State>, bool>> & trajectories,
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & pub)
{
  visualization_msgs::msg::MarkerArray v_trajectories;
  for (size_t i = 0; i < trajectories.size(); ++i) {
    std_msgs::msg::ColorRGBA color;
    if (trajectories[i].second) {
      color.g = 1.0;
    } else {
      color.r = 0.5;
      color.b = 0.5;
    }
    visualization_msgs::msg::Marker v_trajectory = create_marker_msg(
      i, v_path_width_ * 0.4, color,
      trajectories[i].first);
    v_trajectories.markers.push_back(v_trajectory);
  }
  pub->publish(v_trajectories);
}

// Visualize footprints
void DWAPlanner::visualize_footprints(
  const std::vector<State> & trajectory,
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & pub)
{
  std_msgs::msg::ColorRGBA color;
  color.b = 1.0;
  visualization_msgs::msg::MarkerArray v_footprints;
  for (size_t i = 0; i < trajectory.size(); ++i) {
    const geometry_msgs::msg::PolygonStamped footprint = move_footprint(trajectory[i]);
    visualization_msgs::msg::Marker v_footprint = create_marker_msg(
      i, v_path_width_ * 0.2, color,
      trajectory, footprint);
    v_footprints.markers.push_back(v_footprint);
  }
  pub->publish(v_footprints);
}

// --- Time Measurement and Debugging Functions ---
void DWAPlanner::init_time_measurement()
{
  // Initialize logging settings
  // Note: ENABLE_DETAILED_LOGGING parameter is declared in load_params()
  this->get_parameter("ENABLE_DETAILED_LOGGING", enable_detailed_logging_);
  
  // Determine log directory in a robust way
  namespace fs = std::filesystem;
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);

  fs::path log_dir;
  const char * home_env = std::getenv("HOME");
  if (home_env) {
    log_dir = fs::path(home_env) / "final_ws";
  } else {
    log_dir = fs::current_path();
  }
  std::error_code ec;
  fs::create_directories(log_dir, ec);
  if (ec) {
    RCLCPP_WARN(this->get_logger(), "Could not create log directory: %s", ec.message().c_str());
  }

  std::stringstream ss;
  ss << "custom_dwa_planner_debug_" << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S") << ".log";
  log_file_path_ = (log_dir / ss.str()).string();
  
  // Initialize log file with header
  std::ofstream log_file(log_file_path_);
  if (log_file.is_open()) {
    log_file << "=== Custom DWA Planner Debug Log ===\n";
    log_file << "Started at: " << std::put_time(std::localtime(&time_t_now), "%Y-%m-%d %H:%M:%S") << "\n";
    log_file << "====================================\n\n";
    log_file.close();
    RCLCPP_INFO(this->get_logger(), "Debug log initialized: %s", log_file_path_.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to create debug log file: %s", log_file_path_.c_str());
  }
  
  timing_active_ = false;
  run_counter_ = 0;
  
  log_debug_info("Custom DWA Planner debugging and timing system initialized", "INFO");
}

void DWAPlanner::start_timing()
{
  if (!timing_active_) {
    goal_start_time_ = std::chrono::high_resolution_clock::now();
    timing_active_ = true;
    run_counter_++;
    log_debug_info("Started timing for run #" + std::to_string(run_counter_), "INFO");
  }
}

void DWAPlanner::stop_timing_and_log(const Eigen::Vector3d& goal)
{
  if (timing_active_) {
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - goal_start_time_);
    double time_taken_seconds = duration.count() / 1000.0;
    
    // Log to debug file
    std::ofstream log_file(log_file_path_, std::ios::app);
    if (log_file.is_open()) {
      auto now = std::chrono::system_clock::now();
      auto time_t_now = std::chrono::system_clock::to_time_t(now);
      
      log_file << "[" << std::put_time(std::localtime(&time_t_now), "%Y-%m-%d %H:%M:%S") << "] "
               << "GOAL_REACHED - Run #" << run_counter_
               << " - Goal: (" << std::fixed << std::setprecision(6) 
               << goal.x() << ", " << goal.y() << ", " << goal.z() << ")"
               << " - Time: " << time_taken_seconds << "s\n";
      log_file.close();
    }
    
    // Log to console
    std::stringstream msg;
    msg << "Goal reached! Run #" << run_counter_
        << " - Goal: (" << std::fixed << std::setprecision(3) 
        << goal.x() << ", " << goal.y() << ", " << goal.z() << ")"
        << " - Time: " << time_taken_seconds << "s";
    log_debug_info(msg.str(), "INFO");
    
    timing_active_ = false;
  }
}

void DWAPlanner::log_debug_info(const std::string& message, const std::string& level)
{
  if (!enable_detailed_logging_) {
    return;
  }
  
  // Always log to file when detailed logging is enabled
  std::ofstream log_file(log_file_path_, std::ios::app);
  if (log_file.is_open()) {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    
    log_file << "[" << std::put_time(std::localtime(&time_t_now), "%Y-%m-%d %H:%M:%S") << "] "
             << "[" << level << "] " << message << "\n";
    log_file.close();
  }
  
  // Also log to console
  if (level == "ERROR") {
    RCLCPP_ERROR(this->get_logger(), "[CUSTOM_DWA_DEBUG] %s", message.c_str());
  } else if (level == "WARN") {
    RCLCPP_WARN(this->get_logger(), "[CUSTOM_DWA_DEBUG] %s", message.c_str());
  } else if (level == "DEBUG") {
    RCLCPP_DEBUG(this->get_logger(), "[CUSTOM_DWA_DEBUG] %s", message.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "[CUSTOM_DWA_DEBUG] %s", message.c_str());
  }
}

void DWAPlanner::log_trajectory_info(const std::vector<State>& trajectory, const std::string& type)
{
  if (!enable_detailed_logging_ || trajectory.empty()) {
    return;
  }
  
  std::stringstream msg;
  msg << type << " trajectory - Start: (" 
      << std::fixed << std::setprecision(3)
      << trajectory.front().x_ << ", " << trajectory.front().y_ 
      << ", " << trajectory.front().yaw_ << ")"
      << " End: (" << trajectory.back().x_ << ", " << trajectory.back().y_
      << ", " << trajectory.back().yaw_ << ")"
      << " Velocity: " << trajectory.front().velocity_
      << " Yawrate: " << trajectory.front().yawrate_;
  
  log_debug_info(msg.str(), "DEBUG");
}
