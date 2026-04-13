/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#include <teb_local_planner/teb_local_planner_ros.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/algorithm/string.hpp>

#include <fstream>
#include <sstream>
#include <ctime>  // for std::time_t, std::strftime
#include <iomanip> // for std::put_time

// MBF return codes
#include <mbf_msgs/ExePathResult.h>

// pluginlib macros
#include <pluginlib/class_list_macros.h>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

#define SDT_DEAD_RECKONING_IMPLEMENTATION
#include <teb_local_planner/sdt_dead_reckoning.h>


// register this planner both as a BaseLocalPlanner and as a MBF's CostmapController plugin
PLUGINLIB_EXPORT_CLASS(teb_local_planner::TebLocalPlannerROS, nav_core::BaseLocalPlanner)
PLUGINLIB_EXPORT_CLASS(teb_local_planner::TebLocalPlannerROS, mbf_costmap_core::CostmapController)

namespace teb_local_planner
{
  

TebLocalPlannerROS::TebLocalPlannerROS() : costmap_ros_(NULL), tf_(NULL), costmap_model_(NULL),
                                           costmap_info_(nullptr), distance_field_(nullptr),
                                           px_out_(nullptr), py_out_(nullptr),
                                           costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons"),
                                           dynamic_recfg_(NULL), custom_via_points_active_(true), goal_reached_(false), no_infeasible_plans_(0),
                                           last_preferred_rotdir_(RotType::none), initialized_(false)
{
}


TebLocalPlannerROS::~TebLocalPlannerROS()
{
}

void TebLocalPlannerROS::reconfigureCB(TebLocalPlannerReconfigureConfig& config, uint32_t level)
{
  cfg_.reconfigure(config);
  ros::NodeHandle nh("~/" + name_);
  // lock the config mutex externally
  boost::mutex::scoped_lock lock(cfg_.configMutex());

  // create robot footprint/contour model for optimization
  cfg_.robot_model = getRobotFootprintFromParamServer(nh, cfg_);
  planner_->updateRobotModel(cfg_.robot_model);
}

void TebLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  // check if the plugin is already initialized
  if(!initialized_)
  {	
    name_ = name;
    // create Node Handle with name of plugin (as used in move_base for loading)
    ros::NodeHandle nh("~/" + name);
	        
    // get parameters of TebConfig via the nodehandle and override the default config
    cfg_.loadRosParamFromNodeHandle(nh);
    
    // reserve some memory for obstacles
    obstacles_.reserve(500);
        
    // create visualization instance	
    visualization_ = TebVisualizationPtr(new TebVisualization(nh, cfg_)); 
        
    // create robot footprint/contour model for optimization
    cfg_.robot_model = getRobotFootprintFromParamServer(nh, cfg_);
    
    // create the planner instance
    if (cfg_.hcp.enable_homotopy_class_planning)
    {
      planner_ = PlannerInterfacePtr(new HomotopyClassPlanner(cfg_, &obstacles_, visualization_, &via_points_));
      ROS_INFO("Parallel planning in distinctive topologies enabled.");
    }
    else
    {
      planner_ = PlannerInterfacePtr(new TebOptimalPlanner(cfg_, &obstacles_, visualization_, &via_points_));
      ROS_INFO("Parallel planning in distinctive topologies disabled.");
    }
    
    // init other variables
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.
    
    costmap_model_ = boost::make_shared<base_local_planner::CostmapModel>(*costmap_);

    global_frame_ = costmap_ros_->getGlobalFrameID();

    cfg_.map_frame = global_frame_; // TODO
    robot_base_frame_ = costmap_ros_->getBaseFrameID();
   
    //Initialize a costmap to polygon converter
    if (!cfg_.obstacles.costmap_converter_plugin.empty())
    {
      try
      {
        costmap_converter_ = costmap_converter_loader_.createInstance(cfg_.obstacles.costmap_converter_plugin);
        std::string converter_name = costmap_converter_loader_.getName(cfg_.obstacles.costmap_converter_plugin);
        // replace '::' by '/' to convert the c++ namespace to a NodeHandle namespace
        boost::replace_all(converter_name, "::", "/");
        costmap_converter_->setOdomTopic(cfg_.odom_topic);
        costmap_converter_->initialize(ros::NodeHandle(nh, "costmap_converter/" + converter_name));
        costmap_converter_->setCostmap2D(costmap_);
        
        costmap_converter_->startWorker(ros::Rate(cfg_.obstacles.costmap_converter_rate), costmap_, cfg_.obstacles.costmap_converter_spin_thread);
        ROS_INFO_STREAM("Costmap conversion plugin " << cfg_.obstacles.costmap_converter_plugin << " loaded.");        
      }
      catch(pluginlib::PluginlibException& ex)
      {
        ROS_WARN("The specified costmap converter plugin cannot be loaded. All occupied costmap cells are treaten as point obstacles. Error message: %s", ex.what());
        costmap_converter_.reset();
      }
    }
    else 
      ROS_INFO("No costmap conversion plugin specified. All occupied costmap cells are treaten as point obstacles.");
  
    
    // Get footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
    footprint_spec_ = costmap_ros_->getRobotFootprint();
    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);    
    
    // init the odom helper to receive the robot's velocity from odom messages
    odom_helper_.setOdomTopic(cfg_.odom_topic);

    // setup dynamic reconfigure
    dynamic_recfg_ = boost::make_shared< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> >(nh);
    dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(&TebLocalPlannerROS::reconfigureCB, this, _1, _2);
    dynamic_recfg_->setCallback(cb);
    
    // validate optimization footprint and costmap footprint
    validateFootprints(cfg_.robot_model->getInscribedRadius(), robot_inscribed_radius_, cfg_.obstacles.min_obstacle_dist);
        
    // setup callback for custom obstacles
    custom_obst_sub_ = nh.subscribe("obstacles", 1, &TebLocalPlannerROS::customObstacleCB, this);

    // setup callback for custom via-points
    via_points_sub_ = nh.subscribe("via_points", 1, &TebLocalPlannerROS::customViaPointsCB, this);
    
    // initialize failure detector
    ros::NodeHandle nh_move_base("~");
    double controller_frequency = 5;
    nh_move_base.param("controller_frequency", controller_frequency, controller_frequency);
    failure_detector_.setBufferLength(std::round(cfg_.recovery.oscillation_filter_duration*controller_frequency));
    
    // set initialized flag
    initialized_ = true;

    ROS_DEBUG("teb_local_planner plugin initialized.");
  }
  else
  {
    ROS_WARN("teb_local_planner has already been initialized, doing nothing.");
  }
}

bool TebLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  // check if plugin is initialized
  if(!initialized_)
  {
    ROS_ERROR("teb_local_planner has not been initialized, please call initialize() before using this planner");
    return false;
  }
  
  // store the global plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  goal_reached_ = false;
  
  return true;
}

bool TebLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{

  ROS_DEBUG("computeVelocityCommands");
  std::string dummy_message;
  geometry_msgs::PoseStamped dummy_pose;
  geometry_msgs::TwistStamped dummy_velocity, cmd_vel_stamped;
  uint32_t outcome = computeVelocityCommands(dummy_pose, dummy_velocity, cmd_vel_stamped, dummy_message);
  cmd_vel = cmd_vel_stamped.twist;

  return outcome == mbf_msgs::ExePathResult::SUCCESS;
}

uint32_t TebLocalPlannerROS::computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                                     const geometry_msgs::TwistStamped& velocity,
                                                     geometry_msgs::TwistStamped &cmd_vel,
                                                     std::string &message)
{

  ROS_DEBUG("computeVelocityCommands for real plan");
  // check if plugin initialized
  if(!initialized_)
  {
    ROS_ERROR("teb_local_planner has not been initialized, please call initialize() before using this planner");
    message = "teb_local_planner has not been initialized";
    return mbf_msgs::ExePathResult::NOT_INITIALIZED;
  }

  static uint32_t seq = 0;
  cmd_vel.header.seq = seq++;
  cmd_vel.header.stamp = ros::Time::now();
  cmd_vel.header.frame_id = robot_base_frame_;
  cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
  goal_reached_ = false;  
  
  // Get robot pose
  geometry_msgs::PoseStamped robot_pose;
  costmap_ros_->getRobotPose(robot_pose);
  robot_pose_ = PoseSE2(robot_pose.pose);
    
  // Get robot velocity
  geometry_msgs::PoseStamped robot_vel_tf;
  odom_helper_.getRobotVel(robot_vel_tf);
  robot_vel_.linear.x = robot_vel_tf.pose.position.x;
  robot_vel_.linear.y = robot_vel_tf.pose.position.y;
  robot_vel_.angular.z = tf2::getYaw(robot_vel_tf.pose.orientation);
  
  // 1. prune global plan to cut off parts of the past (spatially before the robot)
  pruneGlobalPlan(*tf_, robot_pose, global_plan_, cfg_.trajectory.global_plan_prune_distance);

  // 2. Transform global plan to the frame of interest (w.r.t. the local costmap)
  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  int goal_idx;
  geometry_msgs::TransformStamped tf_plan_to_global;
  if (!transformGlobalPlan(*tf_, global_plan_, robot_pose, *costmap_, global_frame_, cfg_.trajectory.max_global_plan_lookahead_dist, 
                           transformed_plan, &goal_idx, &tf_plan_to_global))
  {
    ROS_WARN("Could not transform the global plan to the frame of the controller");
    message = "Could not transform the global plan to the frame of the controller";
    return mbf_msgs::ExePathResult::INTERNAL_ERROR;
  }

  // 3. update via-points container
  if (!custom_via_points_active_)
    updateViaPointsContainer(transformed_plan, cfg_.trajectory.global_plan_viapoint_sep);
  else
    updateCustomViaPointsContainer(transformed_plan, *costmap_,  cfg_.trajectory.max_global_plan_lookahead_dist, cfg_.trajectory.global_plan_viapoint_sep);

  nav_msgs::Odometry base_odom;
  odom_helper_.getOdom(base_odom);

  // 4. check if global goal is reached
  geometry_msgs::PoseStamped global_goal;
  tf2::doTransform(global_plan_.back(), global_goal, tf_plan_to_global);
  double dx = global_goal.pose.position.x - robot_pose_.x();
  double dy = global_goal.pose.position.y - robot_pose_.y();
  double delta_orient = g2o::normalize_theta( tf2::getYaw(global_goal.pose.orientation) - robot_pose_.theta() );
  if(fabs(std::sqrt(dx*dx+dy*dy)) < cfg_.goal_tolerance.xy_goal_tolerance)
  {
    goal_reached_ = true;
    return mbf_msgs::ExePathResult::SUCCESS;
  }

  // check if we should enter any backup mode and apply settings
  configureBackupModes(transformed_plan, goal_idx);
  
    
  // Return false if the transformed global plan is empty
  if (transformed_plan.empty())
  {
    ROS_WARN("Transformed plan is empty. Cannot determisne a local plan.");
    message = "Transformed plan is empty";
    return mbf_msgs::ExePathResult::INVALID_PATH;
  }
              
  // Get current goal point (last point of the transformed plan)
  robot_goal_.x() = transformed_plan.back().pose.position.x;
  robot_goal_.y() = transformed_plan.back().pose.position.y;
  // Overwrite goal orientation if needed
  if (cfg_.trajectory.global_plan_overwrite_orientation)
  {
    robot_goal_.theta() = estimateLocalGoalOrientation(global_plan_, transformed_plan.back(), goal_idx, tf_plan_to_global);
    // overwrite/update goal orientation of the transformed plan with the actual goal (enable using the plan as initialization)
    tf2::Quaternion q;
    q.setRPY(0, 0, robot_goal_.theta());
    tf2::convert(q, transformed_plan.back().pose.orientation);
  }  
  else
  {
    robot_goal_.theta() = tf2::getYaw(transformed_plan.back().pose.orientation);
  }

  // overwrite/update start of the transformed plan with the actual robot position (allows using the plan as initial trajectory)
  if (transformed_plan.size()==1) // plan only contains the goal
  {
    transformed_plan.insert(transformed_plan.begin(), geometry_msgs::PoseStamped()); // insert start (not yet initialized)
  }
  transformed_plan.front() = robot_pose; // update start
    
  // clear currently existing obstacles
  obstacles_.clear();
  
  // Update obstacle container with costmap information or polygons provided by a costmap_converter plugin
  if (costmap_converter_)
  {
    ROS_DEBUG("updateObstacleContainerWithCostmapConverter");
    updateObstacleContainerWithCostmapConverter();
  }
  else
  {
    ROS_DEBUG("updateObstacleContainerWithCostmap");
    updateObstacleContainerWithCostmap();
  }
  // also consider custom obstacles (must be called after other updates, since the container is not cleared)
  updateObstacleContainerWithCustomObstacles();
  
    
  // Do not allow config changes during the following optimization step
  boost::mutex::scoped_lock cfg_lock(cfg_.configMutex());

  int look_ahead_idx = cfg_.trajectory.feasibility_check_no_poses;
  double inscribed_radius = robot_inscribed_radius_;

  // Set via-point weights if custom mode is active
  if (custom_via_points_active_ && !via_points_weights_.empty())
  {
    planner_->setViaPointsWeights(&via_points_weights_);
  }

  bool success = planner_->plan(transformed_plan, &robot_vel_, cfg_.goal_tolerance.free_goal_vel);


  ROS_DEBUG("Plan result: %s", success ? "successful" : "failed");

  if (!success)
  {
    planner_->clearPlanner(); // force reinitialization for next time
    ROS_WARN("teb_local_planner was not able to obtain a local plan for the current setting.");
    
    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = ros::Time::now();
    last_cmd_ = cmd_vel.twist;
    message = "teb_local_planner was not able to obtain a local plan";
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }

  ROS_DEBUG("teb_local_planner was able to obtain a local plan for the current setting.");  

  // Check for divergence
  ROS_DEBUG("Check for divergence");

  if (planner_->hasDiverged())
  {
    cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;

    // Reset everything to start again with the initialization of new trajectories.
    planner_->clearPlanner();
    ROS_WARN_THROTTLE(1.0, "TebLocalPlannerROS: the trajectory has diverged. Resetting planner...");

    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = ros::Time::now();
    last_cmd_ = cmd_vel.twist;
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }

  ROS_DEBUG("the trajectory has not diverged.");
         
  // Check feasibility (but within the first few states only)

  if(cfg_.robot.is_footprint_dynamic)
  {
    // Update footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
    footprint_spec_ = costmap_ros_->getRobotFootprint();
    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);
  }
  ROS_DEBUG("Check for feasibility");

  bool feasible = planner_->isTrajectoryFeasible(costmap_model_.get(), footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius, cfg_.trajectory.feasibility_check_no_poses, cfg_.trajectory.feasibility_check_lookahead_distance);
  if (!feasible)
  {
    cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;

    // now we reset everything to start again with the initialization of new trajectories.
    planner_->clearPlanner();
    // ROS_WARN("TebLocalPlannerROS: trajectory is not feasible. Resetting planner...");
    
    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = ros::Time::now();
    last_cmd_ = cmd_vel.twist;
    message = "teb_local_planner trajectory is not feasible";
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }

  // Get the velocity command for this sampling interval
  ROS_DEBUG("Get the velocity command");
  if (!planner_->getVelocityCommand(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, cfg_.trajectory.control_look_ahead_poses))

  {
    planner_->clearPlanner();
    ROS_WARN("TebLocalPlannerROS: velocity command invalid. Resetting planner...");
    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = ros::Time::now();
    last_cmd_ = cmd_vel.twist;
    message = "teb_local_planner velocity command invalid";
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }
  
  // Saturate velocity, if the optimization results violates the constraints (could be possible due to soft constraints).
  saturateVelocity(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z,
                   cfg_.robot.max_vel_x, cfg_.robot.max_vel_y, cfg_.robot.max_vel_trans, cfg_.robot.max_vel_theta, 
                   cfg_.robot.max_vel_x_backwards);

  // convert rot-vel to steering angle if desired (carlike robot).
  // The min_turning_radius is allowed to be slighly smaller since it is a soft-constraint
  // and opposed to the other constraints not affected by penalty_epsilon. The user might add a safety margin to the parameter itself.
  if (cfg_.robot.cmd_angle_instead_rotvel)
  {
    cmd_vel.twist.angular.z = convertTransRotVelToSteeringAngle(cmd_vel.twist.linear.x, cmd_vel.twist.angular.z,
                                                                cfg_.robot.wheelbase, 0.95*cfg_.robot.min_turning_radius);
    if (!std::isfinite(cmd_vel.twist.angular.z))
    {
      cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
      last_cmd_ = cmd_vel.twist;
      planner_->clearPlanner();
      ROS_WARN("TebLocalPlannerROS: Resulting steering angle is not finite. Resetting planner...");
      ++no_infeasible_plans_; // increase number of infeasible solutions in a row
      time_last_infeasible_plan_ = ros::Time::now();
      message = "teb_local_planner steering angle is not finite";
      return mbf_msgs::ExePathResult::NO_VALID_CMD;
    }
  }
  
  // a feasible solution should be found, reset counter
  no_infeasible_plans_ = 0;
  
  // store last command (for recovery analysis etc.)
  last_cmd_ = cmd_vel.twist;
  
  // Now visualize everything    
  planner_->visualize();
  visualization_->publishObstacles(obstacles_, costmap_->getResolution());
  visualization_->publishCustomViaPoints(via_points_);
  visualization_->publishGlobalPlan(global_plan_);

  return mbf_msgs::ExePathResult::SUCCESS;
}


bool TebLocalPlannerROS::isGoalReached()
{
  if (goal_reached_)
  {
    ROS_INFO("GOAL Reached!");
    planner_->clearPlanner();
    return true;
  }
  return false;
}

void TebLocalPlannerROS::updateDistanceField(const costmap_2d::Costmap2D& costmap)
{
    unsigned int map_width = costmap.getSizeInCellsX();
    unsigned int map_height = costmap.getSizeInCellsY();
    double resolution = costmap.getResolution();
    double origin_x = costmap.getOriginX();
    double origin_y = costmap.getOriginY();
    const unsigned char* costmap_data = costmap.getCharMap();

    costmap_info_ = std::make_shared<CostmapInfo>();
    costmap_info_->map_width = map_width;
    costmap_info_->map_height = map_height;
    costmap_info_->resolution = resolution;
    costmap_info_->origin_x = origin_x;
    costmap_info_->origin_y = origin_y;

    distance_field_ = std::make_shared<std::vector<float>>(map_width * map_height, std::numeric_limits<float>::infinity());
    px_out_ = std::make_shared<std::vector<int>>(map_width * map_height, -1);
    py_out_ = std::make_shared<std::vector<int>>(map_width * map_height, -1);

    sdt_dead_reckoning(map_width, map_height, 253, costmap_data,
                       distance_field_->data(), px_out_->data(), py_out_->data());
}

std::vector<std::pair<geometry_msgs::Point, double>> TebLocalPlannerROS::detectNarrowPassages(
    const std::vector<geometry_msgs::PoseStamped>& transformed_plan, 
    const costmap_2d::Costmap2D& costmap,
    double max_lookahead_dist, double min_separation)
{
  std::vector<std::pair<geometry_msgs::Point, double>> medial_axis_point;

  // 1. Trim transformed_plan to max_lookahead_dist
  std::vector<geometry_msgs::PoseStamped> trimmed_plan;
  double accumulated_distance = 0.0;
  
  if (!transformed_plan.empty()) {
    trimmed_plan.push_back(transformed_plan[0]);
    
    for (size_t i = 1; i < transformed_plan.size(); ++i) {
      double dx = transformed_plan[i].pose.position.x - transformed_plan[i-1].pose.position.x;
      double dy = transformed_plan[i].pose.position.y - transformed_plan[i-1].pose.position.y;
      accumulated_distance += std::sqrt(dx * dx + dy * dy);
      
      if (accumulated_distance <= max_lookahead_dist) {
        trimmed_plan.push_back(transformed_plan[i]);
      } else {
        break;
      }
    }
  }

  updateDistanceField(costmap);

  // 2. Generate samples from trimmed plan
  std::vector<geometry_msgs::Point> samples = generateSamples(trimmed_plan, min_separation);
  // visualization_->visualizeSamples(samples);

  double goal_threshold = 0.3;

  geometry_msgs::Point robot_position;
  robot_position.x = robot_pose_.x();
  robot_position.y = robot_pose_.y();
  double yaw = robot_pose_.theta();

  double heading_x = cos(yaw);
  double heading_y = sin(yaw);
  // 3. Get medial axis points for each sample
  for (const auto& point : samples)
  {
    auto medial_result = findMedialBallRadius(point);
    double medial_radius = medial_result.second;
    geometry_msgs::Point final_center = medial_result.first;

    double dx = final_center.x - robot_position.x;
    double dy = final_center.y - robot_position.y;

    double dot = dx * heading_x + dy * heading_y;

    if (dot < 0)
      continue;

    double distance_to_goal = euclideanDistance(final_center, trimmed_plan.back().pose.position);

    if (medial_radius >= 0.05 && distance_to_goal > goal_threshold)
    {
      medial_axis_point.emplace_back(final_center, medial_radius);
      visualization_->visualizeMedialBall(final_center, medial_radius);
    }
  }

  return medial_axis_point;
}

double TebLocalPlannerROS::euclideanDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
{
  return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

std::vector<geometry_msgs::Point> TebLocalPlannerROS::generateSamples(
  const std::vector<geometry_msgs::PoseStamped>& transformed_plan, double min_separation)
{

  std::vector<geometry_msgs::Point> samples;

  if (min_separation <= 0)
    return samples;

  std::size_t prev_idx = 0;
  for (std::size_t i = 1; i < transformed_plan.size(); ++i)
  {
    if (teb_local_planner::distance_points2d(transformed_plan[prev_idx].pose.position, transformed_plan[i].pose.position) < min_separation)
      continue;
    
    geometry_msgs::Point sample;
    sample.x = transformed_plan[i].pose.position.x;
    sample.y = transformed_plan[i].pose.position.y;
    sample.z = 0.0;
    samples.push_back(sample);
    prev_idx = i;
}

return samples;
}

// ========== Gradient-based Medial Axis Algorithms ==========

std::pair<geometry_msgs::Point, double> TebLocalPlannerROS::findMedialBallRadius(
  const geometry_msgs::Point& point)
{
    auto [mp, r] = performMedialAxisClimb(point, costmap_info_->resolution, costmap_info_->origin_x, costmap_info_->origin_y);

    geometry_msgs::Point mp_msg;
    mp_msg.x = mp.x();
    mp_msg.y = mp.y();
    mp_msg.z = 0.0;

    return std::make_pair(mp_msg, r);
}

double TebLocalPlannerROS::distanceFieldAt(double wx, double wy) const
{
    const auto& info = *costmap_info_;
    const auto& df   = *distance_field_;
    int w = info.map_width;
    int h = info.map_height;
    double res = info.resolution;
    double ox = info.origin_x;
    double oy = info.origin_y;

    int grid_x = static_cast<int>((wx - ox) / res);
    int grid_y = static_cast<int>((wy - oy) / res);
    int idx = grid_x + grid_y * w;

    return df[idx] * res;
}

double TebLocalPlannerROS::distanceFieldAtGrid(double wx, double wy) const
{
    const auto& info = *costmap_info_;
    int w = info.map_width;
    int h = info.map_height;
    double res = info.resolution;
    double ox = info.origin_x;
    double oy = info.origin_y;

    int grid_x = static_cast<int>((wx - ox) / res);
    int grid_y = static_cast<int>((wy - oy) / res);
    int idx = grid_x + grid_y * w;

    return idx;
}

Eigen::Vector2d TebLocalPlannerROS::estimateNormal(const Eigen::Vector2d& pt)
{
    const auto& info = *costmap_info_;
    const auto& df = *distance_field_;

    int gx = int((pt.x() - info.origin_x) / info.resolution);
    int gy = int((pt.y() - info.origin_y) / info.resolution);

    if (gx <= 0 || gy <= 0 || gx >= info.map_width - 1 || gy >= info.map_height - 1)
        return Eigen::Vector2d::Zero();  // out of bounds

    // central difference
    double dx = (df[(gx+1) + gy * info.map_width] - df[(gx-1) + gy * info.map_width]) / (2.0 * info.resolution);
    double dy = (df[gx + (gy+1) * info.map_width] - df[gx + (gy-1) * info.map_width]) / (2.0 * info.resolution);

    Eigen::Vector2d grad(dx, dy);
    if (grad.norm() < 1e-6) return Eigen::Vector2d::Zero();  // flat / error

    return grad.normalized();  // boundary normal direction (pointing outward)
}

// Compute closest obstacle boundary point
Eigen::Vector2d TebLocalPlannerROS::getBoundaryPoint(const Eigen::Vector2d& pt)
{
    const auto& info = *costmap_info_;
    int gx = int((pt.x() - info.origin_x) / info.resolution);
    int gy = int((pt.y() - info.origin_y) / info.resolution);
    int idx = gy * info.map_width + gx;

    int bx = (*px_out_)[idx] - 1;
    int by = (*py_out_)[idx] - 1;
    return Eigen::Vector2d(info.origin_x + (bx + 0.5) * info.resolution, info.origin_y + (by + 0.5) * info.resolution);
}


Eigen::Vector2d TebLocalPlannerROS::computePushDirection(const Eigen::Vector2d& from, const Eigen::Vector2d& to, double dist)
{
    Eigen::Vector2d vec = to - from;
    if (vec.norm() < 1e-3) return Eigen::Vector2d::Zero();
    return vec.normalized();
}

// Bresenham line algorithm: generate all cells from (x0,y0) to (x1,y1)
std::vector<Eigen::Vector2i> TebLocalPlannerROS::bresenhamLineWorld(const Eigen::Vector2d& from, const Eigen::Vector2d& to)
{
    const auto& info = *costmap_info_;
    int x0 = int((from.x() - info.origin_x) / info.resolution);
    int y0 = int((from.y() - info.origin_y) / info.resolution);
    int x1 = int((to.x() - info.origin_x) / info.resolution);
    int y1 = int((to.y() - info.origin_y) / info.resolution);

    std::vector<Eigen::Vector2i> cells;
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, x = x0, y = y0;

    while (true)
    {
        cells.emplace_back(x, y);
        if (x == x1 && y == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
    }
    return cells;
}

std::pair<int, double> TebLocalPlannerROS::climbLocalMax(const std::vector<Eigen::Vector2i>& line, double max_dist, double max_iterations)
{
    const auto& info = *costmap_info_;
    const auto& df = *distance_field_;
     int cur = 0;
     double cur_val = df[line[cur].x() + line[cur].y() * info.map_width]*info.resolution;
     for (int it = 0; it < max_iterations; ++it) {
       int nxt = cur + 1;
       if (nxt >= (int)line.size()) break;
       double nxt_val = df[line[nxt].x() + line[nxt].y() * info.map_width]*info.resolution;
       if (nxt_val >= max_dist) {
         cur = nxt; cur_val = nxt_val;
         break;
       }
       if (nxt_val > cur_val) {
         cur = nxt; cur_val = nxt_val;
       } else {
         break;
       }
     }
    return {cur, cur_val};
}

// Perform medial axis climb with iterative refinement
std::tuple<Eigen::Vector2d, double> TebLocalPlannerROS::performMedialAxisClimb(
  const geometry_msgs::Point& start_point,
  double resolution, double origin_x, double origin_y)
{
    const auto& info = *costmap_info_;
    const auto& df = *distance_field_;
    unsigned int map_width = info.map_width;
    unsigned int map_height = info.map_height;

    const double max_distance = 0.4;   // Maximum distance to climb
    const double min_distance = 0.2;   // If found value < this, do refinement
    const int max_iterations = 100;
    const int line_climb_iters = 50;

    Eigen::Vector2d current_pt(start_point.x, start_point.y);
    double current_val = distanceFieldAt(current_pt.x(), current_pt.y());

    Eigen::Vector2d boundary = getBoundaryPoint(current_pt);
    double boundary_val = distanceFieldAt(boundary.x(), boundary.y());

    Eigen::Vector2d n;
    if (current_val == 0.0)
    {
        // case: Boundary value = 0
        n = estimateNormal(current_pt);
        if (n.norm() == 0.0)
            return {current_pt, current_val};
    }
    else if (current_val < 0.40)
    {
        // case: Boundary value != 0
        n = computePushDirection(current_pt, boundary, max_distance);
        if (current_val > 0.0) n = -n;
    }
    else
    {
      return {current_pt, current_val};
    }

    Eigen::Vector2d pt_fwd = boundary + n * max_distance;
    auto line = bresenhamLineWorld(boundary, pt_fwd);
    auto result = climbLocalMax(line, max_distance);

    Eigen::Vector2i bc = line[result.first];
    double best_val = result.second;
    Eigen::Vector2d best_pt(info.origin_x + (bc.x() + 0.5) * info.resolution, info.origin_y + (bc.y() + 0.5) * info.resolution);
    double best_idx = distanceFieldAtGrid(best_pt.x(), best_pt.y());

    if (best_val < min_distance)
    {
        Eigen::Vector2d g = estimateNormal(best_pt); 
        if (g.norm() > 1e-6)
        {
            Eigen::Vector2d pt_fwd2 = best_pt + g * max_distance;
            auto line2 = bresenhamLineWorld(best_pt, pt_fwd2);
            auto result2 = climbLocalMax(line2, max_distance);

            Eigen::Vector2i bc2 = line2[result2.first];
            Eigen::Vector2d best_pt2(info.origin_x + (bc2.x() + 0.5) * info.resolution,
                                     info.origin_y + (bc2.y() + 0.5) * info.resolution);
            double best_idx2 = distanceFieldAtGrid(best_pt2.x(), best_pt2.y());
            double best_val2 = result2.second;

            if (best_val2 > best_val)
                return {best_pt2, best_val2};
        }
    }

    return {best_pt, best_val};
}


void TebLocalPlannerROS::updateCustomViaPointsContainer(const std::vector<geometry_msgs::PoseStamped>& transformed_plan, const costmap_2d::Costmap2D& costmap, double max_lookahead_dist, double min_separation)
{
    // Clear the existing via-points and weights
    via_points_.clear();
    via_points_weights_.clear();
    ROS_DEBUG("clear via point");

    // Get medial axis points (includes radius information)
    auto medial_axis_points = detectNarrowPassages(transformed_plan, costmap, max_lookahead_dist, min_separation);
    ROS_DEBUG("get medial axis points");

    if (medial_axis_points.empty())
        return;

    // (closest_plan_index, (point, radius))
    std::vector<std::pair<int, std::pair<Eigen::Vector2d, double>>> indexed_medial_points;

    // Map each medial axis point to its closest transformed_plan index
    for (const auto& medial_point_pair : medial_axis_points)
    {
        const auto& medial_point = medial_point_pair.first;
        double medial_radius = medial_point_pair.second;
        Eigen::Vector2d medial_point_vec(medial_point.x, medial_point.y);

        int closest_plan_index = -1;
        double min_distance = std::numeric_limits<double>::max();

        for (size_t i = 0; i < transformed_plan.size(); ++i)
        {
            const auto& plan_pose = transformed_plan[i];
            Eigen::Vector2d plan_point_vec(plan_pose.pose.position.x, plan_pose.pose.position.y);
            double distance = (medial_point_vec - plan_point_vec).squaredNorm();

            if (distance < min_distance)
            {
                min_distance = distance;
                closest_plan_index = static_cast<int>(i);
            }
        }

        if (closest_plan_index >= 0)
        {
            indexed_medial_points.emplace_back(
                closest_plan_index,
                std::make_pair(medial_point_vec, medial_radius));
        }
    }

    if (indexed_medial_points.empty())
        return;

    // Sort by closest plan index
    std::sort(indexed_medial_points.begin(), indexed_medial_points.end(),
              [](const std::pair<int, std::pair<Eigen::Vector2d, double>>& a,
                 const std::pair<int, std::pair<Eigen::Vector2d, double>>& b)
              {
                  return a.first < b.first;
              });

    const int n = static_cast<int>(indexed_medial_points.size());

    // ------------------------------------------------------------
    // Compute raw weights and collect radii
    // ------------------------------------------------------------
    std::vector<double> raw_weights(n, 0.0);
    std::vector<double> radii(n, 0.0);
    for (int i = 0; i < n; ++i)
    {
      double radius = indexed_medial_points[i].second.second;
      radii[i] = radius;
      raw_weights[i] = calculateViaPointWeight(radius);
    }

    // Pass radii to postprocessing for radius-based NMS
    applyViaPointPostProcessing(indexed_medial_points, raw_weights, radii);
  }

  void TebLocalPlannerROS::applyViaPointPostProcessing(
    const std::vector<std::pair<int, std::pair<Eigen::Vector2d, double>>>& indexed_medial_points,
    const std::vector<double>& raw_weights,
    const std::vector<double>& radii)
  {
    if (cfg_.via_point_postprocess_mode == "no_nms")
    {
      applyViaPointNoNMS(indexed_medial_points, raw_weights);
      return;
    }

    applyViaPointNMS(indexed_medial_points, raw_weights, radii);
  }

    void TebLocalPlannerROS::applyViaPointNMS(
    const std::vector<std::pair<int, std::pair<Eigen::Vector2d, double>>>& indexed_medial_points,
    const std::vector<double>& raw_weights,
    const std::vector<double>& radii)
    {
    const int n = static_cast<int>(indexed_medial_points.size());

    // ------------------------------------------------------------
    // Step 1. Find local maxima in radius
    // ------------------------------------------------------------
    std::vector<int> peak_candidates;
    peak_candidates.reserve(n);
    for (int i = 0; i < n; ++i)
    {
      double curr = radii[i];
      double left  = (i > 0)     ? radii[i - 1] : -std::numeric_limits<double>::infinity();
      double right = (i < n - 1) ? radii[i + 1] : -std::numeric_limits<double>::infinity();
      if (curr >= left && curr >= right)
        peak_candidates.push_back(i);
    }

    if (peak_candidates.empty())
    {
      // fallback: keep all with raw weight
      for (int i = 0; i < n; ++i)
      {
        via_points_.emplace_back(indexed_medial_points[i].second.first);
        via_points_weights_.push_back(raw_weights[i]);
      }
      return;
    }

    // ------------------------------------------------------------
    // Step 2. Greedy Non-Maximum Suppression on radius peaks
    // Keep stronger peaks, suppress weaker nearby peaks
    // ------------------------------------------------------------
    std::sort(peak_candidates.begin(), peak_candidates.end(),
          [&](int a, int b)
          {
            return radii[a] > radii[b];
          });

    // NMS parameters
    const int peak_suppress_window = 10;   // nearby peak suppression in plan index
    const int weight_decay_window  = 10;   // weight reduction window around selected peak
    const double min_decay_scale   = 0.5; // at peak center, neighbors can be reduced to 50%
    const double min_keep_weight   = 0.5; // discard overly weak points after suppression

    std::vector<bool> suppressed_peak(n, false);
    std::vector<int> selected_peaks;
    selected_peaks.reserve(peak_candidates.size());

    for (int peak_idx : peak_candidates)
    {
      if (suppressed_peak[peak_idx])
        continue;

      selected_peaks.push_back(peak_idx);

      int peak_plan_index = indexed_medial_points[peak_idx].first;

      // suppress weaker nearby peaks
      for (int other_peak_idx : peak_candidates)
      {
        if (other_peak_idx == peak_idx)
          continue;

        int other_plan_index = indexed_medial_points[other_peak_idx].first;
        if (std::abs(other_plan_index - peak_plan_index) <= peak_suppress_window)
          suppressed_peak[other_peak_idx] = true;
      }
    }

    // ------------------------------------------------------------
    // Step 3. Decay weights around selected radius peaks
    // final weight at each point = max contribution from selected peaks
    // ------------------------------------------------------------
    std::vector<double> final_weights = raw_weights;

    for (int peak_idx : selected_peaks)
    {
      int peak_plan_index = indexed_medial_points[peak_idx].first;
      double peak_weight = raw_weights[peak_idx];

      for (int i = 0; i < n; ++i)
      {
        int plan_idx = indexed_medial_points[i].first;
        int gap = std::abs(plan_idx - peak_plan_index);

        if (gap == 0)
        {
          final_weights[i] = peak_weight;
        }
        else if (gap <= weight_decay_window)
        {
          double ratio = static_cast<double>(gap) /
                        static_cast<double>(weight_decay_window);
          double scale = min_decay_scale + (1.0 - min_decay_scale) * ratio;

          double decayed_weight = raw_weights[i] * scale;

          final_weights[i] = std::min(final_weights[i], decayed_weight);
        }
      }
    }

    // If no selected peaks somehow, fallback to raw weights
    if (selected_peaks.empty())
      final_weights = raw_weights;

    // ------------------------------------------------------------
    // Step 5. Export via-points
    // ------------------------------------------------------------
    for (int i = 0; i < n; ++i)
    {
      const Eigen::Vector2d& point = indexed_medial_points[i].second.first;
      double radius = indexed_medial_points[i].second.second;
      double weight = final_weights[i];

      if (weight < min_keep_weight)
        continue;

      via_points_.emplace_back(point);
      via_points_weights_.push_back(weight);

      //ROS_INFO("Via-point added (NMS): index=%d, pos=(%.2f, %.2f), radius=%.2f, weight=%.2f", i, point.x(), point.y(), radius, weight);
    }
    }

  void TebLocalPlannerROS::applyViaPointNoNMS(
      const std::vector<std::pair<int, std::pair<Eigen::Vector2d, double>>>& indexed_medial_points,
      const std::vector<double>& raw_weights)
  {
    const double min_keep_weight = 0.5;
    const int n = static_cast<int>(indexed_medial_points.size());

    for (int i = 0; i < n; ++i)
    {
      const Eigen::Vector2d& point = indexed_medial_points[i].second.first;
      double radius = indexed_medial_points[i].second.second;
      double weight = raw_weights[i];

      if (weight < min_keep_weight)
        continue;

      via_points_.emplace_back(point);
      via_points_weights_.push_back(weight);

      ROS_INFO("Via-point added (no NMS): pos=(%.2f, %.2f), radius=%.2f, weight=%.2f",point.x(), point.y(), radius, weight);
    }
  }

// Helper function: dispatch the selected via-point weighting mode.
double TebLocalPlannerROS::calculateViaPointWeight(double medial_radius) const
{
  if (cfg_.via_point_weight_mode == "step")
    return calculateStepViaPointWeight(medial_radius);

  return calculateLinearViaPointWeight(medial_radius);
}

double TebLocalPlannerROS::calculateLinearViaPointWeight(double medial_radius) const
{
  double min_radius = 0.15;
  double max_radius = 1.0;
  double high_weight = 10.0;
  double low_weight = 0.0;

  if (medial_radius <= min_radius)
    return high_weight;

  if (medial_radius >= max_radius)
    return 0.0;

  double ratio = (medial_radius - min_radius) / (max_radius - min_radius);
  return high_weight - ratio * (high_weight - low_weight);
}

double TebLocalPlannerROS::calculateStepViaPointWeight(double medial_radius) const
{
  double max_radius = 1.0;
  double high_weight = 10.0;

  if (medial_radius >= max_radius)
    return 0.0;

  return high_weight;
}

void TebLocalPlannerROS::updateObstacleContainerWithCostmap()
{  
  // Add costmap obstacles if desired
  if (cfg_.obstacles.include_costmap_obstacles)
  {
    Eigen::Vector2d robot_orient = robot_pose_.orientationUnitVec();
    
    for (unsigned int i=0; i<costmap_->getSizeInCellsX()-1; ++i)
    {
      for (unsigned int j=0; j<costmap_->getSizeInCellsY()-1; ++j)
      {
        if (costmap_->getCost(i,j) == costmap_2d::LETHAL_OBSTACLE)
        {
          Eigen::Vector2d obs;
          costmap_->mapToWorld(i,j,obs.coeffRef(0), obs.coeffRef(1));
            
          // check if obstacle is interesting (e.g. not far behind the robot)
          Eigen::Vector2d obs_dir = obs-robot_pose_.position();
          if ( obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > cfg_.obstacles.costmap_obstacles_behind_robot_dist  )
            continue;
            
          obstacles_.push_back(ObstaclePtr(new PointObstacle(obs)));
        }
      }
    }
  }
}

void TebLocalPlannerROS::updateObstacleContainerWithCostmapConverter()
{
  if (!costmap_converter_)
    return;
    
  //Get obstacles from costmap converter
  costmap_converter::ObstacleArrayConstPtr obstacles = costmap_converter_->getObstacles();
  if (!obstacles)
    return;

  for (std::size_t i=0; i<obstacles->obstacles.size(); ++i)
  {
    const costmap_converter::ObstacleMsg* obstacle = &obstacles->obstacles.at(i);
    const geometry_msgs::Polygon* polygon = &obstacle->polygon;

    if (polygon->points.size()==1 && obstacle->radius > 0) // Circle
    {
      obstacles_.push_back(ObstaclePtr(new CircularObstacle(polygon->points[0].x, polygon->points[0].y, obstacle->radius)));
    }
    else if (polygon->points.size()==1) // Point
    {
      obstacles_.push_back(ObstaclePtr(new PointObstacle(polygon->points[0].x, polygon->points[0].y)));
    }
    else if (polygon->points.size()==2) // Line
    {
      obstacles_.push_back(ObstaclePtr(new LineObstacle(polygon->points[0].x, polygon->points[0].y,
                                                        polygon->points[1].x, polygon->points[1].y )));
    }
    else if (polygon->points.size()>2) // Real polygon
    {
        PolygonObstacle* polyobst = new PolygonObstacle;
        for (std::size_t j=0; j<polygon->points.size(); ++j)
        {
            polyobst->pushBackVertex(polygon->points[j].x, polygon->points[j].y);
        }
        polyobst->finalizePolygon();
        obstacles_.push_back(ObstaclePtr(polyobst));
    }

    // Set velocity, if obstacle is moving
    if(!obstacles_.empty())
      obstacles_.back()->setCentroidVelocity(obstacles->obstacles[i].velocities, obstacles->obstacles[i].orientation);
  }
}


void TebLocalPlannerROS::updateObstacleContainerWithCustomObstacles()
{
  // Add custom obstacles obtained via message
  boost::mutex::scoped_lock l(custom_obst_mutex_);
  ROS_DEBUG("updateObstacleContainerWithCustomObstacles");

  if (!custom_obstacle_msg_.obstacles.empty())
  {
    ROS_DEBUG("Start to updateObstacleContainerWithCustomObstacles");
    // We only use the global header to specify the obstacle coordinate system instead of individual ones
    Eigen::Affine3d obstacle_to_map_eig;
    try 
    {
      geometry_msgs::TransformStamped obstacle_to_map =  tf_->lookupTransform(global_frame_, ros::Time(0),
                                                                              custom_obstacle_msg_.header.frame_id, ros::Time(0),
                                                                              custom_obstacle_msg_.header.frame_id, ros::Duration(cfg_.robot.transform_tolerance));
      obstacle_to_map_eig = tf2::transformToEigen(obstacle_to_map);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      obstacle_to_map_eig.setIdentity();
    }
    
    for (size_t i=0; i<custom_obstacle_msg_.obstacles.size(); ++i)
    {
      if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1 && custom_obstacle_msg_.obstacles.at(i).radius > 0 ) // circle
      {
        Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
        obstacles_.push_back(ObstaclePtr(new CircularObstacle( (obstacle_to_map_eig * pos).head(2), custom_obstacle_msg_.obstacles.at(i).radius)));
      }
      else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1 ) // point
      {
        Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
        obstacles_.push_back(ObstaclePtr(new PointObstacle( (obstacle_to_map_eig * pos).head(2) )));
      }
      else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 2 ) // line
      {
        Eigen::Vector3d line_start( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                                    custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                                    custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
        Eigen::Vector3d line_end( custom_obstacle_msg_.obstacles.at(i).polygon.points.back().x,
                                  custom_obstacle_msg_.obstacles.at(i).polygon.points.back().y,
                                  custom_obstacle_msg_.obstacles.at(i).polygon.points.back().z );
        obstacles_.push_back(ObstaclePtr(new LineObstacle( (obstacle_to_map_eig * line_start).head(2),
                                                           (obstacle_to_map_eig * line_end).head(2) )));
      }
      else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.empty())
      {
        ROS_WARN("Invalid custom obstacle received. List of polygon vertices is empty. Skipping...");
        continue;
      }
      else // polygon
      {
        PolygonObstacle* polyobst = new PolygonObstacle;
        for (size_t j=0; j<custom_obstacle_msg_.obstacles.at(i).polygon.points.size(); ++j)
        {
          Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points[j].x,
                               custom_obstacle_msg_.obstacles.at(i).polygon.points[j].y,
                               custom_obstacle_msg_.obstacles.at(i).polygon.points[j].z );
          polyobst->pushBackVertex( (obstacle_to_map_eig * pos).head(2) );
        }
        polyobst->finalizePolygon();
        obstacles_.push_back(ObstaclePtr(polyobst));
      }

      // Set velocity, if obstacle is moving
      if(!obstacles_.empty())
        obstacles_.back()->setCentroidVelocity(custom_obstacle_msg_.obstacles[i].velocities, custom_obstacle_msg_.obstacles[i].orientation);
    }
  }
}

void TebLocalPlannerROS::updateViaPointsContainer(const std::vector<geometry_msgs::PoseStamped>& transformed_plan, double min_separation)
{
  via_points_.clear();
  
  if (min_separation<=0)
    return;
  
  std::size_t prev_idx = 0;
  for (std::size_t i=1; i < transformed_plan.size(); ++i) // skip first one, since we do not need any point before the first min_separation [m]
  {
    // check separation to the previous via-point inserted
    if (distance_points2d( transformed_plan[prev_idx].pose.position, transformed_plan[i].pose.position ) < min_separation)
      continue;
        
    // add via-point
    via_points_.push_back( Eigen::Vector2d( transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y ) );
    prev_idx = i;
  }
  
}

Eigen::Vector2d TebLocalPlannerROS::tfPoseToEigenVector2dTransRot(const tf::Pose& tf_vel)
{
  Eigen::Vector2d vel;
  vel.coeffRef(0) = std::sqrt( tf_vel.getOrigin().getX() * tf_vel.getOrigin().getX() + tf_vel.getOrigin().getY() * tf_vel.getOrigin().getY() );
  vel.coeffRef(1) = tf::getYaw(tf_vel.getRotation());
  return vel;
}
      
      
bool TebLocalPlannerROS::pruneGlobalPlan(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& global_pose, std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot)
{
  ROS_DEBUG("Prune Global Plan");
  if (global_plan.empty())
    return true;
  
  try
  {
    // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
    geometry_msgs::TransformStamped global_to_plan_transform = tf.lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, ros::Time(0));
    geometry_msgs::PoseStamped robot; 
    tf2::doTransform(global_pose, robot, global_to_plan_transform);
    
    double dist_thresh_sq = dist_behind_robot*dist_behind_robot;
    
    // iterate plan until a pose close the robot is found
    std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
    std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
    while (it != global_plan.end())
    {
      double dx = robot.pose.position.x - it->pose.position.x;
      double dy = robot.pose.position.y - it->pose.position.y;
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < dist_thresh_sq)
      {
         erase_end = it;
         break;
      }
      ++it;
    }
    if (erase_end == global_plan.end())
      return false;
    
    if (erase_end != global_plan.begin())
      global_plan.erase(global_plan.begin(), erase_end);
  }
  catch (const tf::TransformException& ex)
  {
    ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
    return false;
  }
  return true;
}      

bool TebLocalPlannerROS::transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                  const geometry_msgs::PoseStamped& global_pose, const costmap_2d::Costmap2D& costmap, const std::string& global_frame, double max_plan_length,
                  std::vector<geometry_msgs::PoseStamped>& transformed_plan, int* current_goal_idx, geometry_msgs::TransformStamped* tf_plan_to_global) const
{
  // this method is a slightly modified version of base_local_planner/goal_functions.h

  const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

  transformed_plan.clear();

  try 
  {
    if (global_plan.empty())
    {
      ROS_ERROR("Received plan with zero length");
      *current_goal_idx = 0;
      return false;
    }

    // get plan_to_global_transform from plan frame to global_frame
    geometry_msgs::TransformStamped plan_to_global_transform = tf.lookupTransform(global_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp,
                                                                                  plan_pose.header.frame_id, ros::Duration(cfg_.robot.transform_tolerance));

    //let's get the pose of the robot in the frame of the plan
    geometry_msgs::PoseStamped robot_pose;
    tf.transform(global_pose, robot_pose, plan_pose.header.frame_id);

    //we'll discard points on the plan that are outside the local costmap
    double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                     costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
    dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacle that are
                           // located on the border of the local costmap
    

    int i = 0;
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist = 1e10;
    
    //we need to loop to a point on the plan that is within a certain distance of the robot
    bool robot_reached = false;
    for(int j=0; j < (int)global_plan.size(); ++j)
    {
      ROS_DEBUG("global_plan size : %d", global_plan.size());

      double x_diff = robot_pose.pose.position.x - global_plan[j].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[j].pose.position.y;
      double new_sq_dist = x_diff * x_diff + y_diff * y_diff;

      if (robot_reached && new_sq_dist > sq_dist)
        break;

      if (new_sq_dist < sq_dist) // find closest distance
      {
        sq_dist = new_sq_dist;
        i = j;
        if (sq_dist < 0.05)      // 2.5 cm to the robot; take the immediate local minima; if it's not the global
        {
          robot_reached = true;  // minima, probably means that there's a loop in the path, and so we prefer this'
          ROS_DEBUG("find cloest pose robot_pose : (%d, %d), global_plan : (%d, %d)", robot_pose.pose.position.x, robot_pose.pose.position.y, global_plan[j].pose.position.x, global_plan[j].pose.position.y);
        }


      }
    }
    
    geometry_msgs::PoseStamped newer_pose;
    
    double plan_length = 0; // check cumulative Euclidean distance along the plan
    
    //now we'll transform until points are outside of our distance threshold
    while(i < (int)global_plan.size() && sq_dist <= sq_dist_threshold && (max_plan_length<=0 || plan_length <= max_plan_length))
    {
      const geometry_msgs::PoseStamped& pose = global_plan[i];
      tf2::doTransform(pose, newer_pose, plan_to_global_transform);

      transformed_plan.push_back(newer_pose);

      double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;
      
      // caclulate distance to previous pose
      if (i>0 && max_plan_length>0)
        plan_length += teb_local_planner::distance_points2d(global_plan[i-1].pose.position, global_plan[i].pose.position);

      ++i;
    }
        
    // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
    // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
    if (transformed_plan.empty())
    {
      tf2::doTransform(global_plan.back(), newer_pose, plan_to_global_transform);

      transformed_plan.push_back(newer_pose);
      
      // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx) *current_goal_idx = int(global_plan.size())-1;
    }
    else
    {
      // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx) *current_goal_idx = i-1; // subtract 1, since i was increased once before leaving the loop
    }
    
    // Return the transformation from the global plan to the global planning frame if desired
    if (tf_plan_to_global) *tf_plan_to_global = plan_to_global_transform;
  }
  catch(tf::LookupException& ex)
  {
    ROS_ERROR("No Transform available Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ConnectivityException& ex) 
  {
    ROS_ERROR("Connectivity Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ExtrapolationException& ex) 
  {
    ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    if (global_plan.size() > 0)
      ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

    return false;
  }
  
  ROS_DEBUG("Successfully transform global plan");
  return true;
}

double TebLocalPlannerROS::estimateLocalGoalOrientation(const std::vector<geometry_msgs::PoseStamped>& global_plan, const geometry_msgs::PoseStamped& local_goal,
              int current_goal_idx, const geometry_msgs::TransformStamped& tf_plan_to_global, int moving_average_length) const
{
  int n = (int)global_plan.size();
  
  // check if we are near the global goal already
  if (current_goal_idx > n-moving_average_length-2)
  {
    if (current_goal_idx >= n-1) // we've exactly reached the goal
    {
      return tf2::getYaw(local_goal.pose.orientation);
    }
    else
    {
      tf2::Quaternion global_orientation;
      tf2::convert(global_plan.back().pose.orientation, global_orientation);
      tf2::Quaternion rotation;
      tf2::convert(tf_plan_to_global.transform.rotation, rotation);
      // TODO(roesmann): avoid conversion to tf2::Quaternion
      return tf2::getYaw(rotation *  global_orientation);
    }     
  }
  
  // reduce number of poses taken into account if the desired number of poses is not available
  moving_average_length = std::min(moving_average_length, n-current_goal_idx-1 ); // maybe redundant, since we have checked the vicinity of the goal before
  
  std::vector<double> candidates;
  geometry_msgs::PoseStamped tf_pose_k = local_goal;
  geometry_msgs::PoseStamped tf_pose_kp1;
  
  int range_end = current_goal_idx + moving_average_length;
  for (int i = current_goal_idx; i < range_end; ++i)
  {
    // Transform pose of the global plan to the planning frame
    tf2::doTransform(global_plan.at(i+1), tf_pose_kp1, tf_plan_to_global);

    // calculate yaw angle  
    candidates.push_back( std::atan2(tf_pose_kp1.pose.position.y - tf_pose_k.pose.position.y,
        tf_pose_kp1.pose.position.x - tf_pose_k.pose.position.x ) );
    
    if (i<range_end-1) 
      tf_pose_k = tf_pose_kp1;
  }
  return teb_local_planner::average_angles(candidates);
}
      
      
void TebLocalPlannerROS::saturateVelocity(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y, double max_vel_trans, double max_vel_theta, 
              double max_vel_x_backwards) const
{
  double ratio_x = 1, ratio_omega = 1, ratio_y = 1;
  // Limit translational velocity for forward driving
  if (vx > max_vel_x)
    ratio_x = max_vel_x / vx;
  
  // limit strafing velocity
  if (vy > max_vel_y || vy < -max_vel_y)
    ratio_y = std::abs(max_vel_y / vy);
  
  // Limit angular velocity
  if (omega > max_vel_theta || omega < -max_vel_theta)
    ratio_omega = std::abs(max_vel_theta / omega);
  
  // Limit backwards velocity
  if (max_vel_x_backwards<=0)
  {
    ROS_WARN_ONCE("TebLocalPlannerROS(): Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
  }
  else if (vx < -max_vel_x_backwards)
    ratio_x = - max_vel_x_backwards / vx;

  if (cfg_.robot.use_proportional_saturation)
  {
    double ratio = std::min(std::min(ratio_x, ratio_y), ratio_omega);
    vx *= ratio;
    vy *= ratio;
    omega *= ratio;
  }
  else
  {
    vx *= ratio_x;
    vy *= ratio_y;
    omega *= ratio_omega;
  }

  double vel_linear = std::hypot(vx, vy);
  if (vel_linear > max_vel_trans)
  {
    double max_vel_trans_ratio = max_vel_trans / vel_linear;
    vx *= max_vel_trans_ratio;
    vy *= max_vel_trans_ratio;
  }
}
     
     
double TebLocalPlannerROS::convertTransRotVelToSteeringAngle(double v, double omega, double wheelbase, double min_turning_radius) const
{
  if (omega==0 || v==0)
    return 0;
    
  double radius = v/omega;
  
  if (fabs(radius) < min_turning_radius)
    radius = double(g2o::sign(radius)) * min_turning_radius; 

  return std::atan(wheelbase / radius);
}
     

void TebLocalPlannerROS::validateFootprints(double opt_inscribed_radius, double costmap_inscribed_radius, double min_obst_dist)
{
    ROS_WARN_COND(opt_inscribed_radius + min_obst_dist < costmap_inscribed_radius,
                  "The inscribed radius of the footprint specified for TEB optimization (%f) + min_obstacle_dist (%f) are smaller "
                  "than the inscribed radius of the robot's footprint in the costmap parameters (%f, including 'footprint_padding'). "
                  "Infeasible optimziation results might occur frequently!", opt_inscribed_radius, min_obst_dist, costmap_inscribed_radius);
}
   
   
   
void TebLocalPlannerROS::configureBackupModes(std::vector<geometry_msgs::PoseStamped>& transformed_plan,  int& goal_idx)
{
    ros::Time current_time = ros::Time::now();
    
    // reduced horizon backup mode
    if (cfg_.recovery.shrink_horizon_backup && 
        goal_idx < (int)transformed_plan.size()-1 && // we do not reduce if the goal is already selected (because the orientation might change -> can introduce oscillations)
       (no_infeasible_plans_>0 || (current_time - time_last_infeasible_plan_).toSec() < cfg_.recovery.shrink_horizon_min_duration )) // keep short horizon for at least a few seconds
    {
        ROS_INFO_COND(no_infeasible_plans_==1, "Activating reduced horizon backup mode for at least %.2f sec (infeasible trajectory detected).", cfg_.recovery.shrink_horizon_min_duration);


        // Shorten horizon if requested
        // reduce to 50 percent:
        int horizon_reduction = goal_idx/2;
        
        if (no_infeasible_plans_ > 9)
        {
            ROS_INFO_COND(no_infeasible_plans_==10, "Infeasible trajectory detected 10 times in a row: further reducing horizon...");
            horizon_reduction /= 2;
        }
        
        // we have a small overhead here, since we already transformed 50% more of the trajectory.
        // But that's ok for now, since we do not need to make transformGlobalPlan more complex 
        // and a reduced horizon should occur just rarely.
        int new_goal_idx_transformed_plan = int(transformed_plan.size()) - horizon_reduction - 1;
        goal_idx -= horizon_reduction;
        if (new_goal_idx_transformed_plan>0 && goal_idx >= 0)
            transformed_plan.erase(transformed_plan.begin()+new_goal_idx_transformed_plan, transformed_plan.end());
        else
            goal_idx += horizon_reduction; // this should not happen, but safety first ;-) 
    }
    
    
    // detect and resolve oscillations
    if (cfg_.recovery.oscillation_recovery)
    {
        double max_vel_theta;
        double max_vel_current = last_cmd_.linear.x >= 0 ? cfg_.robot.max_vel_x : cfg_.robot.max_vel_x_backwards;
        if (cfg_.robot.min_turning_radius!=0 && max_vel_current>0)
            max_vel_theta = std::max( max_vel_current/std::abs(cfg_.robot.min_turning_radius),  cfg_.robot.max_vel_theta );
        else
            max_vel_theta = cfg_.robot.max_vel_theta;
        
        failure_detector_.update(last_cmd_, cfg_.robot.max_vel_x, cfg_.robot.max_vel_x_backwards, max_vel_theta,
                               cfg_.recovery.oscillation_v_eps, cfg_.recovery.oscillation_omega_eps);
        
        bool oscillating = failure_detector_.isOscillating();
        bool recently_oscillated = (ros::Time::now()-time_last_oscillation_).toSec() < cfg_.recovery.oscillation_recovery_min_duration; // check if we have already detected an oscillation recently
        
        if (oscillating)
        {
            if (!recently_oscillated)
            {
                // save current turning direction
                if (robot_vel_.angular.z > 0)
                    last_preferred_rotdir_ = RotType::left;
                else
                    last_preferred_rotdir_ = RotType::right;
                ROS_WARN("TebLocalPlannerROS: possible oscillation (of the robot or its local plan) detected. Activating recovery strategy (prefer current turning direction during optimization).");
            }
            time_last_oscillation_ = ros::Time::now();  
            planner_->setPreferredTurningDir(last_preferred_rotdir_);
        }
        else if (!recently_oscillated && last_preferred_rotdir_ != RotType::none) // clear recovery behavior
        {
            last_preferred_rotdir_ = RotType::none;
            planner_->setPreferredTurningDir(last_preferred_rotdir_);
            ROS_INFO("TebLocalPlannerROS: oscillation recovery disabled/expired.");
        }
    }

}
     
void TebLocalPlannerROS::customObstacleCB(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg)
{
  boost::mutex::scoped_lock l(custom_obst_mutex_);
  custom_obstacle_msg_ = *obst_msg;  
}

void TebLocalPlannerROS::customViaPointsCB(const nav_msgs::Path::ConstPtr& via_points_msg)
{
  ROS_INFO_ONCE("Via-points received. This message is printed once.");

  if (cfg_.trajectory.global_plan_viapoint_sep > 0)
  {
    ROS_WARN("Via-points are already obtained from the global plan (global_plan_viapoint_sep>0)."
             "Ignoring custom via-points.");
    custom_via_points_active_ = false;
    return;
  }

  boost::mutex::scoped_lock l(via_point_mutex_);
  via_points_.clear();
  for (const geometry_msgs::PoseStamped& pose : via_points_msg->poses)
  {
    via_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }

  custom_via_points_active_ = !via_points_.empty();
}
     
RobotFootprintModelPtr TebLocalPlannerROS::getRobotFootprintFromParamServer(const ros::NodeHandle& nh, const TebConfig& config)
{
  std::string model_name; 
  if (!nh.getParam("footprint_model/type", model_name))
  {
    ROS_INFO("No robot footprint model specified for trajectory optimization. Using point-shaped model.");
    return boost::make_shared<PointRobotFootprint>();
  }
    
  // point  
  if (model_name.compare("point") == 0)
  {
    ROS_INFO("Footprint model 'point' loaded for trajectory optimization.");
    return boost::make_shared<PointRobotFootprint>(config.obstacles.min_obstacle_dist);
  }
  
  // circular
  if (model_name.compare("circular") == 0)
  {
    // get radius
    double radius;
    if (!nh.getParam("footprint_model/radius", radius))
    {
      ROS_ERROR_STREAM("Footprint model 'circular' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/radius' does not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    ROS_INFO_STREAM("Footprint model 'circular' (radius: " << radius <<"m) loaded for trajectory optimization.");
    return boost::make_shared<CircularRobotFootprint>(radius);
  }
  
  // line
  if (model_name.compare("line") == 0)
  {
    // check parameters
    if (!nh.hasParam("footprint_model/line_start") || !nh.hasParam("footprint_model/line_end"))
    {
      ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/line_start' and/or '.../line_end' do not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    // get line coordinates
    std::vector<double> line_start, line_end;
    nh.getParam("footprint_model/line_start", line_start);
    nh.getParam("footprint_model/line_end", line_end);
    if (line_start.size() != 2 || line_end.size() != 2)
    {
      ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/line_start' and/or '.../line_end' do not contain x and y coordinates (2D). Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    
    ROS_INFO_STREAM("Footprint model 'line' (line_start: [" << line_start[0] << "," << line_start[1] <<"]m, line_end: ["
                     << line_end[0] << "," << line_end[1] << "]m) loaded for trajectory optimization.");
    return boost::make_shared<LineRobotFootprint>(Eigen::Map<const Eigen::Vector2d>(line_start.data()), Eigen::Map<const Eigen::Vector2d>(line_end.data()), config.obstacles.min_obstacle_dist);
  }
  
  // two circles
  if (model_name.compare("two_circles") == 0)
  {
    // check parameters
    if (!nh.hasParam("footprint_model/front_offset") || !nh.hasParam("footprint_model/front_radius") 
        || !nh.hasParam("footprint_model/rear_offset") || !nh.hasParam("footprint_model/rear_radius"))
    {
      ROS_ERROR_STREAM("Footprint model 'two_circles' cannot be loaded for trajectory optimization, since params '" << nh.getNamespace()
                       << "/footprint_model/front_offset', '.../front_radius', '.../rear_offset' and '.../rear_radius' do not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    double front_offset, front_radius, rear_offset, rear_radius;
    nh.getParam("footprint_model/front_offset", front_offset);
    nh.getParam("footprint_model/front_radius", front_radius);
    nh.getParam("footprint_model/rear_offset", rear_offset);
    nh.getParam("footprint_model/rear_radius", rear_radius);
    ROS_INFO_STREAM("Footprint model 'two_circles' (front_offset: " << front_offset <<"m, front_radius: " << front_radius 
                    << "m, rear_offset: " << rear_offset << "m, rear_radius: " << rear_radius << "m) loaded for trajectory optimization.");
    return boost::make_shared<TwoCirclesRobotFootprint>(front_offset, front_radius, rear_offset, rear_radius);
  }

  // polygon
  if (model_name.compare("polygon") == 0)
  {

    // check parameters
    XmlRpc::XmlRpcValue footprint_xmlrpc;
    if (!nh.getParam("footprint_model/vertices", footprint_xmlrpc) )
    {
      ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/vertices' does not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    // get vertices
    if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      try
      {
        Point2dContainer polygon = makeFootprintFromXMLRPC(footprint_xmlrpc, "/footprint_model/vertices");
        ROS_INFO_STREAM("Footprint model 'polygon' loaded for trajectory optimization.");
        return boost::make_shared<PolygonRobotFootprint>(polygon);
      } 
      catch(const std::exception& ex)
      {
        ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization: " << ex.what() << ". Using point-model instead.");
        return boost::make_shared<PointRobotFootprint>();
      }
    }
    else
    {
      ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/vertices' does not define an array of coordinates. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    
  }
  
  // otherwise
  ROS_WARN_STREAM("Unknown robot footprint model specified with parameter '" << nh.getNamespace() << "/footprint_model/type'. Using point model instead.");
  return boost::make_shared<PointRobotFootprint>();
}
         
       
       
       
Point2dContainer TebLocalPlannerROS::makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue& footprint_xmlrpc, const std::string& full_param_name)
{
   // Make sure we have an array of at least 3 elements.
   if (footprint_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray ||
       footprint_xmlrpc.size() < 3)
   {
     ROS_FATAL("The footprint must be specified as list of lists on the parameter server, %s was specified as %s",
                full_param_name.c_str(), std::string(footprint_xmlrpc).c_str());
     throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least "
                              "3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
   }
 
   Point2dContainer footprint;
   Eigen::Vector2d pt;
 
   for (int i = 0; i < footprint_xmlrpc.size(); ++i)
   {
     // Make sure each element of the list is an array of size 2. (x and y coordinates)
     XmlRpc::XmlRpcValue point = footprint_xmlrpc[ i ];
     if (point.getType() != XmlRpc::XmlRpcValue::TypeArray ||
         point.size() != 2)
     {
       ROS_FATAL("The footprint (parameter %s) must be specified as list of lists on the parameter server eg: "
                 "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.",
                  full_param_name.c_str());
       throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: "
                               "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
    }

    pt.x() = getNumberFromXMLRPC(point[ 0 ], full_param_name);
    pt.y() = getNumberFromXMLRPC(point[ 1 ], full_param_name);

    footprint.push_back(pt);
  }
  return footprint;
}

double TebLocalPlannerROS::getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name)
{
  // Make sure that the value we're looking at is either a double or an int.
  if (value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
      value.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    std::string& value_string = value;
    ROS_FATAL("Values in the footprint specification (param %s) must be numbers. Found value %s.",
               full_param_name.c_str(), value_string.c_str());
     throw std::runtime_error("Values in the footprint specification must be numbers");
   }
   return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
}

} // end namespace teb_local_planner


