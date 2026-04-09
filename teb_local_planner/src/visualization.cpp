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

#include <teb_local_planner/visualization.h>
#include <teb_local_planner/optimal_planner.h>
#include <teb_local_planner/FeedbackMsg.h>

namespace teb_local_planner
{

TebVisualization::TebVisualization() : initialized_(false)
{
}

TebVisualization::TebVisualization(ros::NodeHandle& nh, const TebConfig& cfg) : initialized_(false)
{
  initialize(nh, cfg);
}

void TebVisualization::initialize(ros::NodeHandle& nh, const TebConfig& cfg)
{
  if (initialized_)
    ROS_WARN("TebVisualization already initialized. Reinitalizing...");
  
  // set config
  cfg_ = &cfg;
  
  // register topics
  global_plan_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1);
  local_plan_pub_ = nh.advertise<nav_msgs::Path>("local_plan",1);
  teb_poses_pub_ = nh.advertise<geometry_msgs::PoseArray>("teb_poses", 100);
  teb_marker_pub_ = nh.advertise<visualization_msgs::Marker>("teb_markers", 1000);
  feedback_pub_ = nh.advertise<teb_local_planner::FeedbackMsg>("teb_feedback", 10);  
  footprint_pub_ = nh.advertise<visualization_msgs::Marker>("footprint_markers", 1000);
  footprintmodel_pub_ = nh.advertise<visualization_msgs::Marker>("footprintmodel_markers", 1000);
  infeasiblefootprint_pub_ = nh.advertise<visualization_msgs::Marker>("infeasiblefootprint_markers", 1000);
  infeasiblefootprintmodel_pub_ = nh.advertise<visualization_msgs::Marker>("infeasiblefootprintmodel_markers", 1000);
  obstacle_pub_ = nh.advertise<visualization_msgs::Marker>("close_obstacle", 1000);
  gap_pub_ = nh.advertise<visualization_msgs::Marker>("gap_marker", 1);
  via_point_pub_ = nh.advertise<visualization_msgs::Marker>("via_point_marker", 1);
  marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 5);
  grid_path_pub_ = nh.advertise<visualization_msgs::Marker>("grid_path_marker", 10);

  initialized_ = true; 
}



void TebVisualization::publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan) const
{
  if ( printErrorWhenNotInitialized() ) return;
  base_local_planner::publishPlan(global_plan, global_plan_pub_); 
}

void TebVisualization::publishLocalPlan(const std::vector<geometry_msgs::PoseStamped>& local_plan) const
{
  if ( printErrorWhenNotInitialized() )
    return;
  base_local_planner::publishPlan(local_plan, local_plan_pub_); 
}

void TebVisualization::publishLocalPlanAndPoses(const TimedElasticBand& teb) const
{
  if ( printErrorWhenNotInitialized() )
    return;
  
    // create path msg
    nav_msgs::Path teb_path;
    teb_path.header.frame_id = cfg_->map_frame;
    teb_path.header.stamp = ros::Time::now();
    
    // create pose_array (along trajectory)
    geometry_msgs::PoseArray teb_poses;
    teb_poses.header.frame_id = teb_path.header.frame_id;
    teb_poses.header.stamp = teb_path.header.stamp;
    
    // fill path msgs with teb configurations
    for (int i=0; i < teb.sizePoses(); i++)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = teb_path.header.frame_id;
      pose.header.stamp = teb_path.header.stamp;
      pose.pose.position.x = teb.Pose(i).x();
      pose.pose.position.y = teb.Pose(i).y();
      pose.pose.position.z = cfg_->hcp.visualize_with_time_as_z_axis_scale*teb.getSumOfTimeDiffsUpToIdx(i);
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(teb.Pose(i).theta());
      teb_path.poses.push_back(pose);
      teb_poses.poses.push_back(pose.pose);
    }
    local_plan_pub_.publish(teb_path);
    teb_poses_pub_.publish(teb_poses);
  }

  void TebVisualization::publishGridSearchPath(const std::vector<geometry_msgs::Point>& path)
  {
    // grid search 경로를 나타내는 LINE_STRIP Marker
    visualization_msgs::Marker grid_path_marker;
    grid_path_marker.header.frame_id = "map";  // 사용 좌표계에 맞게 수정
    grid_path_marker.header.stamp = ros::Time::now();
    grid_path_marker.ns = "grid_search_path";
    grid_path_marker.id = 0;
    grid_path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    grid_path_marker.action = visualization_msgs::Marker::ADD;

    grid_path_marker.scale.x = 0.02;  // 경로 선 두께 (미터 단위)
    grid_path_marker.color.r = 0.0;
    grid_path_marker.color.g = 0.0;
    grid_path_marker.color.b = 1.0;
    grid_path_marker.color.a = 1.0;

    for (const auto& p : path)
    {
        geometry_msgs::Point pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = 0.05;  // 살짝 띄워서 RViz에서 뚜렷하게 보이게
        grid_path_marker.points.push_back(pt);
    }


    grid_path_pub_.publish(grid_path_marker);
  }

  void TebVisualization::visualizeSamples(const std::vector<geometry_msgs::Point>& samples)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "samples";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    for (const auto& sample : samples)
    {
      geometry_msgs::Point p;
      p.x = sample.x;
      p.y = sample.y;
      p.z = 0.0;
      marker.points.push_back(p);
    }

    marker_pub_.publish(marker);
  }

  void TebVisualization::visualizeMedialBall(const geometry_msgs::Point& center, double radius)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "medial_balls";
    marker.id = static_cast<int>(center.x * 1000 + center.y * 1000);
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.02;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    // Draw the medial ball centered at the new position
    for (double angle = 0; angle <= 2 * M_PI; angle += M_PI / 36)
    {
      geometry_msgs::Point p;
      p.x = center.x + radius * std::cos(angle);
      p.y = center.y + radius * std::sin(angle);
      p.z = 0.0;
      marker.points.push_back(p);
    }

    ros::Duration lifetime(1.0);
    marker.lifetime = lifetime;

    marker_pub_.publish(marker);
  }

  void TebVisualization::visualizeNarrowSpace(const geometry_msgs::Point& center, double radius)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map"; // Replace "map" with the appropriate frame if needed
    marker.header.stamp = ros::Time::now();
    marker.ns = "narrow_space";
    marker.id = static_cast<int>(center.x * 1000 + center.y * 1000); // Unique ID for each marker
    marker.type = visualization_msgs::Marker::CYLINDER; // Use CYLINDER to represent the narrow space
    marker.action = visualization_msgs::Marker::ADD;

    // Set the position of the marker
    marker.pose.position.x = center.x;
    marker.pose.position.y = center.y;
    marker.pose.position.z = 0.0; // Adjust the height as needed
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale (diameter = 2 * radius)
    marker.scale.x = 2 * radius;
    marker.scale.y = 2 * radius;
    marker.scale.z = 0.1; // Height of the cylinder (can be adjusted as needed)

    // Set the color (e.g., red for narrow spaces)
    marker.color.a = 0.01; // Transparency
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    // Set lifetime (optional, so it doesn't persist forever)
    marker.lifetime = ros::Duration(1.0); // Adjust the duration as needed

    // Publish the marker
    marker_pub_.publish(marker);
  }

void TebVisualization::publishRobotFootprintModel(const PoseSE2& current_pose, const BaseRobotFootprintModel& robot_model,
                                                  const std::string& ns, const std_msgs::ColorRGBA &color)
{
  if ( printErrorWhenNotInitialized() )
    return;
  
  std::vector<visualization_msgs::Marker> markers;
  robot_model.visualizeRobot(current_pose, markers, color);
  if (markers.empty())
    return;

  int idx = 0;
  for (std::vector<visualization_msgs::Marker>::iterator marker_it = markers.begin(); marker_it != markers.end(); ++marker_it, ++idx)
  { //square
    marker_it->header.frame_id = cfg_->map_frame;
    marker_it->header.stamp = ros::Time::now();
    marker_it->action = visualization_msgs::Marker::ADD;
    marker_it->ns = ns;
    marker_it->id = idx;
    marker_it->lifetime = ros::Duration(2.0);
    footprintmodel_pub_.publish(*marker_it);
  }
  
}

void TebVisualization::publishRobotFootprint(const PoseSE2& current_pose, const std::vector<geometry_msgs::Point>& footprint,
                                             const std::string& ns, const std_msgs::ColorRGBA &color)
{ // circle 
  if ( printErrorWhenNotInitialized() || footprint.empty() )
    return;

  visualization_msgs::Marker vertex_marker;
  vertex_marker.header.frame_id = cfg_->map_frame;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.ns = ns;
  vertex_marker.type = visualization_msgs::Marker::LINE_STRIP;
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.color = color;
  vertex_marker.scale.x = 0.01;
  vertex_marker.lifetime = ros::Duration(2.0);
  vertex_marker.points = footprint;
  vertex_marker.points.push_back(footprint.front());  // close the polygon
  current_pose.toPoseMsg(vertex_marker.pose);
  footprint_pub_.publish(vertex_marker);
}

void TebVisualization::publishInfeasibleRobotFootprintModel(const PoseSE2& current_pose, const BaseRobotFootprintModel& robot_model,
                                                  const std::string& ns, const std_msgs::ColorRGBA &color)
{
  if ( printErrorWhenNotInitialized() )
    return;
  
  std::vector<visualization_msgs::Marker> markers;
  robot_model.visualizeRobot(current_pose, markers, color);
  if (markers.empty())
    return;

  int idx = 0;
  for (std::vector<visualization_msgs::Marker>::iterator marker_it = markers.begin(); marker_it != markers.end(); ++marker_it, ++idx)
  { //square
    marker_it->header.frame_id = cfg_->map_frame;
    marker_it->header.stamp = ros::Time::now();
    marker_it->action = visualization_msgs::Marker::ADD;
    marker_it->ns = ns;
    marker_it->id = idx;
    marker_it->lifetime = ros::Duration(2.0);
    infeasiblefootprintmodel_pub_.publish(*marker_it);
  }
  
}

void TebVisualization::publishInfeasibleRobotFootprint(const PoseSE2& current_pose, const std::vector<geometry_msgs::Point>& footprint,
                                             const std::string& ns, const std_msgs::ColorRGBA &color)
{ // circle 
  if ( printErrorWhenNotInitialized() || footprint.empty() )
    return;

  visualization_msgs::Marker vertex_marker;
  vertex_marker.header.frame_id = cfg_->map_frame;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.ns = ns;
  vertex_marker.type = visualization_msgs::Marker::LINE_STRIP;
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.color = color;
  vertex_marker.scale.x = 0.01;
  vertex_marker.lifetime = ros::Duration(2.0);
  vertex_marker.points = footprint;
  vertex_marker.points.push_back(footprint.front());  // close the polygon
  current_pose.toPoseMsg(vertex_marker.pose);
  infeasiblefootprint_pub_.publish(vertex_marker);
}
void TebVisualization::publishInfeasibleRobotPose(const PoseSE2& infeasible_pose,
                                                  const BaseRobotFootprintModel& robot_model,
                                                  const std::vector<geometry_msgs::Point>& footprint)
{
  publishInfeasibleRobotFootprintModel(infeasible_pose, robot_model, "InfeasibleRobotPose/model", toColorMsg(0.5, 0.8, 0.0, 0.0)); //red
  publishInfeasibleRobotFootprint(infeasible_pose, footprint, "InfeasibleRobotPose/footprint", toColorMsg(0.5, 0.9, 0.7, 0.0)); //yellow
}

void TebVisualization::publishRobotPose(const PoseSE2& pose,
                                                  const BaseRobotFootprintModel& robot_model,
                                                  const std::vector<geometry_msgs::Point>& footprint)
{
  publishRobotFootprintModel(pose, robot_model, "RobotPose/model", toColorMsg(1.0, 0.0, 1.0, 0.0)); //green
  publishRobotFootprint(pose, footprint, "RobotPose/footprint", toColorMsg(1.0, 0.0, 0.0, 1.0)); // blue
}

void TebVisualization::visualizeIntermediatePoint(const PoseSE2& pose, const std::string& ns)
{
  if ( printErrorWhenNotInitialized())
    return;

  geometry_msgs::Point point_msg;
  point_msg.x = pose.x();  // x coordinate of the point
  point_msg.y = pose.y();  // y coordinate of the point
  point_msg.z = 0.0;  // assuming a 2D plane, so z coordinate is 0

  visualization_msgs::Marker marker;
  marker.header.frame_id = cfg_->map_frame; // map frame
  marker.header.stamp = ros::Time::now();  // current time
  marker.ns = ns;
  marker.type = visualization_msgs::Marker::POINTS;  // marker type
  marker.action = visualization_msgs::Marker::ADD;  // add marker action

  // Set the scale of the marker
  marker.scale.x = 0.05;  // x scale (width)
  marker.scale.y = 0.05;  // y scale (height)

  // Set the color of the marker
  marker.color.a = 1.0;  // alpha (transparency)
  marker.color.r = 0.0;  // red component
  marker.color.g = 0.0;  // green component
  marker.color.b = 1.0;  // blue component

  marker.points.push_back(point_msg);

  // Check if the points list is empty
  if (marker.points.empty()) {
    ROS_WARN("Points list is empty for visualization marker. Skipping visualization.");
    return;
  }

  // Publish the marker
  teb_marker_pub_.publish(marker);
}

void TebVisualization::visualizeNarrGap(const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& narrow_gaps)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "Gap";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.02; // 선 두께
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.4;
    marker.color.b = 0.6;

    marker.lifetime = ros::Duration(2.0); // 5초 동안 유지

    for (const auto& gap : narrow_gaps)
    {
        geometry_msgs::Point p1, p2;
        p1.x = gap.first.x();
        p1.y = gap.first.y();
        p1.z = 0;

        p2.x = gap.second.x();
        p2.y = gap.second.y();
        p2.z = 0;

        marker.points.push_back(p1);
        marker.points.push_back(p2);
    }

    // 퍼블리시
    gap_pub_.publish(marker);
}

void TebVisualization::publishObstacles(const ObstContainer& obstacles, double scale) const
{
  if ( obstacles.empty() || printErrorWhenNotInitialized() )
    return;
  
  // Visualize point obstacles
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = cfg_->map_frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = "PointObstacles";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(2.0);
    
    for (ObstContainer::const_iterator obst = obstacles.begin(); obst != obstacles.end(); ++obst)
    {
      boost::shared_ptr<PointObstacle> pobst = boost::dynamic_pointer_cast<PointObstacle>(*obst);      
      if (!pobst)
        continue;

      if (cfg_->hcp.visualize_with_time_as_z_axis_scale < 0.001)
      {
        geometry_msgs::Point point;
        point.x = pobst->x();
        point.y = pobst->y();
        point.z = 0;
        marker.points.push_back(point);
      }
      else // Spatiotemporally point obstacles become a line
      {
        marker.type = visualization_msgs::Marker::LINE_LIST;
        geometry_msgs::Point start;
        start.x = pobst->x();
        start.y = pobst->y();
        start.z = 0;
        marker.points.push_back(start);

        geometry_msgs::Point end;
        double t = 20;
        Eigen::Vector2d pred;
        pobst->predictCentroidConstantVelocity(t, pred);
        end.x = pred[0];
        end.y = pred[1];
        end.z = cfg_->hcp.visualize_with_time_as_z_axis_scale*t;
        marker.points.push_back(end);
      }
    }
    
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    teb_marker_pub_.publish( marker );
  }
  
  // Visualize circular obstacles
  {
    std::size_t idx = 0;
    for (ObstContainer::const_iterator obst = obstacles.begin(); obst != obstacles.end(); ++obst)
    {
      boost::shared_ptr<CircularObstacle> pobst = boost::dynamic_pointer_cast<CircularObstacle>(*obst);
      if (!pobst)
        continue;

      visualization_msgs::Marker marker;
      marker.header.frame_id = cfg_->map_frame;
      marker.header.stamp = ros::Time::now();
      marker.ns = "CircularObstacles";
      marker.id = idx++;
      marker.type = visualization_msgs::Marker::SPHERE_LIST;
      marker.action = visualization_msgs::Marker::ADD;
      marker.lifetime = ros::Duration(2.0);
      geometry_msgs::Point point;
      point.x = pobst->x();
      point.y = pobst->y();
      point.z = 0;
      marker.points.push_back(point);

      marker.scale.x = pobst->radius();
      marker.scale.y = pobst->radius();
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;

      teb_marker_pub_.publish( marker );
    }
  }

  // Visualize line obstacles
  {
    std::size_t idx = 0;
    for (ObstContainer::const_iterator obst = obstacles.begin(); obst != obstacles.end(); ++obst)
    {	
      boost::shared_ptr<LineObstacle> pobst = boost::dynamic_pointer_cast<LineObstacle>(*obst);   
      if (!pobst)
        continue;
      
      visualization_msgs::Marker marker;
      marker.header.frame_id = cfg_->map_frame;
      marker.header.stamp = ros::Time::now();
      marker.ns = "LineObstacles";
      marker.id = idx++;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;
      marker.lifetime = ros::Duration(2.0);
      geometry_msgs::Point start;
      start.x = pobst->start().x();
      start.y = pobst->start().y();
      start.z = 0;
      marker.points.push_back(start);
      geometry_msgs::Point end;
      end.x = pobst->end().x();
      end.y = pobst->end().y();
      end.z = 0;
      marker.points.push_back(end);
  
      marker.scale.x = scale;
      marker.scale.y = scale;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      
      teb_marker_pub_.publish( marker );     
    }
  }
  

  // Visualize polygon obstacles
  {
    std::size_t idx = 0;
    for (ObstContainer::const_iterator obst = obstacles.begin(); obst != obstacles.end(); ++obst)
    {	
      boost::shared_ptr<PolygonObstacle> pobst = boost::dynamic_pointer_cast<PolygonObstacle>(*obst);   
      if (!pobst)
				continue;
      
      visualization_msgs::Marker marker;
      marker.header.frame_id = cfg_->map_frame;
      marker.header.stamp = ros::Time::now();
      marker.ns = "PolyObstacles";
      marker.id = idx++;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;
      marker.lifetime = ros::Duration(2.0);
      
      for (Point2dContainer::const_iterator vertex = pobst->vertices().begin(); vertex != pobst->vertices().end(); ++vertex)
      {
        geometry_msgs::Point point;
        point.x = vertex->x();
        point.y = vertex->y();
        point.z = 0;
        marker.points.push_back(point);
      }
      
      // Also add last point to close the polygon
      // but only if polygon has more than 2 points (it is not a line)
      if (pobst->vertices().size() > 2)
      {
        geometry_msgs::Point point;
        point.x = pobst->vertices().front().x();
        point.y = pobst->vertices().front().y();
        point.z = 0;
        marker.points.push_back(point);
      }
      marker.scale.x = scale;
      marker.scale.y = scale;
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      
      teb_marker_pub_.publish( marker );     
    }
  }
}


void TebVisualization::publishViaPoints(const std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& via_points, const std::string& ns) const
{
  if ( via_points.empty() || printErrorWhenNotInitialized() )
    return;
  
  visualization_msgs::Marker marker;
  marker.header.frame_id = cfg_->map_frame;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(2.0);
  
  for (std::size_t i=0; i < via_points.size(); ++i)
  {
    geometry_msgs::Point point;
    point.x = via_points[i].x();
    point.y = via_points[i].y();
    point.z = 0;
    marker.points.push_back(point);
  }
  
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;

  teb_marker_pub_.publish( marker );
}

void TebVisualization::publishCustomViaPoints(const std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& via_points, const std::string& ns) const
{
  if ( via_points.empty() || printErrorWhenNotInitialized() )
    return;
  
  visualization_msgs::Marker marker;
  marker.header.frame_id = cfg_->map_frame;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(2.0);
  
  for (std::size_t i=0; i < via_points.size(); ++i)
  {
    geometry_msgs::Point point;
    point.x = via_points[i].x();
    point.y = via_points[i].y();
    point.z = 0;
    marker.points.push_back(point);
  }
  
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;

  via_point_pub_.publish( marker );
}


void TebVisualization::publishTebContainer(const TebOptPlannerContainer& teb_planner, const std::string& ns)
{
  if (printErrorWhenNotInitialized())
      return;

  visualization_msgs::Marker line_marker;
  line_marker.header.frame_id = cfg_->map_frame;
  line_marker.header.stamp = ros::Time::now();
  line_marker.ns = ns;
  line_marker.id = 0;
  line_marker.type = visualization_msgs::Marker::LINE_LIST;
  line_marker.action = visualization_msgs::Marker::ADD;

  visualization_msgs::Marker point_marker;
  point_marker.header.frame_id = cfg_->map_frame;
  point_marker.header.stamp = ros::Time::now();
  point_marker.ns = ns;
  point_marker.id = 1; // unique id for the point marker
  point_marker.type = visualization_msgs::Marker::POINTS;
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.scale.x = 0.05; // scale for the points
  point_marker.scale.y = 0.05;
  point_marker.color.a = 1.0;  // alpha
  point_marker.color.r = 1.0;  // red
  point_marker.color.g = 0.0;  // green
  point_marker.color.b = 0.0;  // blue

    // Iterate through teb pose sequence
  for (TebOptPlannerContainer::const_iterator it_teb = teb_planner.begin(); it_teb != teb_planner.end(); ++it_teb)
  {
      // iterate single poses
      PoseSequence::const_iterator it_pose = it_teb->get()->teb().poses().begin();
      TimeDiffSequence::const_iterator it_timediff = it_teb->get()->teb().timediffs().begin();
      PoseSequence::const_iterator it_pose_end = it_teb->get()->teb().poses().end();
      std::advance(it_pose_end, -1); // since we are interested in line segments, reduce end iterator by one.
      double time = 0;

      while (it_pose != it_pose_end)
      {
          geometry_msgs::Point point_start;
          point_start.x = (*it_pose)->x();
          point_start.y = (*it_pose)->y();
          point_start.z = cfg_->hcp.visualize_with_time_as_z_axis_scale * time;
          line_marker.points.push_back(point_start);
          point_marker.points.push_back(point_start); // Add the point to the point marker

          time += (*it_timediff)->dt();

          geometry_msgs::Point point_end;
          point_end.x = (*boost::next(it_pose))->x();
          point_end.y = (*boost::next(it_pose))->y();
          point_end.z = cfg_->hcp.visualize_with_time_as_z_axis_scale * time;
          line_marker.points.push_back(point_end);
          point_marker.points.push_back(point_end); // Add the point to the point marker

          ++it_pose;
          ++it_timediff;
      }
  }

  line_marker.scale.x = 0.01;
  line_marker.color.a = 1.0;
  line_marker.color.r = 0.5;
  line_marker.color.g = 1.0;
  line_marker.color.b = 0.0;

  teb_marker_pub_.publish(line_marker);
  teb_marker_pub_.publish(point_marker); // Publish the point marker
}

void TebVisualization::publishFeedbackMessage(const std::vector< boost::shared_ptr<TebOptimalPlanner> >& teb_planners,
                                              unsigned int selected_trajectory_idx, const ObstContainer& obstacles)
{
  FeedbackMsg msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = cfg_->map_frame;
  msg.selected_trajectory_idx = selected_trajectory_idx;
  
  
  msg.trajectories.resize(teb_planners.size());
  
  // Iterate through teb pose sequence
  std::size_t idx_traj = 0;
  for( TebOptPlannerContainer::const_iterator it_teb = teb_planners.begin(); it_teb != teb_planners.end(); ++it_teb, ++idx_traj )
  {   
    msg.trajectories[idx_traj].header = msg.header;
    it_teb->get()->getFullTrajectory(msg.trajectories[idx_traj].trajectory);
  }
  
  // add obstacles
  msg.obstacles_msg.obstacles.resize(obstacles.size());
  for (std::size_t i=0; i<obstacles.size(); ++i)
  {
    msg.obstacles_msg.header = msg.header;

    // copy polygon
    msg.obstacles_msg.obstacles[i].header = msg.header;
    obstacles[i]->toPolygonMsg(msg.obstacles_msg.obstacles[i].polygon);

    // copy id
    msg.obstacles_msg.obstacles[i].id = i; // TODO: we do not have any id stored yet

    // orientation
    //msg.obstacles_msg.obstacles[i].orientation =; // TODO

    // copy velocities
    obstacles[i]->toTwistWithCovarianceMsg(msg.obstacles_msg.obstacles[i].velocities);
  }
  
  feedback_pub_.publish(msg);
}

void TebVisualization::publishFeedbackMessage(const TebOptimalPlanner& teb_planner, const ObstContainer& obstacles)
{
  FeedbackMsg msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = cfg_->map_frame;
  msg.selected_trajectory_idx = 0;
  
  msg.trajectories.resize(1);
  msg.trajectories.front().header = msg.header;
  teb_planner.getFullTrajectory(msg.trajectories.front().trajectory);
 
  // add obstacles
  msg.obstacles_msg.obstacles.resize(obstacles.size());
  for (std::size_t i=0; i<obstacles.size(); ++i)
  {
    msg.obstacles_msg.header = msg.header;

    // copy polygon
    msg.obstacles_msg.obstacles[i].header = msg.header;
    obstacles[i]->toPolygonMsg(msg.obstacles_msg.obstacles[i].polygon);

    // copy id
    msg.obstacles_msg.obstacles[i].id = i; // TODO: we do not have any id stored yet

    // orientation
    //msg.obstacles_msg.obstacles[i].orientation =; // TODO

    // copy velocities
    obstacles[i]->toTwistWithCovarianceMsg(msg.obstacles_msg.obstacles[i].velocities);
  }
  
  feedback_pub_.publish(msg);
}

std_msgs::ColorRGBA TebVisualization::toColorMsg(double a, double r, double g, double b)
{
  std_msgs::ColorRGBA color;
  color.a = a;
  color.r = r;
  color.g = g;
  color.b = b;
  return color;
}

bool TebVisualization::printErrorWhenNotInitialized() const
{
  if (!initialized_)
  {
    ROS_ERROR("TebVisualization class not initialized. You must call initialize or an appropriate constructor");
    return true;
  }
  return false;
}

} // namespace teb_local_planner
