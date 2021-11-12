//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <cstdlib>
#include <cmath>

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
// #include "amrl_msgs/Localization2DMsg.h"
#include "math/line2d.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
using geometry::line2d;
using namespace std;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_; // publish to /ackermann_curvature_drive
ros::Publisher viz_pub_;  // publish to /visualization
VisualizationMsg local_viz_msg_; // points, lines, arcs, etc.
VisualizationMsg global_viz_msg_; // points, lines, arcs, etc.
AckermannCurvatureDriveMsg drive_msg_; // velocity, curvature
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
float critical_time = 0.0;
float critical_dist = 0.0;
float speed = 0.0;
float accel = 0.0;
bool collision = false;
// float latency; // -0.2;
float del_angle_ = 0.0;
std::vector<Vector2f> proj_point_cloud_;
std::vector<Vector2f> drawn_point_cloud_;
Eigen::Vector2f relative_local_target = Vector2f(0.0,0.0);
Eigen::Vector2f global_target;
int odd_num_paths = 15; // make sure this is odd
double previous_time;
double current_time;
double del_time;
bool obstacle_debug = false;
bool odometry_debug = false;

float map_x_width = 100.0; // -50.0 to +50.0
float map_y_height = 100.0;  // -50.0 to +50.0
float resolution = 0.25; // meters between nodes
float local_target_radius = 3.0; // meters away from robot

struct Node {
    string id;
    float x; // map_frame
    float y; // map_frame
    std::vector<string> neighbors;
    bool visited;
}

Eigen::MatrixXf global_graph = Eigen::MatrixXf::Ones(1 + int(map_x_width/resolution), 1 + int(map_y_height/resolution));
// std::map<std::string, Node> Nodes;
// std::vector<Node> global_path;
// float max_path_sep = 1.5; // meters away from path

} //namespace

namespace navigation {

// -------START HELPER FUNCTIONS--------------

float VelocityToSpeed(const Vector2f& velocity) {
  return sqrt(pow(velocity.x(), 2) + pow(velocity.y(), 2));
}

float VectorAccelToAccel(const Vector2f& accel) {
  return sqrt(pow(accel.x(), 2) + pow(accel.y(), 2));
}

std::vector<Vector2f> ProjectPointCloud1D(const std::vector<Vector2f>& point_cloud_,
                                           const Vector2f& velocity, float speed,
                                           float critical_time, float latency){
  vector<Vector2f> proj_point_cloud_;
  for (const auto& point : point_cloud_){
    Vector2f proj_point = point - (critical_time + latency) * velocity;
    proj_point_cloud_.push_back(proj_point);
  }
  return proj_point_cloud_;
}

std::vector<Vector2f> ProjectPointCloud2D(const std::vector<Vector2f>& point_cloud_,
                                          const Vector2f& velocity, float speed, float critical_time,
                                          float latency, float angle, float angular_vel_, float curvature){
  vector<Vector2f> proj_point_cloud_;
  Eigen::Rotation2Df rot(-angle);
  for (const auto& point : point_cloud_){
    Vector2f proj_point = (rot * (point - (critical_time + latency) * Vector2f(speed, 0.0)));
    proj_point_cloud_.push_back(proj_point);
  }
  return proj_point_cloud_;
}

bool PointWithinSafetyMargin(const Vector2f& proj_point,
                             float width, float length,
                             float axle_offset, float safety_margin_front, float safety_margin_side) {
  bool within_length = (proj_point.x() < (axle_offset + length + safety_margin_front)) && (proj_point.x() > (axle_offset - safety_margin_front));
  bool within_width = (proj_point.y() < (safety_margin_side + width/2.0)) && (proj_point.y() > (-safety_margin_side - width/2.0));
  return within_length && within_width;
}

bool ProjectedPointCloudCollision(const std::vector<Vector2f>& proj_point_cloud_,
                                  float width, float length, float axle_offset,
                                  float safety_margin_front, float safety_margin_side) {
  for (const auto& projected_point: proj_point_cloud_){
    if (PointWithinSafetyMargin(projected_point, width, length, axle_offset, safety_margin_front, safety_margin_side)){
      //draw a red point where identified point is within safety margin
      visualization::DrawPoint(projected_point, 0xeb3434, local_viz_msg_);
      if (obstacle_debug) {ROS_INFO("Collision Alert: Collision Detected!");}
      return true;
    }
  }
  if (obstacle_debug) {ROS_INFO("Collision Alert: None");}
  return false;
}

// Given the previous and current odometry readings
// return the instantaneous velocity
Vector2f GetOdomVelocity(const Vector2f& last_loc, const Vector2f& current_loc, float update_freq) {
  return update_freq * Vector2f(current_loc.x() - last_loc.x(), current_loc.y() - last_loc.y());
}

// Given the previous and current odometry readings
// return the instantaneous velocity
Vector2f GetOdomAcceleration(const Vector2f& last_vel, const Vector2f& current_vel, float update_freq) {
  return (last_vel - current_vel) / update_freq;
}

void PrintPaths(object_avoidance::paths_ptr paths){
  ROS_INFO("----------------------");
  for (const auto& path : *paths){
    ROS_INFO("c= %f, fpl= %f, cl= %f s= %f", path.curvature,  path.free_path_lengthv2, path.clearance, path.score);
  }
}


void DrawPaths(object_avoidance::paths_ptr paths){
  for (auto& path : *paths){
    visualization::DrawPathOption(path.curvature, path.free_path_lengthv2, 0.0, local_viz_msg_);
  }
}

float_t CalculateVelocityMsg(const std::vector<Vector2f>& point_cloud_, object_avoidance::CarSpecs car_specs_, float_t free_path_length, float_t critical_dist, float_t max_vel) {
  for (const auto& point: point_cloud_){
    if (PointWithinSafetyMargin(point, car_specs_.car_width, car_specs_.car_length, car_specs_.rear_axle_offset, car_specs_.car_safety_margin_front, car_specs_.car_safety_margin_side)){
      //draw a red point where identified point is within safety margin
      visualization::DrawPoint(point, 0xeb3434, local_viz_msg_);
      if (obstacle_debug) {ROS_INFO("Collision Alert: Collision Detected!");}
      collision = true;
      return 0.0;
    }
  }

  collision = false;

  if (free_path_length <= critical_dist){
    if (obstacle_debug) {ROS_INFO("Collision Alert: Predicting Collision");}
    return 0.0;
  } else {
    if (obstacle_debug) {ROS_INFO("Collision Alert: None");}
    return max_vel;
    // return max_vel * (min(free_path_length - critical_dist, float(4.0))) / 4.0
    // return 0.5 * max_vel * (min(free_path_length - critical_dist, float(4.0))) / 4.0 + max_vel * 0.5;
  }
}

// given the x and y map coordinates, return the closest index i, j
Eigen::Vector2i PointToIndex(Eigen::Vector2f point){
  return Eigen::Vector2i(int(round(((map_y_height/2.0) - point.y())/resolution)), int(round((point.x() + (map_x_width/2))/resolution)));
}

// given the matrix indices, return the map x and y location
Eigen::Vector2f IndexToPoint(Eigen::Vector2i loc){
                          // 0  - (100/2) = -50      (100/2) - 0 = 50  (0, 0) -> (-50, 50)
                          // 0  - (100/2) = -50      (100/2) - 1 = 49  (1, 0) -> (-50, 49)
  return Eigen::Vector2f((loc.y() * resolution) - (map_x_width / 2), (map_y_height / 2) - (loc.x() * resolution));
}

// given the robot location, highlight nearest node in red
void DrawRobotNode(Eigen::Vector2f loc){
  Eigen::Vector2f point =IndexToPoint(PointToIndex(loc));
  visualization::DrawArc(point, 0.3, 0, 2.0 * M_PI, 0x0045cf, global_viz_msg_);
  visualization::DrawPoint(point, 0x0045cf, global_viz_msg_);
}

// given the target location, highlight nearest node in green
void DrawTargetNode(Eigen::Vector2f loc){
  Eigen::Vector2f point =IndexToPoint(PointToIndex(loc));
  visualization::DrawArc(point, 0.3, 0, 2.0 * M_PI, 0x3ede12, global_viz_msg_);
  visualization::DrawPoint(point, 0x3ede12, global_viz_msg_);
}

// iterate through Matrix global_graph to overlay the nodes on the map
void OverlayGlobalGraph() {
  for (int i = 0; i < global_graph.cols(); i+=1){
    for (int j = 0; j < global_graph.rows(); j+=1){
      if (global_graph(i, j) == 1){
        visualization::DrawPoint(IndexToPoint(Eigen::Vector2i(i, j)), 0x7d7d7d, global_viz_msg_);
      }
    }
  }
}

void UpdateGlobalGraph (geometry::line2f line, float interval) {
  float x1 = line.p0(0,0);
  float y1 = line.p0(1,0);
  float x2 = line.p1(0,0);
  float y2 = line.p1(1,0);

  float length = (Eigen::Vector2f(x1,y1) - Eigen::Vector2f(x2,y2)).norm();
  int num_points = int(length / resolution);
  float x_inc = (x2 - x1) / num_points;
  float y_inc = (y2 - y1) / num_points;

  // ROS_INFO("line (%f, %f) to (%f, %f)", x1, y1, x2, y2);
  // ROS_INFO("length = %f", length);
  // ROS_INFO("increment (%f, %f)", x_inc, y_inc);
  // ROS_INFO(" number of points = %d", num_points);
  for (int i = 0; i < num_points; i++){
    // ROS_INFO("  processing point %d", i);
    Eigen::Vector2f point = Eigen::Vector2f(x1,y1) + Eigen::Vector2f(i * x_inc, i * y_inc);
    // ROS_INFO("  point (%f, %f)", point.x(), point.y());
    Eigen::Vector2i index = PointToIndex(point);
    // ROS_INFO("  index (%d, %d)", index.x(), index.y());
    global_graph(index.x(), index.y()) = 0;
  }
  // ROS_INFO("  processing last point");
  Eigen::Vector2i index = PointToIndex(Eigen::Vector2f(x2, y2));
  global_graph(index.x(), index.y()) = 0;
}

void InitializeGlobalGraph(vector_map::VectorMap map) {
  int32_t num_map_lines = (int32_t)map.lines.size();
  for (int32_t j = 0; j < num_map_lines; j++) {
    ROS_INFO("processing line %d", j);
    UpdateGlobalGraph(map.lines[j], resolution);
  }

  // TODO:
  // need to loop through global_graph
  // 1) create nodes
  // 2) reference global_graph to find valid neighbors

  ROS_INFO("Initialization complete.");
}

// -------END HELPER FUNCTIONS-------------

// Navigation Constructor called when Navigation instantiated in navigation_main.cc
Navigation::Navigation(const string& map_file, const double& latency, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0),
    odom_omega_(0),
    latency(latency) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
  ros::Time::init();
  map_.Load(map_file);
  InitializeGlobalGraph(map_);

  // Create ObjectAvoidance object that will manage path calculations
  path_planner_.reset(
    new object_avoidance::ObjectAvoidance(car_specs_, odd_num_paths, min_turn_radius_));
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  nav_complete_ = false;
  nav_goal_loc_ = loc;
  nav_goal_angle_ = angle;
  global_target = loc;
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

// gets called in navigation_main.cc during ros::spinOnce()
void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {

  if (odom_initialized_) {
    last_odom_loc_ = odom_loc_;
    last_odom_angle_ = odom_angle_;
  }

  robot_omega_ = ang_vel; // ?
  robot_vel_ = vel;
  odom_loc_ = loc;
  odom_angle_ = angle;
  current_time = ros::Time::now().toSec();

  if (odom_initialized_) {
    del_time = current_time - previous_time;
    last_odom_vel_ = odom_vel_;
    odom_vel_ = GetOdomVelocity(last_odom_loc_, odom_loc_, 1.0 / del_time); //update_frequency_);
    odom_accel_ = GetOdomAcceleration(last_odom_vel_, odom_vel_, 1.0 / del_time); //update_frequency_);
    speed = VelocityToSpeed(odom_vel_);
    accel = VectorAccelToAccel(odom_accel_);
    del_angle_ = odom_angle_ - last_odom_angle_;
    odom_omega_ = del_angle_ / del_time; // update_frequency_;
    critical_time =  speed / max_accel_;
    critical_dist = 0.5 * (critical_time + latency) * max_vel_;

    if (odometry_debug){
      ROS_INFO("================START CONTROL================="); 
      ROS_INFO("current_time = %f", current_time);   
      ROS_INFO("previous_time = %f", previous_time);  
      ROS_INFO("del_time = %f", del_time); 
      ROS_INFO("odom_loc_ = (%f, %f)", odom_loc_.x(), odom_loc_.y());
      ROS_INFO("last_odom_loc_ = (%f, %f)", last_odom_loc_.x(), last_odom_loc_.y());
      ROS_INFO("odom_vel_ = (%f, %f)", odom_vel_.x(),odom_vel_.y());
      ROS_INFO("last_odom_vel = (%f, %f)",last_odom_vel_.x(), last_odom_vel_.y());
      ROS_INFO("odom_accel_ = (%f, %f)", odom_accel_.x(), odom_accel_.y());    
      ROS_INFO("speed = %f", speed);
      ROS_INFO("accel = %f", accel);
      ROS_INFO("latency = %f", latency);
      ROS_INFO("critical_time = %f", critical_time);
      ROS_INFO("critical_dist = %f", critical_dist);
      ROS_INFO("----------------------");
      ROS_INFO("odom_angle_ = %f", odom_angle_);
      ROS_INFO("last_odom_angle_ = %f", last_odom_angle_);
      ROS_INFO("del_angle_ = %f", del_angle_);
      ROS_INFO("odom_omega_ = %f", odom_omega_);
      ROS_INFO("----------------------");
    }
  }

  previous_time = current_time;
 
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    return;
  }
  
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;                                     
}

void Navigation::ObstacleAvoid(){

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) {
    return;
  }
  
  // -------START CONTROL---------------------------------------
  
  path_planner_->UpdatePaths(point_cloud_, relative_local_target);
  auto best_path = path_planner_->GetHighestScorePath(); //GetPlannedCurvature();
  drive_msg_.curvature = best_path.curvature;
  drive_msg_.velocity = CalculateVelocityMsg(point_cloud_, car_specs_, best_path.free_path_lengthv2, critical_dist, max_vel_);
  
  // ---------- Visualizations & Terminal Outputs -----------------
  
  DrawPaths(path_planner_->GetPaths());
  visualization::DrawRobot(car_width_, car_length_, rear_axle_offset_, car_safety_margin_front_, car_safety_margin_side_, drive_msg_, local_viz_msg_, collision);
  visualization::DrawLocalTarget(relative_local_target, local_viz_msg_);
  visualization::DrawPathOption(drive_msg_.curvature, local_target_radius, 0, local_viz_msg_);
  
  if (obstacle_debug) {ROS_INFO("drive_msg_.velocity = %f", drive_msg_.velocity);}
  if (obstacle_debug) {ROS_INFO("drive_msg_.curvature = %f", drive_msg_.curvature);}

  // Display Driving Status
  if ((drive_msg_.velocity == 0) && (speed == 0.0)){
    if (obstacle_debug) {ROS_INFO("Status: Stopped");}
  }
  else if (drive_msg_.velocity == 0) {
    if (obstacle_debug) {ROS_INFO("Status: Stopping");}
  }
  else { // driving
    if (drive_msg_.curvature == 0) { 
      if (obstacle_debug) {ROS_INFO("Status: Driving Straight");}
    } else if (drive_msg_.curvature > 0){
      if (obstacle_debug) {ROS_INFO("Status: Turning Left");}
    }
      else if (drive_msg_.curvature < 0){
      if (obstacle_debug) {ROS_INFO("Status: Turning Right");}
    }
  }

  if (obstacle_debug) {PrintPaths(path_planner_->GetPaths());}
  
  if (obstacle_debug) {ROS_INFO("=================END CONTROL==================");}  

}


void Navigation::Run() {

  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // Avoid Obstacles
  ObstacleAvoid();
  // OverlayGlobalGraph();
  DrawTargetNode(global_target);
  DrawRobotNode(robot_loc_);
  visualization::DrawArc(Vector2f(0.0, 0.0), local_target_radius, 0, 2.0 * M_PI, 0x0045cf, local_viz_msg_);
  visualization::DrawArc(robot_loc_, resolution, 0, 2.0 * M_PI, 0x3ede12, global_viz_msg_);
  visualization::DrawGlobalTarget(global_target, global_viz_msg_);
  visualization::DrawLine(robot_loc_, global_target, 0x3ede12, global_viz_msg_);
  
  // ROS_INFO("robot_loc_ = (%f, %f)", robot_loc_.x(), robot_loc_.y());
  // ROS_INFO("distance to global_target = %f",(robot_loc_ - global_target).norm());

  // robot is at global_target
  if ((robot_loc_ - global_target).norm() <= resolution) { // distance to global target
    //  relative_local_target = Vector2f(0.0, 0.0); // TODO: find out why car turns right
     drive_msg_.velocity = 0;
   } else { // robot is not at global target
      
      // robot is straying off global path 
      // if (distance from global path > max_separation) {
      //    // do A* search on global_graph from robot.loc to global_target 
      //    global_path = Plan_Global_Path(robot.loc, global_graph, global_target); 
      // }
      
      // TODO:
      // find intersection of circle around robot with global path, return relative coordinates to robot
      // relative_local_target = Calculate_Local_Target(robot.loc, global_path, local_target_radius);
      relative_local_target = Vector2f(3.0, 0.0);
   }
  
  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

}  // namespace navigation
