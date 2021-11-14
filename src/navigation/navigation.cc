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
#include "math/line2d.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include "graph.h"
// #include "geometry"
#include "global_planner.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
using geometry::line2d;
using geometry::MinDistanceLineArc;
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
float local_target_radius = 2.0; // meters away from robot

Eigen::MatrixXf global_graph = Eigen::MatrixXf::Ones(1 + int(map_x_width/resolution), 1 + int(map_y_height/resolution));
std::map<std::string, Node*> Nodes;
std::vector<std::string> global_path;
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
        Eigen::Vector2f point = IndexToPoint(Eigen::Vector2i(i, j));
        visualization::DrawPoint(point, 0x3ede12, global_viz_msg_);
        // visualization::DrawArc(point, 0.2, 0, 2.0 * M_PI, 0x3ede12, global_viz_msg_);
      }
    }
  }
}

void VisualizeMapPoints(vector_map::VectorMap map){
  int32_t num_map_lines = (int32_t)map.lines.size();
  for (int32_t j = 0; j < num_map_lines; j++) {
    ROS_INFO("processing line %d", j);
    geometry::line2f line = map.lines[j];
    // float interval = resolution;

    float x1 = line.p0(0,0);
    float y1 = line.p0(1,0);
    float x2 = line.p1(0,0);
    float y2 = line.p1(1,0);

    float length = (Eigen::Vector2f(x1,y1) - Eigen::Vector2f(x2,y2)).norm();
    int num_points = 2 * int(length / resolution);
    float x_inc = (x2 - x1) / num_points;
    float y_inc = (y2 - y1) / num_points;

    for (int i = 0; i < num_points; i++){
      // ROS_INFO("  processing point %d", i);
      Eigen::Vector2f point = Eigen::Vector2f(x1,y1) + Eigen::Vector2f(i * x_inc, i * y_inc);
      // ROS_INFO("  point (%f, %f)", point.x(), point.y());
      // Eigen::Vector2i index = PointToIndex(point);
      // ROS_INFO("  index (%d, %d)", index.x(), index.y());
      visualization::DrawArc(point, 0.05, 0, 2.0 * M_PI, 0xd60b26, global_viz_msg_);
    }
  // ROS_INFO("  processing last point");
  // Eigen::Vector2i index = PointToIndex(Eigen::Vector2f(x2, y2));
  // global_graph(index.x(), index.y()) = 0;
  visualization::DrawArc(Eigen::Vector2f(x2, y2), 0.05, 0, 2.0 * M_PI, 0xd60b26, global_viz_msg_); 
  }
}

void UpdateGlobalGraph (geometry::line2f line) {
  float x1 = line.p0(0,0);
  float y1 = line.p0(1,0);
  float x2 = line.p1(0,0);
  float y2 = line.p1(1,0);

  float length = (Eigen::Vector2f(x1,y1) - Eigen::Vector2f(x2,y2)).norm();
  int num_points = 2 * int(length / resolution);
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
    // visualization::DrawArc(point, 0.2, 0, 2.0 * M_PI, 0xd60b26, global_viz_msg_);
  }
  // ROS_INFO("  processing last point");
  Eigen::Vector2i index = PointToIndex(Eigen::Vector2f(x2, y2));
  global_graph(index.x(), index.y()) = 0;
  // visualization::DrawArc(Eigen::Vector2f(x2, y2), 0.2, 0, 2.0 * M_PI, 0xd60b26, global_viz_msg_);
}

void PrintNeighbors (Node* node){
  ROS_INFO("node id = %s", node->id.c_str());
  visualization::DrawArc(Eigen::Vector2f(node->x, node->y), 0.1, 0, 2.0 * M_PI, 0xeb34d2, global_viz_msg_);
  for (std::string n : node->neighbor_ids){
    ROS_INFO("  neighbor = %s", n.c_str());
    Node* node2 = Nodes[n];
    visualization::DrawArc(Eigen::Vector2f(node2->x, node2->y), 0.1, 0, 2.0 * M_PI, 0xeba534, global_viz_msg_);
  } 
}

void InitializeGlobalGraph(vector_map::VectorMap map) {
  
  // global_graph initialized to all ones prior to this
  int32_t num_map_lines = (int32_t)map.lines.size();

  // iterate through all the map lines in the graph
  for (int32_t j = 0; j < num_map_lines; j++) {
    ROS_INFO("processing line %d", j);
    // set any cell in global_graph to 0 if map line intersects
    // that cell
    UpdateGlobalGraph(map.lines[j]);
  }  // updated global_graph has 1s for every node and 
     // zeros corresponding to walls

  // loops through every i, j in global_graph
  for (int i = 0; i < global_graph.rows(); i++){
    for (int j = 0; j < global_graph.cols(); j++){
      if (global_graph(i,j)==1){
        string id = std::to_string(i) + "," + std::to_string(j);
        Eigen::Vector2f point = IndexToPoint(Eigen::Vector2i(i,j));
        //loop through to create list of neighbors
        std::vector<std::string> neighbor_ids;
        // std::vector<std::string> path;
        Node* n = new Node();
        n->id = id;
        n->x = point.x();
        n->y = point.y();
        // TL  T  TR
        // L   -  R
        // BL  B  BR
        // loops through the adjacent 9 nodes
        for (int a = i - 1; a <= i + 1; a++){
          for (int b = j - 1; b <= j + 1; b++){
            if (((a != i) || (b != j)) // node can't be its own neighbor
                  && ((a >= 0) && (a < global_graph.rows())) // valid first dimension
                  && ((b >= 0) && (b < global_graph.cols()))){ // valid second column
                
                if (global_graph(a,b) == 1){ // a valid neigbor exists
                  string n_id = std::to_string(a) + "," + std::to_string(b);
                  neighbor_ids.push_back(n_id);
                }
            }
          }
        }
        n->neighbor_ids = neighbor_ids;
        Nodes[id] = n;
      }
    }
  }
  
  global_path.push_back("50,50");
  global_path.push_back("51,50");
  global_path.push_back("52,50");
  global_path.push_back("53,51");
  global_path.push_back("53,52");

  // ROS_INFO("Matrix.shape = (%ld, %ld)", global_graph.rows(), global_graph.cols());
  // ROS_INFO("Matrix non-zero size = %f", global_graph.sum());
  ROS_INFO("Nodes.size() = %ld", Nodes.size());
  ROS_INFO("Initialization complete.");
}

void DrawGlobalPath() {
  // ROS_INFO("Printing global_path:");
  Node* prev_node;
  bool first_point = true;
  for (std::string p : global_path){
    // ROS_INFO("path point = %s", p.c_str());
    Node* node = Nodes[p];
    // ROS_INFO("path point loc = (%f, %f)", node.x, node.y);
    visualization::DrawArc(Eigen::Vector2f(node->x, node->y), 0.1, 0, 2.0 * M_PI, 0x3ede12, global_viz_msg_);
    if (!first_point) {
      visualization::DrawLine(Eigen::Vector2f(prev_node->x,prev_node->y),Eigen::Vector2f(node->x,node->y), 0x3ede12, global_viz_msg_);
    }
    first_point = false;
    prev_node = node;
  }
}

// Draw intersection points of line A to B that intersect circle c
// https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
Eigen::Vector2f Navigation::DrawIntersectionPoints(Eigen::Vector2f A, 
                                        Eigen::Vector2f B,
                                        Eigen::Vector2f C,
                                        float r) {
  Eigen::Rotation2Df rot(-robot_angle_);
  float Ax = A.x();
  float Ay = A.y();
  float Bx = B.x();
  float By = B.y();
  float Cx = C.x();
  float Cy = C.y();

  // compute the euclidean distance between A and B
  float LAB = sqrt(pow(Bx-Ax,2) + pow(By-Ay,2));

  // compute the direction vector D from A to B
  float Dx = (Bx-Ax) / LAB;
  float Dy = (By-Ay) / LAB;

  // the equation of the line AB is x = Dx*t + Ax, y = Dy*t + Ay with 0 <= t <= LAB.

  // compute the distance between the points A and E, where
  // E is the point of AB closest the circle center (Cx, Cy)
  float t = Dx * (Cx-Ax) + Dy * (Cy-Ay);    

  // compute the coordinates of the point E
  float Ex = t * Dx + Ax;
  float Ey = t * Dy + Ay;

  // compute the euclidean distance between E and C
  float LEC = sqrt(pow(Ex-Cx,2) + pow(Ey-Cy,2));

  // test if the line intersects the circle
  if (LEC < r) {
      // compute distance from t to circle intersection point
      float dt = sqrt(pow(r,2) - pow(LEC,2));

      // compute first intersection point
      // float Fx = (t-dt) * Dx + Ax;
      // float Fy = (t-dt) * Dy + Ay;
      // visualization::DrawCross(Eigen::Vector2f(Fx, Fy), 0.15, 0x0045cf, global_viz_msg_);

      // compute second intersection point
      float Gx = (t+dt) * Dx + Ax;
      float Gy = (t+dt) * Dy + Ay;
      // ROS_INFO("Far Intersection Point = (%f, %f)", Gx-robot_loc_.x(), Gy-robot_loc_.y());

      visualization::DrawCross(rot * Eigen::Vector2f(Gx-robot_loc_.x(), Gy-robot_loc_.y()), 0.15, 0x0045cf, local_viz_msg_);
      return rot * Eigen::Vector2f(Gx-robot_loc_.x(),Gy-robot_loc_.y());
  } else if( LEC == r ) { // else test if the line is tangent to circle
      // tangent point to circle is E
      // ROS_INFO("Tagent Intersection Point = (%f, %f)", Ex-robot_loc_.x(), Ey-robot_loc_.y());
      visualization::DrawCross(rot * Eigen::Vector2f(Ex - robot_loc_.x(), Ey-robot_loc_.y()), 0.15, 0x0045cf, local_viz_msg_);
      return rot * Eigen::Vector2f(Ex-robot_loc_.x(), Ey-robot_loc_.y());
  } else {
      // line doesn't touch circle
      // ROS_INFO("No intersection");
      return Eigen::Vector2f(0.0,0.0);
  }    
}

// TODO: find intersection of circle around robot with global path, return relative coordinates to robot
Eigen::Vector2f Navigation::Calculate_Local_Target() {
  // given robot_loc_, global_path, and local_target_radius

  float min_distance;
  std::string prev_point;
  bool first_point = true;
  for (std::string p : global_path){
    if(!first_point){
      Node* currNode = Nodes[p];
      Node* prevNode = Nodes[prev_point];
      min_distance = geometry::MinDistanceLineArc(Eigen::Vector2f(prevNode->x,prevNode->y), 
                                               Eigen::Vector2f(currNode->x,currNode->y), 
                                               Eigen::Vector2f(robot_loc_.x(),robot_loc_.y()), 
                                               float(3.0),
                                               float(0.0), 
                                               float(2.0 * M_PI), 
                                               1);
      // ROS_INFO("intersection_point = (%f, %f)", int_point.x(), int_point.y());
      ROS_INFO("currNode = (%f,%f)",currNode->x,currNode->y);
      ROS_INFO("prevNode = (%f,%f)",prevNode->x,prevNode->y);
      ROS_INFO("robot loc = (%f, %f)", robot_loc_.x(), robot_loc_.y());
      ROS_INFO("min_distance = %f", min_distance);
      // return int_point;
    }
    prev_point = p;
    first_point = false;
  }

  // int_point = Vector2f(3.0,0.0);
  // Eigen::Vector2f robot_node = IndexToPoint(PointToIndex(robot_loc_));
  // Eigen::Vector2f target_node = IndexToPoint(PointToIndex(global_target));
  // Eigen::Vector2f int_point = geometry::MinDistanceLineArc(robot_node, target_node, robot_loc_, 0, 2 * M_PI, 1);
  // ROS_INFO("intersection_point not found = (%f, %f)", int_point.x(), int_point.y());
  return Vector2f(3.0,0.0);
  // return int_point;
}

// MinDistanceLineArc(line_point_1,
//                      line_point_2,
//                      circle_center_point,
//                      circle_radius,
//                      circle_start_angle,
//                      circle_end_angle,
//                      const int rotation_sign)

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

void Navigation::PlanSimplePath(){
  global_path.clear();
  Eigen::Vector2i robot_id = PointToIndex(robot_loc_);
  global_path.push_back(to_string(robot_id.x()) + "," + to_string(robot_id.y()));
  Eigen::Vector2i target_id = PointToIndex(global_target);
  global_path.push_back(to_string(target_id.x()) + "," + to_string(target_id.y()));
}

//update global_path
void Navigation::PlanGlobalPath(){
  global_path.clear();
  
  Eigen::Vector2i robot_id = PointToIndex(robot_loc_);
  string robot_name = to_string(robot_id.x()) +  "," + to_string(robot_id.y());
  Node* robot = Nodes[robot_name];

  Eigen::Vector2i target_id = PointToIndex(global_target);
  string target_name = to_string(target_id.x()) +  "," + to_string(target_id.y());
  Node* target = Nodes[target_name];
  
  // Given robot_id, target_id, Nodes, => update global_path
  // to be a std::vector of std::strings of ids, ie. "10,10"
  string message = "Running A* with robot position " + robot_name + " and target " + target_name;
  ROS_INFO("%s", message.c_str());

  std::vector<std::string> result = a_star(robot, target, Nodes);
  /* std::vector<std::string> result = a_star_local_vars(robot, target, 
                                  Nodes, global_graph.rows(), global_graph.cols()); */
  
  message = "Ran A*, got result";
  for (string s : result) {
    message = message + " " + s;
  }
  ROS_INFO("%s", message.c_str());

  global_path.push_back(robot_name);
  // example accessing a node pointer Nodes["10,10"]->.neighbor_ids
  global_path.insert(global_path.end(), result.begin(), result.end());
  global_path.push_back(target_name);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  nav_complete_ = false;
  nav_goal_loc_ = loc;
  nav_goal_angle_ = angle;
  global_target = loc;

  // temporary functionality: add robots current 
  // location and target location to global_path
  // PlanSimplePath();
  PlanGlobalPath();
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

  robot_omega_ = ang_vel;
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
    odom_omega_ = del_angle_ / del_time;
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
  //drive_msg_.velocity = 0.0;
  // if (relative_local_target.x() <= 0.0){
  if (relative_local_target == Eigen::Vector2f(0.0,0.0)){
      drive_msg_.curvature = 0.0;
      drive_msg_.velocity = 0.0;
  }
  // ---------- Visualizations & Terminal Outputs -----------------
  
  DrawPaths(path_planner_->GetPaths());
  visualization::DrawRobot(car_width_, car_length_, rear_axle_offset_, car_safety_margin_front_, car_safety_margin_side_, drive_msg_, local_viz_msg_, collision);
  // visualization::DrawLocalTarget(relative_local_target, local_viz_msg_);
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

  ObstacleAvoid(); //
  // OverlayGlobalGraph();
  // VisualizeMapPoints(map_);
  // DrawTargetNode(global_target);
  // DrawRobotNode(robot_loc_);
  visualization::DrawArc(Vector2f(0.0, 0.0), local_target_radius, 0, 2.0 * M_PI, 0x0045cf, local_viz_msg_);
  // visualization::DrawArc(robot_loc_, 2 * resolution, 0, 2.0 * M_PI, 0x3ede12, global_viz_msg_);
  if (global_target != Eigen::Vector2f(0.0,0.0)){
    DrawGlobalPath();
  }
  // visualization::DrawGlobalTarget(global_target, global_viz_msg_);
  // visualization::DrawLine(robot_loc_, global_target, 0x3ede12, global_viz_msg_);
  
  // Debugging to see neighbors are accurate:
  // Eigen::Vector2i index = PointToIndex(global_target);
  // Node* n = Nodes[to_string(index.x()) + "," + to_string(index.y())];
  // PrintNeighbors(n);


  // ROS_INFO("robot_loc_ = (%f, %f)", robot_loc_.x(), robot_loc_.y());
  // ROS_INFO("distance to global_target = %f",(robot_loc_ - global_target).norm());

  // TODO: find out why the car turns right with no laser observations or manually fix
  
  // ROS_INFO("relative_local_target = (%f, %f)", relative_local_target.x(), relative_local_target.y());
  // robot is at global_target
  if ((global_target == Eigen::Vector2f(0.0,0.0)) || ((robot_loc_ - global_target).norm() <= 2 * resolution)) { // distance to global target
     relative_local_target = Vector2f(0.0, 0.0);
   } else { // robot is not at global target
      
      // robot is straying off global path 
      // if (distance from global path > max_separation) {
      //    // do A* search on global_graph from robot.loc to global_target 
      //    global_path = Plan_Global_Path(robot.loc, global_target, Nodes); 
      //    PlanSimplePath();
      // }
      if (!global_path.empty()){
        Eigen::Vector2f start(Nodes[global_path.front()]->x,Nodes[global_path.front()]->y);
        Eigen::Vector2f end(Nodes[global_path.back()]->x,Nodes[global_path.back()]->y);
        relative_local_target = DrawIntersectionPoints(start, end, robot_loc_, local_target_radius); 
      }
      if(relative_local_target == Eigen::Vector2f(0.0,0.0)){
        //PlanSimplePath();
        PlanGlobalPath();
        DrawGlobalPath();
        //Eigen::Vector2f start(Nodes[global_path.front()]->x,Nodes[global_path.front()]->y);
        //Eigen::Vector2f end(Nodes[global_path.back()]->x,Nodes[global_path.back()]->y);
        //relative_local_target = DrawIntersectionPoints(start, end, robot_loc_, local_target_radius); 
      }
      // relative_local_target = Eigen::Vector2f(2.0, 0.0); //Calculate_Local_Target();
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
