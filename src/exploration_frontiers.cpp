#include <termios.h>
#include <stdio.h>
#include <ros/ros.h>
#include <vector>

#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include <visualization_msgs/Marker.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// tractament frontiers
#include <boost/pending/disjoint_sets.hpp>
#include <numeric>
#include <algorithm>


using namespace std;

//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> * ac_;
int robot_status_=1; //0: moving, 1: goal reached, 2: couldn't reach goal
tf::TransformListener * listener_;
geometry_msgs::Pose robot_pose_;
nav_msgs::OccupancyGrid map_;
nav_msgs::OccupancyGrid frontier_map_;
int num_map_cells_=0;

// list of all frontiers in the occupancy grid
vector<int> labels_;
int current_label_=0;
unsigned int labeling_threshold_=5;
vector<int> centroid_markers_;

typedef struct
{
  int id;
  unsigned int size; // frontier size
  geometry_msgs::Point center_point;  // position of the center
  int center_cell;
  vector<int> frontier_cells; // points in grid cells
  vector<geometry_msgs::Point> frontier_points; // points in grid coordinates 
} frontier;

vector<frontier> frontiers_;

//////////// Function headers ////////////

bool moveRobot(float x, float y, float yaw);
void move_baseDone(const actionlib::SimpleClientGoalState& state,  const move_base_msgs::MoveBaseResultConstPtr& result);
void move_baseActive();
std_msgs::ColorRGBA colorets(int n);
void publishMarker(uint i, const float & x, const float & y,int color,int type,const float & yaw);
void occupancy_gridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
int left(int point);
int upleft(int point);
int up(int point);
int upright(int point);
int right(int point);
int downright(int point);
int down(int point);
int downleft(int point);
void getStraightPoints(int point, int points[]);
void getDiagonalPoints(int point, int points[]);
void getAdjacentPoints(int point, int points[]);
bool isFrontier(int point);
int floor0(float value);
int point2cell(const geometry_msgs::Point & point);
geometry_msgs::Point cell2point(const int & cell);
bool refreshRobotPosition();
void twoPassLabeling();
void publishLabels(const std::vector<int> & v);
void clearMarkers(const std::vector<int> & v);
void findFrontiers();

// My function declarations
geometry_msgs::Pose get_closest_frontier_to_robot();
tf::Vector3 Point2V3(const geometry_msgs::Point p);

//////////// My functions ////////////

bool isValidPoint(const geometry_msgs::Point & point)
{
  bool valid=true;
  ////////////////////////////////////////////////////////////////////
  // TODO
  ////////////////////////////////////////////////////////////////////
  //check if the point is not an obstacle
  
  
  ////////////////////////////////////////////////////////////////////
  // TODO END
  ////////////////////////////////////////////////////////////////////
  return valid;
}

bool replan()
{
  bool replan = false;
  ////////////////////////////////////////////////////////////////////
  // TODO
  ////////////////////////////////////////////////////////////////////
  // if robot_status_ is 0 dont replan as we are moving
  if ( robot_status_ != 0 )
      replan = true;

  ////////////////////////////////////////////////////////////////////
  // TODO END
  ////////////////////////////////////////////////////////////////////
  return replan;
}

geometry_msgs::Pose decideGoal()
{
  geometry_msgs::Pose g;
  ////////////////////////////////////////////////////////////////////
  // TODO
  ////////////////////////////////////////////////////////////////////
  // get closest frontier
  // or biggest frontier... or a combination
  // robot pose: robot_pose_
  // frontiers: frontiers_
  g = get_closest_frontier_to_robot();

  ////////////////////////////////////////////////////////////////////
  // TODO END
  ////////////////////////////////////////////////////////////////////
  return g;
}

geometry_msgs::Pose get_closest_frontier_to_robot(){
    tfScalar closest_dist = 99999.9;
    geometry_msgs::Pose closest_frontier_pose;
    int closest_id = 0;
    for (const frontier front : frontiers_)
    {
        // if frontier is just one point, ignore
        ROS_INFO_STREAM("Frontier #" << front.id << " has size: " << front.size);
        if (front.size < 4)
            continue;

        tfScalar tmp_dist = tf::tfDistance(Point2V3(front.center_point), Point2V3(robot_pose_.position));
        if (tmp_dist < closest_dist)
        {
            closest_dist = tmp_dist;
            closest_frontier_pose.position = front.center_point;
            closest_id = front.id;

            tf::Quaternion angle_between_points = tf::shortestArcQuat(Point2V3(front.center_point),
                                                                      Point2V3(robot_pose_.position));

            tf::quaternionTFToMsg(angle_between_points, closest_frontier_pose.orientation);
        }
    }
    ROS_INFO_STREAM("Chosen frontier centroid is: " << closest_id << " at :\n" << closest_frontier_pose.position);
    return closest_frontier_pose;
}

tf::Vector3 Point2V3(const geometry_msgs::Point p){
    return tf::Vector3(p.x, p.y, p.z);
}


// Algorithm execution
void execute()
{
  ros::NodeHandle n;
  ros::Subscriber sub2 = n.subscribe("/map", 1, occupancy_gridCallback);
  ros::Publisher  pub = n.advertise<visualization_msgs::Marker>("candidate_marker", 1);
  ros::Publisher  pub_text = n.advertise<visualization_msgs::Marker>("candidate_marker_text", 1);
  ros::Publisher  pub2 = n.advertise<nav_msgs::OccupancyGrid>("frontiers_map", 1);
  ros::Publisher  pub_frontiers = n.advertise<visualization_msgs::Marker>("frontiers", 1);
  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    if(map_.data.size()!=0)
    {
      if(refreshRobotPosition())
      {
        findFrontiers();
        geometry_msgs::Pose target = decideGoal();
        float x = target.position.x;
        float y = target.position.y;
        float th = tf::getYaw(target.orientation);
        if(replan())
          moveRobot(x,y,th);
      }
      else
        ROS_ERROR("execute: couldn't get robot position!");
    }
    else
      ROS_INFO("execute: waiting for first map");

    ros::spinOnce();
    loop_rate.sleep();
  }
}

// MAIN NODE FUNCTIONS 
void spin(){
  ros::spinOnce();
  boost::thread t(boost::bind(&execute));
  ros::spin();
  t.join();
}

// Main (loop)
int main(int argc, char** argv)
{
  ROS_INFO("Exploration Node");
  ros::init(argc, argv, "exploration");
  listener_ = new tf::TransformListener();
  ac_ = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base",true);
  spin();
  delete listener_;
  return 0;
}

//////////// Function definitions ////////////

// move the robot
bool moveRobot(float x, float y, float yaw)
{
  //tell the action client that we want to spin a thread by default
  //MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  if(!ac_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("moveRobot: Waiting for the move_base action server to come up");
    return false;
  }

  //we'll send a goal to the robot
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;

  //Conversion from roll,pitch,yaw to Quaternion, which is how orientations are represented
  tf::quaternionTFToMsg (tf::createQuaternionFromRPY(0.0,0.0,yaw), goal.target_pose.pose.orientation);

  ROS_INFO("moveRobot: Sending Goal: x=%4.2f, y=%4.2f, orientation=%4.2f in frame=%s",
           goal.target_pose.pose.position.x, 
           goal.target_pose.pose.position.y, yaw , 
           goal.target_pose.header.frame_id.c_str());

  ac_->sendGoal(goal,
            boost::bind(&move_baseDone, _1, _2),
            boost::bind(&move_baseActive));
  robot_status_=0; //MOVING
  return true;
}

void move_baseDone(const actionlib::SimpleClientGoalState& state,  const move_base_msgs::MoveBaseResultConstPtr& result)
{
  ROS_DEBUG("move_baseDone");
  if(state==actionlib::SimpleClientGoalState::SUCCEEDED)
    robot_status_=1; //succedeed
  else
    robot_status_=2; //error
}

void move_baseActive()
{
  ROS_DEBUG("move_baseActive");
  robot_status_ = 0; //moving
}

// assigns colours in a kind of rainbow basis
std_msgs::ColorRGBA colorets(int n)
{
  std_msgs::ColorRGBA c;
  c.a = 1.0;
  c.r = sin(n*0.3);
  c.g = sin(n*0.3 + 2*M_PI/3);
  c.b = sin(n*0.3 + 4*M_PI/3);
  return c;
}

// publish a marker to rviz
void publishMarker(uint i, const float & x, const float & y,int color,int type,const float & yaw)
{
  ros::NodeHandle n;
  ros::Publisher  pub = n.advertise<visualization_msgs::Marker>("candidate_marker", 1);
  ros::Publisher  pub_text = n.advertise<visualization_msgs::Marker>("candidate_marker_text", 1);

  visualization_msgs::Marker marker, marker_text;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = marker_text.header.frame_id = "/map";
  marker.header.stamp = marker_text.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "candidates";
  marker_text.ns = "candidates_text";
  marker.id = marker_text.id = i;
  stringstream ss; ss << i; marker_text.text = ss.str();

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  switch(type)
  {
    default:
    case 0:
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.scale.x = marker.scale.y = 0.10; marker.scale.z = 0.5;
      break;
    case 1:
      marker.type = visualization_msgs::Marker::ARROW;
      marker.scale.x = 0.5;
      marker.scale.y = 0.2;
      marker.scale.z = 0.1;
      break;
  }
  marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker_text.scale.x = marker_text.scale.y = 1;
  marker_text.scale.z = 0.5;

  // Set the marker action.  Options are ADD and DELETE
  marker.action = marker_text.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = marker_text.pose.position.x = x;
  marker.pose.position.y = marker_text.pose.position.y = y;
  marker.pose.position.z = 0;
  marker_text.pose.position.z = 0.5;
  tf::quaternionTFToMsg (tf::createQuaternionFromRPY(0.0,0.0,yaw), marker.pose.orientation);
  tf::quaternionTFToMsg (tf::createQuaternionFromRPY(0.0,0.0,yaw), marker_text.pose.orientation);

  // Set the lifetime of the marker
  marker.lifetime = marker_text.lifetime = ros::Duration(0);

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color = marker_text.color = colorets(color);

  // Publish the marker
  pub.publish(marker);
  pub_text.publish(marker_text);
}

// Occupancy Grid Callback
void occupancy_gridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  ROS_INFO("Occupancy Grid data received: resolution %f cells, width %d cells, height %d cells",
  msg->info.resolution, msg->info.width, msg->info.height);
  map_=*msg;
  num_map_cells_=map_.info.width*map_.info.height;
  frontier_map_=*msg;
}

/*
 These functions calculate the index of an adjacent point 
 (left,upleft,up,upright,right,downright,down,downleft) to the given point. 
 If there is no such point (meaning the point would cause the index to be out of bounds), 
 -1 is returned.
 */
int left(int cell){
  // only go left if no index error and if current cell is not already on the left boundary
  if((cell % map_.info.width != 0))
    return cell+1;
  
  return -1;
}
int upleft(int cell){
  if((cell % map_.info.width != 0) && (cell >= (int)map_.info.width))
    return cell-map_.info.width+1;
  
  return -1;
}
int up(int cell){
  if(cell >= (int)map_.info.width)
    return cell-map_.info.width;
  
  return -1;
}
int upright(int cell){
  if((cell >= (int)map_.info.width) && ((cell + 1) % (int)map_.info.width != 0))
    return cell-map_.info.width-1;
  
  return -1;
}
int right(int cell){
  if((cell + 1) % map_.info.width != 0)
    return cell-1;
  
  return -1;
}
int downright(int cell){
  if(((cell + 1) % map_.info.width != 0) && ((cell/map_.info.width) < (map_.info.height-1)))
    return cell+map_.info.width-1;
  
  return -1;
}
int down(int cell){
  if((cell/map_.info.width) < (map_.info.height-1))
    return cell+map_.info.width;
  
  return -1;
}
int downleft(int cell){
  if(((cell/map_.info.width) < (map_.info.height-1)) && (cell % map_.info.width != 0))
    return cell+map_.info.width+1;
  
  return -1;
}
void getStraightPoints(int cell, int cells[]){
  cells[0] = left(cell);
  cells[1] = up(cell);
  cells[2] = right(cell);
  cells[3] = down(cell);
}
void getDiagonalPoints(int cell, int cells[]){
  cells[0] = upleft(cell);
  cells[1] = upright(cell);
  cells[2] = downright(cell);
  cells[3] = downleft(cell);
}
void getAdjacentPoints(int cell, int cells[]){
  cells[0] = left(cell);
  cells[1] = up(cell);
  cells[2] = right(cell);
  cells[3] = down(cell);
  cells[4] = upleft(cell);
  cells[5] = upright(cell);
  cells[6] = downright(cell);
  cells[7] = downleft(cell);
}

// Check if a cell is frontier
bool isFrontier(int cell)
{
  //check if it is free space
  if(map_.data[cell] == 50){ // was 0, changed this values for poseSLAM
    int adjacentPoints[8];
    getAdjacentPoints(cell,adjacentPoints);
    for(int i = 0; i < 8; ++i)
      if(adjacentPoints[i] != -1 && map_.data[adjacentPoints[i]] == 0) // were -1
        return true;
  }
  // If it is obstacle or unknown could not be frontier
  return false;
}

int floor0(float value)
{
  if (value < 0.0)
    return ceil( value );
  else
    return floor( value );
}

int point2cell(const geometry_msgs::Point & point)
{
  int x_cell = floor0((point.x - map_.info.origin.position.x)/map_.info.resolution);
  int y_cell = floor0((point.y - map_.info.origin.position.y)/map_.info.resolution);

  return(x_cell + (y_cell)*map_.info.width);
}

geometry_msgs::Point cell2point(const int & cell)
{
  geometry_msgs::Point point;
  point.x = (cell % map_.info.width)*map_.info.resolution + map_.info.origin.position.x;
  point.y = floor(cell/map_.info.width)*map_.info.resolution + map_.info.origin.position.y;
  return point;
}


// asks TF for the current position of the robot in map coordinates
bool refreshRobotPosition()
{
  tf::StampedTransform transform;
  ros::Time target_time = ros::Time().now();
  std::string source_frame = "/map";
  std::string target_frame = "/base_footprint";
  try
  {
    if(listener_->waitForTransform(source_frame, target_frame, target_time, ros::Duration(5.0)))
        listener_->lookupTransform(source_frame, target_frame, target_time, transform);
    else
    {
      ROS_ERROR("refreshRobotPosition: no transform between frames %s and %s", source_frame.c_str(), target_frame.c_str());
      return false;
    }
  }
  catch(tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    return false;
  }
  robot_pose_.position.x = transform.getOrigin().x();
  robot_pose_.position.y = transform.getOrigin().y();
  robot_pose_.position.z = 0.0;
  robot_pose_.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(transform.getRotation()));
  ROS_DEBUG("base_link|map: x:%fm y:%fm th:%fº", robot_pose_.position.x, robot_pose_.position.y, tf::getYaw(robot_pose_.orientation));
  return true;
}

// Two pass labeling to label frontiers [http://en.wikipedia.org/wiki/Connected-component_labeling]
void twoPassLabeling()
{
  labels_.assign(frontier_map_.data.begin(), frontier_map_.data.end());
  vector<int> neigh_labels;
  vector<int> rank(100);
  vector<int> parent(100);
  boost::disjoint_sets<int*,int*> dj_set(&rank[0], &parent[0]);
  current_label_=0;

  // 1ST PASS: Assign temporary labels to frontiers and establish relationships
  for (uint i = 0; i < frontier_map_.data.size(); i++)
  {
    if( frontier_map_.data[i] != 0)
    {
      neigh_labels.clear();
      // Find 8-connectivity neighbours
      if(upright(i) != -1 && labels_[upright(i)] != 0) neigh_labels.push_back(labels_[upright(i)]);
      if(up(i)      != -1 && labels_[up(i)]      != 0) neigh_labels.push_back(labels_[up(i)]);
      if(upleft(i)  != -1 && labels_[upleft(i)]  != 0) neigh_labels.push_back(labels_[upleft(i)]);
      if(right(i)   != -1 && labels_[right(i)]   != 0) neigh_labels.push_back(labels_[right(i)]);

      if(neigh_labels.empty())                                                  // case: No neighbours
      {
        dj_set.make_set(current_label_);                                        //   create new set of labels
        labels_[i] = current_label_;                                            //   update cell's label
        current_label_++;                                                       //   update label
      }
      else                                                                      // case: With neighbours
      {
        labels_[i] = *std::min_element(neigh_labels.begin(), neigh_labels.end());//   choose minimum label of the neighbours
        for (unsigned int j = 0; j < neigh_labels.size(); ++j)                   //   update neighbours sets
          dj_set.union_set(labels_[i],neigh_labels[j]);                          //   unite sets minimum label with the others
      }
    }
  }

  // 2ND PASS: Assign final label
  dj_set.compress_sets(labels_.begin(), labels_.end());
  // compress sets for efficiency
  for (unsigned int i = 0; i < frontier_map_.data.size(); i++)
    if( labels_[i] != 0)
      labels_[i] = dj_set.find_set(labels_[i]);                                 // relabel each element with the lowest equivalent label

  // for (unsigned int i = 0; i < labels_.size(); i++)
  // {
  //   if( labels_[i] != 0)
  //     std::cout << labels_[i];
  // }
  // std::cout << std::endl;
}

// publishes the frontiers using only a marker
void publishLabels(const std::vector<int> & v)
{
  // From labels to x-y-z-r-g-b points
  // Each label will have a color
  std::vector<geometry_msgs::Point> p;
  std::vector<std_msgs::ColorRGBA> c;
  p.reserve(v.size());
  c.reserve(v.size());
  for (unsigned int i = 0; i < v.size(); ++i)
  {
    if(v[i]!=0)
    {
      geometry_msgs::Point d = cell2point(i);
      p.push_back(d);
      c.push_back(colorets(v[i]));
    }
  }
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "frontiers";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = marker.pose.position.y = 0.0; marker.pose.position.z = 0;
  marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0; marker.pose.orientation.w = 1;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
  marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0;
  marker.lifetime = ros::Duration();
  marker.points = p;
  marker.colors = c;

  ros::NodeHandle n;
  ros::Publisher pub_frontiers = n.advertise<visualization_msgs::Marker>("frontiers", 1);
  pub_frontiers.publish(marker);
}

// clears frontier centroids markers
void clearMarkers(const std::vector<int> & v)
{
  ros::NodeHandle n;
  ros::Publisher  pub = n.advertise<visualization_msgs::Marker>("candidate_marker", 1);
  ros::Publisher  pub_text = n.advertise<visualization_msgs::Marker>("candidate_marker_text", 1);
  visualization_msgs::Marker marker;
  visualization_msgs::Marker marker_text;
  marker.ns = "candidates";
  marker_text.ns = "candidates_text";
  marker.action = visualization_msgs::Marker::DELETE;
  marker_text.action = visualization_msgs::Marker::DELETE;
  for (unsigned int i = 0; i < v.size(); ++i)
  {
    marker.id = marker_text.id = v[i];
    pub.publish(marker);
    pub_text.publish(marker_text);
  }
}

void findFrontiers()
{
  frontier_map_.data.resize(num_map_cells_);
  frontier_map_.data.assign(frontier_map_.data.size(), 0);
  clearMarkers(centroid_markers_);
  centroid_markers_.clear();

  //check for all cells in the occupancy grid whether or not they are frontier cells
  for(unsigned int i = 0; i < frontier_map_.data.size(); ++i)
  {
    if(isFrontier(i)){
      frontier_map_.data[i] = 100;
    }
    else
      frontier_map_.data[i] = 0;
  }

  ros::NodeHandle n;
  ros::Publisher  pub2 = n.advertise<nav_msgs::OccupancyGrid>("frontiers_map", 1);
  pub2.publish(frontier_map_);

  twoPassLabeling();

  // Clear labels and create frontiers
  frontiers_.clear();
  // Search existing labels
  for (unsigned int i = 0; i < labels_.size(); ++i)
  {
    if(labels_[i]!=0)
    {
      int i_label = frontiers_.size();
      for (uint j = 0; j < frontiers_.size(); j++)
      {
        if (frontiers_[j].id == labels_[i])
        {
          i_label = j;
          break;
        }
      }
      frontiers_.resize(std::max(i_label+1, (int)frontiers_.size()));
      frontiers_[i_label].frontier_cells.push_back(i);
      geometry_msgs::Point point_i = cell2point(i);
      frontiers_[i_label].frontier_points.push_back(point_i);
      frontiers_[i_label].id = labels_[i];
    }
  }

  for(unsigned int i = 0; i < frontiers_.size(); ++i)
  {
    frontiers_[i].size = frontiers_[i].frontier_cells.size();
    float sum_x =0, sum_y = 0;
    for (unsigned int j = 0; j < frontiers_[i].size; ++j)
    {
      sum_x += frontiers_[i].frontier_points[j].x;
      sum_y += frontiers_[i].frontier_points[j].y;
    }
    frontiers_[i].center_point.x = sum_x/frontiers_[i].size;
    frontiers_[i].center_point.y = sum_y/frontiers_[i].size;
    frontiers_[i].center_cell = point2cell(frontiers_[i].center_point);
    centroid_markers_.push_back(i);
    publishMarker(i, frontiers_[i].center_point.x, frontiers_[i].center_point.y, i, 0, 0);
  }
  publishLabels(labels_);
}
