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

#include <numeric>
#include <algorithm>

using namespace std;

//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> * ac_;
int robot_status_=1; //0: moving, 1: goal reached, 2: couldn't reach goal
tf::TransformListener * listener_;
geometry_msgs::Pose robot_pose_;
nav_msgs::OccupancyGrid map_;
int num_map_cells_=0;

//////////// Function headers ////////////

bool moveRobot(float x, float y, float yaw);
void move_baseDone(const actionlib::SimpleClientGoalState& state,  const move_base_msgs::MoveBaseResultConstPtr& result);
void move_baseActive();
std_msgs::ColorRGBA colorets(int n);
void publishMarker(uint i, const float & x, const float & y,int color,int type,const float & yaw);
void occupancy_gridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
int floor0(float value);
int point2cell(const geometry_msgs::Point & point);
geometry_msgs::Point cell2point(const int & cell);
bool refreshRobotPosition();
//////////// My functions ////////////

geometry_msgs::Pose getRandomPose()
{
  geometry_msgs::Pose goal_pose;
  ////////////////////////////////////////////////////////////////////
  // TODO
  ////////////////////////////////////////////////////////////////////
  
  
  
  ////////////////////////////////////////////////////////////////////
  // TODO END
  ////////////////////////////////////////////////////////////////////
  return goal_pose;
}

bool isValidPoint(const geometry_msgs::Point & point)
{
  bool valid=true;
  ////////////////////////////////////////////////////////////////////
  // TODO
  ////////////////////////////////////////////////////////////////////
  
  
  
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
  
  
  
  ////////////////////////////////////////////////////////////////////
  // TODO END
  ////////////////////////////////////////////////////////////////////
  return replan;
}


// Algorithm execution
void execute()
{
  ros::NodeHandle n;
  ros::Subscriber sub2 = n.subscribe("/map", 1, occupancy_gridCallback);
  ros::Publisher  pub = n.advertise<visualization_msgs::Marker>("candidate_marker", 1);
  ros::Publisher  pub_text = n.advertise<visualization_msgs::Marker>("candidate_marker_text", 1);
  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    if(map_.data.size()!=0)
    {
      geometry_msgs::Pose goal_pose;
      do
      {
        goal_pose = getRandomPose();
      }
      while(!isValidPoint(goal_pose.position));
      float x = goal_pose.position.x;
      float y = goal_pose.position.y;
      float th = tf::getYaw(goal_pose.orientation);
      if(replan())
      {
        publishMarker(0,x,y,6,1,th);
        moveRobot(x,y,th);
      }
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