#include <termios.h>
#include <stdio.h>
#include <ros/ros.h>

#include "sensor_msgs/LaserScan.h"

#include <visualization_msgs/Marker.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <tf/transform_datatypes.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// HEADERS OF GIVEN FUNCTIONS
// returns:
//    0: canceled by user
//    1: goal reached
//    2: goal not reached
int moveRobot(float x, float y, float yaw);

void publishMarker(uint i, const float & x, const float & y,int color=0,int type=0,const float & yaw=0.0);


// //// FUNCTIONS TO DO ////////////////////////////////////////////////////////

// transform from laser point data (polar) to xy plane (cartesian)
void getXY(int i,float range,float angle_min,float angle_increment,float & x, float & y)
{
  // \TODO Polar to cartesian
  //x = ...;
  //y = ...;
  // \TODO_END
}

// Callback when a scan message is received
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // Scan msg structure: http://www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html
  ROS_DEBUG("LaserScan number %d received: angle min %f rad, max %f rad, increment %f rad, ranges size %u",
    msg->header.seq, msg->angle_min, msg->angle_max, msg->angle_increment, msg->ranges.size());

  float x=0.0;   // [m]
  float y=0.0;   // [m]
  float yaw=0.0; // [rad]

  // \TODO: Decide a goal (x,y,yaw)
  // ...
  // \TODO_END

  moveRobot(x,y,yaw);
  
  // // Some examples of function usage
  // // \EXAMPLE: Publish a marker in the midpoint of the laser
  // int midpoint_i = msg->ranges.size()/2;
  // getXY(midpoint_i,msg->ranges[midpoint_i],msg->angle_min,msg->angle_increment,x,y);
  // publishMarker(0,x,y,0,0);
  
  // // \EXAMPLE: Go through the laser points
  // for(uint i=1;i<msg->ranges.size();i++)
  // {
  //   // Laser Point i
  //   ROS_INFO("index %i",i);
  // }

  // // \EXAMPLE: Set goal to 1m forward
  // ROS_INFO("Going to goal");
  // moveRobot(1,0,0);
  
}

// //// GIVEN FUNCTIONS ////////////////////////////////////////////////////////

// Main (loop)
int main(int argc, char** argv)
{
  ROS_INFO("Find Goal Node");
  ros::init(argc, argv, "findgoal");

  // init a node handler
  ros::NodeHandle n;

  // subscribe to laserscan topic
  ros::Subscriber sub = n.subscribe("scan", 1, scanCallback);
  ros::Publisher  pub = n.advertise<visualization_msgs::Marker>("candidate_marker", 3);

  ROS_INFO("----");
  ROS_INFO("Press Ctrl+C (+Enter) to exit the program");
  ROS_INFO("----");
  ROS_INFO("Waiting for a Laser Scan to arrive...");

  // make the node loop
  ros::spin();

  return 0;
}

// move the robot
int moveRobot(float x, float y, float yaw)
{
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  ROS_INFO("Do you want to move the robot? (y/n) or Exit the program(E)");
  char c = getchar();
  if(c=='E'){
    exit(0);
  }else if(c=='y')
  {
    // delete enter character
    getchar();   
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
      //ROS_INFO("Waiting for the move_base action server to come up");
    }

    //we'll send a goal to the robot
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "camera_depth_frame";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x+0.1;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = -0.28;

    //Conversion from roll,pitch,yaw to Quaternion, which is how orientations are represented
    tf::quaternionTFToMsg (tf::createQuaternionFromRPY(0.0,0.0,yaw),  goal.target_pose.pose.orientation);

    ROS_INFO("Sending Goal: x=%4.2f, y=%4.2f, orientation=%4.2f in frame=%s",goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, yaw , goal.target_pose.header.frame_id.c_str());
    ac.sendGoal(goal);

    ac.waitForResult(ros::Duration(15)); //wait N seconds
    
    //ROS_INFO("Current State: %s\n", ac.getState().toString().c_str());
    
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Hooray, the base reached the goal!");
      return 1;
    }
    else
    {
      ROS_INFO("Oops, the base failed to reach the goal for some reason!");
      return 2;
    }
  }
  else{
    return 0;
  }
}

// publish a marker to rviz
void publishMarker(uint i, const float & x, const float & y,int color,int type,const float & yaw){

    ROS_DEBUG("Publishing marker: id = %i\npose: %f, %f, %f", i, x, y, yaw);
    ros::NodeHandle n;
    ros::Publisher  pub = n.advertise<visualization_msgs::Marker>("candidate_marker", 1);

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "camera_depth_frame";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "candidates";
    marker.id = i;

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
        marker.scale.x = 0.3; marker.scale.y = marker.scale.z = 0.05;
        break;
    }   

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = -0.28;
    tf::quaternionTFToMsg (tf::createQuaternionFromRPY(0.0,0.0,yaw), marker.pose.orientation);

   // Set the lifetime of the marker
    marker.lifetime = ros::Duration(15);
    
    // Set the color -- be sure to set alpha to something non-zero!
    float r,g,b;
    color=color%7;
    switch(color)
    {
      default:
      case 0: r=0; g=0; b=1; break;
      case 1: r=0; g=1; b=0; break;
      case 2: r=1; g=0; b=0; break;
      case 3: r=1; g=0; b=1; break;
      case 4: r=0; g=1; b=1; break;
      case 5: r=1; g=1; b=0; break;
      case 6: r=0; g=0.5; b=1; break;
    }
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1;

    // Publish the marker
    pub.publish(marker);
}
