#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "upc_mobrob");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot
  goal.target_pose.header.frame_id = "base_footprint"; //fixed frame 'base_footprint'
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.position.y = 0.0;
  //goal.target_pose.pose.orientation; // This is a Quaternion, some stuff is needed here: 
  //yaw: rotation along the z axis, in radians
  double yaw=3.14;
  //Conversion from roll,pitch,yaw to Quaternion, which is how orientations are represented
  tf::quaternionTFToMsg (tf::createQuaternionFromRPY(0.0,0.0,yaw),  goal.target_pose.pose.orientation);

  ROS_INFO("Goal: x=%4.2f, y=%4.2f, orientation=%4.2f in frame=%s", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, yaw , goal.target_pose.header.frame_id.c_str());

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ROS_INFO("Sending goal!");
  ac.sendGoal(goal);

  ac.waitForResult();
  //ac.waitForResult(ros::Duration(20.0)); //wait N seconds

  ROS_INFO("Current State: %s\n", ac.getState().toString().c_str());
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base reached the goal!");
  else
    ROS_INFO("Oops, the base failed to reach the goal for some reason!");

  return 0;
}

//SimpleActionClient C++ API
//http://www.ros.org/doc/api/actionlib/html/classactionlib_1_1SimpleActionClient.html
