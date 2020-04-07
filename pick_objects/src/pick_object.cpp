#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/UInt8.h"

//Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){

  //Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_object"); 
  ros::NodeHandle n; 

  //Robot Location Publisher subscribed by add_markers node
  ros::Publisher location_pub = n.advertise<std_msgs::UInt8>("robot_location", 10); 

  //Tell the action client that we want to spin a thread by default| Wait 5 sec for move_base action server to come up
  MoveBaseClient ac("move_base", true);
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  //Initialize move_base_msg for goal position|std_msgs to publish robot_location
  move_base_msgs::MoveBaseGoal goal;
  std_msgs::UInt8 msg;

  //Set up the frame parameters
  goal.target_pose.header.frame_id = "map"; 
  goal.target_pose.header.stamp = ros::Time::now();
  //Define a pick-up position and orientation for the robot to reach 
  goal.target_pose.pose.position.x = 3.5;
  goal.target_pose.pose.position.y = 7.0;
  goal.target_pose.pose.orientation.w = 3.0;
  //Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending PICK-UP goal point.");
  ac.sendGoal(goal);
  //Wait an infinite time for the results
  ac.waitForResult();
  //Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Hooray, the robot reached pickup point.");
      msg.data= 1;
      location_pub.publish(msg);
      ROS_INFO("The robot successfully picked the object. Now moving toward dropoff location.");
    }
  else{
      ROS_INFO("The robot failed to reach pickup point");
    }


  ros::Duration(5.0).sleep(); // Sleep for five seconds

  //Define a drop-off position and orientation for the robot to reach 
  goal.target_pose.pose.position.x = -3.5;
  goal.target_pose.pose.position.y = 7.0;
  goal.target_pose.pose.orientation.w = 1.0;
  //Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending dropoff goal");
  ac.sendGoal(goal);
  //Wait an infinite time for the results
  ac.waitForResult();
  //Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Hooray, the robot reached dropoff point");
    msg.data= 3;
    location_pub.publish(msg);
    ROS_INFO("The robot successfully placed the object.");
  }
  else{
     ROS_INFO("The robot failed to reach drop point");  
  }

/* //Additional Goal - HOME POSITION
  ros::Duration(5.0).sleep(); // Sleep for two seconds

  //Define a home position and orientation for the robot to reach 
  goal.target_pose.pose.position.x = 0.0;
  goal.target_pose.pose.position.y = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;
  //Send the goal position and orientation for the robot to reach
  ROS_INFO("Moving toward Home Position");
  ac.sendGoal(goal);
  //Wait an infinite time for the results
  ac.waitForResult();
  //Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("TASK COMPLETED.");
  }
  else{
     ROS_INFO("The robot failed to reach HOME POSITION.");  
  }

*/

  return 0;
}