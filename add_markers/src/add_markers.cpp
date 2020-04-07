
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/UInt8.h>


//initialize global variable keeping track of the current search phase
uint8_t cycle=0;

// Define destination subscriber callback to set the search phase to the received value
void pickupDropoffCallback(const std_msgs::UInt8::ConstPtr& msg)
{
  ROS_INFO("Received message!");
  cycle = msg->data;
  return;
}

int main( int argc, char** argv )
{  
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(5);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber location_sub = n.subscribe("robot_location", 1, pickupDropoffCallback);
  uint32_t np= location_sub.getNumPublishers();
  bool done = false;
  bool cond = false;

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "object";
  marker.id = 0;

  marker.type = shape;
  marker.lifetime = ros::Duration();
 
 // Search phase based on receipt of message from pick_objects_node
  while (ros::ok())
  {

    switch (cycle)
    {
      // Currently moving towards first goal, display this goal marker
      case 0:
        ROS_INFO_ONCE("Added object for PICK-UP.");     
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 3.5;
        marker.pose.position.y = 7.0;
        marker.pose.orientation.w = 1.0;
        break;

      // Reached first goal, delete the goal marker
      case 1:
        ROS_INFO_ONCE("Reached Pickup Point. Picked up Object");
        marker.action = visualization_msgs::Marker::DELETE;
        cycle += 1;
        break;

      // Waiting to reach second goal...
      case 2:
        ROS_INFO_ONCE("Waiting for delivery.");
        marker.action = visualization_msgs::Marker::DELETE;
        break;

      // Reached second goal, display marker here
      case 3: 
        ROS_INFO_ONCE("Placing object at drop-off location");   
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = -3.5;
        marker.pose.position.y = 7.0;
        marker.pose.orientation.w = 1.0;
        done = true;
        break;
    }

    // Set the scale and color of the marker -- 1x1x1 here means 1m on a side and -- be sure to set alpha to something non-zero!
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;

    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    //publish the marker
    marker_pub.publish(marker);

    // if last marker published and noted as done exit
    if (done)
      return 0;  
    
    //Do this every cycle to ensure the subscriber receives the message
    ros::spinOnce();
    r.sleep();

  }

  return 0;
}

