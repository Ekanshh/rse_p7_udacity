

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>


int main( int argc, char** argv )
{
  ros::init(argc, argv, "test_addmarkers");
  ros::NodeHandle n;
  ros::Rate r(5);
  ros::Publisher marker_publisher = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  uint8_t value = 0;
  bool done = false;

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = value;
  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  marker.lifetime = ros::Duration();


  while (ros::ok())
  {


    // Cycle between different shapes
    switch (value)
    {
      case 0:
        ROS_INFO_ONCE("Added object for PICK-UP.");     
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 3.5;
        marker.pose.position.y = 7.0;
        marker.pose.orientation.w = 1.0;
        value += 1;
        std::cout << "1";
        break;

      case 1: 
        ROS_INFO_ONCE("Counting for 5 Seconds");       
        std::cout << "2";
        value += 1;
        break;

      case 2: 
        ros::Duration(5).sleep();
        ROS_INFO_ONCE("Placing object at drop-off location");  
        marker.action = visualization_msgs::Marker::ADD; 
        marker.pose.position.x = -3.5;
        marker.pose.position.y = 7.0;
        marker.pose.orientation.w = 1.0;
        done= true;
        break;  
    }

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;


    // Set the color -- be sure to set alpha to something non-zero!

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    // Publish the marker
    marker_publisher.publish(marker);
    std::cout << "Published";
    
    if (done){
      return 0;
    }

    r.sleep();
  }
  return 0;
}
