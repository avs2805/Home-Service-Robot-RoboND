#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <vector>
// pick_up {x, y, w}
std::vector<float> pick_up_point = {2.8585, 0.30, 1.0};
std::vector<float> drop_off_point = {1.8623, -4.75, 1.0};
float threshold = 0.05;
bool reachedX, reachedY, reachedW, reachedPickup, reachedDropoff = false;

bool checkDist(float a, float b, float thr)
{
  if (sqrt(abs(a * a - b * b)) < thr)
  {
    return true;
  }
  else
    return false;
}

bool targetReached(const nav_msgs::Odometry::ConstPtr &odom_msg, std::vector<float> &targetpose)
{
  reachedX = checkDist(odom_msg->pose.pose.position.x, targetpose[0], threshold);
  reachedY = checkDist(odom_msg->pose.pose.position.y, targetpose[1], threshold);
  return (reachedX && reachedY);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  // ROS_INFO("Seq: [%d]", msg->header.seq);
  // ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  // ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  // ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x, msg->twist.twist.angular.z);

  // check if the bot has reached the pickup point
  reachedPickup = targetReached(msg, pick_up_point);

  // check if the bot has reached the dropoff point
  reachedDropoff = targetReached(msg, drop_off_point);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_subscriber = n.subscribe("odom", 100, odomCallback);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    visualization_msgs::Marker pickup_marker;
    visualization_msgs::Marker dropoff_marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    pickup_marker.header.frame_id = "map";
    pickup_marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    pickup_marker.ns = "basic_shapes";
    pickup_marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    pickup_marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    pickup_marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    pickup_marker.pose.position.x = pick_up_point[0];
    pickup_marker.pose.position.y = pick_up_point[1];
    pickup_marker.pose.position.z = 0;
    pickup_marker.pose.orientation.x = 0.0;
    pickup_marker.pose.orientation.y = 0.0;
    pickup_marker.pose.orientation.z = 0.0;
    pickup_marker.pose.orientation.w = pick_up_point[2];

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    pickup_marker.scale.x = 0.5;
    pickup_marker.scale.y = 0.5;
    pickup_marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    pickup_marker.color.r = 0.0f;
    pickup_marker.color.g = 0.0f;
    pickup_marker.color.b = 1.0f;
    pickup_marker.color.a = 1.0;

    pickup_marker.lifetime = ros::Duration();

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    dropoff_marker.header.frame_id = "map";
    dropoff_marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    dropoff_marker.ns = "basic_shapes";
    dropoff_marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    dropoff_marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    dropoff_marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    dropoff_marker.pose.position.x = drop_off_point[0];
    dropoff_marker.pose.position.y = drop_off_point[1];
    dropoff_marker.pose.position.z = 0;
    dropoff_marker.pose.orientation.x = 0.0;
    dropoff_marker.pose.orientation.y = 0.0;
    dropoff_marker.pose.orientation.z = 0.0;
    dropoff_marker.pose.orientation.w = drop_off_point[2];

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    dropoff_marker.scale.x = 0.5;
    dropoff_marker.scale.y = 0.5;
    dropoff_marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    dropoff_marker.color.r = 0.0f;
    dropoff_marker.color.g = 0.0f;
    dropoff_marker.color.b = 1.0f;
    dropoff_marker.color.a = 1.0;

    dropoff_marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(pickup_marker);
    ROS_INFO("Heading to the Pickup Marker!");
    // publish till robot has picked up the object

    while (!reachedPickup)
    {
      ros::spinOnce();
      // ROS_INFO("inside reachedpickup while loop");
      r.sleep();
    }

    ROS_INFO("Reached Pickup Point. Picking Up Object.\n");
    ROS_INFO("Object Picked up, Hiding Pickup Marker");

    pickup_marker.action = visualization_msgs::Marker::DELETE;
    ROS_INFO("sleeping for 5 seconds\n");
    ros::Duration(5).sleep();

    marker_pub.publish(dropoff_marker);
    ROS_INFO("Heading to the Pickup Marker!");

    while (!reachedDropoff)
    {
      ros::spinOnce();
      // ROS_INFO("inside reachedpickup while loop");
      r.sleep();
    }
    // Set the scale of the marker -- 0,0,0 to hide after dropoff
    dropoff_marker.scale.x = 0.0;
    dropoff_marker.scale.y = 0.0;
    dropoff_marker.scale.z = 0.0;
    ros::Duration(5).sleep();
    ROS_INFO("Object Dropped off!");
    pickup_marker.action = visualization_msgs::Marker::DELETE;
  }
}