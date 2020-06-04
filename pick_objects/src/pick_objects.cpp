#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

std::vector<float> pick_up_point = {2.8585, 0.30, 1.0};
std::vector<float> drop_off_point = {1.8623, -4.75, 1.0};

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  move_base_msgs::MoveBaseGoal next_goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = pick_up_point[0];
  goal.target_pose.pose.position.y = pick_up_point[1];
  goal.target_pose.pose.orientation.w = pick_up_point[2];

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("The base has moved to the first goal");
    ros::Duration(5).sleep();
    // set up the frame parameters
    next_goal.target_pose.header.frame_id = "map";
    next_goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    next_goal.target_pose.pose.position.x = drop_off_point[0];
    next_goal.target_pose.pose.position.y = drop_off_point[1];
    next_goal.target_pose.pose.orientation.w = drop_off_point[2];

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending second goal now");
    ac.sendGoal(next_goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Hooray, the base moved to the second position");
      ros::Duration(5).sleep();
    }
    else
    {
      ROS_INFO("The base failed to move to the second goal for some reason");
    }
  }
  else
  {
    ROS_INFO("The base failed to move to the first goal for some reason");
  }

  return 0;
}