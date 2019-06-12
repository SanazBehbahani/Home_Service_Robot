#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Define pick up and drop off coordinates
double pick[2] = {-2.0, 0.5};
double place[2] = {1.0, -2.0};

int main (int argc, char** argv){
    // Initialize the simple_navigation_goals node
    ros::init(argc, argv, "pick_objects");
    ros::NodeHandle n;
    
    // tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Position publisher
    ros::Publisher pos_pub = n.advertise<std_msgs::UInt8>("/robot_position", 1);

    // Wait 5 sec for move_base action server to come up
    while (!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    
    move_base_msgs::MoveBaseGoal goal;

    // Set up the frame parameters
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = pick[0];
    goal.target_pose.pose.position.y = pick[1];
    goal.target_pose.pose.orientation.w = 1.57;

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending pick goal");
    ac.sendGoal(goal);

    // Wait an infinite time for the result
    ac.waitForResult();

    // Check if the robot reached its goal
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
	ROS_INFO("Horray, the robot picked up the object.");
	std_msgs::UInt8 msg1;
	msg1.data = 0;
	ROS_INFO("The message is %d", msg1.data);
	pos_pub.publish(msg1);
    }
    else
	ROS_INFO("The robot failed to pick up the object.");

    sleep(5);

    move_base_msgs::MoveBaseGoal place_goal;

    // Set up the frame parameters
    place_goal.target_pose.header.frame_id = "map";
    place_goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    place_goal.target_pose.pose.position.x = place[0];
    place_goal.target_pose.pose.position.y = place[1];
    place_goal.target_pose.pose.orientation.w = 0;

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending place goal");
    ac.sendGoal(place_goal);

    // Wait an infinite time for the result
    ac.waitForResult();

    // Check if the robot reached its goal
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
	ROS_INFO("Horray, the robot placed the object.");
	std_msgs::UInt8 msg2;
	msg2.data = 0;
	ROS_INFO("The message is %d", msg2.data);
	pos_pub.publish(msg2);
    }
    else
	ROS_INFO("The robot failed to place the object.");


    return 0;
}
