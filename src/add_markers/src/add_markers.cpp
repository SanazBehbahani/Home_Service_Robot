#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

using namespace std;

// Define pick up and drop off coordinates
double pick[2] = {-2.0, 0.5};
double place[2] = {2.0, 0.0};

double robot_x, robot_y, marker_x, marker_y;
double position_error = 5;

nav_msgs::Odometry pose_msg;
visualization_msgs::Marker marker;

uint8_t state = 2;

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    pose_msg = *odom_msg;
    robot_x = pose_msg.pose.pose.position.x;
    robot_y = pose_msg.pose.pose.position.y;
}

void position_error_check()
{
    position_error = pow (robot_x - marker_x, 2) + pow (robot_y - marker_y, 2);
    ROS_INFO("The position error is, %5.2f", position_error);
    ROS_INFO("The robot position is, %5.2f, %5.2f", robot_x, robot_y);
}

void add_marker(double xMarker, double yMarker, bool ToDo)
{
    // Set the frame ID and timestamp
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker. This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the market type. Initially this is CUBE, and cycles between taht and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE; 

    if (ToDo)
    {
	// Set the marker action. Options are ADD, DELETE, and new in ROS Indige: 3 (DELETEALL)
    	marker.action = visualization_msgs::Marker::ADD;
    }
    else
    {
	marker.action = visualization_msgs::Marker::DELETE;
    }
    // Set the pose of the marker. This is a full 6 DOF pose relative to the frame/time specified in the header. 
    marker.pose.position.x = xMarker;
    marker.pose.position.y = yMarker;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1 meter on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    marker_x = marker.pose.position.x;
    marker_y = marker.pose.position.y;
}


int main( int argc, char** argv )
{
    ros::init (argc, argv, "add_markers");
    ros::NodeHandle n;
    ros::Rate r(1);
    bool endMarker = false;
    bool initialMarker = true;
    bool pickReached = false;
    bool placeReached = false;
 
    ros::Subscriber odom_subscriber;
    odom_subscriber = n.subscribe("/odom", 10, odom_callback);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    
    while (ros::ok())
    {
	if(!pickReached)
	{
	    ROS_INFO_ONCE ("Adding object to be picked.");
	    add_marker(pick[0], pick[1], true);
	    while (marker_pub.getNumSubscribers() < 1)
	    {
		if (!ros::ok())
		{
		    return 0;
		}
	   	ROS_WARN_ONCE("Please create a subscriber to the marker.");
		sleep(1);
	    }
	    marker_pub.publish(marker);
	}

	position_error_check();

	if (position_error < 0.1 && !pickReached)
	{
	    pickReached = true;
	    std::cout << "Reached the pick position" << endl;
	    std::cout << "Wait for 5 seconds" << endl;
	    std::cout << "Picking up the object" << endl;
	    ros::Duration(5.0).sleep();
	    std::cout << "Go to drop off position" << endl;
	    // Reset position error
	    position_error = 5.0;
	}

	if(pickReached && !placeReached)
	{
	    add_marker (pick[0], pick[1], false);
	    marker_pub.publish(marker);

	    add_marker (place[0], place[1], true);

	    while (marker_pub.getNumSubscribers() < 1)
	    {
		if (!ros::ok())
		{
		    return 0;
		}
	   	ROS_WARN_ONCE("Please create a subscriber to the marker.");
		sleep(1);
	    }
	    marker_pub.publish(marker);
	}

	if (position_error < 0.1 && placeReached)
	{
	    pickReached = true;
	    std::cout << "Reached the place position" << endl;
	    std::cout << "Wait for 5 seconds" << endl;
	    std::cout << "Placing the object" << endl;
	    ros::Duration(5.0).sleep();
	    std::cout << "Task completed!!" << endl;
	}

	ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
    return 0;
}

