#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/UInt8.h>


// Define pick up and drop off coordinates
double pick[2] = {-2.0, 0.5};
double place[2] = {2.0, 0.0};

visualization_msgs::Marker marker;
uint8_t state = 2;

void robot_status (const std_msgs::UInt8::ConstPtr& msg)
{
    state = msg->data;
    return;
}

void add_marker(double xpos, double ypos, bool ToDo)
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
    marker.pose.position.x = xpos;
    marker.pose.position.y = ypos;
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

}


int main( int argc, char** argv )
{
    ros::init (argc, argv, "add_markers");
    ros::NodeHandle n;
    ros::Rate r(1);
    bool initialPosition = false;
    bool finalPosition = false; 
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;
    
    while (ros::ok())
    {
	if(!initialPosition)
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

	if(state == 0 && !initialPosition)
	{
	    initialPosition = true;
	    add_marker(pick[0], pick[1], false);
	    marker_pub.publish(marker);
	    ros::Duration(5.0).sleep();
	}

	if(initialPosition && !finalPosition)
	{
	    while (marker_pub.getNumSubscribers() < 1)
	    {
		if (!ros::ok())
		{
		    return 0;
		}
	   	ROS_WARN_ONCE("Please create a subscriber to the marker.");
		sleep(1);
	    }
	}

	if(state == 1 && !finalPosition)
	{
	    add_marker(place[0], place[1], true);
	    marker_pub.publish(marker);
	    ros::Duration(5.0).sleep();
	    return 0;
	}

	ros::spinOnce();
    }
}

