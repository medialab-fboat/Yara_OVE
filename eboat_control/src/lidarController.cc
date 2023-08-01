#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // Process the LIDAR data here
    // For example, you might print the range to the closest detected object:
    ROS_INFO("Closest range: %f", *(min_element(msg->ranges.begin(), msg->ranges.end())));
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "lidar_node");

    // Create a handle to this node
    ros::NodeHandle n;

    // Subscribe to the LIDAR topic
    ros::Subscriber sub = n.subscribe("/eboat/lidar", 1000, lidarCallback);

    // Enter a loop, pumping callbacks
    ros::spin();

    return 0;
}
