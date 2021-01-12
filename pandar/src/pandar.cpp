#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <string>
#include <sstream>
static ros::Publisher g_scan_pub;
static void main_topic_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
    sensor_msgs::PointCloud2 msg = *input;
    msg.header.frame_id = "velodyne";
    g_scan_pub.publish(msg);
    std::cout<<"ok111000!"<<std::endl;
}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pandar");
    ros::NodeHandle n;
    g_scan_pub = n.advertise<sensor_msgs::PointCloud2>("points_raw", 4);
    ros::Subscriber main_topic_sub = n.subscribe<sensor_msgs::PointCloud2>("/pandar", 10000, main_topic_callback);
    std::cout<<"ok!"<<std::endl;
    ros::spin();
    return 0;
}
