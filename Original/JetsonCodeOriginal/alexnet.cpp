#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "alexnet.hpp"
#include <string>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "alexnet");
    ros::NodeHandle nh;           // público
    ros::NodeHandle pnh("~");     // privado

    GpuNetPub img_pub("network_out", "network_view", 1, &nh);

    std::string input_topic;
    pnh.param<std::string>("input_topic",
                           input_topic,
                           "/usb_cam/image_raw");   // por defecto webcam USB
    ROS_INFO_STREAM("AlexNet subscribes to: " << input_topic);

    ros::Subscriber img_sub =
        nh.subscribe(input_topic, 1, &GpuNetPub::msgCallback, &img_pub);

    ROS_INFO("Node Started Successfully");
    ros::spin();
    ROS_INFO("Node Shutting Down");
    return 0;
}
