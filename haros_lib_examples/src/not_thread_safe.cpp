#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include "haros/haros.h"

int shared_value = 0;
haros::Publisher<std_msgs::Int32> g_pub;
haros::Subscriber<std_msgs::Int32> g_sub;

void callback(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO_STREAM("Received " << msg->data);
    shared_value = msg->data;
}

void publisherLoop()
{
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        std_msgs::Int32 msg;
        msg.data = shared_value + 1;

        ROS_ASSERT(!g_sub.lastReceive() || g_sub.lastMessage()->data == shared_value);
        g_pub.publish(msg);

        loop_rate.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "not_thread_safe");
    ros::NodeHandle nh;
    g_pub = haros::Publisher<std_msgs::Int32>(nh, "values", 10);
    g_sub = haros::Subscriber<std_msgs::Int32>(nh, "values", 10, callback);
    boost::thread thread(publisherLoop);
    ros::spin();
    thread.join();
    return 0;
}
