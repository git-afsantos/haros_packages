#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include "haros/haros.h"

void publishCounter(ros::Publisher &pub, int counter) {
    //ROS_INFO_STREAM("Publishing " << counter);
    std_msgs::Int32 msg;
    msg.data = counter;
    pub.publish(msg);
}

void publishHello(ros::Publisher &pub) {
    //ROS_INFO_STREAM("Publishing " << counter);
    std_msgs::String msg;
    msg.data = "Hello world!";
    pub.publish(msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::Int32>("chatter", 10);
    ros::Publisher str_pub = n.advertise<std_msgs::String>("chatter_string", 10);
    ros::Rate loop_rate(10);
    int counter = 1;
    while (ros::ok()) {
        publishCounter(pub, counter);
        publishHello(str_pub);
        ++counter;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
