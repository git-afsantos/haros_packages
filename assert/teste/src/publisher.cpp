#include <cassert>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include "haros/haros.h"

void publishHello(ros::Publisher &pub, int counter) {
    //ROS_INFO_STREAM("Publishing " << counter);
    std_msgs::Int32 msg;
    msg.data = counter;
    pub.publish(msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "talker");
    haros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::Int32>("chatter", 10);
    ros::Rate loop_rate(10);
    int counter = 1;
    while (ros::ok()) {
        publishHello(pub, counter);
        ++counter;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
