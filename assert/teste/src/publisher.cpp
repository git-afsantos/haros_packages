#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include "haros/haros.h"

void publishCounter(haros::Publisher<std_msgs::Int32> &pub, int counter) {
    //ROS_INFO_STREAM("Publishing " << counter);
    if (counter % 5 == 0)
    {
      pub.bookmark("5n");
    }
    std_msgs::Int32 msg;
    msg.data = counter;
    pub.publish(msg);
    ROS_ASSERT(pub.lastMessage() && pub.lastMessage()->data == msg.data);
    ROS_ASSERT(counter < 5 ||
        (pub.lastMessage("5n") && pub.lastMessage("5n")->data - msg.data > -5));
}

void publishHello(haros::Publisher<std_msgs::String> &pub) {
    //ROS_INFO_STREAM("Publishing " << counter);
    std_msgs::String msg;
    msg.data = "Hello world!";
    pub.publish(msg);
    ROS_ASSERT(pub.lastMessage() && pub.lastMessage()->data == msg.data);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    haros::Publisher<std_msgs::Int32> pub(n.advertise<std_msgs::Int32>("chatter", 10));
    haros::Publisher<std_msgs::String> str_pub(n.advertise<std_msgs::String>("chatter_string", 10));
    ros::Rate loop_rate(10);
    int counter = 1;
    ROS_ASSERT(!pub.lastPublish());
    ROS_ASSERT(!str_pub.lastPublish());
    while (ros::ok()) {
        publishCounter(pub, counter);
        publishHello(str_pub);
        ++counter;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
