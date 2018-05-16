#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include "haros/haros.h"

bool even(const std_msgs::Int32& msg)
{
  return msg.data % 2 == 0;
}

bool tens(const std_msgs::Int32& msg)
{
  return msg.data % 10 == 0;
}

void publishCounter(haros::Publisher<std_msgs::Int32> &pub, int counter) {
    //ROS_INFO_STREAM("Publishing " << counter);
    std_msgs::Int32 msg;
    msg.data = counter;
    pub.publish(msg);
    ROS_ASSERT(pub.lastMessage() && pub.lastMessage()->data == msg.data);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    haros::Publisher<std_msgs::Int32> pub(n, "chatter", 10);
    haros::Publisher<std_msgs::Int32> g_pub(n, "global_chatter", 10);
    pub.recordIf(even, "2n");
    g_pub.recordIf(tens, "10n");
    ros::Rate loop_rate(10);
    int counter = 1;
    ROS_ASSERT(!pub.lastPublish());
    ROS_ASSERT(!g_pub.lastPublish());
    while (ros::ok()) {
        publishCounter(pub, counter);
        publishCounter(g_pub, counter);

        ROS_ASSERT(!pub.lastPublish("2n") || pub.lastMessage("2n")->data <= counter);
        ROS_ASSERT(!g_pub.lastPublish("10n") || g_pub.lastMessage("10n")->data <= counter);

        ++counter;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
