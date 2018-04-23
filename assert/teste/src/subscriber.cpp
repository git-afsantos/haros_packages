#include <cassert>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include "haros/haros.h"

class Handler
{
public:
    haros::NodeHandle node;
    haros::Subscriber chatter;

    Handler()
    {
        chatter = node.subscribe("chatter", 10, &Handler::callback, this);
    }

    void callback(const std_msgs::Int32::ConstPtr& msg)
    {
        ROS_INFO_STREAM("Received " << msg->data);
        haros::MessageEvent evt = chatter.lastReceive();
        ROS_INFO_STREAM("Last receive has occurred: " << evt.hasOccurred());
        ROS_INFO_STREAM("Last receive has data: " << (evt.msg<std_msgs::Int32>() != NULL));
        if (evt.msg<std_msgs::Int32>())
            ROS_INFO_STREAM("Last receive data: " << evt.msg<std_msgs::Int32>()->data);
        
        //assert(!chatter.lastReceive().hasOccurred() ||
        //        chatter.lastMessage<std_msgs::Int32>()->data < msg->data);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    Handler h;
    ros::spin();
    return 0;
}
