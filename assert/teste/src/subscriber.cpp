#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include "haros/haros.h"

class Handler
{
public:
    haros::NodeHandle node;
    haros::Subscriber chatter;
    haros::Subscriber chatter2;

    Handler()
    {
        chatter = node.subscribe("chatter", 10, &Handler::callback, this);
        chatter2 = node.subscribe("chatter", 10, &Handler::callback2, this);
    }

    void callback(const std_msgs::Int32::ConstPtr& msg)
    {
        //haros::MessageEvent evt = chatter.lastReceive();
        //ROS_INFO_STREAM("Last receive has occurred: " << evt.hasOccurred());
        //ROS_INFO_STREAM("Last receive has data: " << (evt.msg<std_msgs::Int32>() != NULL));
        //if (evt.msg<std_msgs::Int32>())
            //ROS_INFO_STREAM("Last receive data: " << evt.msg<std_msgs::Int32>()->data);
        
        haros::fact(!chatter.lastReceive() ||
                    chatter.lastMessage<std_msgs::Int32>()->data < msg->data);
    }

    void callback2(const std_msgs::Int32::ConstPtr& msg)
    {
        haros::fact(!chatter2.lastReceive() ||
                    chatter2.lastMessage<std_msgs::Int32>()->data < msg->data);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    Handler h;
    ros::spin();
    return 0;
}
