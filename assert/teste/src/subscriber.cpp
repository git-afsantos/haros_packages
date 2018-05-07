#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include "haros/haros.h"

class Handler
{
public:
    ros::NodeHandle node;
    haros::Subscriber<std_msgs::Int32> chatter;
    haros::Subscriber<std_msgs::Int32> chatter2;
    haros::Subscriber<std_msgs::String> str_chatter;

    Handler()
    {
        chatter = haros::Subscriber<std_msgs::Int32>(node.subscribe("chatter", 10, &Handler::callback, this));
        chatter2 = haros::Subscriber<std_msgs::Int32>(node.subscribe("chatter", 10, &Handler::callback2, this));
        str_chatter = haros::Subscriber<std_msgs::String>(node.subscribe("chatter_string", 10, &Handler::callback_str, this));
    }

    void callback(const std_msgs::Int32::ConstPtr& msg)
    {
        if (msg->data % 5 == 0)
            chatter.bookmark("5n");
        haros::MessageEvent<std_msgs::Int32> evt = chatter.lastReceive();
        ROS_INFO_STREAM("Last receive has occurred: " << evt.hasOccurred());
        if (evt.msg)
            ROS_INFO_STREAM("Last receive data: " << evt.msg->data);
        else
            ROS_INFO("Last receive has no data.");

        evt = chatter.lastReceive("5n");
        ROS_INFO_STREAM("Last 5N receive has occurred: " << evt.hasOccurred());
        if (evt.msg)
            ROS_INFO_STREAM("Last 5N receive data: " << evt.msg->data);
        else
            ROS_INFO("Last 5N receive has no data.");
        
        ROS_ASSERT(!chatter.lastReceive()
                   || chatter.lastMessage()->data < msg->data);
        ROS_ASSERT(!chatter.lastReceive("5n")
                   || chatter.lastMessage("5n")->data % 5 == 0);
        ROS_ASSERT(chatter.lastReceive("5n") <= chatter.lastReceive());
        haros::MessageEvent<std_msgs::Int32> m2 =
            chatter.lastReceiveWhere(&Handler::even, this);
        ROS_ASSERT(!m2 || (m2.msg->data % 2 == 0));
        ROS_ASSERT(chatter.lastReceiveWhere(&Handler::multipleFive, this) == chatter.lastReceive("5n"));
        if (m2) ROS_INFO_STREAM("Last 2N receive data: " << m2.msg->data);
    }

    void callback2(const std_msgs::Int32::ConstPtr& msg)
    {
        haros::MessageEvent<std_msgs::Int32> evt = chatter2.lastReceive();
        ROS_INFO_STREAM("Last receive has occurred: " << evt.hasOccurred());
        if (evt.msg)
            ROS_INFO_STREAM("Last receive data: " << evt.msg->data);
        else
            ROS_INFO("Last receive has no data.");

        ROS_ASSERT(!chatter2.lastReceive() ||
                    chatter2.lastMessage()->data < msg->data);
    }

    void callback_str(const std_msgs::String::ConstPtr& msg)
    {
        haros::MessageEvent<std_msgs::String> evt = str_chatter.lastReceive();
        ROS_INFO_STREAM("(String) Last receive has occurred: " << evt.hasOccurred());
        if (evt.msg)
            ROS_INFO_STREAM("(String) Last receive data: " << evt.msg->data);
        else
            ROS_INFO("(String) Last receive has no data.");

        ROS_ASSERT(!str_chatter.lastReceive() ||
                    str_chatter.lastMessage()->data == "Hello world!");
    }

    bool even(haros::MessageEvent<std_msgs::Int32> evt)
    {
      return evt && (evt.msg->data % 2 == 0);
    }

    bool multipleFive(haros::MessageEvent<std_msgs::Int32> evt)
    {
      return evt && (evt.msg->data % 5 == 0);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    Handler h;
    ros::spin();
    return 0;
}
