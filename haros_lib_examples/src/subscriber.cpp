#include <stdexcept>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include "haros/haros.h"

haros::Subscriber<std_msgs::Int32> g_chatter;

void global_callback(const std_msgs::Int32::ConstPtr& msg)
{
    haros::MessageEvent<std_msgs::Int32> evt = g_chatter.lastReceive();
    ROS_INFO_STREAM("Last global receive has occurred: " << evt.hasOccurred());
    if (evt.msg)
        ROS_INFO_STREAM("Last global receive data: " << evt.msg->data);
    else
        ROS_INFO("Last global receive has no data.");
    
    ROS_ASSERT(!g_chatter.lastReceive()
               || g_chatter.lastMessage()->data < msg->data);

    haros::MessageEvent<std_msgs::Int32> n5 = g_chatter.lastReceive("5n");
    ROS_ASSERT(!n5 || (n5.msg->data % 5 == 0));
    if (n5) ROS_INFO_STREAM("Last 5N receive data: " << n5.msg->data);

    g_chatter.updateHistory();
    ROS_ASSERT(g_chatter.lastReceive().msg->data == msg->data);
}

bool multipleFive(const std_msgs::Int32& msg)
{
  return msg.data % 5 == 0;
}

class Handler
{
public:
    haros::Subscriber<std_msgs::Int32> chatter;
    haros::Subscriber<std_msgs::Int32> chatter2;

    Handler(ros::NodeHandle node)
    : chatter(node, "chatter", 10, &Handler::callback, this)
    , chatter2(node, "chatter", 10, &Handler::callback2, this)
    {
      chatter.recordIf(&Handler::even, this, "2n");
      chatter2.recordIf(boost::bind(&Handler::tens, this, _1), "10n");
    }

    void callback(const std_msgs::Int32::ConstPtr& msg)
    {
        haros::MessageEvent<std_msgs::Int32> evt = chatter.lastReceive();
        ROS_INFO_STREAM("Last receive has occurred: " << evt.hasOccurred());
        if (evt.msg)
            ROS_INFO_STREAM("Last receive data: " << evt.msg->data);
        else
            ROS_INFO("Last receive has no data.");
        
        ROS_ASSERT(!chatter.lastReceive()
                   || chatter.lastMessage()->data < msg->data);

        haros::MessageEvent<std_msgs::Int32> n2 = chatter.lastReceive("2n");
        ROS_ASSERT(!n2 || (n2.msg->data % 2 == 0));
        if (n2) ROS_INFO_STREAM("Last 2N receive data: " << n2.msg->data);

        try
        {
          haros::MessageEvent<std_msgs::Int32> n10 = chatter.lastReceive("10n");
          ROS_BREAK();
        }
        catch (const std::out_of_range&) {}
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

        haros::MessageEvent<std_msgs::Int32> n10 = chatter2.lastReceive("10n");
        ROS_ASSERT(!n10 || (n10.msg->data % 10 == 0));
        if (n10) ROS_INFO_STREAM("Last 10N receive data: " << n10.msg->data);

        try
        {
          haros::MessageEvent<std_msgs::Int32> n2 = chatter2.lastReceive("2n");
          ROS_BREAK();
        }
        catch (const std::out_of_range&) {}
    }

    bool even(const std_msgs::Int32& msg)
    {
      return msg.data % 2 == 0;
    }

    bool tens(const std_msgs::Int32& msg)
    {
      return msg.data % 10 == 0;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;
    Handler h(nh);
    g_chatter = haros::Subscriber<std_msgs::Int32>(nh, "global_chatter", 10, global_callback);
    g_chatter.recordIf(multipleFive, "5n");
    ros::spin();
    return 0;
}
