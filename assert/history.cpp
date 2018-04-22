/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Andre Santos
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
********************************************************************/

#include "history.h"

#include <boost/bind.hpp>

namespace haros
{
History::History() {}

History::instance;

MessageEvent History::lastReceive(const std::string& topic)
{
  boost::mutex::scoped_lock lock(sub_mutex_);
  std::map<std::string, Entry>::iterator it = received_.find(topic);
  if (it != received_.end())
  {
    return MessageEvent(it->second.time_, it->second.msg_);
  }
  return MessageEvent();
}

boost::shared_ptr<ros::Subscriber> History::subscribe(const std::string& topic,
                                                      const uint32_t queue_size)
{
  boost::mutex::scoped_lock lock(sub_mutex_);
  std::map<std::string, Entry>::iterator it = received_.find(topic);
  // return subscriber if it already exists
  if (it != received_.end()
      && boost::shared_ptr<ros::Subscriber> p = it->second.sub_.lock())
  {
    return p;
  }
  // if it does not exist (anymore), subscribe again
  ros::SubscribeOptions ops;
  ops.topic = topic;
  ops.queue_size = queue_size;
  ops.md5sum = ros::message_traits::md5sum<topic_tools::ShapeShifter>();
  ops.datatype = ros::message_traits::datatype<topic_tools::ShapeShifter>();
  // bind makes a copy of the topic string, so it is safe to use it
  ops.helper = boost::make_shared<ros::SubscriptionCallbackHelperT<
      const ros::MessageEvent<topic_tools::ShapeShifter const> &> >(
          boost::bind(&History::receive, this, topic, _1));
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe(ops);
  boost::shared_ptr<ros::Subscriber> sub_ptr(new ros::Subscriber(sub));
  Entry& e = received_[topic];
  e.sub_ = sub_ptr;
  return sub_ptr;
}

// relevant:
// http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers#MessageEvent_.5BROS_1.1.2B-.5D
// https://answers.ros.org/question/273964/using-shapeshifter-messageevent-and-boost-bind-together-to-pass-arguments-to-callback/
void receive(const std::string& topic,
             const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event)
{
  boost::mutex::scoped_lock lock(sub_mutex_);
  Entry& e = received_[topic];
  e.time_ = msg_event.getReceiptTime();
  e.msg_ = msg_event.getMessage();
}
} // namespace haros
