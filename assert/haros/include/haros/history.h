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

#ifndef HAROS_ASSERT_HISTORY_H
#define HAROS_ASSERT_HISTORY_H

#include <string>
#include <unordered_map>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>

#include <topic_tools/shape_shifter.h>

#include "haros/message_event.h"

namespace haros
{
  /** History will remain alive for the program duration, since everything else is
   * non-unique and/or temporary (NodeHandle, Subscriber, Publisher).
   * Since History is shared across multiple threads, it has to
   * implement concurrency control.
   */
  template<class M>
  class History
  {
  public:
    static History<M> instance;


    MessageEvent lastReceive(const std::string& topic);

    // This is needed in case the client assigns multiple callbacks
    // to the same topic. We must not invalidate previous pointers,
    // but at the same time we must be able to unsubscribe and subscribe
    // again, to make sure the history callback is called last.
    struct SubscriberHolder
    {
      ros::Subscriber sub_;

      SubscriberHolder() {}
      SubscriberHolder(const ros::Subscriber& sub) : sub_(sub) {}
      ~SubscriberHolder() {}
    };

    typedef boost::shared_ptr<SubscriberHolder> HolderPtr;

    HolderPtr subscribe(const std::string& topic, const uint32_t queue_size);

  private:
    History();

    void receive(const std::string& topic,
                 const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event);
    // msg_event.getMessage() is of type topic_tools::ShapeShifter::ConstPtr

    struct Entry
    {
      boost::weak_ptr<SubscriberHolder> sub_;
      ros::Time time_;
      topic_tools::ShapeShifter::ConstPtr msg_;

      Entry() : time_(ros::Time(0)) {}
    };

    boost::mutex sub_mutex_;
    std::map<std::string, Entry> received_;
  };
} // namespace haros

#endif // HAROS_ASSERT_HISTORY_H
