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

#ifndef HAROS_ASSERT_MESSAGE_EVENT_H
#define HAROS_ASSERT_MESSAGE_EVENT_H

#include <boost/shared_ptr.hpp>

#include <topic_tools/shape_shifter.h>

namespace haros
{
  /** This is supposed to be a very lightweight object.
   * It is basically a pair of a time and a pointer to a message.
   * Thus, it is copyable.
   * Can also be stored directly, instead of storing a pointer.
   */
  class MessageEvent
  {
  public:
    MessageEvent();
    MessageEvent(const ros::Time time, const topic_tools::ShapeShifter::ConstPtr msg);

    template<class M>
    boost::shared_ptr<M> msg();

    bool hasOccurred();

    bool operator< (const MessageEvent& me) const;
    bool operator<= (const MessageEvent& me) const;
    bool operator> (const MessageEvent& me) const;
    bool operator>= (const MessageEvent& me) const;
    bool operator== (const MessageEvent& me) const;

    operator void*() const;

  private:
    ros::Time time_;
    topic_tools::ShapeShifter::ConstPtr msg_;
  };

  template<class M>
  boost::shared_ptr<M> MessageEvent::msg()
  {
    if (msg_)
    {
      return msg_->instantiate<M>();
    }
    return boost::shared_ptr<M>();
  }
} // namespace haros

#endif // HAROS_ASSERT_MESSAGE_EVENT_H
