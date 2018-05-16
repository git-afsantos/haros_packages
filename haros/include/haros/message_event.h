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

namespace haros
{
  /** This is supposed to be a very lightweight object.
   * It is basically a pair of a time and a pointer to a message.
   * Thus, it is copyable.
   * Can also be stored directly, instead of storing a pointer.
   */
  template<class M>
  struct MessageEvent
  {
    //---------------------------------------------------------------------------
    // Member Variables
    //---------------------------------------------------------------------------

    ros::Time time;
    typename M::ConstPtr msg;

    //---------------------------------------------------------------------------
    // Constructors
    //---------------------------------------------------------------------------

    MessageEvent()
    : time(ros::Time(0))
    {}

    MessageEvent(const ros::Time& _time, const typename M::ConstPtr& _msg)
    : time(_time)
    , msg(_msg)
    {}

    //---------------------------------------------------------------------------
    // Methods and Operators
    //---------------------------------------------------------------------------

    bool hasOccurred() const
    {
      return time.isValid() && !time.isZero();
    }

    bool operator< (const MessageEvent& rhs) const
    {
      return time < rhs.time;
    }

    bool operator<= (const MessageEvent& rhs) const
    {
      return time <= rhs.time;
    }

    bool operator> (const MessageEvent& rhs) const
    {
      return time > rhs.time;
    }

    bool operator>= (const MessageEvent& rhs) const
    {
      return time >= rhs.time;
    }

    bool operator== (const MessageEvent& rhs) const
    {
      return time == rhs.time && msg == rhs.msg;
    }

    operator void*() const
    {
      return (time.isValid() && !time.isZero() && msg) ? (void*)1 : (void*)0;
    }
  };
} // namespace haros

#endif // HAROS_ASSERT_MESSAGE_EVENT_H
