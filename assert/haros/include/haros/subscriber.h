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

#ifndef HAROS_ASSERT_SUBSCRIBER_H
#define HAROS_ASSERT_SUBSCRIBER_H

#include <string>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include "haros/message_event.h"

#ifndef NDEBUG
#include "haros/history.h"
#endif

namespace haros
{
  template<class M>
  class Subscriber
  {
  public:
    //--------------------------------------------------------------------------
    // Constructors and Destructors
    //--------------------------------------------------------------------------

    Subscriber() {}

    Subscriber(const ros::Subscriber& sub)
    : ros_sub_(sub)
    {
#ifndef NDEBUG
      if (sub)
      {
        history_sub_ = History<M>::instance.subscribe(sub.getTopic());
      }
#endif
    }

    Subscriber(const Subscriber<M>& rhs)
    : ros_sub_(rhs.ros_sub_)
#ifndef NDEBUG
    , history_sub_(rhs.history_sub_)
#endif
    {}

    ~Subscriber() {}

    //--------------------------------------------------------------------------
    // HAROS Interface
    //--------------------------------------------------------------------------

    void bookmark(const std::string& key = std::string()) const
    {
#ifndef NDEBUG
      History<M>::instance.saveNextReceive(ros_sub_.getTopic(), key);
#endif
    }

    void forget(const std::string& key = std::string()) const
    {
#ifndef NDEBUG
      History<M>::instance.forgetReceive(ros_sub_.getTopic(), key);
#endif
    }

    void forgetAll() const
    {
#ifndef NDEBUG
      History<M>::instance.forgetAllReceive(ros_sub_.getTopic());
#endif
    }

    MessageEvent<M> lastReceive() const
    {
#ifdef NDEBUG
      return MessageEvent<M>();
#else
      return History<M>::instance.lastReceive(ros_sub_.getTopic());
#endif
    }

    MessageEvent<M> lastReceive(const std::string& bookmark) const
    {
#ifdef NDEBUG
      return MessageEvent<M>();
#else
      return History<M>::instance.lastReceive(ros_sub_.getTopic(), bookmark);
#endif
    }

    template<class T>
    MessageEvent<M> lastReceiveWhere(bool(T::*pred)(MessageEvent<M>), T* obj) const
    {
#ifdef NDEBUG
      return MessageEvent<M>();
#else
      return History<M>::instance.lastReceive(ros_sub_.getTopic(), pred, obj);
#endif
    }

    template<class T>
    MessageEvent<M> lastReceiveWhere(bool(T::*pred)(MessageEvent<M>) const, T* obj) const
    {
#ifdef NDEBUG
      return MessageEvent<M>();
#else
      return History<M>::instance.lastReceive(ros_sub_.getTopic(), pred, obj);
#endif
    }

    template<class T>
    MessageEvent<M> lastReceiveWhere(bool(T::*pred)(MessageEvent<M>),
                                     const boost::shared_ptr<T>& obj) const
    {
#ifdef NDEBUG
      return MessageEvent<M>();
#else
      return History<M>::instance.lastReceive(ros_sub_.getTopic(), pred, obj);
#endif
    }

    template<class T>
    MessageEvent<M> lastReceiveWhere(bool(T::*pred)(MessageEvent<M>) const,
                                     const boost::shared_ptr<T>& obj) const
    {
#ifdef NDEBUG
      return MessageEvent<M>();
#else
      return History<M>::instance.lastReceive(ros_sub_.getTopic(), pred, obj);
#endif
    }

    MessageEvent<M> lastReceiveWhere(bool(*pred)(MessageEvent<M>)) const
    {
#ifdef NDEBUG
      return MessageEvent<M>();
#else
      return History<M>::instance.lastReceive(ros_sub_.getTopic(), pred);
#endif
    }

    MessageEvent<M> lastReceiveWhere(
        const boost::function<bool (MessageEvent<M>)>& pred) const
    {
#ifdef NDEBUG
      return MessageEvent<M>();
#else
      return History<M>::instance.lastReceive(ros_sub_.getTopic(), pred);
#endif
    }

    typename M::ConstPtr lastMessage() const
    {
#ifdef NDEBUG
      return typename M::ConstPtr();
#else
      return lastReceive().msg;
#endif
    }

    typename M::ConstPtr lastMessage(const std::string& bookmark) const
    {
#ifdef NDEBUG
      return typename M::ConstPtr();
#else
      return lastReceive(bookmark).msg;
#endif
    }

    template<class T>
    typename M::ConstPtr lastMessageWhere(bool(T::*pred)(MessageEvent<M>), T* obj) const
    {
#ifdef NDEBUG
      return typename M::ConstPtr();
#else
      return lastReceive(pred, obj).msg;
#endif
    }

    template<class T>
    typename M::ConstPtr lastMessageWhere(bool(T::*pred)(MessageEvent<M>) const, T* obj) const
    {
#ifdef NDEBUG
      return typename M::ConstPtr();
#else
      return lastReceive(pred, obj).msg;
#endif
    }

    template<class T>
    typename M::ConstPtr lastMessageWhere(bool(T::*pred)(MessageEvent<M>),
                                          const boost::shared_ptr<T>& obj) const
    {
#ifdef NDEBUG
      return typename M::ConstPtr();
#else
      return lastReceive(pred, obj).msg;
#endif
    }

    template<class T>
    typename M::ConstPtr lastMessageWhere(bool(T::*pred)(MessageEvent<M>) const,
                                          const boost::shared_ptr<T>& obj) const
    {
#ifdef NDEBUG
      return typename M::ConstPtr();
#else
      return lastReceive(pred, obj).msg;
#endif
    }

    typename M::ConstPtr lastMessageWhere(bool(*pred)(MessageEvent<M>)) const
    {
#ifdef NDEBUG
      return typename M::ConstPtr();
#else
      return lastReceive(pred).msg;
#endif
    }

    typename M::ConstPtr lastMessageWhere(
        const boost::function<bool (MessageEvent<M>)>& pred) const
    {
#ifdef NDEBUG
      return typename M::ConstPtr();
#else
      return lastReceive(pred).msg;
#endif
    }

    //--------------------------------------------------------------------------
    // ROS Subscriber Interface
    //--------------------------------------------------------------------------

    /**
     * \brief Unsubscribe the callback associated with this Subscriber
     *
     * This method usually does not need to be explicitly called,
     * as automatic shutdown happens when
     * all copies of this Subscriber go out of scope
     *
     * This method overrides the automatic reference counted unsubscribe,
     * and immediately unsubscribes the callback associated with this Subscriber
     */
    void shutdown()
    {
      ros_sub_.shutdown();
    }

    std::string getTopic() const
    {
      return ros_sub_.getTopic();
    }

    /**
     * \brief Returns the number of publishers this subscriber is connected to
     */
    uint32_t getNumPublishers() const
    {
      return ros_sub_.getNumPublishers();
    }

    operator void*() const
    {
      return (void *) ros_sub_;
    }

    bool operator<(const ros::Subscriber& rhs) const
    {
      return ros_sub_ < rhs;
    }

    bool operator<(const Subscriber& rhs) const
    {
      return ros_sub_ < rhs.ros_sub_;
    }

    bool operator==(const ros::Subscriber& rhs) const
    {
      return ros_sub_ == rhs;
    }

    bool operator==(const Subscriber& rhs) const
    {
      return ros_sub_ == rhs.ros_sub_;
    }

    bool operator!=(const ros::Subscriber& rhs) const
    {
      return ros_sub_ != rhs;
    }

    bool operator!=(const Subscriber& rhs) const
    {
      return ros_sub_ != rhs.ros_sub_;
    }

  private:
    ros::Subscriber ros_sub_;

#ifndef NDEBUG
    typename History<M>::HolderPtr history_sub_;
#endif
  };
} // namespace haros

#endif // HAROS_ASSERT_SUBSCRIBER_H
