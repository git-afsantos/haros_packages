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
#include <boost/bind.hpp>

#include <ros/ros.h>

#include "haros/message_event.h"

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

    /**
     * \brief Subscribe to a topic, version for class member function with bare pointer
     */
    template<class P, class T>
    Subscriber(const ros::NodeHandle& nh, const std::string& topic,
               uint32_t queue_size, void(T::*fp)(P), T* obj, 
               const ros::TransportHints& hints = ros::TransportHints())
    : helper_(new Helper(fp, obj))
    , ros_sub_(nh.subscribe(topic, queue_size,
                            boost::bind(&Helper::call, helper_.get(), _1),
                            ros::VoidConstPtr(), hints))
    {}

    /**
     * \brief Subscribe to a topic, version for const class member function with bare pointer
     */
    template<class P, class T>
    Subscriber(const ros::NodeHandle& nh, const std::string& topic,
               uint32_t queue_size, void(T::*fp)(P) const, T* obj, 
               const ros::TransportHints& hints = ros::TransportHints())
    : helper_(new Helper(fp, obj))
    , ros_sub_(nh.subscribe(topic, queue_size,
                            boost::bind(&Helper::call, helper_.get(), _1),
                            ros::VoidConstPtr(), hints))
    {}

    /**
     * \brief Subscribe to a topic, version for class member function with bare pointer
     */
    template<class T>
    Subscriber(const ros::NodeHandle& nh, const std::string& topic,
               uint32_t queue_size,
               void(T::*fp)(const boost::shared_ptr<M const>&), T* obj, 
               const ros::TransportHints& hints = ros::TransportHints())
    : helper_(new Helper(fp, obj))
    , ros_sub_(nh.subscribe(topic, queue_size,
                            boost::bind(&Helper::call, helper_.get(), _1),
                            ros::VoidConstPtr(), hints))
    {}

    /**
     * \brief Subscribe to a topic, version for const class member function with bare pointer
     */
    template<class T>
    Subscriber(const ros::NodeHandle& nh, const std::string& topic,
               uint32_t queue_size,
               void(T::*fp)(const boost::shared_ptr<M const>&) const, T* obj, 
               const ros::TransportHints& hints = ros::TransportHints())
    : helper_(new Helper(fp, obj))
    , ros_sub_(nh.subscribe(topic, queue_size,
                            boost::bind(&Helper::call, helper_.get(), _1),
                            ros::VoidConstPtr(), hints))
    {}

    /**
     * \brief Subscribe to a topic, version for class member function with shared_ptr
     */
    template<class P, class T>
    Subscriber(const ros::NodeHandle& nh, const std::string& topic,
               uint32_t queue_size,
               void(T::*fp)(P), const boost::shared_ptr<T>& obj, 
               const ros::TransportHints& hints = ros::TransportHints())
    : helper_(new Helper(fp, obj.get()))
    , ros_sub_(nh.subscribe(topic, queue_size,
                            boost::bind(&Helper::call, helper_.get(), _1),
                            ros::VoidConstPtr(), hints))
    {}

    /**
     * \brief Subscribe to a topic, version for const class member function with shared_ptr
     */
    template<class P, class T>
    Subscriber(const ros::NodeHandle& nh, const std::string& topic,
               uint32_t queue_size,
               void(T::*fp)(P) const, const boost::shared_ptr<T>& obj, 
               const ros::TransportHints& hints = ros::TransportHints())
    : helper_(new Helper(fp, obj.get()))
    , ros_sub_(nh.subscribe(topic, queue_size,
                            boost::bind(&Helper::call, helper_.get(), _1),
                            ros::VoidConstPtr(), hints))
    {}

    /**
     * \brief Subscribe to a topic, version for class member function with shared_ptr
     */
    template<class T>
    Subscriber(const ros::NodeHandle& nh, const std::string& topic,
               uint32_t queue_size,
               void(T::*fp)(const boost::shared_ptr<M const>&),
               const boost::shared_ptr<T>& obj, 
               const ros::TransportHints& hints = ros::TransportHints())
    : helper_(new Helper(fp, obj.get()))
    , ros_sub_(nh.subscribe(topic, queue_size,
                            boost::bind(&Helper::call, helper_.get(), _1),
                            ros::VoidConstPtr(), hints))
    {}

    /**
     * \brief Subscribe to a topic, version for const class member function with shared_ptr
     */
    template<class T>
    Subscriber(const ros::NodeHandle& nh, const std::string& topic,
               uint32_t queue_size,
               void(T::*fp)(const boost::shared_ptr<M const>&) const,
               const boost::shared_ptr<T>& obj, 
               const ros::TransportHints& hints = ros::TransportHints())
    : helper_(new Helper(fp, obj.get()))
    , ros_sub_(nh.subscribe(topic, queue_size,
                            boost::bind(&Helper::call, helper_.get(), _1),
                            ros::VoidConstPtr(), hints))
    {}

    /**
     * \brief Subscribe to a topic, version for bare function
     */
    template<class P>
    Subscriber(const ros::NodeHandle& nh, const std::string& topic,
               uint32_t queue_size, void(*fp)(P),
               const ros::TransportHints& hints = ros::TransportHints())
    : helper_(new Helper(fp))
    , ros_sub_(nh.subscribe(topic, queue_size,
                            boost::bind(&Helper::call, helper_.get(), _1),
                            ros::VoidConstPtr(), hints))
    {}

    /**
     * \brief Subscribe to a topic, version for bare function
     */
    Subscriber(const ros::NodeHandle& nh, const std::string& topic,
               uint32_t queue_size, void(*fp)(const boost::shared_ptr<M const>&),
               const ros::TransportHints& hints = ros::TransportHints())
    : helper_(new Helper(fp))
    , ros_sub_(nh.subscribe(topic, queue_size,
                            boost::bind(&Helper::call, helper_.get(), _1),
                            ros::VoidConstPtr(), hints))
    {}

    /**
     * \brief Subscribe to a topic, version for arbitrary boost::function
     */
    Subscriber(const ros::NodeHandle& nh, const std::string& topic,
               uint32_t queue_size,
               const boost::function<void (const boost::shared_ptr<M const>&)>& f,
               const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr(),
               const ros::TransportHints& hints = ros::TransportHints())
    : helper_(new Helper(f))
    , ros_sub_(nh.subscribe(topic, queue_size,
                            boost::bind(&Helper::call, helper_, _1),
                            tracked_object, hints))
    {}

    /**
     * \brief Subscribe to a topic, version for arbitrary boost::function
     */
    template<class P>
    Subscriber(const ros::NodeHandle& nh, const std::string& topic,
               uint32_t queue_size,
               const boost::function<void (P)>& f,
               const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr(),
               const ros::TransportHints& hints = ros::TransportHints())
    : helper_(new Helper(f))
    , ros_sub_(nh.subscribe(topic, queue_size,
                            boost::bind(&Helper::call, helper_, _1),
                            tracked_object, hints))
    {}

    Subscriber(const Subscriber<M>& rhs)
    : ros_sub_(rhs.ros_sub_)
    , helper_(rhs.helper_)
    {}

    ~Subscriber() {}

    //--------------------------------------------------------------------------
    // HAROS Interface
    //--------------------------------------------------------------------------

    MessageEvent<M> lastReceive() const
    {
#if NDEBUG
      return MessageEvent<M>();
#else
      ROS_ASSERT(helper_);
      return MessageEvent<M>(helper_->time, helper_->msg);
#endif
    }

/*
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
*/

    typename M::ConstPtr lastMessage() const
    {
#if NDEBUG
      return typename M::ConstPtr();
#else
      return lastReceive().msg;
#endif
    }

/*
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
*/

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
    //--------------------------------------------------------------------------
    // Helper Class
    //--------------------------------------------------------------------------

    // Adapted from CallbackHelper1T in message_filters
    template<typename P>
    struct Helper
    {
      typedef ros::ParameterAdapter<P> Adapter;
      typedef boost::function<void(typename Adapter::Parameter)> Callback;
      typedef typename Adapter::Event Event;

      Callback callback;
      ros::Time time;
      typename M::ConstPtr msg;
      bool dirty;

      Helper()
      : time(ros::Time(0))
      {}

      Helper(void(*fp)(P))
      : time(ros::Time(0))
      , callback(boost::bind(fp, _1))
      {}

      template<typename T>
      Helper(void(T::*fp)(P), T* t)
      : time(ros::Time(0))
      , callback(boost::bind(fp, t, _1))
      {}

      Helper(const boost::function<void(P)>& f)
      : time(ros::Time(0))
      , callback(f)
      {}

      template<typename C>
      Helper(const C& f)
      : time(ros::Time(0))
      , callback(Callback(f))
      {}

      void call(const ros::MessageEvent<M const>& event)
      {
        dirty = true;
        Event client_event(event, event.nonConstWillCopy());
        callback(Adapter::getParameter(client_event));
        if (dirty)
        {
          msg = event.getConstMessage();
        }
      }
    };

    //--------------------------------------------------------------------------
    // Internal State
    //--------------------------------------------------------------------------

    ros::Subscriber ros_sub_;
    boost::shared_ptr<Helper> helper_;
  };
} // namespace haros

#endif // HAROS_ASSERT_SUBSCRIBER_H
