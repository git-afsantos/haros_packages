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

#ifndef HAROS_ASSERT_PUBLISHER_H
#define HAROS_ASSERT_PUBLISHER_H

#include <string>

#if !(NDEBUG)
#include <stdexcept>
#include <utility>
#include <map>
#endif

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#if !(NDEBUG)
#include <boost/bind.hpp>

#if HAROS_CHECK_THREAD_SAFETY
#include <boost/thread/thread.hpp>
#include <boost/thread/tss.hpp>
#include <boost/thread/mutex.hpp>
#endif // HAROS_CHECK_THREAD_SAFETY
#endif // !(NDEBUG)

#include <ros/ros.h>

#include "haros/publish_event.h"

namespace haros
{
  template<class M>
  class Publisher
  {
  public:
    //---------------------------------------------------------------------------
    // Constructors and Destructors
    //---------------------------------------------------------------------------

    Publisher() {}

    /**
     * \brief Advertise a topic, simple version
     */
    Publisher(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size,
              bool latch = false)
#if !(NDEBUG)
    : helper_(new Helper())
#endif
    , ros_pub_(nh.advertise<M>(topic, queue_size, latch))
    {}

    /**
     * \brief Advertise a topic, with most of the available options, including subscriber status callbacks
     */
    Publisher(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size,
              const ros::SubscriberStatusCallback& connect_cb,
              const ros::SubscriberStatusCallback& disconnect_cb =
                  ros::SubscriberStatusCallback(),
              const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr(),
              bool latch = false)
#if !(NDEBUG)
    : helper_(new Helper())
#endif
    , ros_pub_(nh.advertise<M>(topic, queue_size, connect_cb, disconnect_cb,
                               tracked_object, latch))
    {}

    Publisher(const Publisher<M>& rhs)
    : ros_pub_(rhs.ros_pub_)
#if !(NDEBUG)
    , helper_(rhs.helper_)
#endif
    {}

    ~Publisher() {}

    //---------------------------------------------------------------------------
    // HAROS Message History
    //---------------------------------------------------------------------------

    PublishEvent<M> lastPublish() const
    {
#if NDEBUG
      return PublishEvent<M>();
#else
      ROS_ASSERT(helper_);
      return helper_->lastEvent();
#endif
    }

    PublishEvent<M> lastPublish(const std::string& predicate) const
    {
#if NDEBUG
      return PublishEvent<M>();
#else
      ROS_ASSERT(helper_);
      return helper_->lookup(predicate);
#endif
    }

    typename M::Ptr lastMessage() const
    {
#if NDEBUG
      return typename M::Ptr();
#else
      return lastPublish().msg;
#endif
    }

    typename M::Ptr lastMessage(const std::string& predicate) const
    {
#if NDEBUG
      return typename M::Ptr();
#else
      return lastPublish(predicate).msg;
#endif
    }

    //---------------------------------------------------------------------------
    // HAROS History Predicate Registry
    //---------------------------------------------------------------------------

    template<class T>
    void recordIf(bool(T::*pred)(M const&), T* obj, const std::string& key) const
    {
#if !(NDEBUG)
      ROS_ASSERT(helper_);
      Predicate p(boost::bind(pred, obj, _1));
      helper_->addPredicate(key, p);
#endif
    }

    template<class T>
    void recordIf(bool(T::*pred)(M const&) const, T* obj, const std::string& key) const
    {
#if !(NDEBUG)
      ROS_ASSERT(helper_);
      Predicate p(boost::bind(pred, obj, _1));
      helper_->addPredicate(key, p);
#endif
    }

    void recordIf(bool(*pred)(M const&), const std::string& key) const
    {
#if !(NDEBUG)
      ROS_ASSERT(helper_);
      Predicate p(boost::bind(pred, _1));
      helper_->addPredicate(key, p);
#endif
    }

    void recordIf(const boost::function<bool (M const&)>& pred,
                  const std::string& key) const
    {
#if !(NDEBUG)
      ROS_ASSERT(helper_);
      Predicate p(pred);
      helper_->addPredicate(key, p);
#endif
    }

    //---------------------------------------------------------------------------
    // ROS Publisher Interface
    //---------------------------------------------------------------------------

    /**
     * \brief Publish a message on the topic associated with this Publisher.
     *
     * This version of publish allows fast intra-process message-passing,
     * so you may not mutate the message after it has been passed in here
     * (since it will be passed directly into a callback function).
     *
     */
    void publish(const boost::shared_ptr<M>& message) const
    {
      if (ros_pub_)
      {
        ros_pub_.publish(message);
#if !(NDEBUG)
        ROS_ASSERT(helper_);
        helper_->update(message);
#endif
      }
    }

    /**
     * \brief Publish a message on the topic associated with this Publisher.
     */
    void publish(const M& message) const
    {
      if (ros_pub_)
      {
        ros_pub_.publish(message);
#if !(NDEBUG)
        ROS_ASSERT(helper_);
        helper_->update(typename M::Ptr(new M(message)));
#endif
      }
    }

    /**
     * \brief Shutdown the advertisement associated with this Publisher
     *
     * This method usually does not need to be explicitly called, as automatic shutdown happens when
     * all copies of this Publisher go out of scope
     *
     * This method overrides the automatic reference counted unadvertise, and does so immediately.
     * \note Note that if multiple advertisements were made through NodeHandle::advertise(), this will
     * only remove the one associated with this Publisher
     */
    void shutdown()
    {
      ros_pub_.shutdown();
    }

    /**
     * \brief Returns the topic that this Publisher will publish on.
     */
    std::string getTopic() const
    {
      return ros_pub_.getTopic();
    }

    /**
     * \brief Returns the number of subscribers that are currently connected to this Publisher
     */
    uint32_t getNumSubscribers() const
    {
      return ros_pub_.getNumSubscribers();
    }

    /**
     * \brief Returns whether or not this topic is latched
     */
    bool isLatched() const
    {
      return ros_pub_.isLatched();
    }

    operator void*() const
    {
      return (void *) ros_pub_;
    }

    bool operator<(const ros::Publisher& rhs) const
    {
      return ros_pub_ < rhs;
    }

    bool operator<(const Publisher& rhs) const
    {
      return ros_pub_ < rhs.ros_pub_;
    }

    bool operator==(const ros::Publisher& rhs) const
    {
      return ros_pub_ == rhs;
    }

    bool operator==(const Publisher& rhs) const
    {
      return ros_pub_ == rhs.ros_pub_;
    }

    bool operator!=(const ros::Publisher& rhs) const
    {
      return ros_pub_ != rhs;
    }

    bool operator!=(const Publisher& rhs) const
    {
      return ros_pub_ != rhs.ros_pub_;
    }

  private:
#if !(NDEBUG)
    //---------------------------------------------------------------------------
    // Helper Classes
    //---------------------------------------------------------------------------

    /**
     * Justification for choosing (M const&) as the parameter type comes from
     * [C++ Core Guidelines R.30](https://github.com/isocpp/CppCoreGuidelines):
     * Take smart pointers as parameters only to explicitly express
     * lifetime semantics.
     */
    struct Predicate
    {
      boost::function<bool (M const&)> predicate;
      ros::Time time;
      typename M::Ptr msg;

      Predicate(const boost::function<bool (M const&)>& p)
      : predicate(p)
      , time(ros::Time(0))
      {}

      void evaluate(typename M::Ptr message)
      {
        if (predicate(*message))
        {
          time = ros::Time::now();
          msg = message;
        }
      }
    };

    struct Helper
    {
      ros::Time time;
      typename M::Ptr msg;
      std::map<std::string, Predicate> filters;
#if HAROS_CHECK_THREAD_SAFETY
      boost::thread_specific_ptr<bool> known_thread;
      boost::mutex mutex;
      size_t thread_count;
#endif

      Helper()
      : time(ros::Time(0))
#if HAROS_CHECK_THREAD_SAFETY
      , thread_count(0)
#endif
      {}

      void update(typename M::Ptr message)
      {
#if HAROS_CHECK_THREAD_SAFETY
        countThread();
#endif
        time = ros::Time::now();
        msg = message;
        typename std::map<std::string, Predicate>::iterator it;
        for (it = filters.begin(); it != filters.end(); it++)
        {
          it->second.evaluate(message);
        }
      }

      void addPredicate(const std::string& key, const Predicate& pred)
      {
        // TODO decide if countThread() should be called here too
        std::pair<typename std::map<std::string, Predicate>::iterator, bool> res
            = filters.insert(std::make_pair(key, pred));
        if (!res.second)
        {
          throw std::invalid_argument(key);
        }
      }

      PublishEvent<M> lastEvent()
      {
#if HAROS_CHECK_THREAD_SAFETY
        countThread();
#endif
        return PublishEvent<M>(time, msg);
      }

      PublishEvent<M> lookup(const std::string& key)
      {
#if HAROS_CHECK_THREAD_SAFETY
        countThread();
#endif
        typename std::map<std::string, Predicate>::const_iterator it
            = filters.find(key);
        if (it == filters.end())
        {
          throw std::out_of_range(key);
        }
        return PublishEvent<M>(it->second.time, it->second.msg);
      }

#if HAROS_CHECK_THREAD_SAFETY
      void countThread()
      {
        if (!known_thread.get())
        {
          known_thread.reset(new bool(true));
          boost::mutex::scoped_lock lock(mutex);
          ++thread_count;
          if (thread_count > 1)
          {
            oops();
          }
        }
      }

      void oops() const
      {
        ROS_WARN("Detected multiple threads using the same Publisher. "
                 "Make sure to use proper thread synchronization.");
      }
#endif // HAROS_CHECK_THREAD_SAFETY
    };
#endif // !(NDEBUG)

    //---------------------------------------------------------------------------
    // Internal State
    //---------------------------------------------------------------------------

    ros::Publisher ros_pub_;
#if !(NDEBUG)
    boost::shared_ptr<Helper> helper_;
#endif
  };
} // namespace haros

#endif // HAROS_ASSERT_PUBLISHER_H
