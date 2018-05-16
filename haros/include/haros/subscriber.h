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
    Subscriber(ros::NodeHandle& nh, const std::string& topic,
               uint32_t queue_size, void(T::*fp)(P), T* obj, 
               const ros::TransportHints& hints = ros::TransportHints())
#if NDEBUG
    : ros_sub_(nh.subscribe(topic, queue_size, fp, obj, hints))
    {}
#else
    : helper_(new HelperT<P>(boost::bind(fp, obj, _1)))
    {
      boost::function<void (Param)> cb =
          boost::bind(&Helper::call, helper_.get(), _1);
      ros_sub_ = nh.template subscribe<Param>(topic, queue_size, cb, helper_, hints);
    }
#endif

    /**
     * \brief Subscribe to a topic, version for const class member function with bare pointer
     */
    template<class P, class T>
    Subscriber(ros::NodeHandle& nh, const std::string& topic,
               uint32_t queue_size, void(T::*fp)(P) const, T* obj, 
               const ros::TransportHints& hints = ros::TransportHints())
#if NDEBUG
    : ros_sub_(nh.subscribe(topic, queue_size, fp, obj, hints))
    {}
#else
    : helper_(new HelperT<P>(boost::bind(fp, obj, _1)))
    {
      boost::function<void (Param)> cb =
          boost::bind(&Helper::call, helper_.get(), _1);
      ros_sub_ = nh.template subscribe<Param>(topic, queue_size, cb, helper_, hints);
    }
#endif

    /**
     * \brief Subscribe to a topic, version for class member function with bare pointer
     */
    template<class T>
    Subscriber(ros::NodeHandle& nh, const std::string& topic,
               uint32_t queue_size,
               void(T::*fp)(const boost::shared_ptr<M const>&), T* obj, 
               const ros::TransportHints& hints = ros::TransportHints())
#if NDEBUG
    : ros_sub_(nh.subscribe(topic, queue_size, fp, obj, hints))
    {}
#else
    : helper_(new HelperT<const boost::shared_ptr<M const>&>(boost::bind(fp, obj, _1)))
    {
      boost::function<void (const ros::MessageEvent<M const>&)> cb =
          boost::bind(&Helper::call, helper_.get(), _1);
      ros_sub_ = nh.template subscribe<Param>(topic, queue_size, cb, helper_, hints);
    }
#endif

    /**
     * \brief Subscribe to a topic, version for const class member function with bare pointer
     */
    template<class T>
    Subscriber(ros::NodeHandle& nh, const std::string& topic,
               uint32_t queue_size,
               void(T::*fp)(const boost::shared_ptr<M const>&) const, T* obj, 
               const ros::TransportHints& hints = ros::TransportHints())
#if NDEBUG
    : ros_sub_(nh.subscribe(topic, queue_size, fp, obj, hints))
    {}
#else
    : helper_(new HelperT<const boost::shared_ptr<M const>&>(boost::bind(fp, obj, _1)))
    {
      boost::function<void (const ros::MessageEvent<M const>&)> cb =
          boost::bind(&Helper::call, helper_.get(), _1);
      ros_sub_ = nh.template subscribe<Param>(topic, queue_size, cb, helper_, hints);
    }
#endif

    /**
     * \brief Subscribe to a topic, version for class member function with shared_ptr
     */
    template<class P, class T>
    Subscriber(ros::NodeHandle& nh, const std::string& topic,
               uint32_t queue_size,
               void(T::*fp)(P), const boost::shared_ptr<T>& obj, 
               const ros::TransportHints& hints = ros::TransportHints())
#if NDEBUG
    : ros_sub_(nh.subscribe(topic, queue_size, fp, obj, hints))
    {}
#else
    : helper_(new HelperT<P>(boost::bind(fp, obj.get(), _1)))
    {
      boost::function<void (const ros::MessageEvent<M const>&)> cb =
          boost::bind(&Helper::call, helper_.get(), _1);
      ros_sub_ = nh.template subscribe<Param>(topic, queue_size, cb, helper_, hints);
    }
#endif

    /**
     * \brief Subscribe to a topic, version for const class member function with shared_ptr
     */
    template<class P, class T>
    Subscriber(ros::NodeHandle& nh, const std::string& topic,
               uint32_t queue_size,
               void(T::*fp)(P) const, const boost::shared_ptr<T>& obj, 
               const ros::TransportHints& hints = ros::TransportHints())
#if NDEBUG
    : ros_sub_(nh.subscribe(topic, queue_size, fp, obj, hints))
    {}
#else
    : helper_(new HelperT<P>(boost::bind(fp, obj.get(), _1)))
    {
      boost::function<void (const ros::MessageEvent<M const>&)> cb =
          boost::bind(&Helper::call, helper_.get(), _1);
      ros_sub_ = nh.template subscribe<Param>(topic, queue_size, cb, helper_, hints);
    }
#endif

    /**
     * \brief Subscribe to a topic, version for class member function with shared_ptr
     */
    template<class T>
    Subscriber(ros::NodeHandle& nh, const std::string& topic,
               uint32_t queue_size,
               void(T::*fp)(const boost::shared_ptr<M const>&),
               const boost::shared_ptr<T>& obj, 
               const ros::TransportHints& hints = ros::TransportHints())
#if NDEBUG
    : ros_sub_(nh.subscribe(topic, queue_size, fp, obj, hints))
    {}
#else
    : helper_(new HelperT<const boost::shared_ptr<M const>&>(boost::bind(fp, obj.get(), _1)))
    {
      boost::function<void (const ros::MessageEvent<M const>&)> cb =
          boost::bind(&Helper::call, helper_.get(), _1);
      ros_sub_ = nh.template subscribe<Param>(topic, queue_size, cb, helper_, hints);
    }
#endif

    /**
     * \brief Subscribe to a topic, version for const class member function with shared_ptr
     */
    template<class T>
    Subscriber(ros::NodeHandle& nh, const std::string& topic,
               uint32_t queue_size,
               void(T::*fp)(const boost::shared_ptr<M const>&) const,
               const boost::shared_ptr<T>& obj, 
               const ros::TransportHints& hints = ros::TransportHints())
#if NDEBUG
    : ros_sub_(nh.subscribe(topic, queue_size, fp, obj, hints))
    {}
#else
    : helper_(new HelperT<const boost::shared_ptr<M const>&>(boost::bind(fp, obj.get(), _1)))
    {
      boost::function<void (const ros::MessageEvent<M const>&)> cb =
          boost::bind(&Helper::call, helper_.get(), _1);
      ros_sub_ = nh.template subscribe<Param>(topic, queue_size, cb, helper_, hints);
    }
#endif

    /**
     * \brief Subscribe to a topic, version for bare function
     */
    template<class P>
    Subscriber(ros::NodeHandle& nh, const std::string& topic,
               uint32_t queue_size, void(*fp)(P),
               const ros::TransportHints& hints = ros::TransportHints())
#if NDEBUG
    : ros_sub_(nh.subscribe(topic, queue_size, fp, hints))
    {}
#else
    : helper_(new HelperT<P>(boost::bind(fp, _1)))
    {
      boost::function<void (const ros::MessageEvent<M const>&)> cb =
          boost::bind(&Helper::call, helper_.get(), _1);
      ros_sub_ = nh.template subscribe<Param>(topic, queue_size, cb, helper_, hints);
    }
#endif

    /**
     * \brief Subscribe to a topic, version for bare function
     */
    Subscriber(ros::NodeHandle& nh, const std::string& topic,
               uint32_t queue_size, void(*fp)(const boost::shared_ptr<M const>&),
               const ros::TransportHints& hints = ros::TransportHints())
#if NDEBUG
    : ros_sub_(nh.subscribe(topic, queue_size, fp, hints))
    {}
#else
    : helper_(new HelperT<const boost::shared_ptr<M const>&>(boost::bind(fp, _1)))
    {
      boost::function<void (const ros::MessageEvent<M const>&)> cb =
          boost::bind(&Helper::call, helper_.get(), _1);
      ros_sub_ = nh.template subscribe<Param>(topic, queue_size, cb, helper_, hints);
    }
#endif

    /**
     * \brief Subscribe to a topic, version for arbitrary boost::function
     */
    Subscriber(ros::NodeHandle& nh, const std::string& topic,
               uint32_t queue_size,
               const boost::function<void (const boost::shared_ptr<M const>&)>& f,
               const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr(),
               const ros::TransportHints& hints = ros::TransportHints())
#if NDEBUG
    : ros_sub_(nh.subscribe(topic, queue_size, f, tracked_object, hints))
    {}
#else
    : helper_(new HelperT<const boost::shared_ptr<M const>&>(f))
    {
      boost::function<void (const ros::MessageEvent<M const>&)> cb =
          boost::bind(&Helper::call, helper_, _1);
      ros_sub_ = nh.template subscribe<Param>(topic, queue_size, cb, tracked_object, hints);
    }
#endif

    /**
     * \brief Subscribe to a topic, version for arbitrary boost::function
     */
    template<class P>
    Subscriber(ros::NodeHandle& nh, const std::string& topic,
               uint32_t queue_size,
               const boost::function<void (P)>& f,
               const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr(),
               const ros::TransportHints& hints = ros::TransportHints())
#if NDEBUG
    : ros_sub_(nh.subscribe(topic, queue_size, f, tracked_object, hints))
    {}
#else
    : helper_(new HelperT<P>(f))
    {
      boost::function<void (const ros::MessageEvent<M const>&)> cb =
          boost::bind(&Helper::call, helper_, _1);
      ros_sub_ = nh.template subscribe<Param>(topic, queue_size, cb, tracked_object, hints);
    }
#endif

    Subscriber(const Subscriber<M>& rhs)
    : ros_sub_(rhs.ros_sub_)
#if !(NDEBUG)
    , helper_(rhs.helper_)
#endif
    {}

    ~Subscriber() {}

    //--------------------------------------------------------------------------
    // HAROS Message History
    //--------------------------------------------------------------------------

    MessageEvent<M> lastReceive() const
    {
#if NDEBUG
      return MessageEvent<M>();
#else
      ROS_ASSERT(helper_);
      return helper_->lastEvent();
#endif
    }

    MessageEvent<M> lastReceive(const std::string& predicate) const
    {
#if NDEBUG
      return MessageEvent<M>();
#else
      ROS_ASSERT(helper_);
      return helper_->lookup(predicate);
#endif
    }

    typename M::ConstPtr lastMessage() const
    {
#if NDEBUG
      return typename M::ConstPtr();
#else
      return lastReceive().msg;
#endif
    }

    typename M::ConstPtr lastMessage(const std::string& predicate) const
    {
#if NDEBUG
      return typename M::ConstPtr();
#else
      return lastReceive(predicate).msg;
#endif
    }

    void updateHistory() const
    {
#if !(NDEBUG)
      ROS_ASSERT(helper_);
      helper_->update(true);
#endif
    }

    //--------------------------------------------------------------------------
    // HAROS History Predicate Registry
    //--------------------------------------------------------------------------

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
    typedef const ros::MessageEvent<M const>& Param;

#if !(NDEBUG)
    //--------------------------------------------------------------------------
    // Helper Classes
    //--------------------------------------------------------------------------

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
      typename M::ConstPtr msg;

      Predicate(const boost::function<bool (M const&)>& p)
      : predicate(p)
      , time(ros::Time(0))
      {}

      void evaluate(const ros::MessageEvent<M const>& event)
      {
        if (predicate(*(event.getConstMessage())))
        {
          time = event.getReceiptTime();
          msg = event.getMessage();
        }
      }
    };

    // Adapted from CallbackHelper1 in message_filters
    struct Helper
    {
      bool dirty;
      ros::Time time;
      typename M::ConstPtr msg;
      std::map<std::string, Predicate> filters;
      const ros::MessageEvent<M const> *ptr;
#if HAROS_CHECK_THREAD_SAFETY
      boost::thread_specific_ptr<bool> known_thread;
      boost::mutex mutex;
      size_t thread_count;
      bool manual_update;
      boost::thread::id callback_thread;
#endif

      Helper()
      : time(ros::Time(0))
      , dirty(false)
      , ptr(NULL)
#if HAROS_CHECK_THREAD_SAFETY
      , thread_count(0)
      , manual_update(false)
#endif
      {}

      virtual ~Helper() {}

      virtual void call(const ros::MessageEvent<M const>& event) = 0;

      void update(bool manual)
      {
#if HAROS_CHECK_THREAD_SAFETY
        {
          boost::mutex::scoped_lock lock(mutex);
          if (boost::this_thread::get_id() != callback_thread)
          {
            oops();
          }
          manual_update = manual;
        }
        countThread();
#endif
        if (dirty && ptr)
        {
          time = ptr->getReceiptTime();
          msg = ptr->getMessage();
          typename std::map<std::string, Predicate>::iterator it;
          for (it = filters.begin(); it != filters.end(); it++)
          {
            it->second.evaluate(*ptr);
          }
          dirty = false;
          ptr = NULL;
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

      MessageEvent<M> lastEvent()
      {
#if HAROS_CHECK_THREAD_SAFETY
        countThread();
#endif
        return MessageEvent<M>(time, msg);
      }

      MessageEvent<M> lookup(const std::string& key)
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
        return MessageEvent<M>(it->second.time, it->second.msg);
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
        if (!manual_update)
        {
          ROS_WARN("Detected multiple threads using the same Subscriber. "
                   "Make sure to call updateHistory() within your callback, "
                   "and in a thread-safe manner (e.g. using mutexes).");
        }
      }
#endif
    };

    // Adapted from CallbackHelper1T in message_filters
    template<typename P>
    struct HelperT : Helper
    {
      typedef ros::ParameterAdapter<P> Adapter;
      typedef boost::function<void(typename Adapter::Parameter)> Callback;
      typedef typename Adapter::Event Event;

      Callback callback;

      HelperT(const boost::function<void(P)>& f) : callback(f) {}

      virtual void call(const ros::MessageEvent<M const>& event)
      {
#if HAROS_CHECK_THREAD_SAFETY
        {
          boost::mutex::scoped_lock lock(this->mutex);
          this->callback_thread = boost::this_thread::get_id();
        }
#endif
        this->dirty = true;
        this->ptr = &event;
        Event client_event(event, event.nonConstWillCopy());
        callback(Adapter::getParameter(client_event));
        this->update(false);
      }
    };
#endif // !(NDEBUG)

    //--------------------------------------------------------------------------
    // Internal State
    //--------------------------------------------------------------------------

    ros::Subscriber ros_sub_;
#if !(NDEBUG)
    boost::shared_ptr<Helper> helper_;
#endif
  };
} // namespace haros

#endif // HAROS_ASSERT_SUBSCRIBER_H
