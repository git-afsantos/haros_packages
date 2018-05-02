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
#include <list>
#include <map>
#include <stdexcept>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>

#include "haros/message_event.h"
#include "haros/publish_event.h"

namespace haros
{
  template<class M>
  class Subscriber;

  template<class M>
  class Publisher;

  // NOTE: templated code should not be in cpp files. Results in link error.

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

    //--------------------------------------------------------------------------
    // Publish Events
    //--------------------------------------------------------------------------

    PublishEvent<M> lastPublish(const std::string& topic)
    {
      boost::mutex::scoped_lock lock(pub_mutex_);
      typename std::map<std::string, PublisherEntry>::iterator it =
          published_.find(topic);
      if (it != published_.end())
      {
        return PublishEvent<M>(it->second.time, it->second.msg);
      }
      return PublishEvent<M>();
    }

    PublishEvent<M> lastPublish(const std::string& topic,
                                const std::string& bookmark)
    {
      boost::mutex::scoped_lock lock(pub_mutex_);
      typename std::map<std::string, PublisherEntry>::iterator it =
          published_.find(topic);
      if (it != published_.end())
      {
        typename std::map<std::string, std::list<PublishEvent<M> > >::iterator it2 =
            it->second.bookmarks.find(bookmark);
        if (it2 != it->second.bookmarks.end() && !it2->second.empty())
        {
          return PublishEvent<M>(it2->second.front());
        }
      }
      return PublishEvent<M>();
    }

    //--------------------------------------------------------------------------
    // Receive Events
    //--------------------------------------------------------------------------

    MessageEvent<M> lastReceive(const std::string& topic)
    {
      boost::mutex::scoped_lock lock(sub_mutex_);
      typename std::map<std::string, SubscriberEntry>::iterator it = received_.find(topic);
      if (it != received_.end())
      {
        return MessageEvent<M>(it->second.time, it->second.msg);
      }
      return MessageEvent<M>();
    }

    MessageEvent<M> lastReceive(const std::string& topic,
                                const std::string& bookmark)
    {
      boost::mutex::scoped_lock lock(sub_mutex_);
      typename std::map<std::string, SubscriberEntry>::iterator it = received_.find(topic);
      if (it != received_.end())
      {
        typename std::map<std::string, std::list<MessageEvent<M> > >::iterator it2 =
            it->second.bookmarks.find(bookmark);
        if (it2 != it->second.bookmarks.end() && !it2->second.empty())
        {
          return MessageEvent<M>(it2->second.front());
        }
      }
      return MessageEvent<M>();
    }

  private:
    //--------------------------------------------------------------------------
    // Constructors
    //--------------------------------------------------------------------------
    History() {}
    History(const History& rhs) {}

    //--------------------------------------------------------------------------
    // Publisher Interface
    //--------------------------------------------------------------------------

    struct PublisherEntry
    {
      ros::Time time;
      boost::shared_ptr<M> msg;
      std::map<std::string, std::list<PublishEvent<M> > > bookmarks;
      bool save_next;
      std::string bookmark_key;

      PublisherEntry() : time(ros::Time(0)), save_next(false) {}
    };

    boost::mutex pub_mutex_;
    std::map<std::string, PublisherEntry> published_;

    void saveNextPublish(const std::string& topic, const std::string& key)
    {
      boost::mutex::scoped_lock lock(pub_mutex_);
      typename std::map<std::string, PublisherEntry>::iterator it = published_.find(topic);
      if (it != published_.end())
      {
        it->second.save_next = true;
        it->second.bookmark_key = key;
      }
      else
      {
        throw std::out_of_range(topic);
      }
    }

    void publish(const std::string& topic, const boost::shared_ptr<M>& msg)
    {
      boost::mutex::scoped_lock lock(pub_mutex_);
      PublisherEntry& e = published_[topic];
      e.time = ros::Time::now();
      e.msg = msg;
      if (e.save_next)
      {
        e.bookmarks[e.bookmark_key].push_front(PublishEvent<M>(e.time, e.msg));
        e.save_next = false;
      }
    }

    //--------------------------------------------------------------------------
    // Subscriber Interface
    //--------------------------------------------------------------------------

    // This is needed in case the client assigns multiple callbacks
    // to the same topic. We must not invalidate previous pointers,
    // but at the same time we must be able to unsubscribe and subscribe
    // again, to make sure the history callback is called last.
    struct SubscriberHolder
    {
      ros::Subscriber sub;

      SubscriberHolder() {}
      SubscriberHolder(const ros::Subscriber& _sub) : sub(_sub) {}
      ~SubscriberHolder() {}
    };

    typedef boost::shared_ptr<SubscriberHolder> HolderPtr;

    struct SubscriberEntry
    {
      boost::weak_ptr<SubscriberHolder> sub;
      ros::Time time;
      typename M::ConstPtr msg;
      std::map<std::string, std::list<MessageEvent<M> > > bookmarks;
      bool save_next;
      std::string bookmark_key;

      SubscriberEntry() : time(ros::Time(0)), save_next(false) {}
    };

    boost::mutex sub_mutex_;
    std::map<std::string, SubscriberEntry> received_;

    void saveNextReceive(const std::string& topic, const std::string& key)
    {
      boost::mutex::scoped_lock lock(sub_mutex_);
      typename std::map<std::string, SubscriberEntry>::iterator it = received_.find(topic);
      if (it != received_.end())
      {
        it->second.save_next = true;
        it->second.bookmark_key = key;
      }
      else
      {
        throw std::out_of_range(topic);
      }
    }

    HolderPtr subscribe(const std::string& topic)
    {
      boost::mutex::scoped_lock lock(sub_mutex_);
      HolderPtr sub_ptr;
      typename std::map<std::string, SubscriberEntry>::iterator it = received_.find(topic);
      if (it != received_.end())
      {
        sub_ptr = it->second.sub.lock();
        if (sub_ptr)
        {
          // if the client subscribes multiple times to the same topic,
          // make sure this callback is the last to be executed.
          sub_ptr->sub = ros::Subscriber();
        }
      }
      // at this point, the history callback does not exist (anymore)
      ros::NodeHandle nh;
      ros::Subscriber sub = nh.subscribe<M>(topic, 1,
          boost::bind(&History<M>::receive, this, topic, _1));
      if (!sub_ptr)
      {
        sub_ptr.reset(new SubscriberHolder());
      }
      sub_ptr->sub = sub;
      SubscriberEntry& e = received_[topic];
      e.sub = sub_ptr;
      return sub_ptr;
    }

    // relevant:
    // http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers#MessageEvent_.5BROS_1.1.2B-.5D
    // https://answers.ros.org/question/273964/using-shapeshifter-messageevent-and-boost-bind-together-to-pass-arguments-to-callback/
    void receive(const std::string& topic,
                 const ros::MessageEvent<M const>& msg_event)
    {
      boost::mutex::scoped_lock lock(sub_mutex_);
      SubscriberEntry& e = received_[topic];
      e.time = msg_event.getReceiptTime();
      e.msg = msg_event.getMessage();
      if (e.save_next)
      {
        e.bookmarks[e.bookmark_key].push_front(MessageEvent<M>(e.time, e.msg));
        e.save_next = false;
      }
    }

    friend class Subscriber<M>;
    friend class Publisher<M>;
  };

  template<class M>
  History<M> History<M>::instance;
} // namespace haros

#endif // HAROS_ASSERT_HISTORY_H
