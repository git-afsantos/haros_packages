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

#ifndef HAROS_ASSERT_NODE_HANDLE_H
#define HAROS_ASSERT_NODE_HANDLE_H

#include <cstdint>
#include <string>

#include <boost/core/ref.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>

#include "history.h"
#include "publisher.h"
#include "subscriber.h"

namespace haros
{
  typedef std::map<std::string, std::string> M_string;


  class NodeHandle : public ros::NodeHandle
  {
  public:
    // The following constructors should do the same as the regular ros::NodeHandle
    // ones, except for the initialisation of additional variables.
    NodeHandle(const std::string& ns = std::string(),
               const M_string& remappings = M_string());
    NodeHandle(const ros::NodeHandle& rhs);
    NodeHandle(const NodeHandle& rhs);
    NodeHandle(const ros::NodeHandle& parent, const std::string& ns);
    NodeHandle(const ros::NodeHandle& parent, const std::string& ns,
               const M_string& remappings);
    ~NodeHandle();

    ////////////////////////////////////////////////////////////////////////////
    // Versions of advertise()
    ////////////////////////////////////////////////////////////////////////////

    // all inherited, except for
    // Publisher advertise(AdvertiseOptions& ops);

    ////////////////////////////////////////////////////////////////////////////
    // Versions of subscribe()
    ////////////////////////////////////////////////////////////////////////////

    // TODO: define all other versions of subscribe.
    // Since the functions in ros::NodeHandle are not virtual, this is not
    // true polymorphism and method overriding, it is just name hiding.
    // We must re-define all functions, so that they call our version.

    // TODO: if we ever need to compact callbacks into one with bindings:
    // typedef typename ros::ParameterAdapter<M>::Message MessageType;
    // create callback for const MessageType::ConstPtr&

    /** Subscribe to a topic, version for class member function with bare pointer */
    template<class M, class T>
    Subscriber subscribe(const std::string& topic, uint32_t queue_size,
                         void(T::*fp)(M), T* obj,
                         const ros::TransportHints& transport_hints = ros::TransportHints())
    {
      ros::SubscribeOptions ops;
      ops.template initByFullCallbackType<M>(topic, queue_size, boost::bind(fp, obj, _1));
      ops.transport_hints = transport_hints;
      return subscribe(ops);
    }

    /** Subscribe to a topic, version for const class member function with bare pointer */
    template<class M, class T>
    Subscriber subscribe(const std::string& topic, uint32_t queue_size,
                         void(T::*fp)(M) const, T* obj,
                         const ros::TransportHints& transport_hints = ros::TransportHints())
    {
      ros::SubscribeOptions ops;
      ops.template initByFullCallbackType<M>(topic, queue_size, boost::bind(fp, obj, _1));
      ops.transport_hints = transport_hints;
      return subscribe(ops);
    }

    /** Subscribe to a topic, version for class member function with bare pointer */
    template<class M, class T>
    Subscriber subscribe(const std::string& topic, uint32_t queue_size, 
                         void(T::*fp)(const boost::shared_ptr<M const>&), T* obj, 
                         const ros::TransportHints& transport_hints = ros::TransportHints())
    {
      ros::SubscribeOptions ops;
      ops.template init<M>(topic, queue_size, boost::bind(fp, obj, _1));
      ops.transport_hints = transport_hints;
      return subscribe(ops);
    }

    /** Subscribe to a topic, version for const class member function with bare pointer */
    template<class M, class T>
    Subscriber subscribe(const std::string& topic, uint32_t queue_size, 
                         void(T::*fp)(const boost::shared_ptr<M const>&) const, T* obj, 
                         const ros::TransportHints& transport_hints = ros::TransportHints())
    {
      ros::SubscribeOptions ops;
      ops.template init<M>(topic, queue_size, boost::bind(fp, obj, _1));
      ops.transport_hints = transport_hints;
      return subscribe(ops);
    }

    /** Subscribe to a topic, version for class member function with shared_ptr */
    template<class M, class T>
    Subscriber subscribe(const std::string& topic, uint32_t queue_size,
                         void(T::*fp)(M), const boost::shared_ptr<T>& obj,
                         const ros::TransportHints& transport_hints = ros::TransportHints())
    {
      ros::SubscribeOptions ops;
      ops.template initByFullCallbackType<M>(topic, queue_size, boost::bind(fp, obj.get(), _1));
      ops.tracked_object = obj;
      ops.transport_hints = transport_hints;
      return subscribe(ops);
    }

    /** Subscribe to a topic, version for const class member function with shared_ptr */
    template<class M, class T>
    Subscriber subscribe(const std::string& topic, uint32_t queue_size,
                         void(T::*fp)(M) const, const boost::shared_ptr<T>& obj,
                         const ros::TransportHints& transport_hints = ros::TransportHints())
    {
      ros::SubscribeOptions ops;
      ops.template initByFullCallbackType<M>(topic, queue_size, boost::bind(fp, obj.get(), _1));
      ops.tracked_object = obj;
      ops.transport_hints = transport_hints;
      return subscribe(ops);
    }

    /** Subscribe to a topic, version for class member function with shared_ptr */
    template<class M, class T>
    Subscriber subscribe(const std::string& topic, uint32_t queue_size, 
                         void(T::*fp)(const boost::shared_ptr<M const>&), 
                         const boost::shared_ptr<T>& obj,
                         const ros::TransportHints& transport_hints = ros::TransportHints())
    {
      ros::SubscribeOptions ops;
      ops.template init<M>(topic, queue_size, boost::bind(fp, obj.get(), _1));
      ops.tracked_object = obj;
      ops.transport_hints = transport_hints;
      return subscribe(ops);
    }

    /** Subscribe to a topic, version for const class member function with shared_ptr */
    template<class M, class T>
    Subscriber subscribe(const std::string& topic, uint32_t queue_size,
                         void(T::*fp)(const boost::shared_ptr<M const>&) const,
                         const boost::shared_ptr<T>& obj,
                         const ros::TransportHints& transport_hints = ros::TransportHints())
    {
      ros::SubscribeOptions ops;
      ops.template init<M>(topic, queue_size, boost::bind(fp, obj.get(), _1));
      ops.tracked_object = obj;
      ops.transport_hints = transport_hints;
      return subscribe(ops);
    }

    /** Subscribe to a topic, version for bare function */
    template<class M>
    Subscriber subscribe(const std::string& topic, uint32_t queue_size, void(*fp)(M),
                         const ros::TransportHints& transport_hints = ros::TransportHints())
    {
      ros::SubscribeOptions ops;
      ops.template initByFullCallbackType<M>(topic, queue_size, fp);
      ops.transport_hints = transport_hints;
      return subscribe(ops);
    }

    /** Subscribe to a topic, version for bare function */
    template<class M>
    Subscriber subscribe(const std::string& topic, uint32_t queue_size,
                         void(*fp)(const boost::shared_ptr<M const>&),
                         const ros::TransportHints& transport_hints = ros::TransportHints())
    {
      ros::SubscribeOptions ops;
      ops.template init<M>(topic, queue_size, fp);
      ops.transport_hints = transport_hints;
      return subscribe(ops);
    }

    /** Subscribe to a topic, version for arbitrary boost::function object */
    template<class M>
    Subscriber subscribe(const std::string& topic, uint32_t queue_size,
                         const boost::function<void (const boost::shared_ptr<M const>&)>& callback,
                         const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr(),
                         const ros::TransportHints& transport_hints = ros::TransportHints())
    {
      ros::SubscribeOptions ops;
      ops.template init<M>(topic, queue_size, callback);
      ops.tracked_object = tracked_object;
      ops.transport_hints = transport_hints;
      return subscribe(ops);
    }

    /** Subscribe to a topic, version for arbitrary boost::function object */
    template<class M, class C>
    Subscriber subscribe(const std::string& topic, uint32_t queue_size,
                         const boost::function<void (C)>& callback,
                         const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr(),
                         const ros::TransportHints& transport_hints = ros::TransportHints())
    {
      ros::SubscribeOptions ops;
      ops.template initByFullCallbackType<C>(topic, queue_size, callback);
      ops.tracked_object = tracked_object;
      ops.transport_hints = transport_hints;
      return subscribe(ops);
    }

    /** Subscribe to a topic, version with full range of SubscribeOptions */
    Subscriber subscribe(ros::SubscribeOptions& ops);

    ////////////////////////////////////////////////////////////////////////////
    // HAROS Interface
    ////////////////////////////////////////////////////////////////////////////

    MessageEvent lastReceive(const std::string& topic)
    {
      return History::instance.lastReceive(topic);
    }
  };
} // namespace haros

#endif // HAROS_ASSERT_NODE_HANDLE_H
