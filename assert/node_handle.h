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

#include <ros/transport_hints.h>
#include <ros/parameter_adapter.h>
#include <ros/subscribe_options.h>
#include <ros/publisher.h>
#include <ros/node_handle.h>

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
    NodeHandle(const std::string& ns = std::string(), const M_string& remappings = M_string());
    NodeHandle(const NodeHandle& rhs);
    NodeHandle(const NodeHandle& parent, const std::string& ns);
    NodeHandle(const NodeHandle& parent, const std::string& ns, const M_string& remappings);
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

    /** Subscribe to a topic, version with full range of SubscribeOptions */
    Subscriber subscribe(SubscribeOptions& ops);
    {
      ros::Subscriber main_sub = ros::NodeHandle::subscribe(ops);
      // ops.topic is changed to the fully resolved topic
      boost::shared_ptr<ros::Subscriber> history_sub =
          History::instance.subscribe(ops.topic, ops.queue_size);
      return Subscriber(main_sub, history_sub);
    }

    ////////////////////////////////////////////////////////////////////////////
    // HAROS Interface
    ////////////////////////////////////////////////////////////////////////////

    MessageEvent lastReceive(const std::string& topic);


  private:
    boost::shared_ptr<History> history_;

    template<class M, class P>
    void subscribeCallback(const std::string& topic, const M::ConstPtr& msg,
                           const boost::function<void (P)>& callback)
    {
      // this binding crap may not be needed after all, since ROS supports
      // multiple callbacks on the same topic.
      // I think it may not be as performant as a single callback, though.
      history_.template receive<M>(topic, msg);
      callback();
    }
  };
} // namespace haros

#endif // HAROS_ASSERT_NODE_HANDLE_H
