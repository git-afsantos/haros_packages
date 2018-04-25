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

#include "haros/node_handle.h"

#include <boost/shared_ptr.hpp>

namespace haros
{
NodeHandle::NodeHandle(const std::string& ns, const M_string& remappings)
: ros::NodeHandle(ns, remappings)
{}

NodeHandle::NodeHandle(const ros::NodeHandle& rhs)
: ros::NodeHandle(rhs)
{}

NodeHandle::NodeHandle(const NodeHandle& rhs)
: ros::NodeHandle(rhs)
{}

NodeHandle::NodeHandle(const ros::NodeHandle& parent, const std::string& ns)
: ros::NodeHandle(parent, ns)
{}

NodeHandle::NodeHandle(const ros::NodeHandle& parent, const std::string& ns,
                       const M_string& remappings)
: ros::NodeHandle(parent, ns, remappings)
{}

NodeHandle::~NodeHandle() {}

Publisher NodeHandle::advertise(ros::AdvertiseOptions& ops)
{
  return Publisher(ros::NodeHandle::advertise(ops));
}

Subscriber NodeHandle::subscribe(ros::SubscribeOptions& ops)
{
  ros::Subscriber main_sub = ros::NodeHandle::subscribe(ops);
  // ops.topic is changed to the fully resolved topic
  if (main_sub)
  {
    History::HolderPtr history_sub =
        History::instance.subscribe(ops.topic, ops.queue_size);
    return Subscriber(main_sub, history_sub);
  }
  return Subscriber(main_sub);
}
} // namespace haros
