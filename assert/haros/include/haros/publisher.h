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

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include "haros/history.h"

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

    Publisher(const ros::Publisher& pub)
    : ros_pub_(pub)
    {}

    Publisher(const Publisher<M>& rhs)
    : ros_pub_(rhs.ros_pub_)
    {}

    ~Publisher() {}

    //---------------------------------------------------------------------------
    // HAROS Interface
    //---------------------------------------------------------------------------

    PublishEvent<M> lastPublish() const
    {
      return History<M>::instance.lastPublish(ros_pub_.getTopic());
    }

    boost::shared_ptr<M> lastMessage() const
    {
      return lastPublish().msg;
    }

    //---------------------------------------------------------------------------
    // ROS Publisher Interface
    //---------------------------------------------------------------------------

    /**
     * \brief Publish a message on the topic associated with this Publisher.
     *
     * This version of publish will allow fast intra-process message-passing in the future,
     * so you may not mutate the message after it has been passed in here (since it will be
     * passed directly into a callback function)
     *
     */
    void publish(const boost::shared_ptr<M>& message) const
    {
      if (ros_pub_)
      {
        ros_pub_.publish(message);
        History<M>::instance.publish(ros_pub_.getTopic(), message);
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
        History<M>::instance.publish(ros_pub_.getTopic(),
                                     boost::shared_ptr<M>(new M(message)));
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
      return ros_pub_.getNumPublishers();
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
    ros::Publisher ros_pub_;
  };
} // namespace haros

#endif // HAROS_ASSERT_PUBLISHER_H
