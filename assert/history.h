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

#include <cstdint>
#include <string>
#include <map>

#include "message_event.h"

namespace haros
{
  /** History will remain alive for the program duration, since everything else is
   * non-unique and/or temporary (NodeHandle, Subscriber, Publisher).
   * Since History is shared across multiple threads, it has to
   * implement concurrency control.
   */
  class History
  {
  public:
    static const History instance;


    MessageEvent lastReceive(const std::string& topic);

    void receive(const std::string& topic,
                 const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event);
    // msg_event.getMessage() is of type topic_tools::ShapeShifter::ConstPtr

  private:
    History();

    /** An Entry has to be allocated on the heap.
     *  It must stay alive for as long as History, since it will be
     *  the tracked object for the history callback.
     */
    struct Entry
    {
      std::string topic_;                   // registered topic
      uint32_t time_;                       // time of the last event
      boost::shared_ptr<void const> msg_;   // last message

      Entry(const std::string& topic);

      
    };

    uint32_t clock_;
    std::map<std::string, MessageEventPtr> received_;
    boost::mutex mutex_;
  };
} // namespace haros

#endif // HAROS_ASSERT_HISTORY_H
