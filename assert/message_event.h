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

#ifndef HAROS_ASSERT_MESSAGE_EVENT_H
#define HAROS_ASSERT_MESSAGE_EVENT_H

#include <cstdint>
#include <boost/shared_ptr.hpp>

namespace haros
{
  /** This is supposed to be a very lightweight object. It is basically a pair
   * of a clock and a (shared) pointer to a message. Thus, it can be copyable.
   * Can also be stored directly, instead of storing a pointer.
   */
  class MessageEvent
  {
  public:
    MessageEvent();
    MessageEvent(const uint32_t time, const boost::shared_ptr<void> data);

    template<class M>
    boost::shared_ptr<M> msg();
    // return boost::static_pointer_cast<M>(data_);

    bool hasOccurred();

    bool operator < (const MessageEvent& me) const;
    bool operator > (const MessageEvent& me) const;
    bool operator == (const MessageEvent& me) const;

  private:
    uint32_t time_;
    boost::shared_ptr<void const> data_;
  };

  typedef MessageEventPtr boost::shared_ptr<MessageEvent>;
} // namespace haros

#endif // HAROS_ASSERT_MESSAGE_EVENT_H
