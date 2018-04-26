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

#include <string>

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>

#include "haros/history.h"
#include "haros/publisher.h"
#include "haros/subscriber.h"

namespace haros
{
  typedef std::map<std::string, std::string> M_string;


  class NodeHandle
  {
  public:
    //---------------------------------------------------------------------------
    // Constructors and Destructors
    //---------------------------------------------------------------------------

    /**
     * \brief Constructor
     *
     * When a NodeHandle is constructed, it checks to see if the global
     * node state has already been started.  If so, it increments a
     * global reference count.  If not, it starts the node with
     * ros::start() and sets the reference count to 1.
     *
     * \param ns Namespace for this NodeHandle.  This acts in addition to any namespace assigned to this ROS node.
     *           eg. If the node's namespace is "/a" and the namespace passed in here is "b", all 
     *           topics/services/parameters will be prefixed with "/a/b/"
     * \param remappings Remappings for this NodeHandle.
     * \throws InvalidNameException if the namespace is not a valid graph resource name
     */
    NodeHandle(const std::string& ns = std::string(),
               const M_string& remappings = M_string());

    /**
     * \brief Copy constructor
     *
     * When a NodeHandle is copied, it inherits the namespace of the
     * NodeHandle being copied, and increments the reference count of
     * the global node state by 1.
     */
    NodeHandle(const ros::NodeHandle& rhs);

    /**
     * \brief Copy constructor
     *
     * When a NodeHandle is copied, it inherits the namespace of the
     * NodeHandle being copied, and increments the reference count of
     * the global node state by 1.
     */
    NodeHandle(const NodeHandle& rhs);

    /**
     * \brief Parent constructor
     *
     * This version of the constructor takes a "parent" NodeHandle.
     * If the passed "ns" is relative (does not start with a slash), it is equivalent to calling:
     \verbatim
     NodeHandle child(parent.getNamespace() + "/" + ns);
     \endverbatim
     *
     * If the passed "ns" is absolute (does start with a slash), it is equivalent to calling:
     \verbatim
     NodeHandle child(ns);
     \endverbatim
     *
     * When a NodeHandle is copied, it inherits the namespace of the
     * NodeHandle being copied, and increments the reference count of
     * the global node state by 1.
     *
     * \throws InvalidNameException if the namespace is not a valid
     * graph resource name
     */
    NodeHandle(const ros::NodeHandle& parent, const std::string& ns);

    /**
     * \brief Parent constructor
     *
     * This version of the constructor takes a "parent" NodeHandle.
     * If the passed "ns" is relative (does not start with a slash), it is equivalent to calling:
     \verbatim
     NodeHandle child(parent.getNamespace() + "/" + ns, remappings);
     \endverbatim
     *
     * If the passed "ns" is absolute (does start with a slash), it is equivalent to calling:
     \verbatim
     NodeHandle child(ns, remappings);
     \endverbatim
     *
     * This version also lets you pass in name remappings that are specific to this NodeHandle
     *
     * When a NodeHandle is copied, it inherits the namespace of the NodeHandle being copied, 
     * and increments the reference count of the global node state
     * by 1.
     * \throws InvalidNameException if the namespace is not a valid graph resource name
     */
    NodeHandle(const ros::NodeHandle& parent, const std::string& ns,
               const M_string& remappings);

    /**
     * \brief Destructor
     *
     * When a NodeHandle is destroyed, it decrements a global reference
     * count by 1, and if the reference count is now 0, shuts down the
     * node.
     */
    ~NodeHandle();

    //---------------------------------------------------------------------------
    // Versions of advertise()
    //---------------------------------------------------------------------------

    /**
     * \brief Advertise a topic, simple version
     *
     * This call connects to the master to publicize that the node will be
     * publishing messages on the given topic.
     * This method returns a Publisher that allows you to
     * publish a message on this topic.
     *
     * \param topic Topic to advertise on
     *
     * \param queue_size Maximum number of outgoing messages to be
     * queued for delivery to subscribers
     *
     * \param latch (optional) If true, the last message published on
     * this topic will be saved and sent to new subscribers when they
     * connect
     *
     * \return On success, a Publisher that, when it goes out of scope,
     * will automatically release a reference on this advertisement.  On
     * failure, an empty Publisher.
     *
     * \throws InvalidNameException If the topic name begins with a
     * tilde, or is an otherwise invalid graph resource name, or is an
     * otherwise invalid graph resource name
     */
    template <class M>
    Publisher advertise(const std::string& topic, uint32_t queue_size,
                        bool latch = false)
    {
      return Publisher(ros::NodeHandle::advertise<M>(topic, queue_size, latch));
    }

    /**
     * \brief Advertise a topic, with most of the available options, including subscriber status callbacks
     *
     * This call connects to the master to publicize that the node will be
     * publishing messages on the given topic.  This method returns a Publisher that allows you to
     * publish a message on this topic.
     *
     * This version of advertise allows you to pass functions to be called when
     * new subscribers connect and disconnect.
     *
     * \param topic Topic to advertise on
     *
     * \param queue_size Maximum number of outgoing messages to be queued for delivery to subscribers
     *
     * \param connect_cb Function to call when a subscriber connects
     *
     * \param disconnect_cb (optional) Function to call when a subscriber disconnects
     *
     * \param tracked_object (optional) A shared pointer to an object to track for these callbacks.  If set, the a weak_ptr will be created to this object,
     * and if the reference count goes to 0 the subscriber callbacks will not get called.
     * Note that setting this will cause a new reference to be added to the object before the
     * callback, and for it to go out of scope (and potentially be deleted) in the code path (and therefore
     * thread) that the callback is invoked from.
     * \param latch (optional) If true, the last message published on this topic will be saved and sent to new subscribers when they connect
     * \return On success, a Publisher that, when it goes out of scope, will automatically release a reference
     * on this advertisement.  On failure, an empty Publisher.
     *
     * \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
     */
    template <class M>
    Publisher advertise(const std::string& topic, uint32_t queue_size,
        const ros::SubscriberStatusCallback& connect_cb,
        const ros::SubscriberStatusCallback& disconnect_cb = ros::SubscriberStatusCallback(),
        const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr(),
        bool latch = false)
    {
      return Publisher(ros::NodeHandle::advertise<M>(topic, queue_size, connect_cb,
          disconnect_cb, tracked_object, latch));
    }

    /**
     * \brief Advertise a topic, with full range of AdvertiseOptions
     *
     * This call connects to the master to publicize that the node will be
     * publishing messages on the given topic.  This method returns a Publisher that allows you to
     * publish a message on this topic.
     *
     * This is an advanced version advertise() that exposes all options (through the AdvertiseOptions structure)
     *
     * \param ops Advertise options to use
     * \return On success, a Publisher that, when it goes out of scope, will automatically release a reference
     * on this advertisement.  On failure, an empty Publisher.
     *
     * \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
     */
    Publisher advertise(ros::AdvertiseOptions& ops);

    //---------------------------------------------------------------------------
    // Versions of subscribe()
    //---------------------------------------------------------------------------

    // TODO: if we ever need to compact callbacks into one with bindings:
    // typedef typename ros::ParameterAdapter<M>::Message MessageType;
    // create callback for const MessageType::ConstPtr&

    /**
     * \brief Subscribe to a topic, version for class member function with bare pointer
     *
     * This method connects to the master to register interest in a given
     * topic.  The node will automatically be connected with publishers on
     * this topic.  On each message receipt, fp is invoked and passed a shared pointer
     * to the message received.  This message should \b not be changed in place, as it
     * is shared with any other subscriptions to this topic.
     *
     * This version of subscribe is a convenience function for using member functions.
     *
     * \param M [template] M here is the callback parameter type (e.g. const boost::shared_ptr<M const>& or const M&), \b not the message type, and should almost always be deduced
     * \param topic Topic to subscribe to
     * \param queue_size Number of incoming messages to queue up for
     * processing (messages in excess of this queue capacity will be
     * discarded).
     * \param fp Member function pointer to call when a message has arrived
     * \param obj Object to call fp on
     * \param transport_hints a TransportHints structure which defines various transport-related options
     * \return On success, a Subscriber that, when all copies of it go out of scope, will unsubscribe from this topic.
     * On failure, an empty Subscriber.
     *  \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
     *  \throws ConflictingSubscriptionException If this node is already subscribed to the same topic with a different datatype
     */
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

    /**
     * \brief Subscribe to a topic, version for class member function with bare pointer
     *
     * This method connects to the master to register interest in a given
     * topic.  The node will automatically be connected with publishers on
     * this topic.  On each message receipt, fp is invoked and passed a shared pointer
     * to the message received.  This message should \b not be changed in place, as it
     * is shared with any other subscriptions to this topic.
     *
     * This version of subscribe is a convenience function for using member functions.
     *
     * \param M [template] M here is the callback parameter type (e.g. const boost::shared_ptr<M const>& or const M&), \b not the message type, and should almost always be deduced
     * \param topic Topic to subscribe to
     * \param queue_size Number of incoming messages to queue up for
     * processing (messages in excess of this queue capacity will be
     * discarded).
     * \param fp Member function pointer to call when a message has arrived
     * \param obj Object to call fp on
     * \param transport_hints a TransportHints structure which defines various transport-related options
     * \return On success, a Subscriber that, when all copies of it go out of scope, will unsubscribe from this topic.
     * On failure, an empty Subscriber.
     *  \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
     *  \throws ConflictingSubscriptionException If this node is already subscribed to the same topic with a different datatype
     */
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

    /**
     * \brief Subscribe to a topic, version for class member function with bare pointer
     *
     * This method connects to the master to register interest in a given
     * topic.  The node will automatically be connected with publishers on
     * this topic.  On each message receipt, fp is invoked and passed a shared pointer
     * to the message received.  This message should \b not be changed in place, as it
     * is shared with any other subscriptions to this topic.
     *
     * This version of subscribe is a convenience function for using member functions.
     *
     * \param M [template] M here is the message type
     * \param topic Topic to subscribe to
     * \param queue_size Number of incoming messages to queue up for
     * processing (messages in excess of this queue capacity will be
     * discarded).
     * \param fp Member function pointer to call when a message has arrived
     * \param obj Object to call fp on
     * \param transport_hints a TransportHints structure which defines various transport-related options
     * \return On success, a Subscriber that, when all copies of it go out of scope, will unsubscribe from this topic.
     * On failure, an empty Subscriber.
     *  \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
     *  \throws ConflictingSubscriptionException If this node is already subscribed to the same topic with a different datatype
     */
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

    /**
     * \brief Subscribe to a topic, version for class member function with bare pointer
     *
     * This method connects to the master to register interest in a given
     * topic.  The node will automatically be connected with publishers on
     * this topic.  On each message receipt, fp is invoked and passed a shared pointer
     * to the message received.  This message should \b not be changed in place, as it
     * is shared with any other subscriptions to this topic.
     *
     * This version of subscribe is a convenience function for using member functions.
     *
     * \param M [template] M here is the message type
     * \param topic Topic to subscribe to
     * \param queue_size Number of incoming messages to queue up for
     * processing (messages in excess of this queue capacity will be
     * discarded).
     * \param fp Member function pointer to call when a message has arrived
     * \param obj Object to call fp on
     * \param transport_hints a TransportHints structure which defines various transport-related options
     * \return On success, a Subscriber that, when all copies of it go out of scope, will unsubscribe from this topic.
     * On failure, an empty Subscriber.
     *  \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
     *  \throws ConflictingSubscriptionException If this node is already subscribed to the same topic with a different datatype
     */
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

    /**
     * \brief Subscribe to a topic, version for class member function with shared_ptr
     *
     * This method connects to the master to register interest in a given
     * topic.  The node will automatically be connected with publishers on
     * this topic.  On each message receipt, fp is invoked and passed a shared pointer
     * to the message received.  This message should \b not be changed in place, as it
     * is shared with any other subscriptions to this topic.
     *
     * This version of subscribe is a convenience function for using member functions on a shared_ptr.
     *
     * \param M [template] M here is the callback parameter type (e.g. const boost::shared_ptr<M const>& or const M&), \b not the message type, and should almost always be deduced
     * \param topic Topic to subscribe to
     * \param queue_size Number of incoming messages to queue up for
     * processing (messages in excess of this queue capacity will be
     * discarded).
     * \param fp Member function pointer to call when a message has arrived
     * \param obj Object to call fp on.  Since this is a shared pointer, the object will automatically be tracked with a weak_ptr
     * so that if it is deleted before the Subscriber goes out of scope the callback will no longer be called (and therefore will not crash).
     * \param transport_hints a TransportHints structure which defines various transport-related options
     * \return On success, a Subscriber that, when all copies of it go out of scope, will unsubscribe from this topic.
     * On failure, an empty Subscriber.
     *  \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
     *  \throws ConflictingSubscriptionException If this node is already subscribed to the same topic with a different datatype
     */
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

    /**
     * \brief Subscribe to a topic, version for class member function with shared_ptr
     *
     * This method connects to the master to register interest in a given
     * topic.  The node will automatically be connected with publishers on
     * this topic.  On each message receipt, fp is invoked and passed a shared pointer
     * to the message received.  This message should \b not be changed in place, as it
     * is shared with any other subscriptions to this topic.
     *
     * This version of subscribe is a convenience function for using member functions on a shared_ptr.
     *
     * \param M [template] M here is the callback parameter type (e.g. const boost::shared_ptr<M const>& or const M&), \b not the message type, and should almost always be deduced
     * \param topic Topic to subscribe to
     * \param queue_size Number of incoming messages to queue up for
     * processing (messages in excess of this queue capacity will be
     * discarded).
     * \param fp Member function pointer to call when a message has arrived
     * \param obj Object to call fp on.  Since this is a shared pointer, the object will automatically be tracked with a weak_ptr
     * so that if it is deleted before the Subscriber goes out of scope the callback will no longer be called (and therefore will not crash).
     * \param transport_hints a TransportHints structure which defines various transport-related options
     * \return On success, a Subscriber that, when all copies of it go out of scope, will unsubscribe from this topic.
     * On failure, an empty Subscriber.
     *  \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
     *  \throws ConflictingSubscriptionException If this node is already subscribed to the same topic with a different datatype
     */
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

    /**
     * \brief Subscribe to a topic, version for class member function with shared_ptr
     *
     * This method connects to the master to register interest in a given
     * topic.  The node will automatically be connected with publishers on
     * this topic.  On each message receipt, fp is invoked and passed a shared pointer
     * to the message received.  This message should \b not be changed in place, as it
     * is shared with any other subscriptions to this topic.
     *
     * This version of subscribe is a convenience function for using member functions on a shared_ptr.
     *
     * \param M [template] M here is the message type
     * \param topic Topic to subscribe to
     * \param queue_size Number of incoming messages to queue up for
     * processing (messages in excess of this queue capacity will be
     * discarded).
     * \param fp Member function pointer to call when a message has arrived
     * \param obj Object to call fp on.  Since this is a shared pointer, the object will automatically be tracked with a weak_ptr
     * so that if it is deleted before the Subscriber goes out of scope the callback will no longer be called (and therefore will not crash).
     * \param transport_hints a TransportHints structure which defines various transport-related options
     * \return On success, a Subscriber that, when all copies of it go out of scope, will unsubscribe from this topic.
     * On failure, an empty Subscriber.
     *  \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
     *  \throws ConflictingSubscriptionException If this node is already subscribed to the same topic with a different datatype
     */
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

    /**
     * \brief Subscribe to a topic, version for class member function with shared_ptr
     *
     * This method connects to the master to register interest in a given
     * topic.  The node will automatically be connected with publishers on
     * this topic.  On each message receipt, fp is invoked and passed a shared pointer
     * to the message received.  This message should \b not be changed in place, as it
     * is shared with any other subscriptions to this topic.
     *
     * This version of subscribe is a convenience function for using member functions on a shared_ptr.
     *
     * \param M [template] M here is the message type
     * \param topic Topic to subscribe to
     * \param queue_size Number of incoming messages to queue up for
     * processing (messages in excess of this queue capacity will be
     * discarded).
     * \param fp Member function pointer to call when a message has arrived
     * \param obj Object to call fp on.  Since this is a shared pointer, the object will automatically be tracked with a weak_ptr
     * so that if it is deleted before the Subscriber goes out of scope the callback will no longer be called (and therefore will not crash).
     * \param transport_hints a TransportHints structure which defines various transport-related options
     * \return On success, a Subscriber that, when all copies of it go out of scope, will unsubscribe from this topic.
     * On failure, an empty Subscriber.
     *  \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
     *  \throws ConflictingSubscriptionException If this node is already subscribed to the same topic with a different datatype
     */
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

    /**
     * \brief Subscribe to a topic, version for bare function
     *
     * This method connects to the master to register interest in a given
     * topic.  The node will automatically be connected with publishers on
     * this topic.  On each message receipt, fp is invoked and passed a shared pointer
     * to the message received.  This message should \b not be changed in place, as it
     * is shared with any other subscriptions to this topic.
     *
     * This version of subscribe is a convenience function for using bare functions.
     *
     * \param M [template] M here is the callback parameter type (e.g. const boost::shared_ptr<M const>& or const M&), \b not the message type, and should almost always be deduced
     * \param topic Topic to subscribe to
     * \param queue_size Number of incoming messages to queue up for
     * processing (messages in excess of this queue capacity will be
     * discarded).
     * \param fp Function pointer to call when a message has arrived
     * \param transport_hints a TransportHints structure which defines various transport-related options
     * \return On success, a Subscriber that, when all copies of it go out of scope, will unsubscribe from this topic.
     * On failure, an empty Subscriber.
     *  \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
     *  \throws ConflictingSubscriptionException If this node is already subscribed to the same topic with a different datatype
     */
    template<class M>
    Subscriber subscribe(const std::string& topic, uint32_t queue_size, void(*fp)(M),
                         const ros::TransportHints& transport_hints = ros::TransportHints())
    {
      ros::SubscribeOptions ops;
      ops.template initByFullCallbackType<M>(topic, queue_size, fp);
      ops.transport_hints = transport_hints;
      return subscribe(ops);
    }

    /**
     * \brief Subscribe to a topic, version for bare function
     *
     * This method connects to the master to register interest in a given
     * topic.  The node will automatically be connected with publishers on
     * this topic.  On each message receipt, fp is invoked and passed a shared pointer
     * to the message received.  This message should \b not be changed in place, as it
     * is shared with any other subscriptions to this topic.
     *
     * This version of subscribe is a convenience function for using bare functions.
     *
     * \param M [template] M here is the message type
     * \param topic Topic to subscribe to
     * \param queue_size Number of incoming messages to queue up for
     * processing (messages in excess of this queue capacity will be
     * discarded).
     * \param fp Function pointer to call when a message has arrived
     * \param transport_hints a TransportHints structure which defines various transport-related options
     * \return On success, a Subscriber that, when all copies of it go out of scope, will unsubscribe from this topic.
     * On failure, an empty Subscriber.
     *  \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
     *  \throws ConflictingSubscriptionException If this node is already subscribed to the same topic with a different datatype
     */
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

    /**
     * \brief Subscribe to a topic, version for arbitrary boost::function object
     *
     * This method connects to the master to register interest in a given
     * topic.  The node will automatically be connected with publishers on
     * this topic.  On each message receipt, callback is invoked and passed a shared pointer
     * to the message received.  This message should \b not be changed in place, as it
     * is shared with any other subscriptions to this topic.
     *
     * This version of subscribe allows anything bindable to a boost::function object
     *
     * \param M [template] M here is the message type
     * \param topic Topic to subscribe to
     * \param queue_size Number of incoming messages to queue up for
     * processing (messages in excess of this queue capacity will be
     * discarded).
     * \param callback Callback to call when a message has arrived
     * \param tracked_object A shared pointer to an object to track for these callbacks.  If set, the a weak_ptr will be created to this object,
     * and if the reference count goes to 0 the subscriber callbacks will not get called.
     * Note that setting this will cause a new reference to be added to the object before the
     * callback, and for it to go out of scope (and potentially be deleted) in the code path (and therefore
     * thread) that the callback is invoked from.
     * \param transport_hints a TransportHints structure which defines various transport-related options
     * \return On success, a Subscriber that, when all copies of it go out of scope, will unsubscribe from this topic.
     * On failure, an empty Subscriber.
     *  \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
     *  \throws ConflictingSubscriptionException If this node is already subscribed to the same topic with a different datatype
     */
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

    /**
     * \brief Subscribe to a topic, version for arbitrary boost::function object
     *
     * This method connects to the master to register interest in a given
     * topic.  The node will automatically be connected with publishers on
     * this topic.  On each message receipt, callback is invoked and passed a shared pointer
     * to the message received.  This message should \b not be changed in place, as it
     * is shared with any other subscriptions to this topic.
     *
     * This version of subscribe allows anything bindable to a boost::function object
     *
     * \param M [template] the message type
     * \param C [template] the callback parameter type (e.g. const boost::shared_ptr<M const>& or const M&)
     * \param topic Topic to subscribe to
     * \param queue_size Number of incoming messages to queue up for
     * processing (messages in excess of this queue capacity will be
     * discarded).
     * \param callback Callback to call when a message has arrived
     * \param tracked_object A shared pointer to an object to track for these callbacks.  If set, the a weak_ptr will be created to this object,
     * and if the reference count goes to 0 the subscriber callbacks will not get called.
     * Note that setting this will cause a new reference to be added to the object before the
     * callback, and for it to go out of scope (and potentially be deleted) in the code path (and therefore
     * thread) that the callback is invoked from.
     * \param transport_hints a TransportHints structure which defines various transport-related options
     * \return On success, a Subscriber that, when all copies of it go out of scope, will unsubscribe from this topic.
     * On failure, an empty Subscriber.
     *  \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
     *  \throws ConflictingSubscriptionException If this node is already subscribed to the same topic with a different datatype
     */
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

    /**
     * \brief Subscribe to a topic, version with full range of SubscribeOptions
     *
     * This method connects to the master to register interest in a given
     * topic.  The node will automatically be connected with publishers on
     * this topic.  On each message receipt, fp is invoked and passed a shared pointer
     * to the message received.  This message should \b not be changed in place, as it
     * is shared with any other subscriptions to this topic.
     *
     * This version of subscribe allows the full range of options, exposed through the SubscribeOptions class
     *
     * \param ops Subscribe options
     * \return On success, a Subscriber that, when all copies of it go out of scope, will unsubscribe from this topic.
     * On failure, an empty Subscriber.
     *  \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
     *  \throws ConflictingSubscriptionException If this node is already subscribed to the same topic with a different datatype
     */
    Subscriber subscribe(ros::SubscribeOptions& ops);

    //---------------------------------------------------------------------------
    // ROS NodeHandle Interface
    //---------------------------------------------------------------------------

    NodeHandle& operator=(const NodeHandle& rhs);

    /**
     * \brief Set the default callback queue to be used by this NodeHandle.
     *
     * Setting this will cause any callbacks from
     * advertisements/subscriptions/services/etc. to happen through the
     * use of the specified queue.  NULL (the default) causes the global
     * queue (serviced by ros::spin() and ros::spinOnce()) to be used.
     */
    void setCallbackQueue(ros::CallbackQueueInterface* queue)
    {
      node_handle_.setCallbackQueue(queue);
    }

    /**
     * \brief Returns the callback queue associated with this
     * NodeHandle.  If none has been explicitly set, returns the global
     * queue.
     */
    ros::CallbackQueueInterface* getCallbackQueue() const 
    { 
      return node_handle_.getCallbackQueue();
    }

    /**
     * \brief Returns the namespace associated with this NodeHandle
     */
    const std::string& getNamespace() const
    {
      return node_handle_.getNamespace();
    }

    /**
     * \brief Returns the namespace associated with this NodeHandle as
     * it was passed in (before it was resolved)
     */
    const std::string& getUnresolvedNamespace() const
    {
      return node_handle_.getUnresolvedNamespace();
    }

    /** \brief Resolves a name into a fully-qualified name
     *
     * Resolves a name into a fully qualified name, eg. "blah" =>
     * "/namespace/blah". By default also applies any matching
     * name-remapping rules (which were usually supplied on the command
     * line at startup) to the given name, returning the resulting
     * remapped name.
     *
     * \param name Name to remap
     *
     * \param remap Whether to apply name-remapping rules
     *
     * \return Resolved name.
     *
     * \throws InvalidNameException If the name begins with a tilde, or is an otherwise invalid graph resource name
     */
    std::string resolveName(const std::string& name, bool remap = true) const
    {
      return node_handle_.resolveName(name, remap);
    }

    /**
     * \brief Shutdown every handle created through this NodeHandle.
     *
     * This method will unadvertise every topic and service advertisement,
     * and unsubscribe every subscription created through this NodeHandle.
     */
    void shutdown()
    {
      node_handle_.shutdown();
    }

    /** \brief Check whether it's time to exit.
     *
     * This method checks to see if both ros::ok() is true and shutdown() has not been called on this NodeHandle, to see whether it's yet time
     * to exit.  ok() is false once either ros::shutdown() or NodeHandle::shutdown() have been called
     *
     * \return true if we're still OK, false if it's time to exit
     */
    bool ok() const
    {
      return node_handle_.ok();
    }

    ros::NodeHandle ros()
    {
      return node_handle_;
    }

  private:
    ros::NodeHandle node_handle_;
  };
} // namespace haros

#endif // HAROS_ASSERT_NODE_HANDLE_H
