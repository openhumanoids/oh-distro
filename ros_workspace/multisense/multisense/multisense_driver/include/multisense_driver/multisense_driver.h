#ifndef MULTISENSE_DRIVER_MULTISENSE_DRIVER_H_
#define MULTISENSE_DRIVER_MULTISENSE_DRIVER_H_


#include <multisense_driver/multisense_subscriber.h>
#include <boost/shared_ptr.hpp>
#include <boost/signals2.hpp>
#include <boost/thread.hpp>
#include <map>
#include <LibSensorPodCommunications/SensorPodCommunications.h>
#include <cstdio>

namespace multisense_driver
{

/**
 * @class SignalWrapper
 * @brief Wrapper around a typed signal so that we can store them in one std map
 */
class SignalWrapper
{
 public:
  /**
   * @brief   Calls a callback for a given message
   * @param  msg The message to process
   */
  //TODO: MAKE THIS CONST CORRECT, NEEDS FIX TO DESERIALIZE IN ALL MESSAGE STRUCTS
  //Can't do this until the driver switches from stringstream
  //virtual void call(const SensorPodMessageBuffer& msg) = 0;
  virtual void call(SensorPodMessageBuffer& msg) = 0;
};

/**
 * @class SignalWrapperTemplated
 * @brief The actual implementation of a signal wrapper
 */
template <class T>
class SignalWrapperTemplated : public SignalWrapper
{
 public:
  typedef const boost::shared_ptr<const T> MessageConstPtr;
  typedef boost::function<void (MessageConstPtr&)> CallbackType;

  SignalWrapperTemplated()
  {
  }

  /**
   * @brief   Connects this signal wrapper to a callback for a certain message
   * @param   cb The callback to connect to this signal
   * @return  A connection object that allows for the callback to be
   * disconnected from the signal later if desired
   */
  boost::signals2::connection connect(const CallbackType& cb)
  {
    boost::signals2::connection c = sig_.connect(cb);
    return c;
  }

  /**
   * @brief   Calls a callback for a given message
   * @param  msg The message to process
   */
  //TODO: MAKE THIS CONST CORRECT, NEEDS FIX TO DESERIALIZE IN ALL MESSAGE STRUCTS
  //Can't do this until the driver switches from stringstream
  //void call(const SensorPodMessageBuffer& msg)
  void call(SensorPodMessageBuffer& msg)
  {
    //convert the generic message to the proper type
    boost::shared_ptr<T> p(new T);
    p->deserialize(msg);
    sig_(p);
  }

 private:
  boost::signals2::signal<void (MessageConstPtr&)> sig_; /**< @brief The underlying boost signal */
};

/**
 * @class MultisenseDriver
 * @brief A driver class for the Multisense unit. Allows users to register
 * callbacks for different data streams as well as to publish information to the
 * driver
 */
class MultisenseDriver
{
 public:
  /**
   * @brief   Constructor for the driver
   * @param
   */
  MultisenseDriver(const std::string& dest_ip);

  /**
   * @brief   Publishes a request to the sensor
   * @param  msg The message to publish
   */
  template <class T>
  //TODO: MAKE THIS CONST CORRECT, NEEDS FIX TO SERIALIZE IN ALL MESSAGE STRUCTS
  //OK, we actually can't do this because of the reliance on stringstreams in
  //the driver code. This should probably be fixed at some point, but I'm not
  //going to do it now
  //void publish(const T& msg)
  void publish(T& msg)
  {
    SensorPodMessageBuffer message_buffer(sensor_pod_address_, sensor_pod_command_port_);
    msg.serialize(message_buffer);
    //TODO: Do we need to put this in some sort of retry loop like in their
    //example program, or can we handle that elsewhere
    comm_.publish(message_buffer);
  }

  /**
   * @brief   Subscribe to a certain message stream and receive callbacks
   * @param  msg_type The type of message to subscribe to
   * @param  cb The callback that should occur
   * @return
   */
  template <class T>
  MultisenseSubscriber<T> subscribe(const typename SignalWrapperTemplated<T>::CallbackType& cb)
  {
    uint8_t msg_type = T::MSG_ID;
    std::map<uint8_t, boost::shared_ptr<SignalWrapper> >::iterator sig_it = signal_map_.find(msg_type);

    if(sig_it == signal_map_.end())
    {
      fprintf(stderr, "Message type %u is unsupported!!!!!!!.\n", msg_type);
      throw std::runtime_error("Invalid message type.");
    }

    boost::shared_ptr<SignalWrapperTemplated<T> > msg_sig = boost::dynamic_pointer_cast<SignalWrapperTemplated<T> >(sig_it->second);
    if(!msg_sig)
    {
      fprintf(stderr, "Dynamic cast to specific type failed: %u!!!!!!\n", msg_type);
      throw std::runtime_error("Dynamic cast failed");
    }

    MultisenseSubscriber<T> sub;

    typename SignalWrapperTemplated<T>::CallbackType sub_cb(sub.getCallback());
    boost::signals2::connection c = msg_sig->connect(sub_cb);
    sub.init(c, cb);

    return sub;
  }
 private:
  friend void listener(SensorPodMessageBuffer&, const void*);
  /**
   * @brief   Processes a message when it comes over the wire
   * @param  message_buffer The message to be processed
   */
  void processMessage(SensorPodMessageBuffer& message_buffer);


  std::map<uint8_t, boost::shared_ptr<SignalWrapper> > signal_map_; /**< @brief A map that holds a signaler for each message type */
  SensorPodCommunications comm_; /**< @brief The Carnegie Robotics provided communication layer to the sensor */
  int sensor_pod_address_; /**< @brief The address on which to connect to the sensor */
  unsigned int sensor_pod_command_port_; /**< @brief The post on which to connect to the sensor */

};


}
#endif
