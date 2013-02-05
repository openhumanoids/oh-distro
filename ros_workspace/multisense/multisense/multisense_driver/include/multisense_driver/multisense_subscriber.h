#ifndef MULTISENSE_ROS_MULTISENSE_SUBSCRIBER_H_
#define MULTISENSE_ROS_MULTISENSE_SUBSCRIBER_H_

#include <boost/shared_ptr.hpp>
#include <boost/signals2.hpp>
#include <boost/thread.hpp>

namespace multisense_driver
{

/**
 * @class MultisenseSubscriberImp
 * @brief Threadsafe wrapper around a boost::signals2::connection that allows users to
 * unsubscribe from driver callbacks
 */
template <class T>
class MultisenseSubscriberImp
{
 public:
  typedef boost::function<void (const boost::shared_ptr<const T>&)> CallbackType;

  void init(boost::signals2::connection c, const CallbackType& cb)
  {
    boost::recursive_mutex::scoped_lock lock(mutex_);
    c_ = c;
    cb_ = cb;
    subscribed_ = true;
  }

  CallbackType getCallback()
  {
    boost::recursive_mutex::scoped_lock lock(mutex_);
    return boost::bind(&MultisenseSubscriberImp::call, this, _1);
  }

  /**
   * @brief   Disconnects the callback referenced by this subscriber object
   */
  void unsubscribe()
  {
    boost::recursive_mutex::scoped_lock lock(mutex_);
    if (subscribed_)
    {
      subscribed_ = false;
      c_.disconnect();
    }
  }

  /**
   * @brief   Destructor for the subscriber object, will unsubscribe from the
   * associated callback when called.
   */
  ~MultisenseSubscriberImp()
  {
    unsubscribe();
  }

 private:
  void call(const boost::shared_ptr<const T>& msg)
  {
    boost::recursive_mutex::scoped_lock lock(mutex_);
    if (subscribed_)
      cb_(msg);
  }

  boost::recursive_mutex mutex_;
  CallbackType cb_;
  boost::signals2::connection c_; /**< @brief Used to disconnect the callback associated with the subscriber */

  bool subscribed_;
};

template <class T>
class MultisenseSubscriber
{
 public:
  typedef boost::function<void (const boost::shared_ptr<const T>&)> CallbackType;

  /**
   * @brief   Default constructor, doesn't do anything with connection objects
   */
  MultisenseSubscriber()
  {
    sub_.reset(new MultisenseSubscriberImp<T>());
  }

  /**
   * @brief   Disconnects the callback referenced by this subscriber object
   */
  void unsubscribe()
  {
    sub_->unsubscribe();
  }

  CallbackType getCallback()
  {
    return sub_->getCallback();
  }

 private:
  friend class MultisenseDriver;

  /**
   * @brief   Private constructor for the subscriber object.
   * MultisenseSubscribers can only be created by a MultisenseDriver
   * @param
   * @return
   */
  void init(boost::signals2::connection c, const CallbackType& cb)
  {
    sub_->init(c, cb);
  }


  boost::shared_ptr<MultisenseSubscriberImp<T> > sub_;
};

}

#endif

