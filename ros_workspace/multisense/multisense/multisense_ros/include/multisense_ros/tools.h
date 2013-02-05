#ifndef MULTISENSE_ROS_TOOLS_H
#define MULTISENSE_ROS_TOOLS_H

#include <multisense_driver/multisense_driver.h>
#include <LibPlatform/TimeStamp.hh>
#include <ros/ros.h>

namespace multisense_ros
{


ros::Time convertTime(const crl::TimeStamp& t);


class CommandBase
{
public:
  virtual ~CommandBase() {};
  virtual void run() = 0;
};



template <class M, class A>
class Command: public CommandBase
{
public:
  Command(const M& msg, multisense_driver::MultisenseDriver* driver, boost::function<void(const boost::shared_ptr<const A>&)> cb)
    :have_cb_(true), cb_(cb)
  {
    init(msg, driver);
  }

  Command(const M& msg, multisense_driver::MultisenseDriver* driver)
    :have_cb_(false)
  {
    init(msg, driver);
  }

  ~Command()
  {
    timer_.stop();
    ack_sub_.unsubscribe();
  }

  void run()
  {
    // execute command once
    cmd_();

    // repeat command if needed
    ros::NodeHandle nh;
    timer_ = nh.createTimer(ros::Duration(1.0), boost::bind(&Command::timerCB, this, _1));
  }


private:
  void init(const M& msg, multisense_driver::MultisenseDriver* driver)
  {
    finished_ = false;

    cmd_     = boost::bind(&multisense_driver::MultisenseDriver::publish<M>, driver, msg);
    ack_sub_ = driver->subscribe<A>(boost::bind(&Command::ackCB, this, _1));
    ROS_DEBUG("Subscribing to %02x", A::MSG_ID);
  }


  void ackCB(const boost::shared_ptr<const A>& res)
  {
    ROS_DEBUG("Ack received of type %s", typeid(A).name());
    finished_ = true;
    timer_.stop();
    ack_sub_.unsubscribe();

    if (have_cb_)
      cb_(res);
  }


  void timerCB(const ros::TimerEvent& t)
  {
    if (!finished_)
      cmd_();
    else
      timer_.stop();
  }

  bool have_cb_;
  boost::function<void(const boost::shared_ptr<const A>&)> cb_;
  boost::function<void()> cmd_;
  multisense_driver::MultisenseSubscriber<A> ack_sub_;

  ros::Timer timer_;
  bool finished_;

}; //class



}// namespace

#endif
