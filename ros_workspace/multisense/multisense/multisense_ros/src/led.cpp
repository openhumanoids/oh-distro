#include <multisense_ros/led.h>

namespace multisense_ros
{

Led::Led(multisense_driver::MultisenseDriver* driver):
    driver_(driver)
{
  //create publishers
  ros::NodeHandle nh;
  as_.reset(new LedAS(nh, "led_control", false));
  as_->registerGoalCallback(boost::bind(&Led::ledControlGoal, this));
  as_->start();
}

void Led::ledControlGoal()
{
  LedSetMessage cmd;
  {
    boost::mutex::scoped_lock lock(as_mutex_);
    led_goal_ = as_->acceptNewGoal();

    //command leds
    cmd.mask = 0xFF;
    for(size_t i = 0; i < led_goal_->intensities.size(); ++i)
    {
      cmd.intensity[i] = uint8_t(led_goal_->intensities[i] * 255);
    }
    cmd.flash = led_goal_->flashing;
  }

  led_commander_.reset(new LedCommand(cmd, driver_, boost::bind(&Led::ledControlResult, this, _1)));
  led_commander_->run();
}

void Led::ledControlResult(const boost::shared_ptr<const LedSetAckMessage>& res)
{
  boost::mutex::scoped_lock lock(as_mutex_);

  if(as_->isActive())
  {
    as_->setSucceeded();
  }
}

}
