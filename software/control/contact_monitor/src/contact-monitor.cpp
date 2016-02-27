#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <ConciseArgs>
#include <map>

#include "lcmtypes/bot_core/robot_state_t.hpp"
#include "lcmtypes/drc/foot_contact_estimate_t.hpp"


enum FootID {RIGHT, LEFT};

class ContactMonitor {
public:
  ~ContactMonitor() {}
  ContactMonitor(double threshold, std::string output_channel, double debounce_time) {
    this->fz_threshold = threshold;
    this->last_change_utime[RIGHT] = 0;
    this->last_change_utime[LEFT] = 0;
    this->contact_state[RIGHT] = false;
    this->contact_state[LEFT] = false;
    this->output_channel = output_channel;
    this->debounce_time = debounce_time;
  }

  void handleState(const lcm::ReceiveBuffer* rbuf,
                   const std::string& chan,
                   const bot_core::robot_state_t* msg) {
    if (msg->utime - this->last_change_utime.at(RIGHT) > static_cast<int64_t>(this->debounce_time * 1e6)) {
      this->contact_state.at(RIGHT) = msg->force_torque.r_foot_force_z > this->fz_threshold;
      this->last_change_utime.at(RIGHT) = msg->utime;
    }
    if (msg->utime - this->last_change_utime.at(LEFT) > static_cast<int64_t>(this->debounce_time * 1e6)) {
      this->contact_state.at(LEFT) = msg->force_torque.l_foot_force_z > this->fz_threshold;
      this->last_change_utime.at(LEFT) = msg->utime;
    }

    drc::foot_contact_estimate_t output;
    output.utime = msg->utime;
    output.detection_method = 0;
    output.left_contact = this->contact_state.at(LEFT);
    output.right_contact = this->contact_state.at(RIGHT);

    this->lcm.publish(this->output_channel, &output);
  }

  void run(std::string input_channel = "EST_ROBOT_STATE") {
    this->lcm.subscribe(input_channel, &ContactMonitor::handleState, this);
    while(0 == this->lcm.handle());
  }


private:
  std::map<FootID, int64_t> last_change_utime;
  std::map<FootID, bool> contact_state;
  double fz_threshold;
  double debounce_time;
  std::string output_channel;
  lcm::LCM lcm;
};


int main(int argc, char** argv) {
  ConciseArgs parser(argc, argv, "contact-monitor");
  double threshold = 100;
  std::string channel = "FOOT_CONTACT_ESTIMATE";
  double debounce_time = 0.05;

  parser.add(threshold, "t", "threshold", "Force threshold (N)");
  parser.add(channel, "c", "channel", "Output channel");
  parser.add(debounce_time, "d", "debounce", "Debounce time (s)");
  parser.parse();

  std::cout << "threshold: " << threshold << std::endl;
  std::cout << "channel: " << channel << std::endl;
  std::cout << "debounce_time: " << debounce_time << std::endl;

  ContactMonitor monitor(threshold, channel, debounce_time);
  monitor.run();
  return 0;
}
