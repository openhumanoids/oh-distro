#include <stdexcept>
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "drake/lcmt_qp_controller_input.hpp"
#include "drc/controller_foot_contact_t.hpp"


#define R_FOOT_NAME "r_foot"
#define L_FOOT_NAME "l_foot"

void copyContactPts(const drake::lcmt_support_data &supp, std::vector< std::vector< double > > &contact_pts) {
  contact_pts.resize(3);
  for (int i=0; i < 3; ++i) {
    contact_pts[i].resize(supp.num_contact_pts);
    for (int j=0; j < supp.num_contact_pts; ++j) {
      contact_pts[i][j] = supp.contact_pts[i][j];
    }
  }
}

class Handler {
  private: 
    lcm::LCM lcm;

  public:
    ~Handler() {}

    void handleMessage(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan,
                       const drake::lcmt_qp_controller_input* msg) {
      drc::controller_foot_contact_t foot_contact_msg;
      foot_contact_msg.num_right_foot_contacts = 0;
      foot_contact_msg.num_left_foot_contacts = 0;
      for (std::vector<drake::lcmt_support_data>::const_iterator supp = msg->support_data.begin(); supp != msg->support_data.end(); ++supp) {
        if (supp->body_name == R_FOOT_NAME) {
          foot_contact_msg.num_right_foot_contacts = supp->num_contact_pts;
          copyContactPts(*supp, foot_contact_msg.right_foot_contacts);
        } else if (supp->body_name == L_FOOT_NAME) {
          foot_contact_msg.num_left_foot_contacts = supp->num_contact_pts;
          copyContactPts(*supp, foot_contact_msg.left_foot_contacts);
        }
      }
      this->lcm.publish("CONTROLLER_FOOT_CONTACT", &foot_contact_msg);
    }
};

int main(int argc, char** argv) {
  lcm::LCM lcm;
  if (!lcm.good()) {
    throw std::runtime_error("LCM is not good");
  }

  Handler handlerObj;
  lcm.subscribe("QP_CONTROLLER_INPUT", &Handler::handleMessage, &handlerObj);
  std::cout << "Controller contact passthrough (c++) running" << std::endl;

  while (0 == lcm.handle());

  return 0;
}