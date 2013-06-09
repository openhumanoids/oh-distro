/*
 * unit_tests_imu.cpp
 *
 *  Created on: Jun 1, 2013
 *      Author: drc
 */


#include <iostream>
#include <Eigen/Dense>

#include "LegOdometry_LCM_Handler.hpp"
#include "Odometry.hpp"


using  namespace std;


int main() {

	cout << "This is a unit test function\n";

	command_switches switches;

	  switches.publish_footcontact_states = false;
	  switches.do_estimation = false;
	  switches.draw_footsteps = false;
	  switches.log_data_files = false;
	  switches.lcm_add_ext = false;
	  switches.lcm_read_trues = false;
	  switches.use_true_z = false;
	  switches.print_computation_time = false;
	  switches.OPTION_A = false;
	  switches.OPTION_B = false;
	  switches.OPTION_C = false;
	  switches.OPTION_D = false;
	  switches.OPTION_E = false;
	  switches.grab_true_init = false;
	  switches.medianlength=9;



	boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
	  if(!lcm->good())
		return 1;

	  //_leg_odo = new TwoLegs::TwoLegOdometry(); // This is excessive, as the class is also invoked by LegOdometry_Handler() object.
	  LegOdometry_Handler loh(lcm, &switches);

	  InertialOdometry::DynamicState LegO;
	  InertialOdometry::DynamicState IneO;
	  InertialOdometry::DynamicState VO;

	  LegO.P << 0., 0., 0.;
	  IneO.P << 0., 0., 0.;
	  VO.P << 0., 0., 0.;
	  VO.V << 0., 0., 0.;



	  IneO.q.setIdentity();

	  loh.data_fusion(0,LegO, IneO,VO);

	  LegO.P << 0.2, 0., 0.;

	  loh.data_fusion(0,LegO, IneO,VO);
	  cout << "DATA_FUSION UNIT TEST bias result: " << loh.getInerAccBiases().transpose() << endl;
	  loh.data_fusion(1E6,LegO, IneO,VO);
	  cout << "DATA_FUSION UNIT TEST bias result: " << loh.getInerAccBiases().transpose() << endl;
	  loh.data_fusion(2E6,LegO, IneO,VO);
	  cout << "DATA_FUSION UNIT TEST bias result: " << loh.getInerAccBiases().transpose() << endl;
	  loh.data_fusion(3E6,LegO, IneO,VO);
	  cout << "DATA_FUSION UNIT TEST bias result: " << loh.getInerAccBiases().transpose() << endl;
	  loh.data_fusion(4E6,LegO, IneO,VO);
	  cout << "DATA_FUSION UNIT TEST bias result: " << loh.getInerAccBiases().transpose() << endl;
	  loh.data_fusion(5E6,LegO, IneO,VO);
	  cout << "DATA_FUSION UNIT TEST bias result: " << loh.getInerAccBiases().transpose() << endl;

	return 0;
}



