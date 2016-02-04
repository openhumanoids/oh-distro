#include <iostream>
#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <Eigen/Dense>

#include "capabilityMap/capabilityMap.hpp"
#include "finalPosePlanner/finalPosePlanner.hpp"
#include "bot_lcmgl_client/lcmgl.h"

using namespace std;
using namespace Eigen;

int main()
{
	CapabilityMap cm;
	cm.loadFromMatlabBinFile("/home/marco/drc-testing-data/final_pose_planner/val_description/eigenexport.bin");


	boost::shared_ptr<lcm::LCM> theLCM(new lcm::LCM);
	if(!theLCM->good()){
		std::cerr <<"ERROR: lcm is not good()" <<std::endl;
	}
	bot_lcmgl_t* lcmgl = bot_lcmgl_init(theLCM->getUnderlyingLCM(), "Capability map");
	cm.drawActiveMap(lcmgl);

	RigidBodyTree robot("/home/marco/oh-distro/software/models/val_description/urdf/valkyrie_sim_simple.urdf");
	std::vector<RigidBodyConstraint> constraints;
	FinalPosePlanner fpp;
	VectorXd start_configuration;
	start_configuration.resize(robot.num_positions);
	start_configuration <<	0, 0, 1.0250, 0, 0 ,0 ,0, 0 ,0 ,0 ,0 ,0 ,0.3002,1.2500, 0, 0.7854, 1.5710 ,0, 0 ,0.3002, -1.2500,
			0, -0.7854, 1.5710 ,0, 0, 0, 0, -0.4900, 1.2050 ,-0.7100, 0 ,0, 0, -0.4900 ,1.2050, -0.7100 ,0;
	fpp.findFinalPose(robot, "leftPalm", start_configuration, VectorXd(), constraints, VectorXd() , cm, IKoptions(&robot));
	return 0;
}
