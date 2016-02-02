#include <iostream>
#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include "capabilityMap/capabilityMap.hpp"
#include "bot_lcmgl_client/lcmgl.h"
#include "lcmtypes/bot_lcmgl_data_t.h"

using namespace std;

void handler(const lcm::ReceiveBuffer* buf, const string& channel, const bot_lcmgl_data_t* msg)
{};

int main()
{
	CapabilityMap cm;
	cm.loadFromMatlabBinFile("/home/marco/drc-testing-data/final_pose_planner/val_description/eigenexport.bin");
	cm.drawCapabilityMap();


	boost::shared_ptr<lcm::LCM> theLCM(new lcm::LCM);
	if(!theLCM->good()){
		std::cerr <<"ERROR: lcm is not good()" <<std::endl;
	}
	bot_lcmgl_t* lcmgl = bot_lcmgl_init(theLCM->getUnderlyingLCM(), "Capability map");
	return 0;
}
