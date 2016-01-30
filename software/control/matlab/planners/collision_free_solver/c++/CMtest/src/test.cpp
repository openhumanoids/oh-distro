#include <iostream>
#include "capabilityMap.hpp"

using namespace std;

int main()
{
	CapabilityMap cm;
	cm.loadFromMatlabBinFile("/home/marco/drc-testing-data/final_pose_planner/val_description/eigenexport.bin");

	return 0;
}
