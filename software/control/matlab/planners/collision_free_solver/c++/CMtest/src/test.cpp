#include <iostream>
#include "capabilityMap.hpp"

using namespace std;

int main()
{
	CapabilityMap cm;
	cm.loadFromFile("/home/marco/drc-testing-data/final_pose_planner/val_description/eigenexport.bin");
	cout<<cm.getMapSize()<<'\n';

	return 0;
}
