#include <iostream>
#include "capabilityMap.hpp"

using namespace std;

int main()
{
	CapabilityMap cm;
	CapabilityMap cm2;

	cout<<cm.getNVoxels()<<'\n';
	cm.setNVoxels(200);
	cout<<cm.getNVoxels()<<'\n';
	cm.saveToFile("test.dat");
	cout<<cm2.getNVoxels()<<'\n';
	cm2.loadFromFile("test.dat");
	cout<<cm2.getNVoxels()<<'\n';

	return 0;
}
