#include "capabilityMap.hpp"
#include <fstream>

CapabilityMap::CapabilityMap():nVoxels(50)
{

}

CapabilityMap::CapabilityMap(const std::string & urdf_filename):nVoxels(50)
{

}

void CapabilityMap::loadFromFile(const std::string mapFile)
{
	std::cout << "Loading capability map from" << mapFile << '\n';
	std::ifstream inputFile(mapFile.c_str(), std::ifstream::binary);
	inputFile.read((char *) &this->nVoxels, sizeof(this->nVoxels));
	inputFile.close();
	std::cout << "Capability map successfully loaded\n";
}

void CapabilityMap::saveToFile(const std::string mapFile)
{
	std::cout << "Saving capability map...\n";
	std::ofstream outputFile(mapFile.c_str(), std::ofstream::binary);
	outputFile.write((char *) &this->nVoxels, sizeof(this->nVoxels));
	std::cout << "Capability map saved in " << mapFile << '\n';
	outputFile.close();
}

void CapabilityMap::generateCapabilityMap(const double vox_edge, const unsigned int n_samples, const unsigned int n_directions_per_voxel)
{
	this->nVoxels = 200;
	this->map.setConstant(this->nVoxels, n_directions_per_voxel, false);
}

int CapabilityMap::getNVoxels()
{
	return this->nVoxels;
}

void CapabilityMap::setNVoxels(int nVoxels)
{
	this->nVoxels = nVoxels;
}

Eigen::RowVector2d CapabilityMap::getCapabilityMapSize()
{
	Eigen::RowVector2d size;
	size << this->map.rows(), this->map.cols();
	return size;
}
