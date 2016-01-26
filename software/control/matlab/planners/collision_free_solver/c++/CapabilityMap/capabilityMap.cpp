#include "capabilityMap.hpp"
#include <fstream>

CapabilityMap::CapabilityMap():nVoxels(0), nDirectionsPerVoxel(0)
{

}

CapabilityMap::CapabilityMap(const std::string & urdf_filename):nVoxels(0), nDirectionsPerVoxel(0)
{

}

void CapabilityMap::loadFromFile(const std::string mapFile)
{
	typedef Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> MatrixXb;
	MatrixXb denseMap(this->nVoxels, this->nDirectionsPerVoxel);

	std::cout << "Loading capability map from" << mapFile << '\n';
	std::ifstream inputFile(mapFile.c_str(), std::ifstream::binary);
//	inputFile.read((char *) &urdfLength, sizeof(urdfLength));
//	inputFile.read((char *) this->urdf.c_str(), sizeof(char)*urdfLength);
	inputFile.read((char *) this->mapCentre.left.data(), sizeof(this->mapCentre.left));
	std::cout << "Map Centre: \n" << this->mapCentre.left << '\n';
	inputFile.read((char *) &this->nVoxels, sizeof(this->nVoxels));
	inputFile.read((char *) &this->nDirectionsPerVoxel, sizeof(this->nDirectionsPerVoxel));
	this->map.resize(this->nVoxels, this->nDirectionsPerVoxel);
	inputFile.read((char *) denseMap.data(), this->nVoxels * this->nDirectionsPerVoxel * sizeof(MatrixXb::Scalar));
	this->map = denseMap.sparseView();
	inputFile.close();
	std::cout << "Capability map successfully loaded\n";
}

void CapabilityMap::saveToFile(const std::string mapFile)
{
	std::cout << "Saving capability map...\n";
	std::ofstream outputFile(mapFile.c_str(), std::ofstream::binary);
	outputFile.write((char *) &this->nVoxels, sizeof(this->nVoxels));
	outputFile.write((char *) &this->nDirectionsPerVoxel, sizeof(this->nDirectionsPerVoxel));
	std::cout << "Capability map saved in " << mapFile << '\n';
	outputFile.close();
}

Eigen::Vector2i CapabilityMap::getMapSize()
{
	Eigen::Vector2i size;
	size << this->map.rows(), this->map.cols();
	return size;
}

int CapabilityMap::getNVoxels()
{
	return this->nVoxels;
}

void CapabilityMap::setNVoxels(unsigned int nVoxels)
{
	this->nVoxels = nVoxels;
}

void CapabilityMap::setNDirectionsPerVoxel(unsigned int nDir)
{
	this->nDirectionsPerVoxel = nDir;
}

Eigen::RowVector2d CapabilityMap::getCapabilityMapSize()
{
	Eigen::RowVector2d size;
	size << this->map.rows(), this->map.cols();
	return size;
}
