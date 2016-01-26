#ifndef SRC_CAPABILITYMAP_HPP_
#define SRC_CAPABILITYMAP_HPP_

#include <string>
#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <drake/systems/plants/RigidBodyTree.h>

struct map_centre {
	Eigen::Vector3d left;
	Eigen::Vector3d right;
};

class CapabilityMap
{
public:
	CapabilityMap();
	CapabilityMap(const std::string & urdf_filename);
	void loadFromFile(const std::string mapFile);
	void saveToFile(const std::string mapFile);
	Eigen::Vector2i getMapSize();
	int getNVoxels();
	void setNVoxels(unsigned int nVoxels);
	void setNDirectionsPerVoxel(unsigned int nDir);
	Eigen::RowVector2d getCapabilityMapSize();
private:
	size_t nVoxels;
	size_t nDirectionsPerVoxel;
	map_centre mapCentre;
	Eigen::SparseMatrix<bool> map;
	Eigen::VectorXd reachabilityIndex;
	RigidBodyTree rigidBodyTree;
};


#endif
