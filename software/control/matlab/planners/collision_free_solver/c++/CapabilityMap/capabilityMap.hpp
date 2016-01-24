#ifndef SRC_CAPABILITYMAP_HPP_
#define SRC_CAPABILITYMAP_HPP_

#include <string>
#include <Eigen/Dense>
#include <drake/systems/plants/RigidBodyTree.h>

class CapabilityMap
{
public:
	CapabilityMap();
	CapabilityMap(const std::string & urdf_filename);
	void generateCapabilityMap(const double vox_edge=0.05, const unsigned int n_samples=1e6, const unsigned int n_directions_per_voxel=50);
	void loadFromFile(const std::string mapFile);
	void saveToFile(const std::string mapFile);
	int getNVoxels();
	void setNVoxels(int nVoxels);
	Eigen::RowVector2d getCapabilityMapSize();
private:
	Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> map;
	unsigned int nVoxels;
	Eigen::VectorXd reachabilityIndex;
	RigidBodyTree rigidBodyTree;
};


#endif
