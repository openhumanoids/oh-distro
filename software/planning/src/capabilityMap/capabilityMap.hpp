#ifndef SRC_CAPABILITYMAP_HPP_
#define SRC_CAPABILITYMAP_HPP_

#include <string>

#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <drake/systems/plants/RigidBodyTree.h>
#include "drake/drakeShapes_export.h"
#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include "bot_lcmgl_client/lcmgl.h"

struct map_centre
{
	Eigen::Vector3d left;
	Eigen::Vector3d right;
};

struct ee_link
{
	std::string left;
	std::string right;
};

struct orient
{
	Eigen::VectorXd roll;
	Eigen::VectorXd pitch;
	Eigen::VectorXd yaw;
};

enum Side
{
	RIGHT,
	LEFT
};

typedef std::vector<Eigen::SparseMatrix<bool>> OccupancyMap;

class CapabilityMap
{
public:
	CapabilityMap();
	CapabilityMap(const std::string & urdf_filename);
	void loadFromMatlabBinFile(const std::string mapFile);
	void saveToFile(const std::string mapFile);
	Eigen::Vector2i getMapSize();
	int getNVoxels();
	void setNVoxels(unsigned int nVoxels);
	void setNDirectionsPerVoxel(unsigned int nDir);
	Eigen::RowVector2d getCapabilityMapSize();
	void setActiveSide(Side side);
	void drawCapabilityMap(bot_lcmgl_t* lcmgl);
private:
	unsigned int nVoxels;
	unsigned int nDirectionsPerVoxel;
	unsigned int nVoxelsPerEdge;
	map_centre mapCentre;
	ee_link endEffectorLink;
	Eigen::Vector3d endEffectorAxis;
	std::string baseLink;
	unsigned int nJoints;
	Eigen::VectorXd nominalConfiguration;
	Eigen::SparseMatrix<bool> map;
	Eigen::VectorXd reachabilityIndex;
	double voxelEdge;
	double angularTolerance;
	double positionTolerance;
	Eigen::Vector3d mapLowerBound;
	Eigen::Vector3d mapUpperBound;
	unsigned int nOccupancyVoxels;
	unsigned int nOccupancyOrient;
	OccupancyMap occupancyMapLeft;
	OccupancyMap occupancyMapRight;
	double occupancyMapResolution;
	Eigen::Vector3d occupancyMapLowerBound;
	Eigen::Vector3d occupancyMapUpperBound;
	Eigen::Vector3i occupancyMapDimensions;
	orient occupancyMapOrientSteps;
	RigidBodyTree rigidBodyTree;
	Side activeSide;
	std::vector<Eigen::Vector3d> voxelCentres;

	void computeVoxelCentres();
};


#endif
