#ifndef SRC_CAPABILITYMAP_HPP_
#define SRC_CAPABILITYMAP_HPP_

#include <string>
#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <drake/systems/plants/RigidBodyTree.h>
#include "drake/drakeShapes_export.h"
#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>


using namespace Eigen;

struct map_centre
{
	Vector3d left;
	Vector3d right;
};

struct ee_link
{
	std::string left;
	std::string right;
};

struct orient
{
	VectorXd roll;
	VectorXd pitch;
	VectorXd yaw;
};

enum Side
{
	RIGHT,
	LEFT
};

typedef std::vector<SparseMatrix<bool>> OccupancyMap;

class CapabilityMap
{
public:
	CapabilityMap();
	CapabilityMap(const std::string & urdf_filename);
	void loadFromMatlabBinFile(const std::string mapFile);
	void saveToFile(const std::string mapFile);
	Vector2i getMapSize();
	int getNVoxels();
	void setNVoxels(unsigned int nVoxels);
	void setNDirectionsPerVoxel(unsigned int nDir);
	RowVector2d getCapabilityMapSize();
	void setActiveSide(Side side);
	void drawCapabilityMap();
private:
	unsigned int nVoxels;
	unsigned int nDirectionsPerVoxel;
	unsigned int nVoxelsPerEdge;
	map_centre mapCentre;
	ee_link endEffectorLink;
	Vector3d endEffectorAxis;
	std::string baseLink;
	unsigned int nJoints;
	VectorXd nominalConfiguration;
	SparseMatrix<bool> map;
	VectorXd reachabilityIndex;
	double voxelEdge;
	double angularTolerance;
	double positionTolerance;
	Vector3d mapLowerBound;
	Vector3d mapUpperBound;
	unsigned int nOccupancyVoxels;
	unsigned int nOccupancyOrient;
	OccupancyMap occupancyMapLeft;
	OccupancyMap occupancyMapRight;
	double occupancyMapResolution;
	Vector3d occupancyMapLowerBound;
	Vector3d occupancyMapUpperBound;
	Vector3i occupancyMapDimensions;
	orient occupancyMapOrientSteps;
	RigidBodyTree rigidBodyTree;
	Side activeSide;
	MatrixX3d voxelCentres;

	void computeVoxelCentres();
};


#endif
