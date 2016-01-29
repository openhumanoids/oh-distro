#ifndef SRC_CAPABILITYMAP_HPP_
#define SRC_CAPABILITYMAP_HPP_

#include <string>
#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <drake/systems/plants/RigidBodyTree.h>

using namespace std;
using namespace Eigen;

struct map_centre
{
	Vector3d left;
	Vector3d right;
};

struct ee_link
{
	string left;
	string right;
};

struct orient
{
	VectorXf roll;
	VectorXf pitch;
	VectorXf yaw;
};

typedef vector<SparseMatrix<bool>> OccupancyMap;

class CapabilityMap
{
public:
	CapabilityMap();
	CapabilityMap(const string & urdf_filename);
	void loadFromFile(const string mapFile);
	void saveToFile(const string mapFile);
	Vector2i getMapSize();
	int getNVoxels();
	void setNVoxels(unsigned int nVoxels);
	void setNDirectionsPerVoxel(unsigned int nDir);
	RowVector2d getCapabilityMapSize();
private:
	unsigned int nVoxels;
	unsigned int nDirectionsPerVoxel;
	map_centre mapCentre;
	ee_link endEffectorLink;
	Vector3d endEffectorAxis;
	string baseLink;
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
};


#endif
