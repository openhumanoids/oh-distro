#ifndef SRC_CAPABILITYMAP_HPP_
#define SRC_CAPABILITYMAP_HPP_

#include <string>
#include <math.h>
#include <fstream>

#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <drake/systems/plants/RigidBodyTree.h>
#include "drake/drakeShapes_export.h"
#include "bot_lcmgl_client/lcmgl.h"

struct MapCentre
{
	Eigen::Vector3d left;
	Eigen::Vector3d right;
};

struct EndEffectorLink
{
	std::string left;
	std::string right;
};

struct Orient
{
	Eigen::VectorXd roll;
	Eigen::VectorXd pitch;
	Eigen::VectorXd yaw;
};

//typedef std::vector<Eigen::SparseMatrix<bool>> OccupancyMap;

class CapabilityMap
{
public:

	enum Side
	{
		RIGHT,
		LEFT
	};

	template <typename T> int sign(T val) {
	    return (T(0) < val) - (val < T(0));
	}

	CapabilityMap(const std::string & log_filename, const std::string & urdf_filename = "");
	void loadFromMatlabBinFile(const std::string map_file);
	void saveToFile(const std::string map_file);
	Eigen::Vector2i getMapSize();
	int getNVoxels();
	Eigen::Vector3d getMapCentre(){return map_centre;}
	Eigen::Vector3d getMapUpperBound(){return map_upper_bound;}
	Eigen::Vector3d getMapLowerBound(){return map_lower_bound;}
	void setNVoxels(unsigned int n_voxels);
	std::vector<Eigen::Vector3d> getActiveVoxelCentres();
	void setNDirectionsPerVoxel(unsigned int n_dir);
	Eigen::RowVector2d getCapabilityMapSize();
	void setActiveSide(Side side);
	void setActiveSide(std::string side);
	void drawCapabilityMap(bot_lcmgl_t* lcmgl, int orient = 0, Eigen::Vector3d centre = Eigen::Vector3d(0, 0, 0), bool draw_cubes = true);
	void drawActiveMap(bot_lcmgl_t* lcmgl, int orient = 0, Eigen::Vector3d centre = Eigen::Vector3d(0, 0, 0), bool draw_cubes = true);
	void setEndeffectorPose(Eigen::Matrix<double, 7, 1> pose);
	void drawOccupancyMap(bot_lcmgl_t* lcmgl, unsigned int capability_map_voxel, unsigned int orient, Eigen::Vector3d centre = Eigen::Vector3d(0, 0, 0), bool draw_cubes = true);
	void reduceActiveSet(bool reset_active, std::vector<Eigen::Vector3d> point_cloud, Eigen::Vector2d sagittal_range = Eigen::Vector2d(-M_PI/3, M_PI/3),
			Eigen::Vector2d transverse_range =  Eigen::Vector2d(-M_PI/3, M_PI/3), Eigen::Vector2d height_range =  Eigen::Vector2d(0.6, 1.1),
			double direction_threshold = M_PI/6);
	void computePositionProbabilityDistribution(Eigen::Vector3d mu = Eigen::Vector3d(0, 0, 0), Eigen::Vector3d sigma = Eigen::Vector3d(1e10, 1e10, 0.01));
private:
	std::ofstream log;
	unsigned int n_voxels;
	unsigned int n_directions_per_voxel;
	unsigned int n_voxels_per_edge;
	Eigen::Vector3d map_centre;
	Eigen::Vector3d map_centre_left;
	Eigen::Vector3d map_centre_right;
	EndEffectorLink endeffector_link;
	Eigen::Vector3d endeffector_axis;
	Eigen::Matrix<double, 7, 1> endeffector_pose;
	std::string base_link;
	unsigned int n_joints;
	Eigen::VectorXd nominal_configuration;
	Eigen::SparseMatrix<bool> map;
	Eigen::VectorXd reachability_index;
	double voxel_edge;
	double angular_tolerance;
	double position_tolerance;
	Eigen::Vector3d map_lower_bound;
	Eigen::Vector3d map_upper_bound;
	unsigned int n_occupancy_voxels;
	unsigned int n_occupancy_orient;
	std::vector<std::vector<std::vector<unsigned int>>> occupancy_map; // n_OM_voxels * n_orient * CM_voxels(variable)
	double occupancy_map_resolution;
	Eigen::Vector3d occupancy_map_lower_bound;
	Eigen::Vector3d occupancy_map_upper_bound;
	Eigen::Vector3i occupancy_map_dimensions;
	Orient occupancy_map_orient_steps;
	RigidBodyTree rigid_body_tree;
	Side active_side;
	std::vector<Eigen::Vector3d> voxel_centres;
	std::vector<unsigned int> active_voxels;
	std::vector<std::vector<unsigned int>> active_orientations; // n_CM_voxels * n_active_orientations(variable)
	std::vector<Eigen::Vector3d> occupancy_voxel_centres;
	std::vector<Eigen::Vector3d> occupancy_map_orientations;
	std::vector<double> position_probability;

	void activateVoxels(std::vector<int> idx);
	void deactivateVoxels(std::vector<int> idx);
	void resetActiveVoxels(bool include_zero_reachability = false);
	void resetActiveOrientations();
	void deactivateVoxelsOutsideAngleRanges(Eigen::Vector2d sagittal_range, Eigen::Vector2d transverse_range, bool reset_active = false);
	void deactivateVoxelsOutsideBaseHeightRange(Eigen::Vector2d range, bool reset_active = false);
	void deactivateVoxelsByDirection(Eigen::Vector3d direction, double direction_threshold, bool reset_active = false);
	void deactivateCollidingVoxels(std::vector<Eigen::Vector3d> point_cloud, bool reset_active = false);
	std::vector<unsigned int> findVoxelsFromDirection(Eigen::Vector3d direction, double threshold, bool active_set_only = true);
	std::vector<unsigned int> findPointsFromDirection(Eigen::Vector3d direction, double threshold);
	std::vector<Eigen::Vector3d> distributePointsOnSphere();
	void computeVoxelCentres(std::vector<Eigen::Vector3d> &centre_array, Eigen::Vector3d lower_bound, Eigen::Vector3d upper_bound, double resolution);
	void setOccupancyMapOrientations();
	void drawMap(bot_lcmgl_t *lcmgl, std::vector<unsigned int> &voxels, Eigen::Vector3d orient = Eigen::Vector3d(0, 0, 0),
			Eigen::Vector3d centre = Eigen::Vector3d(0, 0, 0), bool draw_cubes = true);
	void drawMapCubes(bot_lcmgl_t *lcmgl, Eigen::Vector3d lb, Eigen::Vector3d ub, double resolution,
			Eigen::Vector3d orient = Eigen::Vector3d(0, 0, 0), Eigen::Vector3d centre = Eigen::Vector3d(0, 0, 0));
};


#endif
