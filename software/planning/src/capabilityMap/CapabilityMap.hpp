#ifndef SRC_CAPABILITYMAP_HPP_
#define SRC_CAPABILITYMAP_HPP_

#include <fstream>

#include <Eigen/SparseCore>
#include <drake/systems/plants/RigidBodyTree.h>
#include "bot_lcmgl_client/lcmgl.h"
#include "finalPosePlanner/FPPOutput.hpp"

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
	int getNVoxels(){return n_voxels;}
	int getNActiveVoxels(){return count(active_voxels.begin(), active_voxels.end(), 1);}
	int getNActiveOrientations(){return this->active_orientations.count();}
	Eigen::Vector3d getMapCentre(){return map_centre;}
	Eigen::Vector3d getMapUpperBound(){return map_upper_bound;}
	Eigen::Vector3d getMapLowerBound(){return map_lower_bound;}
	std::string getBaseLink() {return base_link;}
	Eigen::Vector3d getOrientation(int orient){return occupancy_map_orientations[orient];}
	Eigen::Vector3d getVoxelCentre(int vox){return voxel_centres[vox];}
	std::vector<Eigen::Vector3d> getVoxelCentres(){return voxel_centres;}
	void setNVoxels(unsigned int n_voxels);
	std::vector<Eigen::Vector3d> getActiveVoxelCentres();
	void setNDirectionsPerVoxel(unsigned int n_dir);
	Eigen::RowVector2d getCapabilityMapSize();
	void setActiveSide(const Side side);
	void setActiveSide(const std::string side);
	void drawCapabilityMap(bot_lcmgl_t* lcmgl, const int orient = 0, const Eigen::Vector3d centre = Eigen::Vector3d(0, 0, 0), const bool draw_cubes = true);
	void drawActiveMap(bot_lcmgl_t* lcmgl, const int orient = 0, const Eigen::Vector3d centre = Eigen::Vector3d(0, 0, 0), const bool draw_cubes = true);
	void setEndeffectorPose(const Eigen::Matrix<double, 7, 1> pose);
	void drawOccupancyMap(bot_lcmgl_t* lcmgl, const unsigned int capability_map_voxel, const unsigned int orient, const Eigen::Vector3d centre = Eigen::Vector3d(0, 0, 0), const bool draw_cubes = true);
	void reduceActiveSet(const bool reset_active, const std::vector<Eigen::Vector3d> point_cloud, FPPOutput &output, const Eigen::Vector2d sagittal_range = Eigen::Vector2d(-M_PI/3, M_PI/3),
			const Eigen::Vector2d transverse_range =  Eigen::Vector2d(-M_PI/3, M_PI/3), const Eigen::Vector2d height_range =  Eigen::Vector2d(0.6, 1.1),
			const double direction_threshold = M_PI/6);

	/**
	 * compute a 6D multivariate Gaussian distribution for all active voxels and orientations.
	 * \param mu A 6D vector specifying the mean (x, y, z, roll, pitch, yaw) of the multivariate distribution
	 * \param sigma A 6D vector specifying the sigma  (x, y, z, roll, pitch, yaw) matrix diagonal of the multivariate distribution
	 */

	void computeProbabilityDistribution(const Eigen::VectorXd mu, const Eigen::VectorXd sigma);
	/**
	 * draw a random sample from a precomputed probability distribution and update the distribution by removing that sample
	 */
	int drawCapabilityMapSample(std::vector<int> &sample);
	std::ofstream log;

private:
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
	typedef std::vector<std::vector<std::vector<unsigned int>>> occupancy_map; /**< Dimensions:\n  n capability map voxels\n  n orientations\n  occupied occupancy map voxels for each orientation (variable) */
	std::map<Side, occupancy_map> occupancy_maps;
	double occupancy_map_resolution;
	Eigen::Vector3d occupancy_map_lower_bound;
	Eigen::Vector3d occupancy_map_upper_bound;
	Eigen::Vector3i occupancy_map_dimensions;
	Orient occupancy_map_orient_steps;
	RigidBodyTree rigid_body_tree;
	Side active_side;
	std::vector<Eigen::Vector3d> voxel_centres;
	std::vector<bool> active_voxels;
	Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> active_orientations; /**< Dimensions:\n  n capability map voxels\n  n orientations */
	std::vector<Eigen::Vector3d> occupancy_voxel_centres;
	std::vector<Eigen::Vector3d> occupancy_map_orientations;
	Eigen::VectorXd voxel_probability;
	std::vector<int> probability_orientations;
	std::vector<int> probability_voxels;
	Eigen::VectorXd random_sequence;

	void activateVoxels(const std::vector<int> idx);
	void deactivateVoxels(const std::vector<int> idx);
	bool isActiveVoxel(const unsigned int voxel){return active_voxels[voxel];}
	bool isActiveOrient(const unsigned int voxel, const unsigned int orient){return active_orientations(voxel, orient);}
	void resetActiveVoxels(const bool include_zero_reachability = false);
	void resetActiveOrientations();
	void deactivateVoxelsOutsideAngleRanges(Eigen::Vector2d sagittal_range, Eigen::Vector2d transverse_range, const bool reset_active = false);
	void deactivateVoxelsOutsideBaseHeightRange(const Eigen::Vector2d range, const bool reset_active = false);
	void deactivateVoxelsByDirection(const Eigen::Vector3d direction, const double direction_threshold, const bool reset_active = false);
	void deactivateCollidingVoxels(const std::vector<Eigen::Vector3d> point_cloud, const bool reset_active = false);
	std::vector<unsigned int> findVoxelsFromDirection(const Eigen::Vector3d direction, const double threshold, const bool active_set_only = true);
	std::vector<unsigned int> findPointsFromDirection(Eigen::Vector3d direction, const double threshold);
	std::vector<Eigen::Vector3d> distributePointsOnSphere();
	void computeVoxelCentres(std::vector<Eigen::Vector3d> &centre_array, const Eigen::Vector3d lower_bound, const Eigen::Vector3d upper_bound, const double resolution);
	void setOccupancyMapOrientations();
	void drawMap(bot_lcmgl_t *lcmgl, const std::vector<unsigned int> &voxels, const Eigen::Vector3d orient = Eigen::Vector3d(0, 0, 0),
			const Eigen::Vector3d centre = Eigen::Vector3d(0, 0, 0), const bool draw_cubes = true);
	void drawMapCubes(bot_lcmgl_t *lcmgl, const Eigen::Vector3d lb, const Eigen::Vector3d ub, const double resolution,
			const Eigen::Vector3d orient = Eigen::Vector3d(0, 0, 0), const Eigen::Vector3d centre = Eigen::Vector3d(0, 0, 0));
};


#endif
