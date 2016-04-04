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
	Eigen::VectorXd getPositionProbability(){return position_probability;}
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
	void reduceActiveSet(bool reset_active, std::vector<Eigen::Vector3d> point_cloud, FPPOutput &output, Eigen::Vector2d sagittal_range = Eigen::Vector2d(-M_PI/3, M_PI/3),
			Eigen::Vector2d transverse_range =  Eigen::Vector2d(-M_PI/3, M_PI/3), Eigen::Vector2d height_range =  Eigen::Vector2d(0.6, 1.1),
			double direction_threshold = M_PI/6);

	/**
	 * compute a multivariate Gaussian distribution to draw capability map voxels based on their orientation
	 * \param mu A 3D vector specifying the mean of the multivariate distribution
	 * \param sigma A 3D vector specifying the sigma matrix diagonal of the multivariate distribution
	 */
	void computePositionProbabilityDistribution(Eigen::Vector3d mu = Eigen::Vector3d(0, 0, 0), Eigen::Vector3d sigma = Eigen::Vector3d(1e10, 1e10, 0.01)*pow(.65, 2));//magic numbers to match matlab distribution

	/**
	 * compute a multivariate Gaussian distribution to draw capability map voxels based on their position relative to the map centre.
	 * \param mu A 3D vector specifying the mean of the multivariate distribution
	 * \param sigma A 3D vector specifying the sigma matrix diagonal of the multivariate distribution
	 */
	void computeOrientationProbabilityDistribution(Eigen::Vector3d mu = Eigen::Vector3d(0, 0, 0), Eigen::Vector3d sigma = Eigen::Vector3d(5,//*pow(0.08726646259971647390241145103573217056692, 2),
																																		 .5,//*pow(0.1745329251994329478048229020714643411338, 2),
																																		 10));//*pow(0.7853981633974482789994908671360462903976, 2)));//magic numbers to match matlab distribution

	/**
	 * draw a random sample from a precomputed probability distribution and update the distribution by removing that sample
	 */
	int drawCapabilityMapSample(std::vector<int> &sample);
	void generateRandomSequence(int seed);
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
	Eigen::VectorXd position_probability;
	Eigen::VectorXd orientation_probability;
	std::vector<double> total_probability;
	std::vector<int> total_probability_orientations;
	std::vector<int> total_probability_voxels;
	Eigen::VectorXd random_sequence;

	void activateVoxels(std::vector<int> idx);
	void deactivateVoxels(std::vector<int> idx);
	bool isActiveVoxel(unsigned int voxel){return active_voxels[voxel];}
	bool isActiveOrient(unsigned int voxel, unsigned int orient){return active_orientations(voxel, orient);}
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
	/**
	 * compute a 3D multivariate Gaussian distribution for a vector of values.
	 * \param values The samples to compute the probability
	 * \param pdf A pointer to the vector on which probabilities will be written
	 * \param mu A 3D vector specifying the mean of the multivariate distribution
	 * \param sigma A 3D vector specifying the sigma matrix diagonal of the multivariate distribution
	 */
	void computeProbabilityDistribution(std::vector<Eigen::Vector3d> & values, Eigen::VectorXd &pdf, Eigen::Vector3d mu, Eigen::Vector3d sigma);

	/**
	 * compute the orientation-position combined probability
	 */
	void computeTotalProbabilityDistribution();
};


#endif
