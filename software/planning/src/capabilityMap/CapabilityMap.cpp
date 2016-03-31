#include <fstream>
#include <stdlib.h>
#include <algorithm>
#include <numeric>
#include <math.h>
#include <time.h>
#include <boost/range/irange.hpp>
#include <boost/range/algorithm_ext/push_back.hpp>
#include <boost/range/adaptors.hpp>
#include <boost/algorithm/cxx11/copy_if.hpp>
#include <boost/algorithm/string.hpp>

#include "capabilityMap/CapabilityMap.hpp"
#include "drawingUtil/drawingUtil.hpp"
#include "drake/util/drakeGeometryUtil.h"

extern "C" {
double *mt19937ar(uint32_t seed);
}

using namespace std;
using namespace Eigen;

CapabilityMap::CapabilityMap(const string & log_filename, const string &urdf_filename):active_side(Side::LEFT)
{
	if (urdf_filename != "")
	{
		this->loadFromMatlabBinFile(urdf_filename);
	}
	this->log.open(log_filename.c_str());
	if (!this->log.is_open())
	{
		cout << "Failed to open " << log_filename.c_str() << '\n';
	}
}

void CapabilityMap::generateRandomSequence(int seed)
{
//	double* random_sequence_array = mt19937ar(seed);
////	cout << seed << endl;
//	this->random_sequence.assign(random_sequence_array, random_sequence_array+1000);
////	for (auto i : this->random_sequence){cout << i << endl;}
	ifstream random_file("/home/marco/drc-testing-data/final_pose_planner/val_description/random_sequence");
	MatrixXd matrix(1000,100);
	random_file.read((char *) matrix.data(), sizeof(MatrixXd::Scalar)*100000);
	this->random_sequence = matrix.col(seed);
}

void CapabilityMap::loadFromMatlabBinFile(const string map_file)
{
	MatrixX2i idx;
	unsigned int nnz;
	unsigned int string_length;
	bool contains_map;
	bool contains_occupancy_map;
	vector<Triplet<bool>> triplet_list;

	ifstream inputFile(map_file.c_str(), ifstream::binary);

	if (!inputFile.is_open())
	{
		std::cout << "Failed to open " << map_file.c_str() << '\n';
	}
	else
	{
		this->log << "Loading data from" << map_file << '\n';

		inputFile.read((char *) this->map_centre_left.data(), sizeof(this->map_centre_left));
		this->log << "Loaded map_centre_left: " << this->map_centre_left[0] << ";"  << this->map_centre_left[1] << ";"  << this->map_centre_left[2] << '\n';
		inputFile.read((char *) this->map_centre_right.data(), sizeof(this->map_centre_right));
		this->log << "Loaded map_centre_right: " << this->map_centre_right[0] << ";" << this->map_centre_right[1] << ";"  << this->map_centre_right[2] << '\n';

		inputFile.read((char *) &string_length, sizeof(unsigned int));
		char *ee_link_left_str = new char[string_length];
		inputFile.read(ee_link_left_str, string_length);
		this->endeffector_link.left = ee_link_left_str;
		delete [] ee_link_left_str;
		ee_link_left_str = nullptr;
		this->log << "Loaded endeffector_link.left: " << this->endeffector_link.left.c_str() << endl;
		inputFile.read((char *) &string_length, sizeof(unsigned int ));
		char *ee_link_right_str = new char[string_length];
		inputFile.read(ee_link_right_str, string_length * sizeof(char));
		this->endeffector_link.right = ee_link_right_str;
		delete [] ee_link_right_str;
		ee_link_right_str = nullptr;
		this->log << "Loaded endeffector_link.right: " << this->endeffector_link.right.c_str() << endl;

		inputFile.read((char *) this->endeffector_axis.data(), sizeof(this->endeffector_axis));
		this->log << "Loaded endeffector_axis: " << this->endeffector_axis[0] << ";"  << this->endeffector_axis[1] << ";"  << this->endeffector_axis[2] << '\n';

		inputFile.read((char *) &string_length, sizeof(unsigned int));
		char *base_link_str = new char[string_length];
		inputFile.read(base_link_str, string_length * sizeof(char));
		this->base_link = base_link_str;
		delete [] base_link_str;
		base_link_str = nullptr;
		this->log << "Loaded base_link: " << this->base_link.c_str() << endl;

		inputFile.read((char *) &this->n_joints, sizeof(unsigned int));
		this->log << "Loaded n_joints: "<< this->n_joints << endl;

		this->nominal_configuration.resize(this->n_joints);
		inputFile.read((char *) this->nominal_configuration.data(), this->n_joints * sizeof(VectorXd::Scalar));
		this->log << "Loaded nominal_configuration: ";
		for (unsigned int j = 0; j < this->n_joints; j++)
		{
			this->log << this->nominal_configuration[j] << ";";
		}
		this->log << '\n';

		inputFile.read((char *) &contains_map, sizeof(bool));
		if (contains_map)
		{
			this->log << "Found capability map data" << endl;
			inputFile.read((char *) &this->n_voxels, sizeof(unsigned int));
			inputFile.read((char *) &this->n_directions_per_voxel, sizeof(unsigned int));
			this->n_voxels_per_edge = cbrt(this->n_voxels);

			this->active_voxels.resize(this->n_voxels);
			this->map.resize(this->n_voxels, this->n_directions_per_voxel);
			inputFile.read((char *) &nnz, sizeof(unsigned int));
			idx.resize(nnz, 2);
			inputFile.read((char *) idx.data(), nnz * 2 * sizeof(MatrixXi::Scalar));
			triplet_list.reserve(nnz);
			for (unsigned int i = 0; i < nnz; i++)
			{
				triplet_list.push_back(Triplet<bool>(idx(i, 0), idx(i, 1), true));
			}
			this->map.setFromTriplets(triplet_list.begin(), triplet_list.end());
			this->log << "Loaded Capability Map (" << this->map.rows() << "x" << this->map.cols() << ")\n";

			this->reachability_index.resize(this->n_voxels);
			inputFile.read((char *) this->reachability_index.data(), this->n_voxels * sizeof(VectorXd::Scalar));
			this->log << "Loaded Reachability_index (" << this->reachability_index.rows() << ")\n";

			inputFile.read((char *) &this->voxel_edge, sizeof(double));
			this->log << "Loaded voxel_edge :" << this->voxel_edge << endl;

			inputFile.read((char *) &this->angular_tolerance, sizeof(double));
			this->log << "Loaded angular_tolerance :" << this->angular_tolerance << endl;

			inputFile.read((char *) &this->position_tolerance, sizeof(double));
			this->log << "Loaded position_tolerance :" << this->position_tolerance << endl;

			inputFile.read((char *) this->map_lower_bound.data(), sizeof(this->map_lower_bound));
			this->log << "Loaded map_lower_bound: " << this->map_lower_bound[0] << ";"  << this->map_lower_bound[1] << ";"  << this->map_lower_bound[2] << '\n';

			inputFile.read((char *) this->map_upper_bound.data(), sizeof(this->map_upper_bound));
			this->log << "Loaded map_upper_bound: " << this->map_upper_bound[0] << ";"  << this->map_upper_bound[1] << ";"  << this->map_upper_bound[2] << '\n';
		}
		else
		{
			this->log << "No capability map data found" << endl;
		}

		inputFile.read((char *) &contains_occupancy_map, sizeof(bool));
		if (contains_occupancy_map)
		{
			this->log << "Found occupancy map data" << endl;
			inputFile.read((char *) &this->n_occupancy_voxels, sizeof(unsigned int));
			this->log << "Loaded n_occupancy_voxels: " << this->n_occupancy_voxels << endl;
			inputFile.read((char *) &this->n_occupancy_orient, sizeof(unsigned int));
			this->log << "Loaded n_occupancy_orient: " << this->n_occupancy_orient << endl;
			this->occupancy_maps[this->Side::LEFT].resize(this->n_voxels);
			this->occupancy_maps[this->Side::RIGHT].resize(this->n_voxels);
			this->active_orientations.resize(this->n_voxels, this->n_occupancy_orient);
			unsigned int n_colliding_om_voxels;
			unsigned int colliding_voxel;

			vector<Side> sides;
			sides.push_back(Side::LEFT);
			sides.push_back(Side::RIGHT);
			for (auto s : sides)
			{
				switch(s)
				{
					case Side::LEFT:
						this->log << "Loading left occupancy map ..." << endl;
						break;
					case Side::RIGHT:
						this->log << "Loading right occupancy map ..." << endl;
						break;
				}
				for (int vox = 0; vox < this->n_voxels; vox++)
				{
					this->occupancy_maps[s][vox].resize(this->n_occupancy_orient);
					for (int orient = 0; orient < this->n_occupancy_orient; orient++)
					{
						inputFile.read((char *) &n_colliding_om_voxels, sizeof(n_colliding_om_voxels));
						if (n_colliding_om_voxels > 0)
						{
							this->occupancy_maps[s][vox][orient].resize(n_colliding_om_voxels);
							for (int cm_vox = 0; cm_vox < n_colliding_om_voxels; cm_vox++)
							{
								inputFile.read((char *) &colliding_voxel, sizeof(colliding_voxel));
								this->occupancy_maps[s][vox][orient][cm_vox] = colliding_voxel - 1; // because matlab arrays are zero based
							}
						}
					}
				}
				switch(s)
				{
					case Side::LEFT:
						this->log << "Left occupancy map loaded" << endl;
						break;
					case Side::RIGHT:
						this->log << "Right occupancy map loaded" << endl;
						break;
				}
			}

			inputFile.read((char *) &this->occupancy_map_resolution, sizeof(double));
			this->log << "Loaded occupancy_map_resolution :" << this->occupancy_map_resolution << endl;

			inputFile.read((char *) this->occupancy_map_dimensions.data(), sizeof(this->occupancy_map_dimensions));
			this->log << "Loaded occupancy_map_dimensions: " << this->occupancy_map_dimensions[0] << ";"  << this->occupancy_map_dimensions[1] << ";"  << this->occupancy_map_dimensions[2] << '\n';

			inputFile.read((char *) this->occupancy_map_lower_bound.data(), sizeof(this->occupancy_map_lower_bound));
			this->log << "Loaded occupancy_map_lower_bound: " << this->occupancy_map_lower_bound[0] << ";"  << this->occupancy_map_lower_bound[1] << ";"  << this->occupancy_map_lower_bound[2] << '\n';

			inputFile.read((char *) this->occupancy_map_upper_bound.data(), sizeof(this->occupancy_map_upper_bound));
			this->log << "Loaded occupancy_map_upper_bound: " << this->occupancy_map_upper_bound[0] << ";"  << this->occupancy_map_upper_bound[1] << ";"  << this->occupancy_map_upper_bound[2] << '\n';

			unsigned int n_roll_steps;
			inputFile.read((char *) &n_roll_steps, sizeof(n_roll_steps));
			this->occupancy_map_orient_steps.roll.resize(n_roll_steps);
			inputFile.read((char *) this->occupancy_map_orient_steps.roll.data(), n_roll_steps * sizeof(double));
			this->log << "Loaded occupancy_map_orient_steps.roll (" << this->occupancy_map_orient_steps.roll.rows() << ")\n";
			IOFormat fmt(30);
			cout << this->occupancy_map_orient_steps.roll.format(fmt) << endl;

			unsigned int n_pitch_steps;
			inputFile.read((char *) &n_pitch_steps, sizeof(n_pitch_steps));
			this->occupancy_map_orient_steps.pitch.resize(n_pitch_steps);
			inputFile.read((char *) this->occupancy_map_orient_steps.pitch.data(), n_pitch_steps * sizeof(double));
			this->log << "Loaded occupancy_map_orient_steps.pitch (" << this->occupancy_map_orient_steps.pitch.rows() << ")\n";

			unsigned int n_yaw_steps;
			inputFile.read((char *) &n_yaw_steps, sizeof(n_yaw_steps));
			this->occupancy_map_orient_steps.yaw.resize(n_yaw_steps);
			inputFile.read((char *) this->occupancy_map_orient_steps.yaw.data(), n_yaw_steps * sizeof(double));
			this->log << "Loaded occupancy_map_orient_steps.yaw (" << this->occupancy_map_orient_steps.yaw.rows() << ")\n";
		}
		else
		{
			this->log << "No occupancy map data found" << endl;
		}


		inputFile.close();

		if (contains_map)
		{
			this->resetActiveVoxels();
			this->computeVoxelCentres(this->voxel_centres, this->map_lower_bound, this->map_upper_bound, this->voxel_edge);
		}
		if (contains_occupancy_map)
		{
			this->setOccupancyMapOrientations();
			this->resetActiveOrientations();
			this->computeVoxelCentres(this->occupancy_voxel_centres, this->occupancy_map_lower_bound, this->occupancy_map_upper_bound, this->occupancy_map_resolution);
		}
		this->log << "Data successfully loaded\n";
	}
}

//void CapabilityMap::saveToFile(const string map_file)
//{
//	std::cout << "Saving capability map...\n";
//	ofstream outputFile(map_file.c_str(), ofstream::binary);
//	outputFile.write((char *) &this->n_voxels, sizeof(this->n_voxels));
//	outputFile.write((char *) &this->n_directions_per_voxel, sizeof(this->n_directions_per_voxel));
//	std::cout << "Capability map saved in " << map_file << '\n';
//	outputFile.close();
//}

Vector2i CapabilityMap::getMapSize()
{
	Vector2i size;
	size << this->map.rows(), this->map.cols();
	return size;
}

void CapabilityMap::setNVoxels(unsigned int n_voxels)
{
	this->n_voxels = n_voxels;
}

void CapabilityMap::setNDirectionsPerVoxel(unsigned int n_dir)
{
	this->n_directions_per_voxel = n_dir;
}

RowVector2d CapabilityMap::getCapabilityMapSize()
{
	RowVector2d size;
	size << this->map.rows(), this->map.cols();
	return size;
}

vector<Vector3d> CapabilityMap::getActiveVoxelCentres()
{
	vector<Vector3d> centres;
	for (int vox = 0; vox < this->n_voxels; vox++)
	{
		if (this->active_voxels[vox])
		{
			centres.push_back(this->voxel_centres[vox]);
		}
	}
	return centres;
}

void CapabilityMap::activateVoxels(vector<int> idx)
{
	for (int i : idx)
	{
		this->active_voxels[i] = true;
	}
}

void CapabilityMap::deactivateVoxels(vector<int> idx)
{
	for (int i : idx)
	{
		this->active_voxels[i] = false;
		this->active_orientations.row(i).setZero();
	}
}

void CapabilityMap::resetActiveVoxels(bool include_zero_reachability)
{
	std::vector<int> idx(this->n_voxels);
	iota(idx.begin(), idx.end(), 0);
	if (!include_zero_reachability)
	{
		this->deactivateVoxels(idx);
		idx.clear();
		boost::push_back(idx, boost::irange(0, (int)this->n_voxels) | boost::adaptors::filtered([this](int vox){return this->reachability_index[vox] > 1e-6;}));
	}
	this->activateVoxels(idx);
}

void CapabilityMap::resetActiveOrientations()
{
	this->active_orientations = Matrix<bool, Dynamic, Dynamic>::Ones(this->n_voxels, this->n_occupancy_orient);
	for (int vox = 0; vox < this->n_voxels; vox++)
	{
		if (!this->active_voxels[vox])
		{
			this->active_orientations.row(vox).setZero();
		}
	}
}

void CapabilityMap::reduceActiveSet(bool reset_active, vector<Vector3d> point_cloud, FPPOutput &output, Vector2d sagittal_range, Vector2d transverse_range,
		Vector2d height_range, double direction_threshold)
{
	FPPTimer angle_timer, height_timer, direction_timer, collision_timer;

	angle_timer.start();
	this->deactivateVoxelsOutsideAngleRanges(sagittal_range, transverse_range, reset_active);
	angle_timer.stop();

	height_timer.start();
	this->deactivateVoxelsOutsideBaseHeightRange(height_range);
	height_timer.stop();

	direction_timer.start();
	Vector3d direction = quat2rotmat(this->endeffector_pose.block<4,1>(3,0)) * this->endeffector_axis;
	this->deactivateVoxelsByDirection(direction, direction_threshold);
	direction_timer.stop();

	collision_timer.start();
	this->deactivateCollidingVoxels(point_cloud);
	collision_timer.stop();

	output.cm_angle_time = angle_timer.getDuration();
	output.cm_base_hight_time = height_timer.getDuration();
	output.cm_direction_time = direction_timer.getDuration();
	output.cm_collision_time = collision_timer.getDuration();
}

void CapabilityMap::deactivateVoxelsOutsideAngleRanges(Eigen::Vector2d sagittal_range, Eigen::Vector2d transverse_range, bool reset_active)
{
	if (reset_active)
	{
		this->resetActiveVoxels();
		this->resetActiveOrientations();
	}
	vector<int> voxels_to_deactivate;
	sagittal_range(1) = sagittal_range(1) - this->sign(sagittal_range(1)) * M_PI;
	sagittal_range(0) = sagittal_range(0) - this->sign(sagittal_range(0)) * M_PI;
	sagittal_range = sagittal_range.reverse();
	transverse_range(1) = transverse_range(1) - this->sign(transverse_range(1)) * M_PI;
	transverse_range(0) = transverse_range(0) - this->sign(transverse_range(0)) * M_PI;
	transverse_range = transverse_range.reverse();
	double sagittal_angle, transverse_angle;
	for (int vox = 0; vox < this->n_voxels; vox++)
	{
		if (this->active_voxels[vox])
		{
			sagittal_angle = atan2(this->voxel_centres[vox](2), this->voxel_centres[vox](0));
			transverse_angle = atan2(this->voxel_centres[vox](1), this->voxel_centres[vox](0));
			if (sagittal_angle > sagittal_range(0) && sagittal_angle < sagittal_range(1) || transverse_angle > transverse_range(0) && transverse_angle < transverse_range(1))
			{
				voxels_to_deactivate.push_back(vox);
			}
		}
	}
	if (voxels_to_deactivate.size() > 0)
	{
		this->deactivateVoxels(voxels_to_deactivate);
	}
}

void CapabilityMap::deactivateVoxelsOutsideBaseHeightRange(Eigen::Vector2d range, bool reset_active)
{
	Vector3d position;
	vector<unsigned int> active_orients;
	if (reset_active)
	{
		this->resetActiveVoxels();
		this->resetActiveOrientations();
	}
	for (int vox = 0; vox < this->n_voxels; vox++)
	{
		if (this->active_voxels[vox])
		{
			for (int orient = 0; orient < this->n_occupancy_orient; orient++)
			{
				if (this->active_orientations(vox, orient))
				{
					position = rpy2rotmat(this->occupancy_map_orientations[orient]) * (this->voxel_centres[vox] + this->endeffector_pose.block<3,1>(0,0) - this->map_centre);
					if (position(2) < range(0) || position(2) > range(1))
					{
						this->active_orientations(vox, orient) = false;
					}
				}
			}
			if (!this->active_orientations.row(vox).any())
			{
				this->deactivateVoxels({vox});
			}
		}
	}
}

void CapabilityMap::deactivateVoxelsByDirection(Vector3d direction, double direction_threshold, bool reset_active)
{
	if (reset_active)
	{
		this->resetActiveVoxels();
		this->resetActiveOrientations();
	}
	vector<unsigned int> voxels_to_keep;
	for (Vector3d orient : this->occupancy_map_orientations)
	{
		for (int idx : this->findVoxelsFromDirection(rpy2rotmat(orient).transpose()*direction, direction_threshold))
		{
			if (find(voxels_to_keep.begin(), voxels_to_keep.end(), idx) == voxels_to_keep.end())
			{
				voxels_to_keep.push_back(idx);
			}
		}
	}
	vector<int> voxels_to_deactivate;
	for (int vox = 0; vox < this->n_voxels; vox++)
	{
		if (this->active_voxels[vox])
		{
			if (find(voxels_to_keep.begin(), voxels_to_keep.end(), vox) == voxels_to_keep.end())
			{
				voxels_to_deactivate.push_back(vox);
			}
		}
	}
	this->deactivateVoxels(voxels_to_deactivate);
}

void CapabilityMap::deactivateCollidingVoxels(vector<Vector3d> point_cloud, bool reset_active)
{
	if (reset_active)
	{
		this->resetActiveVoxels();
		this->resetActiveOrientations();
	}
	Vector3d ee_pose = this->endeffector_pose.block<3,1>(0,0);
	Vector3d lower_bound = this->occupancy_map_lower_bound + ee_pose;
	Vector3d upper_bound = this->occupancy_map_upper_bound + ee_pose;
	Vector3d sub_d;
	Vector3i sub;
	int idx;
	vector<int> om_occupied_voxels;
	vector<int> voxels;
	for (int point = 0; point < point_cloud.size(); point++)
	{
		if (((Array3d)point_cloud[point] > (Array3d)lower_bound).all() &&
			((Array3d)point_cloud[point] < (Array3d)upper_bound).all())
		{
			sub_d = ((point_cloud[point] - lower_bound) / this->occupancy_map_resolution);
			transform(sub_d.data(), sub_d.data() + 3, sub_d.data(), ptr_fun((double(*)(double))floor));
			sub = sub_d.cast<int>();
			idx = sub(2) * this->occupancy_map_dimensions(1) * this->occupancy_map_dimensions(0) + sub(1) * this->occupancy_map_dimensions(0) + sub(0);
			if (find(om_occupied_voxels.begin(), om_occupied_voxels.end(), idx) == om_occupied_voxels.end())
			{
				om_occupied_voxels.push_back(idx);
			}
		}
	}
	for (int vox = 0; vox < this->n_voxels; vox++)
	{
		if (this->isActiveVoxel(vox))
		{
			for (int orient = 0; orient < this->n_occupancy_orient; orient++)
			{
				if (this->isActiveOrient(vox, orient))
				{
					for (auto om_vox : this->occupancy_maps[this->active_side][vox][orient])
					{
						if (find(om_occupied_voxels.begin(), om_occupied_voxels.end(), om_vox) != om_occupied_voxels.end())
						{
							this->active_orientations(vox, orient) = false;
							break;
						}
					}
					if (!this->active_orientations.row(vox).any())
					{
						this->deactivateVoxels({vox});
						break;
					}
				}
			}
		}
	}
}

vector<unsigned int> CapabilityMap::findVoxelsFromDirection(Vector3d direction, double threshold, bool active_set_only)
{
	vector<unsigned int> voxels;
	vector<unsigned int> voxel_set;
	vector<unsigned int> points = this->findPointsFromDirection(direction, threshold);
	for (int vox = 0; vox < this->n_voxels; vox++)
	{
		if (active_set_only && this->active_voxels[vox] || !active_set_only)
		{
			for (unsigned int point : points)
			{
				if (this->map.coeffRef(vox, point))
				{
					voxels.push_back(vox);
					break;
				}
			}
		}
	}
	return voxels;
}

vector<unsigned int> CapabilityMap::findPointsFromDirection(Vector3d direction, double threshold)
{
	vector<Vector3d> sphere_points = this->distributePointsOnSphere();
	direction = direction / direction.norm();
	if (this->active_side == Side::RIGHT)
	{
		direction(1) = -direction(1);
	}
	vector<unsigned int> points;
	for (int idx = 0; idx < this->n_directions_per_voxel; idx++)
	{
		if (acos(sphere_points[idx].dot(direction)) <= threshold)
		{
			points.push_back(idx);
		}
	}
	return points;
}

vector<Vector3d> CapabilityMap::distributePointsOnSphere()
{
	vector<double> h, theta, phi;
	vector<Vector3d> points;
	for (int k = 1; k <= this->n_directions_per_voxel; k++)
	{
		h.push_back(-1 + 2.*(k-1)/(this->n_directions_per_voxel - 1));
		theta.push_back(acos(h.back()));
	}
	phi.push_back(0);
	for (int i = 1; i < this->n_directions_per_voxel-1; i++)
	{
		phi.push_back(fmod(phi[i-1] + 3.6/(sqrt(this->n_directions_per_voxel) * sqrt(1 - pow(h[i], 2))), 2*M_PI));
	}
	phi.push_back(0);
	for (int i = 0; i < this->n_directions_per_voxel; i++)
	{
		points.push_back(Vector3d(cos(phi[i]) * sin(theta[i]), sin(phi[i]) * sin(theta[i]), cos(theta[i])));
	}
	return points;
}

void CapabilityMap::computeVoxelCentres(vector<Eigen::Vector3d> &centre_array, Vector3d lower_bound, Vector3d upper_bound, double resolution)
{
	Vector3d dimensions_double = (upper_bound - lower_bound) / resolution;
	transform(dimensions_double.data(), dimensions_double.data() + 3, dimensions_double.data(), ptr_fun((double(*)(double))round));
	Vector3i dimensions = dimensions_double.cast<int>();

	unsigned int n_voxels = dimensions.prod();
	centre_array.reserve(n_voxels);
	unsigned int n_voxels_per_face = dimensions(0) * dimensions(1);
	for (int i = 0; i < n_voxels; i++)
	{
		Vector3d vox;
		vox[0] = lower_bound(0) + (i % dimensions(0) + 0.5) * resolution;
		vox[1] = lower_bound(1) + (i / dimensions(0) % dimensions(1) + 0.5) * resolution;
		vox[2] = lower_bound(2) + (i / n_voxels_per_face % dimensions(2) + 0.5) * resolution;
		centre_array.push_back(vox);
	}
}

void CapabilityMap::computePositionProbabilityDistribution(Vector3d mu, Vector3d sigma)
{
	this->computeProbabilityDistribution(this->voxel_centres, this->position_probability, mu, sigma);
	IOFormat fmt(30);
//	int w = 10;
//	for (int i = 0; i < this->position_probability.rows(); i++)
//	{
//		this->log.width(w);
//		this->log << right << this->getVoxelCentre(i)(0) << " ";
//		this->log.width(w);
//		this->log << right << this->getVoxelCentre(i)(1) << " ";
//		this->log.width(w);
//		this->log << right << this->getVoxelCentre(i)(2) << " ";
//		this->log.width(w);
//		this->log << right << this->position_probability[i].format(fmt) << endl;
//	}
//	this->log << this->position_probability.format(fmt) << endl << endl << endl;
}

void CapabilityMap::computeOrientationProbabilityDistribution(Vector3d mu, Vector3d sigma)
{
	this->computeProbabilityDistribution(this->occupancy_map_orientations, this->orientation_probability, mu, sigma);
	IOFormat fmt(30);
//	int w = 10;
//	for (int i = 0; i < this->orientation_probability.rows(); i++)
//	{
//		this->log.width(w);
//		this->log << right << this->getVoxelCentre(i)(0) << " ";
//		this->log.width(w);
//		this->log << right << this->getVoxelCentre(i)(1) << " ";
//		this->log.width(w);
//		this->log << right << this->getVoxelCentre(i)(2) << " ";
//		this->log.width(w);
//		this->log << right << this->orientation_probability[i].format(fmt) << endl;
//	}
//	this->log << this->orientation_probability.format(fmt) << endl << endl << endl;
}

void CapabilityMap::computeProbabilityDistribution(vector<Vector3d> & values, VectorXd &pdf, Vector3d mu, Vector3d sigma)
{
	Matrix3d sigma_inverse = (Vector3d(1, 1, 1).cwiseQuotient(sigma)).asDiagonal();
	pdf.resize(values.size());
	for (int p = 0; p < pdf.size(); p++)
	{
		pdf(p) = (exp(-.5*(values[p] - mu).transpose() * sigma_inverse * (values[p] - mu)));
	}
	double p_sum = pdf.sum();
	for (int p = 0; p < pdf.size(); p++)
	{
		pdf(p) /= p_sum;
	}
}

void CapabilityMap::computeTotalProbabilityDistribution()
{
	int n_orient = this->getNActiveOrientations();
	this->total_probability.resize(n_orient);
	this->total_probability_orientations.resize(n_orient);
	this->total_probability_voxels.resize(n_orient);
	int idx = 0;
	for (int vox = 0; vox < this->active_orientations.rows(); vox++)
	{
		for (int orient = 0; orient < this->active_orientations.cols(); orient++)
		{
			if (this->active_orientations(vox, orient))
			{
				this->total_probability[idx] = this->position_probability(vox) * this->orientation_probability(orient);
				this->total_probability_orientations[idx] = orient;
				this->total_probability_voxels[idx] = vox;
				idx++;
			}
		}
	}
}

int CapabilityMap::drawCapabilityMapSample(vector<int> &sample)
{
	if (this->total_probability.size() == 0)
	{
		this->computeTotalProbabilityDistribution();
	}
	std::vector<double> cumulative_probability(this->total_probability.size());
	partial_sum(this->total_probability.begin(), this->total_probability.end(), cumulative_probability.data());
	srand(time(NULL));
//	double rnd = rand() / (double)RAND_MAX * cumulative_probability.back();
	double rnd = this->random_sequence(sample[0]);
//	this->random_sequence.erase(this->random_sequence.begin());
	rnd *= cumulative_probability.back();
//	cout <<  cumulative_probability.back() << endl;
//	this->log << rnd << endl;
//	cout << "random number: " << rnd << endl;
	ArrayXd eigen_cumulative_probability =  Map<ArrayXd> (&cumulative_probability[0], cumulative_probability.size());
	Array<bool, Dynamic, 1> isGreater = eigen_cumulative_probability > rnd;
	int idx = find(isGreater.data(), isGreater.data() + isGreater.size(), 1) - isGreater.data();
//	cout << idx << endl;
	sample[0] = this->total_probability_voxels[idx];
	sample[1] = this->total_probability_orientations[idx];
	this->total_probability.erase(this->total_probability.begin() + idx);
	this->total_probability_voxels.erase(this->total_probability_voxels.begin() + idx);
	this->total_probability_orientations.erase(this->total_probability_orientations.begin() + idx);
	return 1;
}

void CapabilityMap::setOccupancyMapOrientations()
{
	for (int yaw = 0; yaw < this->occupancy_map_orient_steps.yaw.rows(); yaw++)
	{
		for (int pitch = 0; pitch < this->occupancy_map_orient_steps.pitch.rows(); pitch++)
		{
			for (int roll = 0; roll < this->occupancy_map_orient_steps.roll.rows(); roll++)
			{
				this->occupancy_map_orientations.push_back(Vector3d(this->occupancy_map_orient_steps.roll[roll],
																  this->occupancy_map_orient_steps.pitch[pitch],
																  this->occupancy_map_orient_steps.yaw[yaw]));
			}
		}
	}
}

void CapabilityMap::setActiveSide(Side side)
{
	this->active_side = side;
	if (side == this->Side::LEFT)
	{
		this->map_centre = this->map_centre_left;
	}
	else
	{
		this->map_centre = this->map_centre_right;
	}
}

void CapabilityMap::setActiveSide(string side_str)
{
	Side side = this->active_side;
	if (boost::iequals(side_str, "left"))
	{
		side = Side::LEFT;
	}
	else if (boost::iequals(side_str, "right"))
	{
		side = Side::RIGHT;
	}
	else
	{
		cout << "ERROR: CapabilityMap::setActiveSide Side must be either \"left\" or \"right\"" << endl;
	}
	this->setActiveSide(side);
}

void CapabilityMap::setEndeffectorPose(Matrix<double, 7, 1> pose)
{
	this->endeffector_pose = pose;
}



void CapabilityMap::drawCapabilityMap(bot_lcmgl_t *lcmgl, int orient, Vector3d centre, bool draw_cubes)
{
	std::vector<unsigned int> idx;
	boost::push_back(idx, boost::irange(0, (int)this->n_voxels));
	this->drawMap(lcmgl, idx, this->occupancy_map_orientations[orient], centre, draw_cubes);
}

void CapabilityMap::drawActiveMap(bot_lcmgl_t *lcmgl, int orient, Vector3d centre, bool draw_cubes)
{
	std::vector<unsigned int> idx;
	for (int vox = 0; vox < this->n_voxels; vox++)
	{
		if (this->isActiveOrient(vox, orient))
		{
			idx.push_back(vox);
		}
	}
	this->drawMap(lcmgl, idx, this->occupancy_map_orientations[orient], centre, draw_cubes);
}

void CapabilityMap::drawOccupancyMap(bot_lcmgl_t *lcmgl, unsigned int capability_map_voxel, unsigned int orient, Eigen::Vector3d centre, bool draw_cubes)
{
	if (draw_cubes)
	{
		this->drawMapCubes(lcmgl, this->occupancy_map_lower_bound, this->occupancy_map_upper_bound, this->occupancy_map_resolution, this->occupancy_map_orientations[orient], centre);
	}
	bot_lcmgl_point_size(lcmgl, 10);
	bot_lcmgl_color3f(lcmgl, 1, 0, 0);
	bot_lcmgl_begin(lcmgl, LCMGL_POINTS);
	for(auto vox : this->occupancy_maps[this->active_side][capability_map_voxel][orient])
	{
		cout << vox << endl;
		Vector3d point = rpy2rotmat(this->occupancy_map_orientations[orient]) * this->occupancy_voxel_centres[vox] + centre;
		bot_lcmgl_vertex3d(lcmgl, point(0), point(1), point(2));
	}
	bot_lcmgl_end(lcmgl);
	bot_lcmgl_switch_buffer(lcmgl);
}

void CapabilityMap::drawMap(bot_lcmgl_t *lcmgl, vector<unsigned int> &voxels, Vector3d orient, Vector3d centre, bool draw_cubes)
{
	int start_idx;
	if (draw_cubes)
	{
		this->drawMapCubes(lcmgl, this->map_lower_bound, this->map_upper_bound, this->voxel_edge, orient, centre);
		start_idx = 0;
	}
	else
	{
		start_idx = 1;
	}
	for (int i = start_idx; i < this->n_directions_per_voxel; i++)
	{
		float h = (1 - ((float)i / this->n_directions_per_voxel * 2./3.)) * 360.;
		float r, g, b;
		HSVtoRGB(r, g, b, h, 1, 1);
		bot_lcmgl_color3f(lcmgl, r, g, b);
		if (i == 0)
		{
			bot_lcmgl_point_size(lcmgl, 1);
			bot_lcmgl_color3f(lcmgl, 1, 0, 0);
		}
		else
		{
			bot_lcmgl_point_size(lcmgl, 10);
		}
		bot_lcmgl_begin(lcmgl, LCMGL_POINTS);
		for (auto vox : voxels)
		{
			if (abs((float)this->reachability_index(vox) - (float)i / this->n_directions_per_voxel) < 1e-6)
			{
				Vector3d voxel = rpy2rotmat(orient) * this->voxel_centres[vox] + centre;
				bot_lcmgl_vertex3d(lcmgl, voxel(0), voxel(1), voxel(2));
			}
		}
		bot_lcmgl_end(lcmgl);
	}
	bot_lcmgl_switch_buffer(lcmgl);
}
void CapabilityMap::drawMapCubes(bot_lcmgl_t *lcmgl, Vector3d lb, Vector3d ub, double resolution, Vector3d orient, Vector3d centre)
{
	Vector3d dim = (ub-lb)/resolution;
	Vector3i dimensions = dim.cast<int>();

	bot_lcmgl_color3f(lcmgl, .3, .3, .3);
	bot_lcmgl_line_width(lcmgl, .1);

	Vector3d start, end;
	for (int z = 0; z <= dimensions(1); z++)
	{
		for (int x = 0; x <= dimensions(0); x++)
		{
			start = rpy2rotmat(orient) * (lb + Vector3d(resolution * x, 0, resolution * z)) + centre;
			end = rpy2rotmat(orient) * (lb + Vector3d(resolution * x, ub(1)-lb(1), resolution * z)) + centre;
			draw3dLine(lcmgl, start(0), start(1), start(2), end(0), end(1), end(2));
		}
	}
	for (int z = 0; z <= dimensions(1); z++)
	{
		for (int y = 0; y <= dimensions(0); y++)
		{
			start = rpy2rotmat(orient) * (lb + Vector3d(0, resolution * y, resolution * z)) + centre;
			end = rpy2rotmat(orient) * (lb + Vector3d(ub(0)-lb(0), resolution * y, resolution * z)) + centre;
			draw3dLine(lcmgl, start(0), start(1), start(2), end(0), end(1), end(2));
		}
	}
	for (int x = 0; x <= dimensions(1); x++)
	{
		for (int y = 0; y <= dimensions(0); y++)
		{
			start = rpy2rotmat(orient) * (lb + Vector3d(resolution * x, resolution * y, 0)) + centre;
			end = rpy2rotmat(orient) * (lb + Vector3d(resolution * x, resolution * y, ub(2)-lb(2))) + centre;
			draw3dLine(lcmgl, start(0), start(1), start(2), end(0), end(1), end(2));
		}
	}
}
