#include <fstream>
#include <stdlib.h>
#include <algorithm>
#include <numeric>
#include <math.h>
#include <boost/range/irange.hpp>
#include <boost/range/algorithm_ext/push_back.hpp>
#include <boost/range/adaptors.hpp>
#include <boost/algorithm/cxx11/copy_if.hpp>
#include <boost/algorithm/string.hpp>

#include "capabilityMap/CapabilityMap.hpp"
#include "drawingUtil/drawingUtil.hpp"
#include "drake/util/drakeGeometryUtil.h"

using namespace std;
using namespace Eigen;

CapabilityMap::CapabilityMap(const string & log_filename, const string &urdf_filename):active_side(Side::LEFT)
{
	this->log.open(log_filename.c_str());
	if (!this->log.is_open())
	{
		cout << "Failed to open " << log_filename.c_str() << '\n';
	}
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
		std::cout << "Loading data from" << map_file << '\n';

		inputFile.read((char *) this->map_centre_left.data(), sizeof(this->map_centre_left));
		std::cout << "Loaded map_centre_left: " << this->map_centre_left[0] << ";"  << this->map_centre_left[1] << ";"  << this->map_centre_left[2] << '\n';
		inputFile.read((char *) this->map_centre_right.data(), sizeof(this->map_centre_right));
		std::cout << "Loaded map_centre_right: " << this->map_centre_right[0] << ";" << this->map_centre_right[1] << ";"  << this->map_centre_right[2] << '\n';

		inputFile.read((char *) &string_length, sizeof(unsigned int));
		char *ee_link_left_str = new char[string_length];
		inputFile.read(ee_link_left_str, string_length);
		this->endeffector_link.left = ee_link_left_str;
		delete [] ee_link_left_str;
		ee_link_left_str = nullptr;
		std::cout << "Loaded endeffector_link.left: " << this->endeffector_link.left.c_str() << endl;
		inputFile.read((char *) &string_length, sizeof(unsigned int ));
		char *ee_link_right_str = new char[string_length];
		inputFile.read(ee_link_right_str, string_length * sizeof(char));
		this->endeffector_link.right = ee_link_right_str;
		delete [] ee_link_right_str;
		ee_link_right_str = nullptr;
		std::cout << "Loaded endeffector_link.right: " << this->endeffector_link.right.c_str() << endl;

		inputFile.read((char *) this->endeffector_axis.data(), sizeof(this->endeffector_axis));
		std::cout << "Loaded endeffector_axis: " << this->endeffector_axis[0] << ";"  << this->endeffector_axis[1] << ";"  << this->endeffector_axis[2] << '\n';

		inputFile.read((char *) &string_length, sizeof(unsigned int));
		char *base_link_str = new char[string_length];
		inputFile.read(base_link_str, string_length * sizeof(char));
		this->base_link = base_link_str;
		delete [] base_link_str;
		base_link_str = nullptr;
		std::cout << "Loaded base_link: " << this->base_link.c_str() << endl;

		inputFile.read((char *) &this->n_joints, sizeof(unsigned int));
		std::cout << "Loaded n_joints: "<< this->n_joints << endl;

		this->nominal_configuration.resize(this->n_joints);
		inputFile.read((char *) this->nominal_configuration.data(), this->n_joints * sizeof(VectorXd::Scalar));
		std::cout << "Loaded nominal_configuration: ";
		for (unsigned int j = 0; j < this->n_joints; j++)
		{
			std::cout << this->nominal_configuration[j] << ";";
		}
		std::cout << '\n';

		inputFile.read((char *) &contains_map, sizeof(bool));
		if (contains_map)
		{
			std::cout << "Found capability map data" << endl;
			inputFile.read((char *) &this->n_voxels, sizeof(unsigned int));
			inputFile.read((char *) &this->n_directions_per_voxel, sizeof(unsigned int));
			this->n_voxels_per_edge = cbrt(this->n_voxels);

			boost::push_back(this->active_voxels, boost::irange(0, (int)this->n_voxels));
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
			std::cout << "Loaded Capability Map (" << this->map.rows() << "x" << this->map.cols() << ")\n";

			this->reachability_index.resize(this->n_voxels);
			inputFile.read((char *) this->reachability_index.data(), this->n_voxels * sizeof(VectorXd::Scalar));
			std::cout << "Loaded Reachability_index (" << this->reachability_index.rows() << ")\n";

			inputFile.read((char *) &this->voxel_edge, sizeof(double));
			std::cout << "Loaded voxel_edge :" << this->voxel_edge << endl;

			inputFile.read((char *) &this->angular_tolerance, sizeof(double));
			std::cout << "Loaded angular_tolerance :" << this->angular_tolerance << endl;

			inputFile.read((char *) &this->position_tolerance, sizeof(double));
			std::cout << "Loaded position_tolerance :" << this->position_tolerance << endl;

			inputFile.read((char *) this->map_lower_bound.data(), sizeof(this->map_lower_bound));
			std::cout << "Loaded map_lower_bound: " << this->map_lower_bound[0] << ";"  << this->map_lower_bound[1] << ";"  << this->map_lower_bound[2] << '\n';

			inputFile.read((char *) this->map_upper_bound.data(), sizeof(this->map_upper_bound));
			std::cout << "Loaded map_upper_bound: " << this->map_upper_bound[0] << ";"  << this->map_upper_bound[1] << ";"  << this->map_upper_bound[2] << '\n';
		}
		else
		{
			std::cout << "No capability map data found" << endl;
		}

		inputFile.read((char *) &contains_occupancy_map, sizeof(bool));
		if (contains_occupancy_map)
		{
			std::cout << "Found occupancy map data" << endl;
			inputFile.read((char *) &this->n_occupancy_voxels, sizeof(unsigned int));
			std::cout << "Loaded n_occupancy_voxels: " << this->n_occupancy_voxels << endl;
			inputFile.read((char *) &this->n_occupancy_orient, sizeof(unsigned int));
			std::cout << "Loaded n_occupancy_orient: " << this->n_occupancy_orient << endl;
			this->occupancy_map.resize(this->n_occupancy_voxels);
			this->active_orientations.resize(this->n_voxels);
			unsigned int n_colliding_orient;
			unsigned int n_colliding_voxels;
			unsigned int colliding_orient;
			unsigned int colliding_voxel;

			std::cout << "Loading left occupancy map ..." << endl;
			for (int vox = 0; vox < this->n_occupancy_voxels; vox++)
			{
				this->occupancy_map[vox].resize(this->n_occupancy_orient);
				inputFile.read((char *) &n_colliding_orient, sizeof(n_colliding_orient));
				for (int orient = 0; orient < n_colliding_orient; orient++)
				{
					inputFile.read((char *) &n_colliding_voxels, sizeof(n_colliding_voxels));
					inputFile.read((char *) &colliding_orient, sizeof(colliding_orient));
					for (int cmVox = 0; cmVox < n_colliding_voxels; cmVox++)
					{
						inputFile.read((char *) &colliding_voxel, sizeof(colliding_voxel));
						this->occupancy_map[vox][colliding_orient - 1].push_back(colliding_voxel - 1);
					}
				}
			}
			std::cout << "Left occupancy map loaded" << endl;

			inputFile.read((char *) &this->occupancy_map_resolution, sizeof(double));
			std::cout << "Loaded occupancy_map_resolution :" << this->occupancy_map_resolution << endl;

			inputFile.read((char *) this->occupancy_map_dimensions.data(), sizeof(this->occupancy_map_dimensions));
			std::cout << "Loaded occupancy_map_dimensions: " << this->occupancy_map_dimensions[0] << ";"  << this->occupancy_map_dimensions[1] << ";"  << this->occupancy_map_dimensions[2] << '\n';

			inputFile.read((char *) this->occupancy_map_lower_bound.data(), sizeof(this->occupancy_map_lower_bound));
			std::cout << "Loaded occupancy_map_lower_bound: " << this->occupancy_map_lower_bound[0] << ";"  << this->occupancy_map_lower_bound[1] << ";"  << this->occupancy_map_lower_bound[2] << '\n';

			inputFile.read((char *) this->occupancy_map_upper_bound.data(), sizeof(this->occupancy_map_upper_bound));
			std::cout << "Loaded occupancy_map_upper_bound: " << this->occupancy_map_upper_bound[0] << ";"  << this->occupancy_map_upper_bound[1] << ";"  << this->occupancy_map_upper_bound[2] << '\n';

			unsigned int n_roll_steps;
			inputFile.read((char *) &n_roll_steps, sizeof(n_roll_steps));
			this->occupancy_map_orient_steps.roll.resize(n_roll_steps);
			inputFile.read((char *) this->occupancy_map_orient_steps.roll.data(), n_roll_steps * sizeof(double));
			std::cout << "Loaded occupancy_map_orient_steps.roll (" << this->occupancy_map_orient_steps.roll.rows() << ")\n";

			unsigned int n_pitch_steps;
			inputFile.read((char *) &n_pitch_steps, sizeof(n_pitch_steps));
			this->occupancy_map_orient_steps.pitch.resize(n_pitch_steps);
			inputFile.read((char *) this->occupancy_map_orient_steps.pitch.data(), n_pitch_steps * sizeof(double));
			std::cout << "Loaded occupancy_map_orient_steps.pitch (" << this->occupancy_map_orient_steps.pitch.rows() << ")\n";

			unsigned int n_yaw_steps;
			inputFile.read((char *) &n_yaw_steps, sizeof(n_yaw_steps));
			this->occupancy_map_orient_steps.yaw.resize(n_yaw_steps);
			inputFile.read((char *) this->occupancy_map_orient_steps.yaw.data(), n_yaw_steps * sizeof(double));
			std::cout << "Loaded occupancy_map_orient_steps.yaw (" << this->occupancy_map_orient_steps.yaw.rows() << ")\n";
		}
		else
		{
			std::cout << "No occupancy map data found" << endl;
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
		std::cout << "Data successfully loaded\n";
	}
}

void CapabilityMap::saveToFile(const string map_file)
{
	std::cout << "Saving capability map...\n";
	ofstream outputFile(map_file.c_str(), ofstream::binary);
	outputFile.write((char *) &this->n_voxels, sizeof(this->n_voxels));
	outputFile.write((char *) &this->n_directions_per_voxel, sizeof(this->n_directions_per_voxel));
	std::cout << "Capability map saved in " << map_file << '\n';
	outputFile.close();
}

Vector2i CapabilityMap::getMapSize()
{
	Vector2i size;
	size << this->map.rows(), this->map.cols();
	return size;
}

int CapabilityMap::getNVoxels()
{
	return this->n_voxels;
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
	for (unsigned int i : this->active_voxels)
	{
		centres.push_back(this->voxel_centres[i]);
	}
	return centres;
}

void CapabilityMap::activateVoxels(vector<int> idx)
{
	for (int i : idx)
	{
		if (find(this->active_voxels.begin(), this->active_voxels.end(), i) == this->active_voxels.end())
		{
			this->active_voxels.push_back(i);
		}
	}
}

void CapabilityMap::deactivateVoxels(vector<int> idx)
{
	for (int i : idx)
	{
		this->active_voxels.erase(remove(this->active_voxels.begin(), this->active_voxels.end(), i), this->active_voxels.end());
		this->active_orientations[i].clear();
	}
}

void CapabilityMap::resetActiveVoxels(bool include_zero_reachability)
{
	std::vector<int> idx;
	boost::push_back(idx, boost::irange(0, (int)this->n_voxels));
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
	for (int vox : this->active_voxels)
	{
		this->active_orientations[vox].clear();
		boost::push_back(this->active_orientations[vox], boost::irange(0, (int)this->n_occupancy_orient));
	}
}

void CapabilityMap::reduceActiveSet(bool reset_active, vector<Vector3d> point_cloud, Vector2d sagittal_range, Vector2d transverse_range,
		Vector2d height_range, double direction_threshold)
{
	this->deactivateVoxelsOutsideAngleRanges(sagittal_range, transverse_range, reset_active);
	this->deactivateVoxelsOutsideBaseHeightRange(height_range);
	Vector3d direction = quat2rotmat(this->endeffector_pose.block<4,1>(3,0)) * this->endeffector_axis;
	this->deactivateVoxelsByDirection(direction, direction_threshold);
	this->deactivateCollidingVoxels(point_cloud);
}

void CapabilityMap::deactivateVoxelsOutsideAngleRanges(Eigen::Vector2d sagittal_range, Eigen::Vector2d transverse_range, bool reset_active)
{
	if (reset_active)
	{
		this->resetActiveVoxels();
		this->resetActiveOrientations();
	}
	vector<int> voxels_to_deactivate;
	for (int vox : this->active_voxels)
	{
		double sagittal_angle = atan2(this->voxel_centres[vox](2), this->voxel_centres[vox](0));
		double transverse_angle = atan2(this->voxel_centres[vox](1), this->voxel_centres[vox](0));
		sagittal_angle = sagittal_angle - this->sign(sagittal_angle) * M_PI;
		transverse_angle = transverse_angle - this->sign(transverse_angle) * M_PI;
		if (sagittal_angle < sagittal_range(0) || sagittal_angle > sagittal_range(1) || transverse_angle < transverse_range(0) || transverse_angle > transverse_range(1))
		{
			voxels_to_deactivate.push_back(vox);
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
	vector<unsigned int> active_voxels = this->active_voxels;
	vector<unsigned int> active_orients;
	if (reset_active)
	{
		this->resetActiveVoxels();
		this->resetActiveOrientations();
	}
	for (int vox : active_voxels)
	{
		active_orients = this->active_orientations[vox];
		for (int orient : active_orients)
		{
			position = rpy2rotmat(this->occupancy_map_orientations[orient]) * (this->voxel_centres[vox] + this->endeffector_pose.block<3,1>(0,0) - this->map_centre);
			if (position(2) < range(0) || position(2) > range(1))
			{
				this->active_orientations[vox].erase(remove(this->active_orientations[vox].begin(), this->active_orientations[vox].end(), orient), this->active_orientations[vox].end());
			}
		}
		if (this->active_orientations[vox].size() == 0)
		{
			this->active_voxels.erase(remove(this->active_voxels.begin(), this->active_voxels.end(), vox), this->active_voxels.end());
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
	for (unsigned int vox : this->active_voxels)
	{
		if (find(voxels_to_keep.begin(), voxels_to_keep.end(), vox) == voxels_to_keep.end())
		{
			voxels_to_deactivate.push_back(vox);
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
	Vector3d sub_d;
	Vector3i sub;
	int idx;
	vector<int> om_voxels;
	vector<int> voxels;
	for (int point = 0; point < point_cloud.size(); point++)
	{
		if (((Array3d)point_cloud[point] > (Array3d)(this->occupancy_map_lower_bound + ee_pose)).all() &&
			((Array3d)point_cloud[point] < (Array3d)(this->occupancy_map_upper_bound + ee_pose)).all())
		{
			sub_d = ((point_cloud[point] - this->occupancy_map_lower_bound - ee_pose) /
					this->occupancy_map_resolution);
			transform(sub_d.data(), sub_d.data() + 3, sub_d.data(), ptr_fun((double(*)(double))floor));
			sub = sub_d.cast<int>();
			idx = sub(2) * this->occupancy_map_dimensions(1) * this->occupancy_map_dimensions(0) + sub(1) * this->occupancy_map_dimensions(0) + sub(0);
			if (find(om_voxels.begin(), om_voxels.end(), idx) == om_voxels.end())
			{
				om_voxels.push_back(idx);
			}
		}
	}
	for(int om_vox : om_voxels)
	{
		for (int orient = 0; orient < this->n_occupancy_orient; orient++)
		{
			for (int vox : this->occupancy_map[om_vox][orient])
			{
				if (find(this->active_orientations[vox].begin(), this->active_orientations[vox].end(), orient) != this->active_orientations[vox].end() && orient == 52){voxels.push_back(vox);}
				this->active_orientations[vox].erase(remove(this->active_orientations[vox].begin(), this->active_orientations[vox].end(), orient),
					this->active_orientations[vox].end());
				if (this->active_orientations[vox].size() == 0)
				{
					this->active_voxels.erase(remove(this->active_voxels.begin(), this->active_voxels.end(), vox), this->active_voxels.end());
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
	if (active_set_only)
	{
		voxel_set = this->active_voxels;
	}
	else
	{
		voxel_set.resize(this->n_voxels);
		iota(voxel_set.begin(), voxel_set.end(), 0);
	}
	for (unsigned int vox : voxel_set)
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
}

void CapabilityMap::computeOrientationProbabilityDistribution(Vector3d mu, Vector3d sigma)
{
	this->computeProbabilityDistribution(this->occupancy_map_orientations, this->orientation_probability, mu, sigma);
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
//	MatrixXd compound_probability = this->position_probability * this->orientation_probability.transpose();
//	compound_probability = compound_probability.cwiseProduct(this)
	this->total_probability.resize(this->n_voxels * this->n_occupancy_orient);
//	cout << this->total_probability.rows() << "x" << this->total_probability.cols() << endl;
//	for(auto i : this->active_voxels){cout << i << endl;}
	for (int p = 0; p < this->active_orientations.size(); p++)
	{
//		this->log << p << endl;
		for (int o : this->active_orientations[p])
		{
//			cout << "test" << endl;
//			cout << this->active_orientations[p][o] << endl << endl;
//			this->log << p*(o+1) << " " << this->position_probability(p) * this->orientation_probability(o) << endl;
//			this->log << this->active_orientations[p][o];
			this->total_probability(p*this->n_occupancy_orient+o) = this->position_probability(p) * this->orientation_probability(o);
		}
	}
	for (int i = 0; i < this->total_probability.rows(); i++){
		if (this->total_probability(i) > 0)
			{this->log << i << " " << this->total_probability(i) << endl;}
	}
//	this->log << this->total_probability.segment(2000, 2000) << endl;
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
	for (int vox : this->active_voxels)
	{
		if (find(this->active_orientations[vox].begin(), this->active_orientations[vox].end(), orient) != this->active_orientations[vox].end())
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
	for (int vox = 0; vox < this->occupancy_map.size(); vox++)
	{
		for(int i = 0; i < this->occupancy_map[vox].size(); i++)
		{
			if (this->occupancy_map[vox][orient].size() > 0 && find(this->occupancy_map[vox][orient].begin(), this->occupancy_map[vox][orient].end(), capability_map_voxel) !=this->occupancy_map[vox][orient].end())
			{
				Vector3d point = rpy2rotmat(this->occupancy_map_orientations[orient]) * this->occupancy_voxel_centres[vox] + centre;
				bot_lcmgl_vertex3d(lcmgl, point(0), point(1), point(2));
			}
		}
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
