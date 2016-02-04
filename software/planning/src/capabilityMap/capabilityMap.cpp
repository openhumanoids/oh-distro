#include <fstream>
#include <stdlib.h>
#include <algorithm>
#include <boost/range/irange.hpp>
#include <boost/range/algorithm_ext/push_back.hpp>
#include <boost/range/adaptors.hpp>
#include <boost/algorithm/cxx11/copy_if.hpp>

#include "capabilityMap.hpp"
#include "drawingUtil/drawingUtil.hpp"
#include "drake/util/drakeGeometryUtil.h"

using namespace std;
using namespace Eigen;

CapabilityMap::CapabilityMap():activeSide(Side::LEFT)
{

}

CapabilityMap::CapabilityMap(const string &urdf_filename):activeSide(Side::LEFT)
{

}

void CapabilityMap::loadFromMatlabBinFile(const string mapFile)
{
	MatrixX2i idx;
	unsigned int nnz;
	unsigned int stringLength;
	bool containsMap;
	bool containsOccupancyMap;
	vector<Triplet<bool>> tripletList;

	ifstream inputFile(mapFile.c_str(), ifstream::binary);

	if (!inputFile.is_open())
	{
		std::cout << "Failed to open " << mapFile.c_str() << '\n';
	}
	else
	{
		std::cout << "Loading data from" << mapFile << '\n';

		inputFile.read((char *) this->mapCentre.left.data(), sizeof(this->mapCentre.left));
		std::cout << "Loaded mapCentre.left: " << this->mapCentre.left[0] << ";"  << this->mapCentre.left[1] << ";"  << this->mapCentre.left[2] << '\n';
		inputFile.read((char *) this->mapCentre.right.data(), sizeof(this->mapCentre.right));
		std::cout << "Loaded mapCentre.right: " << this->mapCentre.right[0] << ";" << this->mapCentre.right[1] << ";"  << this->mapCentre.right[2] << '\n';

		inputFile.read((char *) &stringLength, sizeof(unsigned int));
		char *eeLinkLeftstr = new char[stringLength];
		inputFile.read(eeLinkLeftstr, stringLength);
		this->endeffectorLink.left = eeLinkLeftstr;
		delete [] eeLinkLeftstr;
		eeLinkLeftstr = nullptr;
		std::cout << "Loaded endeffectorLink.left: " << this->endeffectorLink.left.c_str() << endl;
		inputFile.read((char *) &stringLength, sizeof(unsigned int ));
		char *eeLinkRightstr = new char[stringLength];
		inputFile.read(eeLinkRightstr, stringLength * sizeof(char));
		this->endeffectorLink.right = eeLinkRightstr;
		delete [] eeLinkLeftstr;
		eeLinkLeftstr = nullptr;
		std::cout << "Loaded endeffectorLink.right: " << this->endeffectorLink.right.c_str() << endl;

		inputFile.read((char *) this->endeffectorAxis.data(), sizeof(this->endeffectorAxis));
		std::cout << "Loaded endeffectorAxis: " << this->endeffectorAxis[0] << ";"  << this->endeffectorAxis[1] << ";"  << this->endeffectorAxis[2] << '\n';

		inputFile.read((char *) &stringLength, sizeof(unsigned int));
		char *baseLinkStr = new char[stringLength];
		inputFile.read(baseLinkStr, stringLength * sizeof(char));
		this->baseLink = baseLinkStr;
		delete [] baseLinkStr;
		baseLinkStr = nullptr;
		std::cout << "Loaded baseLink: " << this->baseLink.c_str() << endl;

		inputFile.read((char *) &this->nJoints, sizeof(unsigned int));
		std::cout << "Loaded nJoints: "<< this->nJoints << endl;

		this->nominalConfiguration.resize(this->nJoints);
		inputFile.read((char *) this->nominalConfiguration.data(), this->nJoints * sizeof(VectorXd::Scalar));
		std::cout << "Loaded nominalConfiguration: ";
		for (unsigned int j = 0; j < this->nJoints; j++)
		{
			std::cout << this->nominalConfiguration[j] << ";";
		}
		std::cout << '\n';

		inputFile.read((char *) &containsMap, sizeof(bool));
		if (containsMap)
		{
			std::cout << "Found capability map data" << endl;
			inputFile.read((char *) &this->nVoxels, sizeof(unsigned int));
			inputFile.read((char *) &this->nDirectionsPerVoxel, sizeof(unsigned int));
			this->nVoxelsPerEdge = cbrt(this->nVoxels);

			boost::push_back(this->activeVoxels, boost::irange(0, (int)this->nVoxels));
			this->map.resize(this->nVoxels, this->nDirectionsPerVoxel);
			inputFile.read((char *) &nnz, sizeof(unsigned int));
			idx.resize(nnz, 2);
			inputFile.read((char *) idx.data(), nnz * 2 * sizeof(MatrixXi::Scalar));
			tripletList.reserve(nnz);
			for (unsigned int i = 0; i < nnz; i++)
			{
				tripletList.push_back(Triplet<bool>(idx(i, 0), idx(i, 1), true));
			}
			this->map.setFromTriplets(tripletList.begin(), tripletList.end());
			std::cout << "Loaded Capability Map (" << this->map.rows() << "x" << this->map.cols() << ")\n";

			this->reachabilityIndex.resize(this->nVoxels);
			inputFile.read((char *) this->reachabilityIndex.data(), this->nVoxels * sizeof(VectorXd::Scalar));
			std::cout << "Loaded Reachability Index (" << this->reachabilityIndex.rows() << ")\n";

			inputFile.read((char *) &this->voxelEdge, sizeof(double));
			std::cout << "Loaded voxelEdge :" << this->voxelEdge << endl;

			inputFile.read((char *) &this->angularTolerance, sizeof(double));
			std::cout << "Loaded angularTolerance :" << this->angularTolerance << endl;

			inputFile.read((char *) &this->positionTolerance, sizeof(double));
			std::cout << "Loaded positionTolerance :" << this->positionTolerance << endl;

			inputFile.read((char *) this->mapLowerBound.data(), sizeof(this->mapLowerBound));
			std::cout << "Loaded mapLowerBound: " << this->mapLowerBound[0] << ";"  << this->mapLowerBound[1] << ";"  << this->mapLowerBound[2] << '\n';

			inputFile.read((char *) this->mapUpperBound.data(), sizeof(this->mapUpperBound));
			std::cout << "Loaded mapUpperBound: " << this->mapUpperBound[0] << ";"  << this->mapUpperBound[1] << ";"  << this->mapUpperBound[2] << '\n';

			this->resetActiveVoxels();
			this->computeVoxelCentres();
		}
		else
		{
			std::cout << "No capability map data found" << endl;
		}

		inputFile.read((char *) &containsOccupancyMap, sizeof(bool));
		if (containsOccupancyMap)
		{
			std::cout << "Found occupancy map data" << endl;
			inputFile.read((char *) &this->nOccupancyVoxels, sizeof(unsigned int));
			std::cout << "Loaded nOccupancyVoxels: " << this->nOccupancyVoxels << endl;
			inputFile.read((char *) &this->nOccupancyOrient, sizeof(unsigned int));
			std::cout << "Loaded nOccupancyOrient: " << this->nOccupancyOrient << endl;
			this->occupancyMapLeft.resize(this->nOccupancyOrient);
			this->occupancyMapRight.resize(this->nOccupancyOrient);
/*
			std::cout << "Loading left occupancy map ..." << endl;
			for (auto map = this->occupancyMapLeft.begin(); map < this->occupancyMapLeft.end(); map++)
			{
				inputFile.read((char *) &nnz, sizeof(unsigned int));
				idx.resize(nnz, 2);
				inputFile.read((char *) idx.data(), nnz * 2 * sizeof(MatrixXi::Scalar));
				tripletList.clear();
				tripletList.reserve(nnz);
				for (unsigned int i = 0; i < nnz; i++)
				{
					tripletList.push_back(Triplet<bool>(idx(i, 0), idx(i, 1), true));
				}
				map->resize(this->nOccupancyVoxels, this->nVoxels);
				map->setFromTriplets(tripletList.begin(), tripletList.end());
			}
			std::cout << "Left occupancy map loaded" << endl;

			std::cout << "Loading right occupancy map ..." << endl;
			for (auto map = this->occupancyMapRight.begin(); map < this->occupancyMapRight.end(); map++)
			{
				inputFile.read((char *) &nnz, sizeof(unsigned int));
				idx.resize(nnz, 2);
				inputFile.read((char *) idx.data(), nnz * 2 * sizeof(MatrixXi::Scalar));
				tripletList.clear();
				tripletList.reserve(nnz);
				for (unsigned int i = 0; i < nnz; i++)
				{
					tripletList.push_back(Triplet<bool>(idx(i, 0), idx(i, 1), true));
				}
				map->resize(this->nOccupancyVoxels, this->nVoxels);
				map->setFromTriplets(tripletList.begin(), tripletList.end());
			}
			std::cout << "Right occupancy map loaded" << endl;
*/
			inputFile.read((char *) &this->occupancyMapResolution, sizeof(double));
			std::cout << "Loaded occupancyMapResolution :" << this->occupancyMapResolution << endl;

			inputFile.read((char *) this->occupancyMapDimensions.data(), sizeof(this->occupancyMapDimensions));
			std::cout << "Loaded occupancyMapDimensions: " << this->occupancyMapDimensions[0] << ";"  << this->occupancyMapDimensions[1] << ";"  << this->occupancyMapDimensions[2] << '\n';

			inputFile.read((char *) this->occupancyMapLowerBound.data(), sizeof(this->occupancyMapLowerBound));
			std::cout << "Loaded occupancyMapLowerBound: " << this->occupancyMapLowerBound[0] << ";"  << this->occupancyMapLowerBound[1] << ";"  << this->occupancyMapLowerBound[2] << '\n';

			inputFile.read((char *) this->occupancyMapUpperBound.data(), sizeof(this->occupancyMapUpperBound));
			std::cout << "Loaded occupancyMapUpperBound: " << this->occupancyMapUpperBound[0] << ";"  << this->occupancyMapUpperBound[1] << ";"  << this->occupancyMapUpperBound[2] << '\n';

			unsigned int nRollSteps;
			inputFile.read((char *) &nRollSteps, sizeof(nRollSteps));
			this->occupancyMapOrientSteps.roll.resize(nRollSteps);
			inputFile.read((char *) this->occupancyMapOrientSteps.roll.data(), nRollSteps * sizeof(double));
			std::cout << "Loaded occupancyMapOrientSteps.roll (" << this->occupancyMapOrientSteps.roll.rows() << ")\n";

			unsigned int nPitchSteps;
			inputFile.read((char *) &nPitchSteps, sizeof(nPitchSteps));
			this->occupancyMapOrientSteps.pitch.resize(nPitchSteps);
			inputFile.read((char *) this->occupancyMapOrientSteps.pitch.data(), nPitchSteps * sizeof(double));
			std::cout << "Loaded occupancyMapOrientSteps.pitch (" << this->occupancyMapOrientSteps.pitch.rows() << ")\n";

			unsigned int nYawSteps;
			inputFile.read((char *) &nYawSteps, sizeof(nYawSteps));
			this->occupancyMapOrientSteps.yaw.resize(nYawSteps);
			inputFile.read((char *) this->occupancyMapOrientSteps.yaw.data(), nYawSteps * sizeof(double));
			std::cout << "Loaded occupancyMapOrientSteps.yaw (" << this->occupancyMapOrientSteps.yaw.rows() << ")\n";
		}
		else
		{
			std::cout << "No occupancy map data found" << endl;
		}


		inputFile.close();
		std::cout << "Data successfully loaded\n";
	}
}

void CapabilityMap::saveToFile(const string mapFile)
{
	std::cout << "Saving capability map...\n";
	ofstream outputFile(mapFile.c_str(), ofstream::binary);
	outputFile.write((char *) &this->nVoxels, sizeof(this->nVoxels));
	outputFile.write((char *) &this->nDirectionsPerVoxel, sizeof(this->nDirectionsPerVoxel));
	std::cout << "Capability map saved in " << mapFile << '\n';
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
	return this->nVoxels;
}

void CapabilityMap::setNVoxels(unsigned int nVoxels)
{
	this->nVoxels = nVoxels;
}

void CapabilityMap::setNDirectionsPerVoxel(unsigned int nDir)
{
	this->nDirectionsPerVoxel = nDir;
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
	for (unsigned int i : this->activeVoxels)
	{
		centres.push_back(this->voxelCentres[i]);
	}
	return centres;
}

void CapabilityMap::activateVoxels(vector<int> idx)
{
	for (int i : idx)
	{
		if (find(this->activeVoxels.begin(), this->activeVoxels.end(), i) == this->activeVoxels.end())
		{
			this->activeVoxels.push_back(i);
		}
	}
}

void CapabilityMap::deactivateVoxels(vector<int> idx)
{
	for (int i : idx)
	{
		vector<unsigned int>::iterator activeIdx = find(this->activeVoxels.begin(), this->activeVoxels.end(), i);
		if (activeIdx != this->activeVoxels.end())
		{
			this->activeVoxels.erase(activeIdx);
		}
	}
}

void CapabilityMap::resetActiveVoxels(bool includeZeroReachability)
{
	std::vector<int> idx;
	boost::push_back(idx, boost::irange(0, (int)this->nVoxels));
	if (!includeZeroReachability)
	{
		this->deactivateVoxels(idx);
		idx.clear();
		boost::push_back(idx, boost::irange(0, (int)this->nVoxels) | boost::adaptors::filtered([this](int vox){return this->reachabilityIndex[vox] > 1e-6;}));
	}
	this->activateVoxels(idx);
}

void CapabilityMap::computeVoxelCentres()
{
	this->voxelCentres.reserve(this->nVoxels);
	unsigned int nVoxelsPerSqEdge = pow(this->nVoxelsPerEdge, 2);
	for (int i = 0; i < this->nVoxels; i++)
	{
		Vector3d vox;
		vox[0] = this->mapLowerBound(0) + (i % this->nVoxelsPerEdge + 0.5) * this->voxelEdge;
		vox[1] = this->mapLowerBound(1) + (i / this->nVoxelsPerEdge % this->nVoxelsPerEdge + 0.5) * this->voxelEdge;
		vox[2] = this->mapLowerBound(2) + (i / nVoxelsPerSqEdge % this->nVoxelsPerEdge + 0.5) * this->voxelEdge;
		this->voxelCentres.push_back(vox);
	}
}

void CapabilityMap::setActiveSide(Side side)
{
	if (this->activeSide != side)
	{
		this->activeSide = side;
	}
}

void CapabilityMap::setEndeffectorPose(Matrix<double, 7, 1> pose)
{
	this->endeffectorPose = pose;
}

void CapabilityMap::drawCapabilityMap(bot_lcmgl_t *lcmgl, Vector3d orient, Vector3d centre, bool drawCubes)
{
	std::vector<unsigned int> idx;
	boost::push_back(idx, boost::irange(0, (int)this->nVoxels));
	this->drawMap(lcmgl, idx, orient, centre, drawCubes);
}

void CapabilityMap::drawActiveMap(bot_lcmgl_t *lcmgl, Vector3d orient, Vector3d centre, bool drawCubes)
{
	this->drawMap(lcmgl, this->activeVoxels, orient, centre, drawCubes);
}

void CapabilityMap::drawMap(bot_lcmgl_t *lcmgl, vector<unsigned int> &voxels, Vector3d orient, Vector3d centre, bool drawCubes)
{
	int startIdx;
	if (drawCubes)
	{
		this->drawMapCubes(lcmgl, this->mapLowerBound, this->mapUpperBound, this->voxelEdge, orient, centre);
		startIdx = 0;
	}
	else
	{
		startIdx = 1;
	}
	for (int i = startIdx; i < this->nDirectionsPerVoxel; i++)
	{
		float h = (1 - ((float)i / this->nDirectionsPerVoxel * 2./3.)) * 360.;
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
			if (abs((float)this->reachabilityIndex(vox) - (float)i / this->nDirectionsPerVoxel) < 1e-6)
			{
				Vector3d voxel = rpy2rotmat(orient) * this->voxelCentres[vox] + centre;
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
