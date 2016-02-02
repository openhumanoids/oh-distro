#include <fstream>
#include <stdlib.h>

#include "capabilityMap.hpp"
#include "drawingUtil/drawingUtil.hpp"

using namespace std;
using namespace Eigen;

CapabilityMap::CapabilityMap()//:activeSide(Side::LEFT)
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
		std::cout << stringLength << endl;
		char *eeLinkLeftstr = new char[stringLength];
		inputFile.read(eeLinkLeftstr, stringLength);
		this->endEffectorLink.left = eeLinkLeftstr;
		delete [] eeLinkLeftstr;
		eeLinkLeftstr = nullptr;
		std::cout << "Loaded endEffectorLink.left: " << this->endEffectorLink.left.c_str() << endl;
		inputFile.read((char *) &stringLength, sizeof(unsigned int ));
		std::cout << stringLength << endl;
		char *eeLinkRightstr = new char[stringLength];
		inputFile.read(eeLinkRightstr, stringLength * sizeof(char));
		this->endEffectorLink.right = eeLinkRightstr;
		delete [] eeLinkLeftstr;
		eeLinkLeftstr = nullptr;
		std::cout << "Loaded endEffectorLink.right: " << this->endEffectorLink.right.c_str() << endl;

		inputFile.read((char *) this->endEffectorAxis.data(), sizeof(this->endEffectorAxis));
		std::cout << "Loaded endEffectorAxis: " << this->endEffectorAxis[0] << ";"  << this->endEffectorAxis[1] << ";"  << this->endEffectorAxis[2] << '\n';

		inputFile.read((char *) &stringLength, sizeof(unsigned int));
		std::cout << stringLength << endl;
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

void CapabilityMap::drawCapabilityMap(bot_lcmgl_t *lcmgl)
{
	this->drawMap(lcmgl, this->voxelCentres);
}

void CapabilityMap::drawMap(bot_lcmgl_t *lcmgl, vector<Vector3d> &voxels, Vector3d orient, Vector3d offset, bool drawCubes)
{
	int startIdx;
	if (drawCubes)
	{
		this->drawMapCubes(lcmgl, this->mapLowerBound, this->mapUpperBound, this->voxelEdge);
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
		for (int vox = 0; vox < this->nVoxels; vox++)
		{
			if (abs((float)this->reachabilityIndex(vox) - (float)i / this->nDirectionsPerVoxel) < 1e-6)
			{
				bot_lcmgl_vertex3d(lcmgl, this->voxelCentres[vox](0), this->voxelCentres[vox](1), this->voxelCentres[vox](2));
			}
		}
		bot_lcmgl_end(lcmgl);
	}
	bot_lcmgl_switch_buffer(lcmgl);
}
void CapabilityMap::drawMapCubes(bot_lcmgl_t *lcmgl, Vector3d lb, Vector3d ub, double resolution, Vector3d centre, Vector3d orientation)
{
	Vector3d dim = (ub-lb)/resolution;
	Vector3i dimensions = dim.cast<int>();
	bot_lcmgl_color3f(lcmgl, .3, .3, .3);
	bot_lcmgl_line_width(lcmgl, .1);
	for (int z = 0; z <= dimensions(1); z++)
	{
		for (int x = 0; x <= dimensions(0); x++)
		{
			draw3dLine(lcmgl, lb(0) + resolution * x, lb(1), lb(2) + resolution * z,
							  lb(0) + resolution * x, ub(1), lb(2) + resolution * z);
		}
	}
	for (int z = 0; z <= dimensions(1); z++)
	{
		for (int y = 0; y <= dimensions(0); y++)
		{
			draw3dLine(lcmgl, lb(0), lb(1) + resolution * y, lb(2) + resolution * z,
							  ub(0), lb(1) + resolution * y, lb(2) + resolution * z);
		}
	}
	for (int x = 0; x <= dimensions(1); x++)
	{
		for (int y = 0; y <= dimensions(0); y++)
		{
			draw3dLine(lcmgl, lb(0) + resolution * x, lb(1) + resolution * y, lb(2),
							  lb(0) + resolution * x, lb(1) + resolution * y, ub(2));
		}
	}
}
