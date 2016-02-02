#include <iostream>

#include "capabilityMap.hpp"
//#include "lcmtypes/bot_core.hpp"
#include "bot_lcmgl_client/lcmgl.h"
//#include "lcmtypes/bot_lcmgl/data_t.hpp"
#include "lcmtypes/bot_lcmgl_data_t.h"

#include <fstream>
#include <math.h>
#include <lcm/lcm-cpp.hpp>

using namespace std;
using namespace Eigen;

CapabilityMap::CapabilityMap()//:activeSide(Side::LEFT)
{

}

CapabilityMap::CapabilityMap(const string & urdf_filename):activeSide(Side::LEFT)
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
	this->voxelCentres.resize(this->nVoxels, 3);
	unsigned int nVoxelsPerSqEdge = pow(this->nVoxelsPerEdge, 2);
	for (unsigned int vox = 0; vox < this->nVoxels; vox++)
	{
		this->voxelCentres(vox,0) = this->mapLowerBound(0) + (vox % this->nVoxelsPerEdge + 0.5) * this->voxelEdge;
		this->voxelCentres(vox,1) = this->mapLowerBound(1) + (vox / this->nVoxelsPerEdge % this->nVoxelsPerEdge + 0.5) * this->voxelEdge;
		this->voxelCentres(vox,2) = this->mapLowerBound(2) + (vox / nVoxelsPerSqEdge % this->nVoxelsPerEdge + 0.5) * this->voxelEdge;
	}
	std::cout << this->voxelCentres << endl;
}

void CapabilityMap::setActiveSide(Side side)
{
	if (this->activeSide != side)
	{
		this->activeSide = side;
	}
}

void CapabilityMap::drawCapabilityMap()
{
/*
//	bot_lcmgl_t* lcmgl;
	lcm::LCM lcm;
//	lcmgl->lcmgl.channel_name = "LCMGL";
//	lcm_t* lcm;
	lcm = lcm_create(NULL);
	lcmgl *lcmgl_ = bot_lcmgl_init(lcm.getUnderlyingLCM() ,"Capability Map");
//	lcmgl_->channel_name = "LCMGL";
	bot_lcmgl_begin(lcmgl_, LCMGL_POINTS);
	bot_lcmgl_vertex3d(lcmgl_, 0, 0, 0);
	bot_lcmgl_end(lcmgl_);
	bot_lcmgl_switch_buffer(lcmgl_);
	lcm.subscribe("",)
*/
}

/*
void CapabilityMap::drawCapabilityMap()
{
	union doubleByte
	{
	  float d;
	  unsigned char b[sizeof(float)];
	};
	union intByte
	{
	  int i;
	  unsigned char b[sizeof(int)];
	};
//	doubleByte d;
	float f;
	double d;
	intByte i;
	int32_t datalen = 0;
	lcm::LCM lcm;
	bot_lcmgl::data_t data;
	data.name = "CapabilityMap";
	data.scene = 0;
	data.sequence = 0;
	//point size
	data.data.push_back((uint8_t) 10);
	datalen++;
	f = 5;
	data.data.insert(data.data.end(), static_cast<char*>(static_cast<void*>(&f)), static_cast<char*>(static_cast<void*>(&f)) + sizeof(f));
	datalen+=sizeof(f);
	//color3f
	data.data.push_back((uint8_t) 8);
	datalen++;
	f = 1;
	data.data.insert(data.data.end(), static_cast<char*>(static_cast<void*>(&f)), static_cast<char*>(static_cast<void*>(&f)) + sizeof(f));
	datalen+=sizeof(f);
	f = 0;
	data.data.insert(data.data.end(), static_cast<char*>(static_cast<void*>(&f)), static_cast<char*>(static_cast<void*>(&f)) + sizeof(f));
	datalen+=sizeof(f);
	f = 0;
	data.data.insert(data.data.end(), static_cast<char*>(static_cast<void*>(&f)), static_cast<char*>(static_cast<void*>(&f)) + sizeof(f));
	datalen+=sizeof(f);
	//begin
	data.data.push_back((uint8_t) 4);
	datalen++;
	//points
	data.data.push_back((uint8_t) 0x0000);
	datalen++;
	//vertex3d
	data.data.push_back((uint8_t) 6);
	datalen++;

	for (int i = 0; i < 3; i++)
	{
		f = 0;
		data.data.insert(data.data.end(), static_cast<char*>(static_cast<void*>(&f)), static_cast<char*>(static_cast<void*>(&f)) + sizeof(f));
		datalen+=sizeof(f);
	}

	//end
	data.data.push_back((uint8_t) 5);
	datalen++;
	/*
	data.data.push_back(4); //begin
	datalen++;
	data.data.push_back(0x0000); //points
	datalen++;
	data.data.push_back(7); //vertex3d
	datalen++;
	for (int i = 0; i < 3; i++)
	{
		d = {0};
		data.data.insert(data.data.end(), d.b[0], d.b[3]);
	}
	datalen+=4;
	data.data.push_back(5); //end
	datalen++;

	data.datalen = datalen;
	lcm.publish("LCMGL", &data);

}
template <typename T>;
void writeNumber(bot_lcmgl::data_t &data, T number)
{

}
*/
