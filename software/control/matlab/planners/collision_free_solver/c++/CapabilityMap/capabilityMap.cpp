#include "capabilityMap.hpp"
#include <fstream>

using namespace std;
using namespace Eigen;

CapabilityMap::CapabilityMap()
{

}

CapabilityMap::CapabilityMap(const string & urdf_filename)
{

}

void CapabilityMap::loadFromFile(const string mapFile)
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
		cout << "Failed to open " << mapFile.c_str() << '\n';
	}
	else
	{
		cout << "Loading data from" << mapFile << '\n';

	//	inputFile.read((char *) &urdfLength, sizeof(urdfLength));
	//	inputFile.read((char *) this->urdf.c_str(), sizeof(char)*urdfLength);

		inputFile.read((char *) this->mapCentre.left.data(), sizeof(this->mapCentre.left));
		cout << "Loaded mapCentre.left: " << this->mapCentre.left[0] << ";"  << this->mapCentre.left[1] << ";"  << this->mapCentre.left[2] << '\n';
		inputFile.read((char *) this->mapCentre.right.data(), sizeof(this->mapCentre.right));
		cout << "Loaded mapCentre.right: " << this->mapCentre.right[0] << ";" << this->mapCentre.right[1] << ";"  << this->mapCentre.right[2] << '\n';

		inputFile.read((char *) &stringLength, sizeof(unsigned int));
		cout << stringLength << endl;
		char *eeLinkLeftstr = new char[stringLength];
		inputFile.read(eeLinkLeftstr, stringLength);
		this->endEffectorLink.left = eeLinkLeftstr;
		delete [] eeLinkLeftstr;
		eeLinkLeftstr = nullptr;
		cout << "Loaded endEffectorLink.left: " << this->endEffectorLink.left.c_str() << endl;
		inputFile.read((char *) &stringLength, sizeof(unsigned int ));
		cout << stringLength << endl;
		char *eeLinkRightstr = new char[stringLength];
		inputFile.read(eeLinkRightstr, stringLength * sizeof(char));
		this->endEffectorLink.right = eeLinkRightstr;
		delete [] eeLinkLeftstr;
		eeLinkLeftstr = nullptr;
		cout << "Loaded endEffectorLink.right: " << this->endEffectorLink.right.c_str() << endl;

		inputFile.read((char *) this->endEffectorAxis.data(), sizeof(this->endEffectorAxis));
		cout << "Loaded endEffectorAxis: " << this->endEffectorAxis[0] << ";"  << this->endEffectorAxis[1] << ";"  << this->endEffectorAxis[2] << '\n';

		inputFile.read((char *) &stringLength, sizeof(unsigned int));
		cout << stringLength << endl;
		char *baseLinkStr = new char[stringLength];
		inputFile.read(baseLinkStr, stringLength * sizeof(char));
		this->baseLink = baseLinkStr;
		delete [] baseLinkStr;
		baseLinkStr = nullptr;
		cout << "Loaded baseLink: " << this->baseLink.c_str() << endl;

		inputFile.read((char *) &this->nJoints, sizeof(unsigned int));
		cout << "Loaded nJoints: "<< this->nJoints << endl;

		this->nominalConfiguration.resize(this->nJoints);
		inputFile.read((char *) this->nominalConfiguration.data(), this->nJoints * sizeof(VectorXd::Scalar));
		cout << "Loaded nominalConfiguration: ";
		for (unsigned int j = 0; j < this->nJoints; j++)
		{
			cout << this->nominalConfiguration[j] << ";";
		}
		cout << '\n';

		inputFile.read((char *) &containsMap, sizeof(bool));
		if (containsMap)
		{
			cout << "Found capability map data" << endl;
			inputFile.read((char *) &this->nVoxels, sizeof(unsigned int));
			inputFile.read((char *) &this->nDirectionsPerVoxel, sizeof(unsigned int));
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
			cout << "Loaded Capability Map (" << this->map.rows() << "x" << this->map.cols() << ")\n";

			this->reachabilityIndex.resize(this->nVoxels);
			inputFile.read((char *) this->reachabilityIndex.data(), this->nVoxels * sizeof(VectorXd::Scalar));
			cout << "Loaded Reachability Index (" << this->reachabilityIndex.rows() << ")\n";

			inputFile.read((char *) &this->voxelEdge, sizeof(double));
			cout << "Loaded voxelEdge :" << this->voxelEdge << endl;

			inputFile.read((char *) &this->angularTolerance, sizeof(double));
			cout << "Loaded angularTolerance :" << this->angularTolerance << endl;

			inputFile.read((char *) &this->positionTolerance, sizeof(double));
			cout << "Loaded positionTolerance :" << this->positionTolerance << endl;

			inputFile.read((char *) this->mapLowerBound.data(), sizeof(this->mapLowerBound));
			cout << "Loaded mapLowerBound: " << this->mapLowerBound[0] << ";"  << this->mapLowerBound[1] << ";"  << this->mapLowerBound[2] << '\n';

			inputFile.read((char *) this->mapUpperBound.data(), sizeof(this->mapUpperBound));
			cout << "Loaded mapUpperBound: " << this->mapUpperBound[0] << ";"  << this->mapUpperBound[1] << ";"  << this->mapUpperBound[2] << '\n';
		}
		else
		{
			cout << "No capability map data found" << endl;
		}

		inputFile.read((char *) &containsOccupancyMap, sizeof(bool));
		if (containsOccupancyMap)
		{
			inputFile.read((char *) &this->nOccupancyVoxels, sizeof(unsigned int));
			cout << "Loaded nOccupancyVoxels: " << this->nOccupancyVoxels << endl;
			inputFile.read((char *) &this->nOccupancyOrient, sizeof(unsigned int));
			cout << "Loaded nOccupancyOrient: " << this->nOccupancyOrient << endl;
			this->occupancyMapLeft.resize(this->nOccupancyOrient);
			this->occupancyMapRight.resize(this->nOccupancyOrient);

			cout << "Loading left occupancy map ..." << endl;
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
			cout << "Left occupancy map loaded" << endl;

			cout << "Loading right occupancy map ..." << endl;
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
			cout << "Right occupancy map loaded" << endl;

		}


		inputFile.close();
		cout << "Data successfully loaded\n";
	}
}

void CapabilityMap::saveToFile(const string mapFile)
{
	cout << "Saving capability map...\n";
	ofstream outputFile(mapFile.c_str(), ofstream::binary);
	outputFile.write((char *) &this->nVoxels, sizeof(this->nVoxels));
	outputFile.write((char *) &this->nDirectionsPerVoxel, sizeof(this->nDirectionsPerVoxel));
	cout << "Capability map saved in " << mapFile << '\n';
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
