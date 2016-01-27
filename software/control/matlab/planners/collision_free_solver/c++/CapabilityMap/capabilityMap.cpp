#include "capabilityMap.hpp"
#include <fstream>

using namespace std;
using namespace Eigen;

CapabilityMap::CapabilityMap():nVoxels(0), nDirectionsPerVoxel(0)
{

}

CapabilityMap::CapabilityMap(const string & urdf_filename):nVoxels(0), nDirectionsPerVoxel(0)
{

}

void CapabilityMap::loadFromFile(const string mapFile)
{
	typedef Matrix<bool, Dynamic, Dynamic> MatrixXb;
	MatrixX2i idx;
	unsigned int nnz;
	unsigned int stringLength;
	char *buffer;
	bool containsMap;
	bool containsOccupancyMap;

	cout << "Loading data from" << mapFile << '\n';
	ifstream inputFile(mapFile.c_str(), ifstream::binary);

//	inputFile.read((char *) &urdfLength, sizeof(urdfLength));
//	inputFile.read((char *) this->urdf.c_str(), sizeof(char)*urdfLength);

	inputFile.read((char *) this->mapCentre.left.data(), sizeof(this->mapCentre.left));
//	cout << "Loaded mapCentre.left: " << this->mapCentre.left[0] << ";"  << this->mapCentre.left[1] << ";"  << this->mapCentre.left[2] << '\n';
	inputFile.read((char *) this->mapCentre.right.data(), sizeof(this->mapCentre.right));
//	cout << "Loaded mapCentre.right: " << this->mapCentre.right[0] << ";" << this->mapCentre.right[1] << ";"  << this->mapCentre.right[2] << '\n';

	inputFile.read((char *) &stringLength, sizeof(unsigned int));
	buffer = new char[stringLength];
	inputFile.read(buffer, stringLength * sizeof(char));
	this->endEffectorLink.left = buffer;
//	cout << "Loaded endEffectorLink.left: " << this->endEffectorLink.left.c_str() << endl;
	inputFile.read((char *) &stringLength, sizeof(unsigned int));
	buffer = new char[stringLength];
	inputFile.read(buffer, stringLength * sizeof(char));
	this->endEffectorLink.right = buffer;
//	cout << "Loaded endEffectorLink.right: " << this->endEffectorLink.right.c_str() << endl;

	inputFile.read((char *) this->endEffectorAxis.data(), sizeof(this->endEffectorAxis));
//	cout << "Loaded endEffectorAxis: " << this->endEffectorAxis[0] << ";"  << this->endEffectorAxis[1] << ";"  << this->endEffectorAxis[2] << '\n';

	inputFile.read((char *) &stringLength, sizeof(unsigned int));
	buffer = new char[stringLength];
	inputFile.read(buffer, stringLength * sizeof(char));
	this->baseLink = buffer;
//	cout << "Loaded baseLink: " << this->baseLink.c_str() << endl;

	inputFile.read((char *) &this->nJoints, sizeof(unsigned int));
	this->nominalConfiguration.resize(nJoints);
	inputFile.read((char *) this->nominalConfiguration.data(), this->nJoints * sizeof(VectorXd::Scalar));
//	cout << "Loaded nominalConfiguration: ";
//	for (unsigned int j = 0; j < this->numJoints; j++)
//	{
//		cout << this->nominalConfiguration[j] << ";";
//	}
//	cout << '\n';

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
		cout << idx.row(0) <<endl;
		vector<Triplet<bool>> tripletList;
		tripletList.reserve(nnz);
		for (unsigned int i = 0; i < nnz; i++)
		{
			tripletList.push_back(Triplet<bool>(idx(i, 0), idx(i, 1), true));
		}
		cout<<tripletList[0].row()<<" "<<tripletList[0].col()<<" "<<tripletList[0].value()<<endl;
		this->map.setFromTriplets(tripletList.begin(), tripletList.end());
//		cout << "Loaded Capability Map (" << this->map.rows() << "x" << this->map.cols() << ")\n";

		this->reachabilityIndex.resize(this->nVoxels);
		inputFile.read((char *) this->reachabilityIndex.data(), this->nVoxels * sizeof(VectorXd::Scalar));
//		cout << "Loaded Reachability Index (" << this->reachabilityIndex.rows() << ")\n";

		inputFile.read((char *) &this->voxelEdge, sizeof(double));
//		cout << "Loaded voxelEdge :" << this->voxelEdge << endl;

		inputFile.read((char *) &this->angularTolerance, sizeof(double));
//		cout << "Loaded angularTolerance :" << this->angularTolerance << endl;

		inputFile.read((char *) &this->positionTolerance, sizeof(double));
//		cout << "Loaded positionTolerance :" << this->positionTolerance << endl;

		inputFile.read((char *) this->mapLowerBound.data(), sizeof(this->mapLowerBound));
//		cout << "Loaded mapLowerBound: " << this->mapLowerBound[0] << ";"  << this->mapLowerBound[1] << ";"  << this->mapLowerBound[2] << '\n';

		inputFile.read((char *) this->mapUpperBound.data(), sizeof(this->mapUpperBound));
//		cout << "Loaded mapUpperBound: " << this->mapUpperBound[0] << ";"  << this->mapUpperBound[1] << ";"  << this->mapUpperBound[2] << '\n';
	}
	else
	{
		cout << "No capability map data found" << endl;
	}

	inputFile.read((char *) &containsOccupancyMap, sizeof(bool));
	if (containsOccupancyMap)
	{

	}

	inputFile.close();
	cout << "Data successfully loaded\n";
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
