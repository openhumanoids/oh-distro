/*
 * PclSurrogateUtils.cpp
 *
 *  Created on: Jan 19, 2012
 *      Author: mfleder
 */

#include "PclSurrogateUtils.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/common/pca.h>
#include <pcl/pcl_macros.h> //is_finite()
//#include <pcl/common/impl/centroid.hpp> //for centroid
//#include <pcl/common/common_headers.h>

#include <set>

#include "LinearAlgebra.h"

using namespace std;
using namespace pcl;
using namespace Eigen;

namespace surrogate_gui
{
	//----------conversions

	//========utilities
	typedef PointXYZRGB PointT;
	typedef PointCloud<PointT> CloudT;

	PointCloud<PointXYZRGB>::Ptr PclSurrogateUtils::downsampleWithVoxel(CloudT::Ptr input_cloud)
	{
		CloudT::Ptr cloud_downsampled(new CloudT);
		VoxelGrid<PointT> sor;
		sor.setInputCloud(input_cloud);
		double leaf = 0.00000001;
		sor.setLeafSize(leaf, leaf, leaf);
		sor.filter(*cloud_downsampled);
		return cloud_downsampled;
	}

	/**downsample by skipping every other point
	 * @param input_cloud cloud to downsample
	 * @param skipValue 2 : returns 1/2 the points
	 * 				 	 3 : return 1/3 of the points*/
	PointCloud<PointXYZRGB>::Ptr PclSurrogateUtils::downsample(CloudT::ConstPtr input_cloud, uint skipValue)
	{
		/*if (!input_cloud->isOrganized())
			throw SurrogateException("Can't downsample unorganized point cloud");
		if (!input_cloud->is_dense)
			throw SurrogateException("Cloud isn't dense");
		*/
		CloudT::Ptr cloud_downsampled(new CloudT);
		cloud_downsampled->reserve(input_cloud->size()/skipValue);

		/*for (uint i = 0; i < input_cloud->width; i+=skipValue)
		{
			for (uint j = 0; j < input_cloud->height; j+=skipValue)
			{
				cloud_downsampled->push_back(input_cloud->at(i,j));
			}
		}*/

		for(uint i = 0; i < input_cloud->size(); i+=skipValue)
		{
			cloud_downsampled->push_back((*input_cloud)[i]);
		}

		return cloud_downsampled;
	}


	/**@returns the points corresponding to indices in originalCloud*/
	PointCloud<PointXYZRGB>::Ptr PclSurrogateUtils::extractIndexedPoints(const SetIntPtr indices,
																		 const PointCloud<PointXYZRGB>::ConstPtr originalCloud)
	{
		PointCloud<PointXYZRGB>::Ptr pcl_cloud(new PointCloud<PointXYZRGB>);

		//don't know the width / height.  should have sent it over from the kinect
		pcl_cloud->width = indices->size();
		pcl_cloud->height = 1;
		pcl_cloud->points.resize(pcl_cloud->width * pcl_cloud->height);

		int i = 0; //index into the  Pcl Point Cloud
		for(set<int>::const_iterator indexItr = indices->begin(); //go thru all indices in object points
			indexItr != indices->end(); ++indexItr)
		{
			int whichIndex = *indexItr;
			pcl_cloud->points[i].x = originalCloud->points[whichIndex].x;
			pcl_cloud->points[i].y = originalCloud->points[whichIndex].y;
			pcl_cloud->points[i].z = originalCloud->points[whichIndex].z;
			pcl_cloud->points[i].rgb = originalCloud->points[whichIndex].rgb;
			i++;
		}

		//cout << "\nindices as cloud size = " << pcl_cloud->points.size() << "\n" << endl;
		return pcl_cloud;
	}


	/**@return pcl format of lcm_cloud*/
	PointCloud<PointXYZRGB>::Ptr PclSurrogateUtils::toPcl(const lkr_color_point_cloud_t *lcm_cloud)
	{
		PointCloud<PointXYZRGB>::Ptr pcl_cloud(new PointCloud<PointXYZRGB>);

		//don't know the width / height.  should have sent it over from the kinect
		pcl_cloud->width = lcm_cloud->width;
		pcl_cloud->height = lcm_cloud->height;
		pcl_cloud->points.resize(pcl_cloud->width * pcl_cloud->height);

		for (int i = 0; i < lcm_cloud->num_points; i++)
		{
			if (   !pcl_isfinite(lcm_cloud->points[i].x)
				|| !pcl_isfinite(lcm_cloud->points[i].y)
				|| !pcl_isfinite(lcm_cloud->points[i].z))
			{
				lcm_cloud->points[i].x
				= lcm_cloud->points[i].y
				= lcm_cloud->points[i].z
				= numeric_limits<float>::quiet_NaN();
			}

			pcl_cloud->points[i].x = lcm_cloud->points[i].x;
			pcl_cloud->points[i].y = lcm_cloud->points[i].y;
			pcl_cloud->points[i].z = lcm_cloud->points[i].z;
			pcl_cloud->points[i].r = lcm_cloud->points[i].r;
			pcl_cloud->points[i].g = lcm_cloud->points[i].g;
			pcl_cloud->points[i].b = lcm_cloud->points[i].b;
		}

		return pcl_cloud;
	}

	/**@return as point cloud*/
	PointCloud<PointXYZ>::Ptr PclSurrogateUtils::toPcl(const list<PointXYZ> &input)
	{
		PointCloud<PointXYZ>::Ptr pcl_cloud(new PointCloud<PointXYZ>);

		//don't know the width / height.  should have sent it over from the kinect
		pcl_cloud->width = input.size();
		pcl_cloud->height = 1;
		pcl_cloud->points.resize(pcl_cloud->width * pcl_cloud->height);

		int i = 0;
		for (list<PointXYZ>::const_iterator iter = input.begin();
			 iter != input.end(); iter++, i++)
		{
			PointXYZ next = *iter;
			pcl_cloud->points[i].x = next.x;
			pcl_cloud->points[i].y = next.y;
			pcl_cloud->points[i].z = next.z;
		}
		return pcl_cloud;
	}


	PointCloud<PointXYZRGB>::Ptr PclSurrogateUtils::toPcl(const vector<int> &indices, const lkr_color_point_cloud_t *originalCloud)
	{
		PointCloud<PointXYZRGB>::Ptr pcl_cloud(new PointCloud<PointXYZRGB>);

		//don't know the width / height.  should have sent it over from the kinect
		pcl_cloud->width = indices.size();
		pcl_cloud->height = 1;
		pcl_cloud->points.resize(pcl_cloud->width * pcl_cloud->height);

		int i = 0; //index into the  Pcl Point Cloud
		for(vector<int>::const_iterator nextLcmInd = indices.begin(); //go thru all indices in object points
			nextLcmInd != indices.end(); ++nextLcmInd)
		{
			int whichLcmIndex = *nextLcmInd;
			pcl_cloud->points[i].x = originalCloud->points[whichLcmIndex].x;
			pcl_cloud->points[i].y = originalCloud->points[whichLcmIndex].y;
			pcl_cloud->points[i].z = originalCloud->points[whichLcmIndex].z;
			pcl_cloud->points[i].r = originalCloud->points[whichLcmIndex].r;
			pcl_cloud->points[i].g = originalCloud->points[whichLcmIndex].g;
			pcl_cloud->points[i].b = originalCloud->points[whichLcmIndex].b;
			i++;
		}

		//cout << "\nindices as cloud size = " << pcl_cloud->points.size() << "\n" << endl;
		return pcl_cloud;
	}

	/**@return pcl format of objPts*/
	PointCloud<PointXYZRGB>::Ptr PclSurrogateUtils::toPcl(const ObjectPointsPtr objPts, const lkr_color_point_cloud_t *originalCloud)
	{
		SetIntPtr setInd = objPts->indices;

		//convert to vector
		vector<int> indices;
		indices.reserve(setInd->size());

		for(set<int>::const_iterator iter = setInd->begin();
			iter != setInd->end(); ++iter)
		{
			indices.push_back(*iter);
		}

		return toPcl(indices, originalCloud);
	}

	/**Converts indices into pcl PointIndices format*/
	PointIndices::Ptr PclSurrogateUtils::toPclIndices(const SetIntPtr indices)
	{
		PointIndices *pi = new PointIndices;
		for(set<int>::iterator indexIter = indices->begin();
			indexIter != indices->end();
			indexIter++)
		{
			pi->indices.push_back(*indexIter);
		}

		PointIndices::Ptr pclIndices(pi);
		return pclIndices;
	}

	/**@return a copy of indices*/
	PointIndices::Ptr PclSurrogateUtils::copyIndices(const PointIndices::Ptr indices)
	{
		PointIndices::Ptr copy(new PointIndices);
		copy->indices.insert(copy->indices.end(),
							 indices->indices.begin(),
							 indices->indices.end());
		return copy;
	}

	/**removes RGB info and converts to XYZ
	 * @param input unmodified input to convert to xyz
	 * @param ouput will be input w/o RGB*/
	void PclSurrogateUtils::convert(const vector<PointXYZRGB> &input,
									list<PointXYZ> &output)
	{
		if (input.size() == 0)
			throw SurrogateException("convert: no points to convert");

		output.clear();
		for(uint i = 0; i < input.size(); i++)
		{
			const PointXYZRGB &next = input[i];
			output.push_back(PointXYZ(next.x, next.y, next.z));
		}
	}



	//========================linear algebra on point clouds
	Eigen::Vector3f PclSurrogateUtils::getCentroid(const PointCloud<PointXYZ>::Ptr &cloud)
	{
		if (cloud->points.size() == 0)
			throw SurrogateException("Can't get centroid of cempty cloud");
		Eigen::Vector4f centroid4f;
		compute3DCentroid(*cloud, centroid4f);
		return Vector3f(centroid4f[0], centroid4f[1], centroid4f[2]);
	}

	/**@param cloud input cloud
	 * @param centroid point to subtract from each item in cloud
	 * @return cloud - centroid*/
	PointCloud<PointXYZ>::Ptr PclSurrogateUtils::subtractCentroid(const PointCloud<PointXYZ>::Ptr &cloud,
															 const Vector3f &centroid)
	{
		PointCloud<PointXYZ>::Ptr shiftedCloud(new PointCloud<PointXYZ>);
		shiftedCloud->width = cloud->points.size();
		shiftedCloud->height = 1;
		shiftedCloud->points.resize(cloud->points.size());

		if (cloud->points.size() == 0)
			throw SurrogateException("Can't subtract centroid from empty cloud");

		Vector3f newMean(0,0,0); //for defensive checking at the end
		for(uint i = 0; i < cloud->points.size(); i++)
		{
			Vector3f nextShifted = toVec(cloud->points[i]) - centroid;
			shiftedCloud->points[i] = toPt(nextShifted);

			newMean += nextShifted; //to defensively check that centroid was the centroid
		}

		//---sanity check to verify that centroid was the centroid
		newMean /= cloud->points.size();
		if (newMean.norm() > 0.00001)
			throw SurrogateException("Centroid was not the centroid!");

		return shiftedCloud;
	}


	 /**Projects inputCloud into the plane spanned by v0 and v1 and re-writes
	  * using the ordered basis {v0, v1}
	  * @param inputCloud cloud to project into v0,v1 plane
	  * @param v0 first new basis vector.  should be unit vector
	  * @param v1 2nd new basis vector.  should be unit vector*/
	 PointCloud<PointXY>::Ptr PclSurrogateUtils::projectAndChangeBasis(const PointCloud<PointXYZ>::Ptr &inputCloud,
																	   const Eigen::Vector3f &v0,
																	   const Eigen::Vector3f &v1)
	 {
		 //---defensive checks
		 if (abs(v0.norm() - 1) > 0.000001 || abs(v1.norm() - 1) > 0.000001)
			 throw SurrogateException("Expected unit vectors");
		 if (abs(v0.dot(v1)) > 0.00001)
			 throw SurrogateException("Expected orthonormal basis");

		 //setup return cloud
		 PointCloud<PointXY>::Ptr projectedCloud(new PointCloud<PointXY>);
		 projectedCloud->width = inputCloud->points.size();
		 projectedCloud->height = 1;
		 projectedCloud->points.resize(inputCloud->points.size());

		 //actually project
		 for (uint i = 0; i < inputCloud->points.size(); i++)
		 {
			 //project
			 Eigen::Vector3f p = toVec(inputCloud->points[i]);
			 Eigen::Vector2f projection(v0.dot(p),
					                    v1.dot(p));
			 //store
			 projectedCloud->points[i] = toPt2D(projection);
		 }

		 return projectedCloud;
	 }


	/**@param points data set for which we want a maximally spaced subset of size numPoints
	 * @param numPoint number of points we want to return. will return all of points if
	 * numPoints < points.size()
	 * @return subset of points of size numPoints that is maximally spaced.  Will clear first.
	 * Currently, we always take the first and last points in points*/
	 void PclSurrogateUtils::getMaximallySpacedSubset(const list<PointXYZ> &points, uint numPoints,
													   list<PointXYZ> &spacedSubset)
	{
		//--defensive checks
		if (numPoints <= 1)
			throw SurrogateException("Why are you calling this to get a set of size <= 1? getMaximallySpacedSubset");
		if (points.size() == 0)
			throw SurrogateException("Can't get a maximally spaced subset of an empty set");

		//Anything to return?
		if (points.size() <= numPoints)
		{
			spacedSubset = list<PointXYZ>(points.begin(), points.end());
			return;
		}

		//----setup data structures
		spacedSubset.clear();
		vector<PointXYZ> output;
		vector<PointXYZ> input(points.begin(), points.end());

		//-----------compute
		set<int> usedIndices; //indices into points that have been included in the output so far
		usedIndices.insert(0);//definitely include the first and last points
		usedIndices.insert(input.size()-1);
		output.push_back(input[0]);
		output.push_back(input[input.size()-1]);

		while(output.size() < numPoints) //continue until we have enough points
		{
			//get the point that has the largest minimal distance from anything
			//already in output (that isn't marked in usedIndices);
			int bestIndex = getFarthestPointIndex(input, usedIndices,output);
			usedIndices.insert(bestIndex);
			output.push_back(input[bestIndex]);
		}

		//---output
		spacedSubset = list<PointXYZ>(output.begin(), output.end());
	}


	 /**@param set set of points from which we want to select the 1 that has the largest minimum distance from
	  * any point in ptsToMeasure
	  * @param dontConsiderIndices set of indices into availablePts that we should NOT consider
	  * @param ptsToMeasure points for which we want a pt from availalbePts that is far away
	  * @return index into availalbePts*/
	 int PclSurrogateUtils::getFarthestPointIndex(const vector<PointXYZ> &availablePts, const set<int> &dontConsiderIndices,
												  const vector<PointXYZ> &ptsToMeasure)
	 {
		 if (dontConsiderIndices.size() == availablePts.size())
			 throw SurrogateException("availablePts.size() == dontConsiderIndices.size().  Nothing left to choose");
		 if (availablePts.size() == 0)
			 throw SurrogateException("Emtpy set passed into getFarthestPointIndex");

		 if (ptsToMeasure.size() == 0)
			 throw SurrogateException("can't measure distance to empty set");

		 //debug
		 /*
		 for (uint i = 0; i < availablePts.size(); i++)
		 	 cout << "\n next available pt: \n" << availablePts[i] << endl;
		 for (uint i = 0; i < ptsToMeasure.size(); i++)
		 	 cout << "\n next measure pt: \n" << ptsToMeasure[i] << endl;
		 for (set<int>::iterator it = dontConsiderIndices.begin();
			  it != dontConsiderIndices.end(); it++)
		 {
			 cout << "\nnext dont index = " << *it << endl;
			 cout << "\n corr dont pt = \n" << availablePts[*it] << endl;
		 }

		 cout << "\n\n ==" << endl;
		 */
		 //-

		 double maxAvailDistance = -1; //maximal min distance for a pt from available pts to anything in ptsToMeasure
		 int bestAvailIndex = -1; //index into availablePts
		 for(uint i = 0; i < availablePts.size(); i++)
		 {
			 if (dontConsiderIndices.find(i) != dontConsiderIndices.end())
			 {
				 //cout << "\n set contained index " << i << ".  so skipping" << endl;
				 continue; //index not available
			 }

			 //cout << "\n===examining available pt: \n" << availablePts[i] << endl;

			 //compute the minimum distance from availablePts[i]
			 //to anything in ptsToMeasure
			 double minDistToSet = -1; //minimum distance
			 for (uint n = 0; n < ptsToMeasure.size(); n++)
			 {
				double nextDist = euclideanDistance(availablePts[i], ptsToMeasure[n]);
				//cout << "\n nextMeasuredDist = " << nextDist << endl;
				if (minDistToSet == -1 || minDistToSet > nextDist) //not initialized, or found new min
					minDistToSet = nextDist;
			 }

			 //cout << "\n nextMinDist = " << minDistToSet << endl;

			//we now have the minimum distance from availablePts[i] to
			//anything in ptsToMeasure.
			//If this distance is farther (better) than anything found so far,
			//then update the maxAvailDistance and bestAvailIndex
			if (bestAvailIndex == -1 || maxAvailDistance < minDistToSet)
			{
				maxAvailDistance = minDistToSet;
				bestAvailIndex = i;

				//cout << "\n updated max to " << maxAvailDistance << endl;
			}
		 }

		 return bestAvailIndex;
	 }


	//====misc
	void PclSurrogateUtils::print(PointCloud<PointXYZ> &cloud)
	{
		for (uint i = 0; i < cloud.size(); i++)
			cout << "\n" << cloud.at(i) << endl;
	}

	//==========================model fitting
	Vector3f PclSurrogateUtils::toVec(const PointXYZ &p)
	{
		return Vector3f(p.x, p.y, p.z);
	}

	PointXYZ PclSurrogateUtils::toPt(const Vector3f &p)
	{
		return PointXYZ(p[0], p[1], p[2]);
	}

	PointXY PclSurrogateUtils::toPt2D(const Vector2f &p)
	{
		PointXY r;
		r.x = p[0];
		r.y = p[1];

		return r;
	}


	void PclSurrogateUtils::runPCA(const list<PointXYZ> &points,
								   PCA_Results &results)
	{
		if (points.size() < 3)
			throw SurrogateException("called line3D fit with < 3 points");

		//---run PCA
		PointCloud<PointXYZ>::Ptr cloud = toPcl(points);
		pcl::PCA<PointXYZ> pca(*cloud, false); //basis_only = true --> compute coefficients
		results.inputCloud = cloud;

		//----extract eigenvectors + eigenvalues
		Matrix3f eigenVectors = pca.getEigenVectors(); //eigen vectors are the columns
		Vector3f eigenValues = pca.getEigenValues();

		//determine the eigen vector with the largest eigen value
		uint bestIndex = (eigenValues[0] > eigenValues[1]) ? 0 : 1;
		if (eigenValues[2] > eigenValues[bestIndex])
			bestIndex = 2;
		results.bestEigen0 = eigenVectors.col(bestIndex);
		results.lamda0 = eigenValues[bestIndex];

		//determine eigen vector with 2nd largest eigen value
		vector<uint> remainingIndices;
		for (uint i = 0; i < 3; i++)
		{
			if (bestIndex != i)
				remainingIndices.push_back(i);
		}
		if (remainingIndices.size() != 2)
			throw SurrogateException("Should only be 2 indices");

		uint index1 = (eigenValues[remainingIndices[0]] > eigenValues[remainingIndices[1]])
					 ? remainingIndices[0] : remainingIndices[1];
		results.lamda1 = eigenValues[index1];
		results.eigen1 = eigenVectors.col(index1);

		//determine 3rd eigen
		uint index2 = 20;
		for (uint i = 0; i < 3; i++)
		{
			if (i != bestIndex && i != index2)
				index2 = i;
		}
		if (index2 == 20)
			throw SurrogateException("why didn't index2 get set?");
		results.lamda2 = eigenValues[index2];

		//------the offset for the line is the cloud centroid
		//get centroid
		results.centroid = getCentroid(cloud);

		//printf
		//cout << "\nEigenMatrix: \n" << eigenVectors << endl;
		//cout << "\nEigenValues: \n" << eigenValues << endl;
	}

	void PclSurrogateUtils::printPcaResults(PCA_Results &pca)
	{
		cout << "\nEigen 0 = \n" << pca.bestEigen0 << endl;
		cout << "\nEigen 1 = \n" << pca.eigen1 << endl;

		cout << "\nLamda 0 = " << pca.lamda0 << endl;
		cout << "\nLamda 1 = " << pca.lamda1 << endl;

		cout << "\nCentroid = \n" << pca.centroid << endl;
	}


	/**@param points to which we will fit a 3D line using PCA
	 * @param endA end point of the best fit line
	 * @param endB other end point of the best fit line
	 * @return biased estimate of the population variance =
	 *         sum(squared distance of pt to line)/n where n is points.size()*/
	double PclSurrogateUtils::line3DFit(const list<PointXYZ> &points,
										LINE_SEG_3D &lineSeg)
	{
		PCA_Results pca;
		runPCA(points, pca);
		//printPcaResults(pca);

		return line3DFit(pca, lineSeg);
	}


	/**@param points to which we will fit a 3D line using PCA
	 * @param PCA computation the data set
	 * @param endA end point of the best fit line
	 * @param endB other end point of the best fit line
	 * @return biased estimate of the population variance =
	 *         sum(squared distance of pt to line)/n where n is points.size()*/
	double PclSurrogateUtils::line3DFit(const PCA_Results &pca,
										LINE_SEG_3D &lineSeg)

	{
		if (pca.lamda0 == 0)
		{
			cout << "error: got lamda0 = " << pca.lamda0 << " and lamda1 = " << pca.lamda1 << endl;
			throw SurrogateException("Can't run a line fit: PCA failed to return a non-zero eigenvalue");
		}


		//line is defined by: v(t) = centroid + t*bestEigen
		//compute 2 points on the line
		Vector3f line0 = pca.centroid; //t = 0
		Vector3f line1 = pca.centroid + pca.bestEigen0; //t = 1

		//cout << "\n\nline(t = 0) = \n" << line0 << endl;
		//cout << "\n\nline(t = 1) = \n" << line1 << endl;
		//cout << "\n\nBest Eign = \n" << bestEigenVec << endl;

		//compute residuals and compute end points
		//First, remember:  line(t) = line0 + t*(line1 - line0)
		//Next, project each pt onto the line to find t (as above)
		//The min/max t values give the maximum spacing on the line
		//between projected points
		double sumSquares = 0;
		double minT = 0, maxT = 0;
		for (uint i = 0; i < pca.inputCloud->points.size(); i++)
		{
			//residual
			Vector3f nextDataPt = toVec(pca.inputCloud->points[i]);
			double nextRes = LinearAlgebra::pointToLineDist(line0, line1,
															nextDataPt);
			sumSquares += nextRes*nextRes;
			//cout << "\nResidual for pt: \n" << nextDataPt << "\n = " << nextRes << endl;

			//project
			Vector3f projection;
			double t = LinearAlgebra::projectPointToLine(line0, line1, nextDataPt, projection);
			if (i == 0)
			{
				minT = t;
				maxT = t;
			}
			else
			{
				minT = min(minT, t);
				maxT = max(maxT, t);
			}
		}

		if (minT == maxT)
			throw SurrogateException("Shouldn't have same end pts");
		if (minT == 0 || maxT == 0)
			throw SurrogateException("Don't think either end pt should be at the centroid");

		//plug in the t values to get the line end points
		lineSeg.endA = toPt(line0 + minT*(line1 - line0));
		lineSeg.endB = toPt(line0 + maxT*(line1 - line0));

		return sumSquares/pca.inputCloud->points.size();
	}


	/**@param points points in R3 we're trying to fit a circle to
	 * @param circle parameters we are returning
	 * @return biased estimate of the variance =
	 *  sum(squared distance of pt to circle)/n where n = points.size()*/
	double PclSurrogateUtils::arc3DFit(const std::list<pcl::PointXYZ> &points,
									   Circle3D &circle)
	{

		//run pca
		PCA_Results pca;
		runPCA(points, pca);
		//printPcaResults(pca);

		return arc3DFit(pca, circle);
	}

	double PclSurrogateUtils::arc3DFit(const PCA_Results &pca,
									   Circle3D &circle)
	{
		//project all the points onto the plane of the
		//returned eigen vectors
		if (abs(pca.lamda0) == 0 || abs(pca.lamda1) == 0)
		{
			cout << "\n\nlambdas: " << pca.lamda0 << " | " << pca.lamda1 << endl;
			throw SurrogateException("PCA didn't return 2 non-zero lamdas");
		}
		if (pca.eigen1 == Eigen::Vector3f(0,0,0) || pca.bestEigen0 == Eigen::Vector3f(0,0,0))
			throw SurrogateException("Don't have 2 non-zero eigenvectors");

		//---planeVec1 and planeVec2 are determined from PCA
		circle.planeVec0 = toPt(pca.bestEigen0);
		circle.planeVec1 = toPt(pca.eigen1);

		//subtract centroid and project onto eigenvectors from pca
		PointCloud<PointXY>::Ptr projectedCloud = projectAndChangeBasis(subtractCentroid(pca.inputCloud, pca.centroid),
																		pca.bestEigen0, pca.eigen1);
		if (projectedCloud->points.size() != pca.inputCloud->points.size())
			throw new SurrogateException("How do these have diff sizes?");

		//circle fitting using Modified Least Squares from:
		//http://www.cs.bsu.edu/homepages/kerryj/kjones/circles.pdf
		//"A Few Methods for Fitting Circles to Data"
		//by Dale Umbach, Kerry N. Jones
		double sumX = 0; //sum of x_i's
		double sumX2 = 0; //sum(x_i ^2)
		double sumX3 = 0; //sum(x_i ^3)

		double sumY = 0; //sum of y_i's
		double sumY2 = 0; //sum(y_i ^2)
		double sumY3 = 0; //sum(y_i ^3)

		double sumXY = 0; //sum(x_i * y_i)
		double sumX_Y2 = 0; //sum(x_i * y_i^2)
		double sumX2_Y = 0; //sum(x_i^2 * y_i)

		for(uint i = 0; i < projectedCloud->points.size(); i++)
		{
			PointXY next = projectedCloud->points[i];
			double x = next.x;
			double y = next.y;

			sumX  += x;
			sumX2 += x * x;
			sumX3 += x * x * x;

			sumY  += y;
			sumY2 += y * y;
			sumY3 += y * y * y;

			sumXY 	+= x * y;
			sumX_Y2 += x * y * y;
			sumX2_Y += x * x * y;
		}

		uint n = projectedCloud->points.size();


		//these variable names: A,B,C,D,E are from the paper referenced above
		double A = (n*sumX2) - (sumX*sumX);
		double B = (n*sumXY) - (sumX*sumY);
		double C = (n*sumY2) - (sumY*sumY);
		double D = (n*sumX_Y2) - (sumX*sumY2)
				   + (n*sumX3) - (sumX*sumX2);
		D *= 0.5; //moved to 2nd line to make above line cleaner
		double E = (n*sumX2_Y) - (sumY*sumX2)
				   + (n*sumY3) - (sumY*sumY2);
		E *= 0.5;

		//compute circle parameters
		double a = (D*C - B*E) / (A*C - B*B);
		double b = (A*E - B*D) / (A*C - B*B);
		circle.radius = 0;
		for (uint i = 0; i < n; i++)
		{
			PointXY next = projectedCloud->points[i];
                        // mfallon:
                        double dxsq = pow(next.x - a,2);
                        double dysq = pow(next.y - b,2);
			double rad = sqrt(dxsq + dysq);
			circle.radius += rad;
		}

		circle.radius /= n;

		//we now have the best-fit circle radius^2 = (x - center.x)^2 + (y - center.y)^2
		//we need to change our basis back to the standard basis of R3.  The only thing
		//in our output that depends on a basis is the center and the 2 plane vectors.
		//we already have the 2 plane vectors from pca.
		//The vector (a,b) is in the ordered basis {bestEigen0, eigen1}
		//and the real location in R3 is shifted by centroid
		Eigen::Vector3f center3f = a*pca.bestEigen0 + b*pca.eigen1 + pca.centroid;
		circle.center = toPt(center3f);

		//compute the variance
		double variance = 0;
		Eigen::Vector3f planeNormal = pca.bestEigen0.cross(pca.eigen1);
		planeNormal.normalize();
		for (uint i = 0; i < n; i++)
		{
			PointXY next = projectedCloud->points[i];
			double nextDist = circle.radius - sqrt(pow(next.x - a, 2) + pow(next.y - b,2));

			//determine distance to plane.
			//Get a vector from center of the circle (real R3 coordinates) to
			//the unprojected point.
			Eigen::Vector3f vectorCenterR3ToPtR3 = center3f - toVec(pca.inputCloud->points[i]);

			//project vectorCenterR3ToPtR3 onto the plane normal to get the plane distance
			double ptToPlaneDist = planeNormal.dot(vectorCenterR3ToPtR3);

			variance += nextDist*nextDist + ptToPlaneDist*ptToPlaneDist;
		}
		variance /= n;

		//todo: min / max theta
		circle.minTheta = 0;
		circle.maxTheta = 2*3.141592;
		return variance;
	}

	/**@param points to classify into some shape
	 * @param circle circle parameters if classified as circle. otherwise garbage
	 * @param lineSeg line parameters if classified as line. otherwise garbage */
	void PclSurrogateUtils::classifyShape(const std::list<pcl::PointXYZ> points,
									      SHAPE_FIT_RESULTS &fitResults)
	{
		if (points.size() < 4)
			throw SurrogateException("Can't classify shape based on < 4 points");

		//---maximally separate points for linear fit
		//Going to take the largest of 10 points
		//or 20% for fitting
		int numPts = max(10, (int) (points.size()*0.2));
		list<PointXYZ> sparseTrace;
		PclSurrogateUtils::getMaximallySpacedSubset(points, numPts, sparseTrace);

		//get linear fit
		PCA_Results pcaSparse;
		PclSurrogateUtils::runPCA(sparseTrace, pcaSparse);
		if (pcaSparse.lamda0 == 0)
		{
			fitResults.shapeType = UNKNOWN;
			return;
		}

		double linearVariance = PclSurrogateUtils::line3DFit(pcaSparse, fitResults.lineSeg);
		//cout << "\n linear variance = " << linearVariance << endl;

		//---see if we have 2 eigenvectors to support arc fitting
		//if 0 variance in line fit, use the line fit
		PCA_Results pca;
		PclSurrogateUtils::runPCA(points, pca);
		if (linearVariance <= 10e-8 || pca.lamda0 == 0 || pca.lamda1 == 0 ||
			pca.bestEigen0.norm() == 0 || pca.eigen1.norm() == 0)
		{
			fitResults.shapeType = LINE_3D;
			return;
		}

		//--can run a circle fit
		double circleVariance = PclSurrogateUtils::arc3DFit(pca, fitResults.circle);
		//cout << "\n circleVariance" << circleVariance << endl;
		//cout << "\n circle vectors: \nv1 = " << fitResults.circle.planeVec0 <<
		//		"\nv2 = " << fitResults.circle.planeVec1 << endl;
		if (circleVariance > 1 && linearVariance > 1)
		{
			fitResults.shapeType = UNKNOWN; //too much variance in fit?
			return;
		}

		/*from dan: check the ratio of the singular values to
		estimate dimensionality.
		let s1 stand for lambda1 ^ 2
		Check:
		if s0 / (s0 + s1 + s2) > 0.99
		 Then 1D
		else if (s0 + s1) / (s0 + s1 + s2) > 0.99
		Then 2D
		else
		 3D
		double s0 = pca.lamda0*pca.lamda0;
		double s1 = pca.lamda1*pca.lamda1;
		double s2 = pca.lamda2*pca.lamda2;
		double sum = s0 + s1 + s2;
		if (s0/sum > 0.99) //99%
			fitResults.estDimensionality = 1;
		else if ((s0 + s1)/sum > 0.99) //99%
			fitResults.estDimensionality = 2;
		else
			fitResults.estDimensionality = 3;
		cout << "\ns0 = " << s0 << " | s1 = " << s1 << endl;
		cout << "\ns0/sum = " << s0/sum << " | (s1+s2)/sum = " << (s0+s1)/sum << endl;
		*/
		if (linearVariance < 10*circleVariance || sparseTrace.size() < 10)
		{
			fitResults.variance = linearVariance;
			fitResults.shapeType = LINE_3D;
		}
		else
		{
			fitResults.variance = circleVariance;
			fitResults.shapeType = ARC_3D;
		}
	}

	//========non-instantiable
	PclSurrogateUtils::PclSurrogateUtils()
	{
		throw exception();
	}

	PclSurrogateUtils::~PclSurrogateUtils()
	{
		throw exception();
	}


	//=========tests
	void PclSurrogateUtils::runPcaLinearTests()
	{
				//----test for line3D fit with the line y = x + 1 in the plane z = 0
		list<PointXYZ> points;
		points.push_back(PointXYZ(0,1,0));
		points.push_back(PointXYZ(1,2,0));
		points.push_back(PointXYZ(2,3,0));

		LINE_SEG_3D lineSeg;
		double variance = line3DFit(points, lineSeg);

		cout << "\n\n\nFirst End Pt: " << lineSeg.endA << endl;
		cout << "\nSecond End Pt: " << lineSeg.endB << endl;
		cout << "\n variance = " << variance;

		//--------------same thing, different z values
		cout << "\n\n\n\n\n=========test 2\n\n\n\n" << endl;
		points.clear();
		points.push_back(PointXYZ(0,1,7));
		points.push_back(PointXYZ(1,2,7));
		points.push_back(PointXYZ(2,3,7));
		variance = line3DFit(points, lineSeg);
		cout << "\n\n\nFirst End Pt: " << lineSeg.endA << endl;
		cout << "\nSecond End Pt: " << lineSeg.endB << endl;
		cout << "\n variance = " << variance;

		//--------------
		cout << "\n\n\n\n\n=========test 3\n\n\n\n" << endl;
		points.push_back(PointXYZ(-1,0,7)); //on the line
		points.push_back(PointXYZ(3,4,7));  //on the line

		points.push_back(PointXYZ(1 + .707, 2 -.707,   7)); //off the line by 1
		points.push_back(PointXYZ(1 - .707, 2 + .707,  7)); //off the line by 1

		variance = line3DFit(points, lineSeg);
		cout << "\n\n\nFirst End Pt: " << lineSeg.endA << endl;
		cout << "\nSecond End Pt: " << lineSeg.endB << endl;
		cout << "\n variance = " << variance;


		cout << "\n\n\n\n\n=========test 4\n\n\n\n" << endl;
		for (int i = 0; i < 100; i++)
			points.push_back(PointXYZ(0,1,i)); //lots of points on the vertical line x = 0, y = 1

		variance = line3DFit(points, lineSeg);
		cout << "\n\n\nFirst End Pt: " << lineSeg.endA << endl;
		cout << "\nSecond End Pt: " << lineSeg.endB << endl;
		cout << "\n variance = " << variance;


	}

	void PclSurrogateUtils::runLinearAlgebraTests()
	{
		cout << "\n\n\n=========Linear Algebra Test 1===== " << endl;
		list<PointXYZ> points;
		points.push_back(PointXYZ(0,0,1));
		points.push_back(PointXYZ(1,1,2));
		points.push_back(PointXYZ(2,2,3));
		points.push_back(PointXYZ(3,3,4));


		PointCloud<PointXY>::Ptr projected = projectAndChangeBasis(toPcl(points),
																  Eigen::Vector3f(0,1,0),
																  Eigen::Vector3f(1,0,0));
		for(uint i = 0; i < projected->points.size(); i++)
			cout << projected->points[i] << endl;


		cout << "\n\n\n=========Linear Algebra Test 2===== " << endl;
		projected = projectAndChangeBasis(toPcl(points),
									      Eigen::Vector3f(1,1,0)/sqrt(2.0),
									      Eigen::Vector3f(0,0,1));
		for(uint i = 0; i < projected->points.size(); i++)
			cout << projected->points[i] << endl;

	}

	void PclSurrogateUtils::runAndPrintCircleFit(const list<PointXYZ> &points)
	{
		Circle3D circle;
		double variance = arc3DFit(points, circle);

		cout << "\n variance = " << variance << endl;
		cout << "\n v0 = \n" << circle.planeVec0 << endl;
		cout << "\n v1 = \n" << circle.planeVec1 << endl;
		cout << "\n radius = " << circle.radius << endl;
		cout << "\n center = \n" << circle.center << endl;
	}

	void PclSurrogateUtils::runCircleTests()
	{
		cout << "\n\n\n=========Circle Test 1===== " << endl;
		list<PointXYZ> points;
		points.push_back(PointXYZ(2, 1,	7));
		points.push_back(PointXYZ(3, 2,	7));
		points.push_back(PointXYZ(2, 3,	7));
		points.push_back(PointXYZ(1, 2,	7));
		points.push_back(PointXYZ(1, 2,	7));
		points.push_back(PointXYZ(1, 2,	7));
		points.push_back(PointXYZ(1, 2,	7));
		points.push_back(PointXYZ(1, 2,	7));

		runAndPrintCircleFit(points);
	}

	void PclSurrogateUtils::runMaximalSpacingTest()
	{
		//setup lots of points at the origin and at 2
		//and then a few spaced up until 5 on the x-axis
		list<PointXYZ> input;
		for(int i = 0; i < 20; i++)
			input.push_back(PointXYZ(0,0,0));
		for(int i = 0; i < 20; i++)
			input.push_back(PointXYZ(2,0,0));
		for(int i = 0; i <= 5; i++)
			input.push_back(PointXYZ(i,0,0));

		set<int> markedIndices;
		markedIndices.insert(0);
		markedIndices.insert(input.size() -1);
		vector<PointXYZ> output;
		output.push_back(input.front());
		output.push_back(input.back());

		//--run test to get farthest point
		vector<PointXYZ> inputAsVec(input.begin(), input.end());
		cout << "\n\n==== farthest point from set test:\n" << endl;
		int index = getFarthestPointIndex(inputAsVec,markedIndices,output);
		cout << "\n best index = " << index << endl;
		cout << "\ncorresponding point = \n" << inputAsVec[index] << endl;


		//--run test to get full set
		list<PointXYZ> spacedSet;
		getMaximallySpacedSubset(input, 6, spacedSet);

		output = vector<PointXYZ>(spacedSet.begin(), spacedSet.end());
		cout << "\n\n\n=======spaced points: \n" << endl;
		for (uint i = 0; i < output.size(); i++)
		{
			cout << output[i] << endl;
		}
	}

	void PclSurrogateUtils::runTests()
	{
		//runPcaLinearTests();
		//runLinearAlgebraTests();
		//runCircleTests();
		runMaximalSpacingTest();
	}
} //namespace surrogate_gui
