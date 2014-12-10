/*
 * PclSurrogateUtils.h
 *
 *  Created on: Jan 19, 2012
 *      Author: mfleder
 */

#ifndef PCLSURROGATEUTILS_H_
#define PCLSURROGATEUTILS_H_

#include "DisplayInfo.h"
#include <lcmtypes/lkr_color_point_cloud_t.h>

#include <set>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>

namespace surrogate_gui
{
	typedef struct
	{
		Eigen::Vector3f bestEigen0;
		Eigen::Vector3f eigen1;
		double lamda0;
		double lamda1;
		double lamda2;
		Eigen::Vector3f centroid;

		pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud;
	} PCA_Results;

	/**non-instantiable class of pcl/surrogate utilities*/
	class PclSurrogateUtils
	{
		public:
			enum SHAPE
			{
				LINE_3D, //in R3
				ARC_3D, //arc of a circle or whole circle
				UNKNOWN
			};

			typedef struct
			{
				SHAPE shapeType;
				Circle3D circle;
				LINE_SEG_3D lineSeg;
				double variance; //variance in fit
				//uint estDimensionality; //estimated dimensionality
			} SHAPE_FIT_RESULTS;

		//non-instantiable
		private:
			PclSurrogateUtils();
			virtual ~PclSurrogateUtils();

		//conversions
		public:
			static pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampleWithVoxel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);
			static pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsample(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input_cloud, uint skipValue);
			static pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractIndexedPoints(const SetIntPtr indices, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr originalCloud);
			static pcl::PointCloud<pcl::PointXYZRGB>::Ptr toPcl(const lkr_color_point_cloud_t *cloud);
			static pcl::PointCloud<pcl::PointXYZ>::Ptr toPcl(const std::list<pcl::PointXYZ> &input);
			static pcl::PointCloud<pcl::PointXYZRGB>::Ptr toPcl(const ObjectPointsPtr objPts, const lkr_color_point_cloud_t *originalCloud);
			static pcl::PointCloud<pcl::PointXYZRGB>::Ptr toPcl(const std::vector<int> &indices, const lkr_color_point_cloud_t *originalCloud);
			static pcl::PointIndices::Ptr toPclIndices(const SetIntPtr indices);
			static pcl::PointIndices::Ptr copyIndices(const pcl::PointIndices::Ptr indices);
			static void toSet(const pcl::PointIndices::Ptr indices,
					  std::set<int> &copy);
			  
			static void convert(const std::vector<pcl::PointXYZRGB> &input,
							    std::list<pcl::PointXYZ> &output);

			static void convertToRgb(pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
						 pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output);

			static Eigen::Vector3f toVec(const pcl::PointXYZ &p);
			static pcl::PointXYZ toPt(const Eigen::Vector3f &p);
			static pcl::PointXY toPt2D(const Eigen::Vector2f &p);

			//--------------linear algebra operations on point clouds
			static Eigen::Vector3f getCentroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

			static pcl::PointCloud<pcl::PointXYZ>::Ptr  subtractCentroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
																		 const Eigen::Vector3f &centroid);

			static pcl::PointCloud<pcl::PointXY>::Ptr projectAndChangeBasis(const pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud,
																		    const Eigen::Vector3f &v0,
																		    const Eigen::Vector3f &v1);

			static int getFarthestPointIndex(const std::vector<pcl::PointXYZ> &availablePts,
											 const std::set<int> &dontConsiderIndices,
											 const std::vector<pcl::PointXYZ> &ptsToMeasure);

			static void getMaximallySpacedSubset(const std::list<pcl::PointXYZ> &points, uint numPoints,
												 std::list<pcl::PointXYZ> &spacedSubset);

			//------printing
			static void print(pcl::PointCloud<pcl::PointXYZ> &cloud);

			//--------model fitting
			static void runPCA(const std::list<pcl::PointXYZ> &points,
							   PCA_Results &pcaResults);

			static void printPcaResults(PCA_Results &pca);


			static double line3DFit(const PCA_Results &pca,
									LINE_SEG_3D &lineSeg);

			static double line3DFit(const std::list<pcl::PointXYZ> &points,
									LINE_SEG_3D &lineSeg);

			static double arc3DFit(const std::list<pcl::PointXYZ> &points,
								  Circle3D &circle);

			static double arc3DFit(const PCA_Results &pcaComputation,
								   Circle3D &circle);

			static void classifyShape(const std::list<pcl::PointXYZ> points,
									  SHAPE_FIT_RESULTS &fitResults);

			//==================testing
			static void runPcaLinearTests();
			static void runLinearAlgebraTests();
			static void runAndPrintCircleFit(const std::list<pcl::PointXYZ> &points);
			static void runCircleTests();
			static void runMaximalSpacingTest();
			static void runTests();
	};


} //namespace surrogate_gui

#endif /* PCLSURROGATEUTILS_H_ */
