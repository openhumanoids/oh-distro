/*
 * OpenCvSrgtUtils.h
 *
 *  Created on: Mar 21, 2012
 *      Author: mfleder
 */

#ifndef OPENCVSRGTUTILS_H_
#define OPENCVSRGTUTILS_H_

//-opencv
#include "opencv-2.3.1/opencv2/opencv.hpp"
#include "opencv-2.3.1/opencv2/highgui/highgui.hpp"

//--pcl
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>

#include <boost/shared_ptr.hpp>

#include <vector>
#include <set>

namespace surrogate_gui
{
	/**Class for EdgeDetection and projecting those edges into R3*/
	class EdgesToR3
	{
		private:
			//---------fields
			cv::Mat3b _rgb_img;
			cv::Mat1b _edge_img;

			const std::string _RGB_WIN_NAME;
			const std::string _EDGE_WIN_NAME;

			int _high_switch_value;
			int _low_switch_value;
			static int _highInt;
			static int _lowInt;

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr _edge_cloud;

			static uint MIN_EDGE_ARC_SIZE;

		//--------constructor/destructor
		public:
			EdgesToR3(uint cloudWidth, uint cloudHeight);
			virtual ~EdgesToR3();

		//---methods
		public:

			//--
			static int cvToPclIndex(const cv::Point &p, uint cloudWidth);

			static void toCvImage(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
								 cv::Mat3b &img);

			static void computeEdgeImg(const cv::Mat3b &rgbImg,
									   const double lowThresh, const double highThresh,
									   cv::Mat1b &edgeImg);

			static void computeContours3D(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
										  const boost::shared_ptr<std::set<int> > contourRestrictedIndices,
										  std::vector<std::vector<cv::Point> > &contoursIndices,
										  std::vector<std::vector<pcl::PointXYZRGB> > &contours3D,
										  cv::Mat1b &edgeImgInput);

			static void edgeToCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &origCloud,
									cv::Mat1b &edgeImg,
									pcl::PointCloud<pcl::PointXYZRGB>::Ptr &edgeCloud);

			//--mutators
			pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr
					  displayRgbToEdgeCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &inputRgbCloud);


			//observers
			void computeContours3D(const boost::shared_ptr<std::set<int> > contourRestrictedIndices,
									std::vector<std::vector<cv::Point> > &contourIndices,
									std::vector<std::vector<pcl::PointXYZRGB> > &contours3D); //todo: make const method


		//--edge detection callbacks
		private:
			static void switch_callback_h(int position, void *obj);
			static void switch_callback_l(int position, void *obj);
	};

}
#endif /* OPENCVSRGTUTILS_H_ */
