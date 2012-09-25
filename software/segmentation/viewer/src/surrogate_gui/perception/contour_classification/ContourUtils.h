/*

 * ContourUtils.h
 *
 *  Created on: Jun 11, 2012
 *      Author: mfleder
 */

#ifndef CONTOURUTILS_H_
#define CONTOURUTILS_H_

#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include "opencv-2.3.1/opencv2/opencv.hpp"
//#include "opencv-2.3.1/opencv2/highgui/highgui.hpp"

#include "display_objects/DisplayInfo.h"
#include "EdgesToR3.h"

#include <set>

namespace surrogate_gui
{

	/**Class for fitting parametric models using 2D edges projected into 3D
	 * using EdgesToR3*/
	class ContourUtils
	{
		//-----------constructor: non-instantiable
		private:
			ContourUtils();
			~ContourUtils();

		//----------static methods
		public:
			static void fitModel(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
								 const boost::shared_ptr<std::set<int> > segmentGuessIndices,
						 		 const AffordanceModel &modelType,
						 		 ModelFit *result);
	};
} //namespace surrogate_gui

#endif /* CONTOURUTILS_H_ */
