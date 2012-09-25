/*
 * ContourUtils.cc
 *
 *  Created on: Jun 11, 2012
 *      Author: mfleder
 */

#include "ContourUtils.h"
#include "SurrogateException.h"
#include "PclSurrogateUtils.h"

#include <pcl/common/distances.h>

#include <vector>
#include <list>

using namespace std;
using namespace boost;
using namespace pcl;

namespace surrogate_gui
{

	/**@param cloud containing the model we're looking to fit
	 * @param segmentGuess indices into cloud containing a guess of the model indices
	 * @return model we've fit*/
	void ContourUtils::fitModel(const PointCloud<PointXYZRGB>::ConstPtr cloud,
								const shared_ptr<set<int> > segmentGuessIndices,
								const AffordanceModel &modelType,
								ModelFit *model)
	{
		//-------defensive checks
		if (segmentGuessIndices->size() == 0)
			throw SurrogateException("0 segment guess indices passed to ContourUtils::classify");
		if (segmentGuessIndices->size() < 3)
		{
			cout << "\n\n===Warning: < 3 segment guess indices passed to ContourUtils::classify\n\n" << endl;
			model->drawCircles = false;
			model->drawLines = false;
			return;
		}

		//-------setup output
		model->drawCircles = true;
		model->drawLines = false;
		model->lines.clear();
		model->circles.clear();
		model->haveModel = false;

		//---extract 3D Contours indices and corresponding points
		vector<vector<PointXYZRGB> > contours3D;
		vector<vector<cv::Point> > contoursIndices;
		cv::Mat1b edgeImg;
		EdgesToR3::computeContours3D(cloud, segmentGuessIndices,
									contoursIndices, contours3D, edgeImg);

		//compute max cloud diameter so we don't have to re-compute in the loop each time
		PointXYZRGB minPt, maxPt; //points on the maximum diameter of the segment guess indices
		double cloudDiameter = pcl::getMaxSegment (*cloud,
												   vector<int>(segmentGuessIndices->begin(), segmentGuessIndices->end()),
												   minPt, maxPt);
		if (cloudDiameter < 0)
			throw SurrogateException("Why Couldn't We Compute The Cloud Diameter?");

		//examine each contour
		for (uint i  = 0; i < contours3D.size(); i++)
		{
			//get next contour 3D XYZRGB points and convert to PointXYZ
			list<PointXYZ> nextContour;
			PclSurrogateUtils::convert(contours3D[i], nextContour);

			//see if next countour is classified as a line or circle
			PclSurrogateUtils::SHAPE_FIT_RESULTS fitResults;
			PclSurrogateUtils::classifyShape(nextContour, fitResults);

			//---if the classification matches up w/ modelType, then we're going to incorporate
			//it into our model fitting
			if (modelType == LEVER)
				throw SurrogateException("fitModel: Lever not implemented");
			else if (modelType != WHEEL)
				throw SurrogateException("Unknown model to fit");

			//---for now, just output the circles
			//wheel?
			if (fitResults.shapeType != PclSurrogateUtils::ARC_3D)
				continue;


			//see if the circle diameter ~ diameter of the segmentGuessIndices
			double circleDiameter = 2*fitResults.circle.radius;
			if (abs(cloudDiameter - circleDiameter)/cloudDiameter > 0.4)
			{
				cout << "\n bad circle diameter = " << 2*fitResults.circle.radius << endl;
				cout << "\n cloud diameter = " << cloudDiameter << endl;
				cout << "\n difference = " << abs(cloudDiameter - circleDiameter)/cloudDiameter << endl;
				continue; //circle diameter is 40% or more beyond the cloud diameter
			}

			model->circles.push_back(fitResults.circle);
			model->haveModel = true;
			model->rotationAxisGuess = fitResults.circle;
			//todo: compute the normal of the segment indices

			//todo: if no good circles, try resorting to ransac

			//todo: draw the circle determined by:
			// (plane normal, max diameter of the cloud, centroid of the cluster)
			//This should only work if we have data uniformly accross the whole wheel.
			//If, for example, we only observe 1/2 the wheel, the centroid will be in the wrong place

		}
	}
} //namespace surrogate_gui
