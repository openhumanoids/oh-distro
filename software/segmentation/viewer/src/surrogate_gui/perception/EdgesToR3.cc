/*
 * EdgesToR3.cpp
 *
 *  Created on: Mar 21, 2012
 *      Author: mfleder
 */

#include "EdgesToR3.h"
#include "display_objects/Color.h"
#include "SurrogateException.h"

//Include for Sobel
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"

using namespace pcl;
using namespace std;
using namespace boost;

namespace surrogate_gui
{
	uint EdgesToR3::MIN_EDGE_ARC_SIZE = 5;
	int  EdgesToR3::_lowInt = 0;
	int  EdgesToR3::_highInt = 0;

	//----------constructor/destructor
	/**Creates windows for display an rgb + edge image.  Also
	 * creates track bars for adjusting edge detection thresholds.
	 * @param cloudWidth width of clouds we will be passing in
	 * @param cloudHeight height of clouds we will be passing in*/
	EdgesToR3::EdgesToR3(uint cloudWidth, uint cloudHeight)
	: _RGB_WIN_NAME("RGB_IMG"), _EDGE_WIN_NAME("EDGE_IMG"),
	   _high_switch_value(0), _low_switch_value(0),
	  _edge_cloud(new PointCloud<PointXYZRGB>)
	{
		cv::namedWindow(_RGB_WIN_NAME, CV_WINDOW_NORMAL);
		cv::namedWindow(_EDGE_WIN_NAME, CV_WINDOW_NORMAL);

		//------edge detection initialization
		cv::createTrackbar("High", _EDGE_WIN_NAME, &_high_switch_value, 4, switch_callback_h, this);
		cv::createTrackbar("Low", _EDGE_WIN_NAME, &_low_switch_value, 4, switch_callback_l, this);

		cv::setTrackbarPos("High", _EDGE_WIN_NAME, 4); //set to highest setting
		cv::setTrackbarPos("Low",  _EDGE_WIN_NAME, 3);

		_rgb_img.create(cloudHeight, cloudWidth); //, CV_8UC3);
		_edge_img.create(cloudHeight, cloudWidth);  //CV_8UC1);

		//------point cloud init
		_edge_cloud->width = cloudWidth;
		_edge_cloud->height = cloudHeight;
		_edge_cloud->points.resize(cloudWidth * cloudHeight);
	}

	EdgesToR3::~EdgesToR3()
	{
		cv::destroyWindow(_RGB_WIN_NAME);
		cv::destroyWindow(_EDGE_WIN_NAME);
	}

	//-----------------utilities---------------

	 int EdgesToR3::cvToPclIndex(const cv::Point &p, uint cloudWidth)
	 {
		 return cloudWidth*p.y + p.x;
	 }


	/**Structured point cloud w/ width != 1 && height != 1*/
	void EdgesToR3::toCvImage(const PointCloud<PointXYZRGB>::ConstPtr &cloud, cv::Mat3b &img)
	{
		img.create(cloud->height, cloud->width); //, CV_8UC3);

		for (uint i = 0; i < cloud->width; i++)
		{
			for (uint j = 0; j < cloud->height; j++)
			{
				unsigned char r,g,b;
				Color::extractColor(cloud->at(i,j).rgb, r, g, b);

				cv::Point point(i,j);
				img(point)[0] = b;
				img(point)[1] = g;
				img(point)[2] = r;
			}
		}
	}

	/**Runs edge detection on img and produces binary edge image
	 * @param img input rgb image
	 * @param lowThresh The first threshold for the hysteresis procedure
	 * @param highThresh The second threshold for the hysteresis procedure
	 * @param edgeImg output edge image*/
	void EdgesToR3::computeEdgeImg(const cv::Mat3b &rgbImg,
								   const double lowThresh, const double highThresh,
								   cv::Mat1b &edgeImg)
	{
		//http://opencv.willowgarage.com/documentation/cpp/imgproc_feature_detection.html

		//--defensive checks
		if (lowThresh < 0 || highThresh < 0)
			throw SurrogateException("getEdgeImg: threshold < 0");

		if (highThresh < lowThresh)
			throw SurrogateException("high threshold < low threshold");

		//------run

		//get grayscale image
		cv::Mat rgbBlur;
		cv::GaussianBlur(rgbImg, rgbBlur, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );

		cv::Mat1b grayScale;
		cv::cvtColor(rgbBlur, grayScale, CV_RGB2GRAY);

		//run edge detection w/ Canny
		const int N = 7;
		const int aperature_size = N;
		cv::Canny(grayScale, edgeImg, lowThresh*N*N, highThresh*N*N, aperature_size);

		//run edge detection w/ Sobel: Sobel_Demo.cpp
		// Generate grad_x and grad_y
		/*cv::Mat grad_x, grad_y;
		cv::Mat abs_grad_x, abs_grad_y;

		// Gradient X
		int ddepth = CV_16S;
		int delta = 0;
		int scale = 1;
		cv::Sobel(grayScale, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );
		cv::convertScaleAbs( grad_x, abs_grad_x );

		// Gradient Y
		cv::Sobel(grayScale, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
		cv::convertScaleAbs( grad_y, abs_grad_y );

		/// Total Gradient (approximate)
		addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, edgeImg);

		//------------threshold
		for (int i = 0; i < edgeImg.size().width; i++)
		{
			for (int j = 0; j < edgeImg.size().height; j++)
			{
  			    cv::Point point(i,j);
				unsigned char val = edgeImg(point);
				edgeImg(point) = (val > 32) ? 255 : 0;
			}
		}

		cv::erode(edgeImg, edgeImg, cv::Mat());*/
	}

	/**Extracts the 3D contour points from cloud using edgeImg,
	 * but limits the points to only things contained in superSetIndices
	 * @param cloud rgbxyz cloud from which the edgeImg was derived.  origCloud and edgeImg
	 * should have the same dimensions
	 * @param superSetIndices.  indices to which we must restrict any contours. we limit any contour points to indices contained here.
	 * if of size 0, then we won't impose any restrictions.
	 * @param contourIndices output: indices into cloud of the contours.
	 *  These are indices into cloud of the 3D contour points
	 *  @param contours3D actual 3D values corresponding to contoursIndices
	 *  @param edgeImg edge image derived from cloud.  Pass this in to save computation.
	 *  If empty, will compute this from the cloud*/
	void EdgesToR3::computeContours3D(const PointCloud<PointXYZRGB>::ConstPtr &cloud,
									  const shared_ptr<set<int> > superSetIndices,
									  vector<vector<cv::Point> > &contoursIndices,
									  vector<std::vector<PointXYZRGB> > &contours3D,
									  cv::Mat1b &edgeImgInput)
	{
		//---see if we need to compute an edge image
		if (edgeImgInput.empty())
		{
			//extract rgb image
			cv::Mat3b rgb_img;
			rgb_img.create(cloud->height, cloud->width);
			toCvImage(cloud, rgb_img);

			//compute edge image
			edgeImgInput.create(cloud->height, cloud->width);
			computeEdgeImg(rgb_img, _lowInt, _highInt, edgeImgInput);
		}

		//-----find contours in edge image
	    vector<vector<cv::Point> > allContours;
	    cv::findContours(edgeImgInput, allContours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	    //setup output
		contoursIndices.clear();
		contours3D.clear();
	    contours3D.reserve(allContours.size());
	    contoursIndices.reserve(allContours.size());

	    //go thru candidate contours
	    //remove any points from contours that aren't in superSetIndices
	    for(uint n = 0; n < allContours.size(); n++)
	    {
	    	vector<cv::Point> &next = allContours[n]; //next from edge detection

	    	vector<cv::Point> nextRestricted;  //restricted by indices
	    	nextRestricted.reserve(next.size());
	    	vector<PointXYZRGB> next3D; //3d version of nextRestricted
	    	next3D.reserve(next.size());

	    	//for the next contour, put any points
	    	//that lie in the superSetIndices into nextRestricted
	    	for (uint i = 0; i < next.size(); i++)
	    	{
	    		//get point
	    		cv::Point p = next[i];

	    		//convert to index into cloud
	    		int pclIndex = cvToPclIndex(p, cloud->width);

	    		//if restriction set is null or empty, then no restriction
	    		//otherwise, need index
	    		if (superSetIndices != shared_ptr<set<int> >() && // if null, want to add
	    			superSetIndices->size() != 0 && // if empty, want to add
	    			superSetIndices->find(pclIndex) == superSetIndices->end()) //dont have index?
	    		{
	    			continue; //inner loop
	    		}

	    		const PointXYZRGB &p3D = cloud->at(p.x, p.y);
	    		if (!pcl_isfinite(p3D.x) || !pcl_isfinite(p3D.y) || !pcl_isfinite(p3D.z))
	    			continue; //don't have depth, don't add

	    		nextRestricted.push_back(p);
    			next3D.push_back(p3D);
	    	}

	    	//size restrictions
	    	if (nextRestricted.size() < MIN_EDGE_ARC_SIZE)
	    		continue;

	    	//todo: more restrictions based on depth using cloud

	    	//finally add
	    	contoursIndices.push_back(nextRestricted);
	    	contours3D.push_back(next3D);
	    }

	    if (contours3D.size() == 0)
	    	cout << "\n Couldn't find any good contours" << endl;
	}


	//-----------edge to cloud
	//Converts the edge image into a point cloud using origCloud to get the depth information
	/**@param origCloud rgbxyz cloud from which the edgeImg was derived.  origCloud and edgeImg
	 * should have the same dimensions
	 * @param edgeImg edge image derived from origCloud.  not const b/c for some reason
	 * findContours won't accept a const Mat1b
	 * @param edgeCloud output : edges projected in R3*/
	void EdgesToR3::edgeToCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &origCloud,
								cv::Mat1b &edgeImg,
								pcl::PointCloud<pcl::PointXYZRGB>::Ptr &edgeCloud)
	{
		//---defensive checks
		if (origCloud->size() == 0)
			throw SurrogateException("orig cloud has size 0");

		uint width = edgeImg.size().width;
		uint height = edgeImg.size().height;
		if (origCloud->width  != width || origCloud->height != height ||
		    edgeCloud->width != width || edgeCloud->height != height)
			throw SurrogateException("Dimensions of clouds and edge img don't agree");

		//---------- set points other than the contours to black

		for (int i = 0; i < edgeImg.size().width; i++)
		{
			for (int j = 0; j < edgeImg.size().height; j++)
			{

			    PointXYZRGB p3d = origCloud->at(i, j); //point in orig cloud
			    PointXYZRGB e3;

			    //set xyz of e3
			    e3.x = p3d.x;
				e3.y = p3d.y;
				e3.z = p3d.z;


			    //set color of e3
			    cv::Point point(i,j);
			    unsigned char binary = edgeImg(point);
			    if (binary == 0)
			    	e3.rgb = Color::toRgbPclFloat(binary, binary, binary); //r, g, b);
			    //else //commented out b/c not using all non-zero pixels
			    // 	e3.rgb = Color::toRgbPclFloat(255,255,255);

				//put e3 in edge_cloud
				edgeCloud->points[origCloud->width*j + i] = e3;
			}
		}

		//-----highlight the contours
		vector<vector<cv::Point> > contours;
		vector<vector<PointXYZRGB> > contours3D;
		EdgesToR3::computeContours3D(origCloud, shared_ptr<set<int> >(), contours, contours3D, edgeImg);
		//computeContours3D(origCloud, shared_ptr<set<int> >(), contours, contours3D, edgeImg);

	    int colorIndex = 0;
	    for(uint i = 0; i < contours.size(); i++)
	    {
	    	vector<cv::Point> nextContour = contours[i];

	    	float nextColor;
			if (colorIndex > 5)
				colorIndex = 0;
			if (colorIndex == 0)
				nextColor = Color::toRgbPclFloat(255, 0, 0); //r, g, b);
			else if (colorIndex == 1)
				nextColor = Color::toRgbPclFloat(0, 255, 0); //r, g, b);
			else if (colorIndex == 2)
				nextColor = Color::toRgbPclFloat(0, 0, 255); //r, g, b);
			else if (colorIndex == 3)
				nextColor = Color::toRgbPclFloat(255, 0, 255); //r, g, b);
			else if (colorIndex == 4)
				nextColor = Color::toRgbPclFloat(255, 255, 0); //r, g, b);
			else
				nextColor = Color::toRgbPclFloat(0, 255, 255);
			colorIndex++;

	    	for (uint j = 0; j < nextContour.size(); j++)
	    	{
	    		cv::Point next2DContourPt = nextContour[j];

	    		//get point in orig cloud
	    		PointXYZRGB p3d = origCloud->at(next2DContourPt.x, next2DContourPt.y);
			    PointXYZRGB e3;
			    //set xyz of e3
			    e3.x = p3d.x;
				e3.y = p3d.y;
				e3.z = p3d.z;

				e3.rgb = nextColor; //r, g, b);

				//put e3 in edge_cloud
				edgeCloud->points[origCloud->width*next2DContourPt.y + next2DContourPt.x] = e3;
	    	}
	    }
	}



	//-----------run

	/**displays 2d rgb and edge images extracted and computed from the
	 * input
	 * @param inputRgbCloud cloud from which we are going to derive and compute
	 * @param outputEdgeCloud edge image projected into R3 (output)
	 * and updated displays*/
	PointCloud<PointXYZRGB>::ConstPtr EdgesToR3::displayRgbToEdgeCloud(const PointCloud<PointXYZRGB>::ConstPtr &inputRgbCloud)
	{
		if (inputRgbCloud->width != _edge_cloud->width ||
			inputRgbCloud->height != _edge_cloud->height)
			throw SurrogateException("Passing in cloud of different dimensions than specified in constructor");

		//--cloud to rgb image
		toCvImage(inputRgbCloud, _rgb_img);
		cv::imshow(_RGB_WIN_NAME, _rgb_img);

		//--rgb to edge image
		computeEdgeImg(_rgb_img, _lowInt, _highInt, _edge_img);
		cv::imshow(_EDGE_WIN_NAME, _edge_img);

		//---edge image to cloud
		EdgesToR3::edgeToCloud(inputRgbCloud, _edge_img, _edge_cloud);
		return _edge_cloud;
	}

	//--------------static callback
	void EdgesToR3::switch_callback_h(int position, void *tmp)
	{
		EdgesToR3 *obj = (EdgesToR3 *) tmp;
		int desiredValue = 200*position;//200, 400, 600, 800
		if (desiredValue < EdgesToR3::_lowInt) //don't let high threshold go below low threshold
			cv::setTrackbarPos("High", obj->_EDGE_WIN_NAME, 4); //set to highest setting
		else
			EdgesToR3::_highInt = desiredValue;
	}

	void EdgesToR3::switch_callback_l(int position, void *tmp)
	{
		EdgesToR3 *obj = (EdgesToR3 *) tmp;
		int desiredValue = 200*position;  //0, 200, 400, 600;
		if (desiredValue > EdgesToR3::_highInt)
			cv::setTrackbarPos("Low", obj->_EDGE_WIN_NAME, 0); //set to lowest setting
		else
			EdgesToR3::_lowInt = desiredValue;
	}
}
