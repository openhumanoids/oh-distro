/********************************************************************************
*
*
*  This program is demonstration for ellipse fitting. Program finds
*  contours and approximate it by ellipses.
*
*  Trackbar specify threshold parametr.
*
*  White lines is contours. Red lines is fitting ellipses.
*
*
*  Autor:  Denis Burenkov.
*
*
*
********************************************************************************/
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
using namespace cv;
using namespace std;

// static void help()
// {
//     cout <<
//             "\nThis program is demonstration for ellipse fitting. The program finds\n"
//             "contours and approximate it by ellipses.\n"
//             "Call:\n"
//             "./fitellipse [image_name -- Default stuff.jpg]\n" << endl;
// }

int sliderPos = 70;

Mat image;

void processImage(int, void*);

int main( int argc, char** argv )
{
    const char* filename = argc == 2 ? argv[1] : (char*)"stuff.jpg";
    image = imread(filename, 0);
    if( image.empty() )
    {
        cout << "Couldn't open image " << filename << "\nUsage: fitellipse <image_name>\n";
        return 0;
    }

    imshow("source", image);
    namedWindow("result", 1);

    // Create toolbars. HighGUI use.
    createTrackbar( "threshold", "result", &sliderPos, 255, processImage );
    processImage(0, 0);

    // Wait for a key stroke; the same function arranges events processing
    waitKey();
    return 0;
}

// Define trackbar callback functon. This function find contours,
// draw it and approximate it by ellipses.
void processImage(int /*h*/, void*)
{
    vector<vector<Point> > contours;
    Mat bimage = image >= sliderPos;

    findContours(bimage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    Mat cimage = Mat::zeros(bimage.size(), CV_8UC3);



   Mat M(20,1, CV_8U, Scalar(255));
   // Mat M = (Mat_<int>(4,1) << 100, 10, 200, 20,  400, 30, 500, 2);


        std::cout << M.rows << "\n";
        std::cout << M.cols << "\n";
    cout << "M = " << endl << " " << M << endl << endl;

   vector<Point2f> b;
   b.push_back( Point2f(100,20) );
   b.push_back( Point2f(200,20) );
   b.push_back( Point2f(300,40) );
   b.push_back( Point2f(200,100) );
   b.push_back( Point2f(100,80) );

   b.push_back( Point2f(105,25) );
   b.push_back( Point2f(210,110) );
   b.push_back( Point2f(110,88) );
   b.push_back( Point2f(112,88) );
   b.push_back( Point2f(114,88) );
   b.push_back( Point2f(116,88) );
   b.push_back( Point2f(118,88) );

   cout << b[0] << endl;

    for(size_t i = 1; i < 2; i++) // contours.size()
    {
        size_t count = contours[i].size();
        if( count < 6 )
            continue;

        Mat pointsf;
        Mat(contours[i]).convertTo(pointsf, CV_32F);
        RotatedRect box = fitEllipse(b);

        std::cout << pointsf.rows << "\n";
        std::cout << pointsf.cols << "\n";
cout << pointsf << endl;

        if( MAX(box.size.width, box.size.height) > MIN(box.size.width, box.size.height)*30 )
            continue;
        drawContours(cimage, contours, (int)i, Scalar::all(255), 1, 8);

        ellipse(cimage, box, Scalar(0,0,255), 1, CV_AA);
        ellipse(cimage, box.center, box.size*0.5f, box.angle, 0, 360, Scalar(0,255,255), 1, CV_AA);
        Point2f vtx[4];
        box.points(vtx);
        for( int j = 0; j < 4; j++ )
            line(cimage, vtx[j], vtx[(j+1)%4], Scalar(0,255,0), 1, CV_AA);
    }

    for (int i=0; i < b.size(); i++){
    circle ( cimage,  b[i], 4, Scalar( 0, 0, 255 ), -1, 8 );
    }


    imshow("result", cimage);
}

