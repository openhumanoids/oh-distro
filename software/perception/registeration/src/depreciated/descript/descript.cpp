#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

// For SURF
#include "opencv2/nonfree/features2d.hpp"

// For common feature2D
#include "opencv2/nonfree/features2d.hpp"




// main
int main( int argc, char** argv ) {
    cv::Mat img_1 = cv::imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE );
    cv::Mat img_2 = cv::imread( argv[2], CV_LOAD_IMAGE_GRAYSCALE );

     if( !img_1.data || !img_2.data )
         { return -1; }

     //-- Step 1: Detect the keypoints and generate their descriptors using SURF
     int minHessian = 3550;//400;
     cv::SURF surf( minHessian );

     std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
     cv::Mat descriptors_1, descriptors_2;

     surf( img_1, cv::Mat(), keypoints_1, descriptors_1, false );
     surf( img_2, cv::Mat(), keypoints_2, descriptors_2, false );


     //-- Step 3: Matching descriptor vectors with a brute force matcher
     cv::BFMatcher matcher( cv::NORM_L2, false );
     std::vector< cv::DMatch > matches;
     matcher.match( descriptors_1, descriptors_2, matches );

std::cout << matches.size() << " matches found\n";

     //-- Draw matches
     cv::Mat img_matches;
     cv::drawMatches( img_1, keypoints_1, img_2, keypoints_2, matches, img_matches );

     //-- Show detected matches
     imshow("Matches", img_matches );

     cv::waitKey(0);

    return 0;
    }
