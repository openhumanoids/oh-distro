#include <stdio.h>
#include "math_utils.hpp"
#include <sys/time.h>

namespace opencv_utils { 

    //----------------------------------
    // Arithmetic
    //----------------------------------


    double l2_dist(const cv::Point& p1, const cv::Point& p2) { 
        cv::Point p = p2 - p1;
        return cv::sqrt(p.dot(p));
    }
    double l2_dist(const cv::Point2f& p1, const cv::Point2f& p2) { 
        cv::Point2f p = p2 - p1;
        return cv::sqrt(p.dot(p));
    }
    double l2_dist(const cv::Point3f& p1, const cv::Point3f& p2) { 
        cv::Point3f p = p2 - p1;
        return cv::sqrt(p.dot(p));
    }

    //----------------------------------
    // Histogram-related
    //----------------------------------

    cv::Mat compute_1D_histogram(cv::Mat& img, cv::Mat& mask, const std::string& name, float min, float max, int nbins) { 

        int bins = nbins;

        // let's quantize the range to nbins
        int histSize[] = {bins};

        float lab_ranges[] = { min, max };
        const float* ranges[] = { lab_ranges };

        // we compute the histogram from the ch-thchannels
        int channels[] = {0};

        cv::Mat hist;
        cv::calcHist( &img, 1, channels, mask, // do not use mask
                      hist, 1, histSize, ranges,
                      true, // the histogram is uniform
                      false );
        cv::normalize(hist, hist, 0, 1, CV_MINMAX);

        //----------------------------------
        // Common Params
        //----------------------------------
        // opencv_utils::plot_1D_hist(hist, bins, name);

        return hist;
    }

    // MatND compute_histogram(Mat& img, Mat& mask, const std::string& name) { 

    //     // let's quantize the L,A,B to 30 levels
    //     int bins = BINS;
    //     int histSize[] = {bins, bins, bins};

    //     // L,A,B varies from 0 to 255, see cvtColor
    //     float lab_ranges[] = { 0, 255 };
    //     const float* ranges[] = { lab_ranges, lab_ranges, lab_ranges };

    //     // we compute the histogram from the 0-th, 1-st, and 2-nd channels
    //     int channels[] = {0, 1, 2};

    //     MatND lab_hist;
    //     calcHist( &img, 1, channels, mask, // do not use mask
    //               lab_hist, 3, histSize, ranges,
    //               true, // the histogram is uniform
    //               false );

    //     //----------------------------------
    //     // Common Params
    //     //----------------------------------

    //     int scale = 5;
    //     double maxVal=0;
    //     for( int l = 0; l < bins; l++ )
    //         for( int a = 0; a < bins; a++ )
    //             for (int b = 0; b < bins; b++) 
    //                 if (lab_hist.at<float>(l, a, b) > maxVal)
    //                     maxVal = lab_hist.at<float>(l, a, b);

    //     Mat histImgLAB = Mat::zeros(bins*scale*3, bins*5, CV_8UC3);
    //     //----------------------------------
    //     // L-A
    //     //----------------------------------
    //     Mat histImg(histImgLAB, Rect(0,0,bins*5,bins*scale));

    //     for( int l = 0; l < bins; l++ )
    //         for( int a = 0; a < bins; a++ )
    //             {
    //                 float binVal = 0;
    //                 for (int b = 0; b < bins; b++) 
    //                     binVal += lab_hist.at<float>(l, a, b);

    //                 int intensity = cvRound(binVal*255/maxVal); // divided by bins to normalize
    //                 rectangle( histImg, Point(l*scale, a*scale),
    //                            Point( (l+1)*scale - 1, (a+1)*scale - 1),
    //                            Scalar::all(intensity),
    //                            CV_FILLED );
    //             }

    //     //----------------------------------
    //     // A-B
    //     //----------------------------------
    //     histImg = Mat(histImgLAB, Rect(0,bins*scale,bins*5,bins*scale));

    //     for( int a = 0; a < bins; a++ )
    //         for( int b = 0; b < bins; b++ )
    //             {
    //                 float binVal = 0;
    //                 for (int l = 0; l < bins; l++) 
    //                     binVal += lab_hist.at<float>(l, a, b);

    //                 int intensity = cvRound(binVal*255/maxVal); // divided by bins to normalize
    //                 rectangle( histImg, Point(a*scale, b*scale),
    //                            Point( (a+1)*scale - 1, (b+1)*scale - 1),
    //                            Scalar::all(intensity),
    //                            CV_FILLED );
    //             }
    //     line(histImg, Point(0,1), Point(histImg.cols-1, 1), Scalar(255,0,0), 1);


    //     //----------------------------------
    //     // L-B
    //     //----------------------------------
    //     histImg = Mat(histImgLAB, Rect(0,2*bins*scale,bins*5,bins*scale));
    //     for( int l = 0; l < bins; l++ )
    //         for( int b = 0; b < bins; b++ )
    //             {
    //                 float binVal = 0;
    //                 for (int a = 0; a < bins; a++) 
    //                     binVal += lab_hist.at<float>(l, a, b);

    //                 int intensity = cvRound(binVal*255/maxVal); // divided by bins to normalize
    //                 rectangle( histImg, Point(l*scale, b*scale),
    //                            Point( (l+1)*scale - 1, (b+1)*scale - 1),
    //                            Scalar::all(intensity),
    //                            CV_FILLED );
    //             }
    //     line(histImg, Point(0,1), Point(histImg.cols-1, 1), Scalar(255,0,0), 1);


    //     opencv_utils::imshow( name, histImgLAB );
    //     return lab_hist;
    // }

}


// template<typename _Tp> double l2_dist(const cv::Point_<_Tp>& p1, const cv::Point_<_Tp>& p2) { 
//     cv::Point_<_Tp> p = p2 - p1;
//     return cv::norm(p.dot(p));
// }
// template<typename _Tp> double l2_dist(const cv::Point3_<_Tp>& p1, const cv::Point3_<_Tp>& p2) { 
//     cv::Point3_<_Tp> p = p2 - p1;
//     return cv::norm(p.dot(p));
// }
