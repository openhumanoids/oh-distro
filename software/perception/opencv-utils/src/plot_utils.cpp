#include <stdio.h>
#include "plot_utils.hpp"
#include "imshow_utils.hpp"
#include <sys/time.h>

namespace opencv_utils { 

    cv::Scalar heatmap_bgr( float val ) { 
        float b = val > 0.333f ? 1 : val;
        val -= 0.333f;
        float g = val > 0.333f ? 1 : val;
        val -= 0.333f;
        float r = val > 0.333f ? 1 : val;
        return cv::Scalar(b * 255, g * 255, r * 255);
    }

    cv::Vec3b heatmap_bgrvec( float val ) { 
        float b = val > 0.333f ? 1 : val;
        val -= 0.333f;
        float g = val > 0.333f ? 1 : val;
        val -= 0.333f;
        float r = val > 0.333f ? 1 : val;
        return cv::Vec3b(b * 255, g * 255, r * 255);
    }

    void fillColors( std::vector<cv::Scalar>& colors ) {
        cv::RNG rng = cv::theRNG();

        for( size_t ci = 0; ci < colors.size(); ci++ )
            colors[ci] = cv::Scalar( rng(256), rng(256), rng(256) );
    }

    void plot_1D_hist(cv::Mat& hist, int bins, const std::string& name) { 

        int scale = 10;
        double maxVal=0;
        cv::minMaxLoc(hist, 0, &maxVal, 0, 0);

        cv::Mat histImgLAB = cv::Mat::zeros(100, bins*scale, CV_8UC3);
        for( int i = 0; i < bins; i++ ) {
            int val = cv::saturate_cast<int>(hist.at<float>(i)*histImgLAB.rows/maxVal);
            cv::rectangle( histImgLAB, cv::Point(i*scale,histImgLAB.rows),
                           cv::Point((i+1)*scale,histImgLAB.rows - val),
                           cv::Scalar::all(255),-1, 8 );
        }
        opencv_utils::imshow( name, histImgLAB );
    }
}

