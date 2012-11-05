#include <stdio.h>
#include "imgproc_utils.hpp"
#include <sys/time.h>

namespace opencv_utils { 

    void simple_sobel(cv::Mat& dst, cv::Mat& img) { 

        int scale = 1;
        int delta = 0;
        int ddepth = CV_16S;

        /// Generate grad_x and grad_y
        cv::Mat grad_x, grad_y;
        cv::Mat abs_grad_x, abs_grad_y;
 
        /// Gradient X
        //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
        cv::Sobel( img, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );   
        cv::convertScaleAbs( grad_x, abs_grad_x );

        /// Gradient Y  
        //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
        cv::Sobel( img, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );   
        cv::convertScaleAbs( grad_y, abs_grad_y );

        /// Total Gradient (approximate)
        cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, dst );

        return;

    }

}
