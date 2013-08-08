// Process to Determine morphological gradient on an 
// unsigned short (16bit) DEPTH IMAGE
// some of the operations in opencv CV dont support 16bit - hence the loop
//
// usage:
// test-morphology-gradient test.png   <--- 16 bit depth image
// mfallon aug 2013

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;


void removeEdges(cv::Mat &src) {
  // 1 Convert to 16S as morphologyEx doesnt support 16U
  Mat src_16s;  
  src.convertTo(src_16s, CV_16S);

  // 2 Determain Morpological Gradient:
  int morph_elem = 0;
  int morph_size = 1;
  int morph_operator = 4;
  Mat element = getStructuringElement( morph_elem, 
            Size( 2*morph_size + 1, 2*morph_size+1 ),
            Point( morph_size, morph_size ) );
  Mat dst;
  morphologyEx( src_16s, dst, morph_operator, element );
  imwrite("test_morph_gray.png",dst);

  // 3 Apply a binary threshold on the gradient and remove high gradient elements
  for(int i=0; i<dst.rows; i++)
    for(int j=0; j<dst.cols; j++) 
      if (  dst.at<short unsigned int>(i,j) > 254)
        src.at<short unsigned int>(i,j)  = 0;
}


int main( int argc, char** argv ){
  // type16 = CV_8UC3 = 3x 8 bit colour
  // type 0 = CV_8U =  8 bit gray
  // type 2 = CV_16U = 16 bit grey scale depth map


  Mat src  = imread( argv[1], CV_LOAD_IMAGE_UNCHANGED );

  removeEdges(src);

  imwrite("test_out.png",src);

  return 0;
}


