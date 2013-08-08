#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;

int main( int argc, char** argv ){
  /// Load an image
  char* window_name = "Morphology Transformations Demo";
  Mat dst, src_gray;
  Mat src = imread( argv[1] );

  if( !src.data )
  { return -1; }
  
  
    cvtColor( src, src_gray, CV_RGB2GRAY );
  imshow( "Input", src );
    imshow( "Input Gray", src_gray );


 /// Create window
 namedWindow( window_name, CV_WINDOW_AUTOSIZE );

  int morph_elem = 0;
  int morph_size = 1;
  int morph_operator = 2;
  // Since MORPH_X : 2,3,4,5 and 6
  int operation = morph_operator + 2;

  std::cout << morph_elem << " " << morph_size << " " << operation << "\n";
  Mat element = getStructuringElement( morph_elem, 
                   Size( 2*morph_size + 1, 2*morph_size+1 ),
                   Point( morph_size, morph_size ) );

  /// Apply the specified morphology operation
  morphologyEx( src, dst, operation, element );
  imshow( window_name, dst );
  
//  threshold( src_gray, dst2, threshold_value, max_BINARY_value,threshold_type );


  /* 0: Binary
     1: Binary Inverted
     2: Threshold Truncated
     3: Threshold to Zero
     4: Threshold to Zero Inverted
   */
  Mat morph_gray, dst2;
  cvtColor( dst, morph_gray, CV_RGB2GRAY );


  bool color_version =false;
  if (color_version){ // rgb

    int threshold_value =10;
    int const max_BINARY_value = 255;
    int threshold_type = 0;

    threshold( morph_gray, dst2, threshold_value, max_BINARY_value,threshold_type );
    imshow( "Threshold", dst2 );  

    cvtColor(dst2, dst2, CV_GRAY2BGR);
    cv::bitwise_or(src, dst2, dst2);


  }else{
    int threshold_value =10;
    int const max_BINARY_value = 255;
    int threshold_type = 1;

    threshold( morph_gray, dst2, threshold_value, max_BINARY_value,threshold_type );
    imshow( "Threshold", dst2 );  

    cv::bitwise_and(src_gray, dst2, dst2);


  }
  
  
  imshow( "Applied", dst2 );  
  
  imwrite("test_out.png",dst2);
  
  waitKey(0);
  return 0;
}


