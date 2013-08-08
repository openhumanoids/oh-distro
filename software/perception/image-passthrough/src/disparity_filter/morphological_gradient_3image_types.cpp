// works for 3 image format: rgb, gray, gray16
//test-morphology-gradient example_range_image.png 0
//test-morphology-gradient example_range_image_gray.png 1
//test-morphology-gradient test.png  2


#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;


// take number image type number (from cv::Mat.type()), get OpenCV's enum string.
string getImgType(int imgTypeInt)
{
    int numImgTypes = 35; // 7 base types, with five channel options each (none or C1, ..., C4)

    int enum_ints[] =       {CV_8U,  CV_8UC1,  CV_8UC2,  CV_8UC3,  CV_8UC4,
                             CV_8S,  CV_8SC1,  CV_8SC2,  CV_8SC3,  CV_8SC4,
                             CV_16U, CV_16UC1, CV_16UC2, CV_16UC3, CV_16UC4,
                             CV_16S, CV_16SC1, CV_16SC2, CV_16SC3, CV_16SC4,
                             CV_32S, CV_32SC1, CV_32SC2, CV_32SC3, CV_32SC4,
                             CV_32F, CV_32FC1, CV_32FC2, CV_32FC3, CV_32FC4,
                             CV_64F, CV_64FC1, CV_64FC2, CV_64FC3, CV_64FC4};

    string enum_strings[] = {"CV_8U",  "CV_8UC1",  "CV_8UC2",  "CV_8UC3",  "CV_8UC4",
                             "CV_8S",  "CV_8SC1",  "CV_8SC2",  "CV_8SC3",  "CV_8SC4",
                             "CV_16U", "CV_16UC1", "CV_16UC2", "CV_16UC3", "CV_16UC4",
                             "CV_16S", "CV_16SC1", "CV_16SC2", "CV_16SC3", "CV_16SC4",
                             "CV_32S", "CV_32SC1", "CV_32SC2", "CV_32SC3", "CV_32SC4",
                             "CV_32F", "CV_32FC1", "CV_32FC2", "CV_32FC3", "CV_32FC4",
                             "CV_64F", "CV_64FC1", "CV_64FC2", "CV_64FC3", "CV_64FC4"};

    for(int i=0; i<numImgTypes; i++)
    {
        if(imgTypeInt == enum_ints[i]) return enum_strings[i];
    }
    return "unknown image type";
}



int main( int argc, char** argv ){
  

  
  /// Load an image
  char* window_name = "Morphology Transformations Demo";
  Mat dst, src_gray;
  Mat src = imread( argv[1], CV_LOAD_IMAGE_UNCHANGED );
  
  int color_version =0;
  if (argc > 2){
    color_version = atoi( argv[2] );
  }

  if( !src.data )
  { return -1; }
  
  // type16 = CV_8UC3 = 3x 8 bit colour
  // type 0 = CV_8U =  8 bit gray
  // type 2 = CV_16U = 16 bit grey scale depth map

  std::cout << src.type() << " " << getImgType(src.type()) << " is the type\n";
  
  
  
  
  
  imshow( "Input", src );
  Mat src_clean  = imread( argv[1], CV_LOAD_IMAGE_UNCHANGED );
  
  if (color_version ==0){
    cvtColor( src, src_gray, CV_RGB2GRAY );
  }else if (color_version==1){
    src_gray = src; 
  }else{
    src.convertTo(src, CV_16S);
    src_gray = src; 
  }
  imshow( "Input Gray", src_gray );

  imwrite("test_gray.png",src_gray);

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
  

  
  if (color_version == 0){ // rgb
Mat morph_gray, dst2;
    cvtColor( dst, morph_gray, CV_RGB2GRAY );
    std::cout << "RGB\n";

    int threshold_value =10;
    int const max_BINARY_value = 255;
    int threshold_type = 0;

    threshold( morph_gray, dst2, threshold_value, max_BINARY_value,threshold_type );
    imshow( "Threshold", dst2 );  

  imwrite("test_morph_gray_threshold.png",dst2);
    
    cvtColor(dst2, dst2, CV_GRAY2BGR);
    cv::bitwise_or(src, dst2, dst2);


  }else if (color_version == 1){ // 8 bit gray
Mat morph_gray, dst2;
    std::cout << "8 Bit Gray\n";
    
    int threshold_value =10;
    int const max_BINARY_value = 255;
    int threshold_type = 1;

    threshold( morph_gray, dst2, threshold_value, max_BINARY_value,threshold_type );
    imshow( "Threshold", dst2 );  

  imwrite("test_morph_gray_threshold.png",dst2);
    
    cv::bitwise_and(src_gray, dst2, dst2);


  }else if (color_version == 2){ // 16 bit gray
    std::cout << "16 Bit Gray\n";

    int threshold_value =254;
    int const max_BINARY_value = 3000;
    int threshold_type = 1;

    cv::Mat morph_gray;//(1021,540, CV_16S);
    morph_gray = dst;

    imwrite("test_morph_gray.png",morph_gray);
    cv::Mat dst2;//(1021,540, CV_16S);

    std::cout << dst.type() << " " << getImgType(dst.type()) << " is the dst type\n";
    std::cout << morph_gray.type() << " " << getImgType(morph_gray.type()) << " is the morph_gray type\n";
    std::cout << dst2.type() << " " << getImgType(dst2.type()) << " is the dst2 type\n";


    threshold( morph_gray, dst2, threshold_value, max_BINARY_value,threshold_type );
    std::cout << "16 Bit Gray 2\n";

    std::cout << dst2.type() << " " << getImgType(dst2.type()) << " is the dst2 type sss\n";
    
    imshow( "Threshold", dst2 );  

    imwrite("test_morph_gray_threshold.png",dst2);

   

    for(int i=0; i<dst2.rows; i++)
      for(int j=0; j<dst2.cols; j++) 
        if (  dst2.at<short unsigned int>(i,j) == 0){
          src_clean.at<short unsigned int>(i,j)  = 0;
        }

          
        // You can now access the pixel value with cv::Vec3b
//        std::cout << (int) src.at<cv::Vec3b>(i,j)[0] << std::endl;
    

    imshow( "Applied", src_clean );  

    imwrite("test_out.png",src_clean);
    
  }else{
    exit(-1); 
  }
  
  
  
  waitKey(0);
  return 0;
}


