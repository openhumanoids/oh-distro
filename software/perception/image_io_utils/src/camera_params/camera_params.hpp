#include <bot_param/param_client.h>
#include <string>
  
class CameraParams {
  public:
    CameraParams ();
  
    void setParams(BotParam *botparam_, std::string key_prefix);
  /*
    cv::Mat_<double> K; 
    cv::Mat_<double> D; */

    int width, height;
    float fx, fy, cx, cy, k1, k2, k3, p1, p2;
    
    /*
    cv::Mat_<double> getK() { 
        K = cv::Mat_<double>::zeros(3,3);
        K(0,0) = fx, K(1,1) = fy; 
        K(0,2) = cx, K(1,2) = cy;
        K(2,2) = 1;
        return K;
    }
    cv::Mat_<double> getD() { 
        D = cv::Mat_<double>::zeros(1,5);
        D(0,0) = k1, D(0,1) = k2; 
        D(0,2) = p1, D(0,2) = p2;
        D(0,4) = k3;
        return D;
    }*/
};



class StereoParams {
  public:
    StereoParams ();
  
    void setParams(BotParam *botparam_, std::string key_prefix);
    
   CameraParams left;
   CameraParams right;
   double rotation[9];
   double translation[3];

};