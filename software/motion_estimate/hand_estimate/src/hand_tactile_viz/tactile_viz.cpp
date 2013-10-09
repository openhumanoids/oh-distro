#include <stdio.h>
#include <cmath>
#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include <ConciseArgs>
#include <string>
#include <cstring>
#include <fstream>
#include "CImg.h"
using namespace cimg_library;

using namespace std;
#include <path_util/path_util.h>

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, bool is_left);
    
    ~App(){
    }
    
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    bool is_left;
    string handtext;  // Put RIGHT / LEFT
    static const int TACTILE_THRESHOLD = 3000;
    static const int NTACTILE = 32;
    void sandiaTactileStateHandler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::hand_tactile_state_t* msg);    
                             
    void readCfg(const std::string& path_cfg);
    void getHeatColor(const float v, unsigned char color [3]);
    CImg<unsigned char> orig_img;
    CImg<unsigned char> disp_img;
    CImgDisplay main_disp;
    int img_x[NTACTILE], img_y[NTACTILE];
};   

App::App(boost::shared_ptr<lcm::LCM> &lcm_, bool is_left):
   lcm_(lcm_), is_left(is_left){
  cout << "enter APP()" << endl;
  
  lcm_->subscribe(is_left ? "SANDIA_LEFT_TACTILE_STATE":"SANDIA_RIGHT_TACTILE_STATE", 
                                        &App::sandiaTactileStateHandler, this);  
                                        

  std::string cfg_path = std::string(getConfigPath()) +"/subsystems/sandia_hands/";
  string path_img, path_cfg;
  string filename = "sandia_hand_r";
  path_img = cfg_path + filename + ".jpg";
  path_cfg = cfg_path + filename + ".cfg";
  
  readCfg(path_cfg);
  
  orig_img.load(path_img.c_str());
  
  disp_img = orig_img;
  main_disp.display(disp_img);
  main_disp.resize(orig_img.width()/2, orig_img.height()/2);
  cout << "leave APP()" << endl;
}

void App::readCfg(const std::string& path_cfg){
  //649 1194         # hand origin on img
  //8.17647058824    # 556/68 px over real metric
  int hand_orig_x, hand_orig_y;
  float pixel_over_real;
  string garbage;
  ifstream fin(path_cfg.c_str());
  if(!fin)
    cout << "open filepath " << path_cfg << " fail" << endl;
  fin >> hand_orig_x >> hand_orig_y;
  getline(fin, garbage);
  fin >> pixel_over_real;
  getline(fin, garbage);
  for(int i=0;i<NTACTILE;i++){
    int id; float rx, ry, rz;
    fin >> id >> rx >> ry >> rz; 
    img_x[i] = rx*pixel_over_real + hand_orig_x;
    img_y[i] = -ry*pixel_over_real + hand_orig_y;
    cout << i << " " << img_x[i] << " " << img_y[i] << endl;
  }
  fin.close();
}

void App::getHeatColor(const float v, unsigned char color [3]){
  // v: [0~1)
  // color: RGB (0,0,0)~(255,255,255)
  if(v < 0.5){
    color[0] = (unsigned char)0;
    color[1] = (unsigned char)(2*256*v);
    color[2] = (unsigned char)(255-2*256*v);
  }
  else{
    color[0] = (unsigned char)(2*256*(v-0.5));
    color[1] = (unsigned char)(255-2*256*(v-0.5));
    color[2] = (unsigned char)(0);
  }
}

void App::sandiaTactileStateHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const  drc::hand_tactile_state_t* msg){
  
  // Draw hand img
  unsigned char color[3];
  
  disp_img = orig_img;
  for(size_t i=0;i<NTACTILE;i++){
    float v = (float)msg->palm[i] / 65536.0 * 2.5;
    if(v<0) v = 0;
    else if(v>=1) v=1-1e-8;
    getHeatColor(v, color);
    //cout << color[0] << " " << color[1] << " " << color[2] <<endl;
    //cout << i <<" " << img_x[i] << " " << img_y[i] <<endl;
    disp_img.draw_circle(img_x[i], img_y[i], /*int radius*/ 20, color);
  }
  main_disp.display(disp_img);
  //main_disp.wait();
  ///////////////////////
  
  
}
 
int main(int argc, char *argv[]){
  bool lhand=false, rhand=false;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(rhand, "r", "right","Process right hand message");
  opt.add(lhand, "l", "left","Process left hand message");
  
  opt.parse();
  if(rhand && lhand){
    printf("Only one hand at a time.\n");
    return 1;
  }
  if(!rhand && !lhand){
    printf("Please specify a hand. Type -h for usage.\n");
    return 1;
  }
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM() );
  if(!lcm->good())
    return 1;  
  
  App app(lcm, lhand);
  while(0 == lcm->handle());
  return 0;
}
