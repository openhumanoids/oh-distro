// mfallon sept 2012

#include <stdio.h>
#include <lcm/lcm.h>
#include <bot_core/bot_core.h>

#include <opencv/cv.h>
//#include <opencv/highgui.h>


#include <ConciseArgs>

using namespace std;
using namespace cv;

class image_splitter{
  public:
    image_splitter(lcm_t* publish_lcm,lcm_t* subscribe_lcm, 
                   std::string camera_, bool output_color_);
    
    ~image_splitter(){
    }
    
  private:
    lcm_t* publish_lcm_;
    lcm_t* subscribe_lcm_;
    std::string camera_;
    bool output_color_;
    static void image_handler_aux(const lcm_recv_buf_t* rbuf, const char* channel,
                                const bot_core_image_t* msg, void* user_data){
      ((image_splitter *) user_data)->image_handler(msg, channel);
    }
    void image_handler(const bot_core_image_t *msg, const char* channel);
    

};    


image_splitter::image_splitter(lcm_t* publish_lcm,lcm_t* subscribe_lcm,
      std::string camera_, bool output_color_):
      publish_lcm_(publish_lcm), subscribe_lcm_(subscribe_lcm),
      camera_(camera_), output_color_(output_color_){
        
  bot_core_image_t_subscribe(subscribe_lcm_, camera_.c_str(),
      image_splitter::image_handler_aux, this);
}


void image_splitter::image_handler(const bot_core_image_t *msg, const char* channel){
  cout << msg->utime << "\n";
  
  if (output_color_){ // output colour
    // this code also converts from BGR to RGB:
    Mat mat_img_bgra = Mat::zeros(1536,1024,CV_8UC4); // h,w
    mat_img_bgra.data = msg->data;

    /*
    imwrite("raw.png",mat_img_bgra);
    Mat mat_img;
    cvtColor( mat_img_bgra, mat_img, CV_RGBA2BGR);
    imwrite("convert.png",mat_img);
    // republish with correct format tag:
    bot_core_image_t* msgc = bot_core_image_t_copy(msg);
    msgc->pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGBA;
    bot_core_image_t_publish(publish_lcm_, "CAM_OUT", msgc);
    */

    // Convert to RGB and republish:
    Mat mat_img2;
    cvtColor( mat_img_bgra, mat_img2, CV_RGBA2RGB);
    //imwrite("convert.png",mat_img2);

    bot_core_image_t msg2;
    msg2.utime = msg->utime;
    msg2.width = msg->width;
    msg2.height = msg->height/2;
    msg2.row_stride = 3*msg->width;
    msg2.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB;
    msg2.size = msg2.width*msg2.height*3;
    msg2.data = mat_img2.data;
    msg2.nmetadata=0;
    msg2.metadata=NULL;
    bot_core_image_t_publish(publish_lcm_, "CAMLCM_LEFT_UNRECTIFIED", &msg2);
  }else{
    bot_core_image_t msg2;
    msg2.utime = msg->utime;
    msg2.width = msg->width;
    msg2.height = msg->height/2; // half the data...
    msg2.row_stride = msg->row_stride;
    msg2.pixelformat = msg->pixelformat;
    msg2.size = msg2.width*msg2.height;
    msg2.data = msg->data;
    msg2.nmetadata=0;
    msg2.metadata=NULL;
    bot_core_image_t_publish(publish_lcm_, string(camera_ + "LEFT").c_str() , &msg2);    
  }
}


int main(int argc, char ** argv) {
  ConciseArgs parser(argc, argv, "registeration-app");
  string camera="CAMLCM_IMAGE_GRAY";
  bool output_color=false;
  parser.add(camera, "c", "camera", "Incoming Camera channel");
  parser.add(output_color, "o", "output_color", "Output Color [t=color, f=grey]");
  parser.parse();
  cout << camera << " is camera\n"; 

  
  lcm_t * lcm;
  lcm = lcm_create(NULL);//"udpm://239.255.76.67:7667?recv_buf_size=100000");

  image_splitter app(lcm,lcm, camera, output_color);

  while(1)
    lcm_handle(lcm);

  lcm_destroy(lcm);
  return 0;
}
