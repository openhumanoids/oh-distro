// kinect2image
// small utility to extract images from raw kinect data 
// and to publish the rgb and depth images
// - currently it only supports albert/abe's kinect work
//   which uses the freenect driver
// - assumes uncompressed rgb and depth data (default driver output)
// - in on_kinect_frame you can choose to output
//   a greyscale, rgb or 16bit version of the depth image
//
// to do:
// support compression (in and out)
// mfallon march 2011

#include <iostream>
#include <stdio.h>
#include <getopt.h>
#include <lcm/lcm.h>
#include <lcmtypes/kinect_frame_msg_t.h>
#include <lcmtypes/bot_core.h>
#include <signal.h>
#include <math.h>

/*
#include <jpeg_utils/jpeg-utils.h>
#include <jpeg_utils/jpeg-utils-ijg.h>
#include <zlib.h>
*/


//#include <ConciseArgs>


uint8_t* depth_img;
using namespace std;

// Switch between freenect and current openni data type:
//#define DEPTH_VAL 2048
#define DEPTH_VAL 8192
uint16_t t_gamma[DEPTH_VAL];


typedef struct _State
{
  lcm_t* subscribe_lcm;
  lcm_t* publish_lcm;
  int width;
  int height;
  bot_core_image_t rgb_image;
  bot_core_image_t depth_image;
  bot_core_image_t person_image;
  int frame_counter;
  int frame_skip;
} State;

sig_atomic_t shutdown_flag = 0;
static void
sig_action(int signal, siginfo_t *s, void *user)
{
  fprintf(stderr,"Shutting Down!\n");
  shutdown_flag = 1;
}

static int 
_pixel_convert_8u_rgb_to_8u_gray (uint8_t *dest, int dstride, int width,
        int height, const uint8_t *src, int sstride)
{
  int i, j;
  for (i=0; i<height; i++) {
    uint8_t *drow = dest + i * dstride;
    const uint8_t *srow = src + i * sstride;
    for (j=0; j<width; j++) {
      drow[j] = 0.2125 * srow[j*3+0] + 
        0.7154 * srow[j*3+1] + 
        0.0721 * srow[j*3+2];
    }
  }
  return 0;
}


void on_kinect_frame(const lcm_recv_buf_t *rbuf, const char *channel,
    const kinect_frame_msg_t *msg, void *user_data)
{
  State* state = static_cast<State*>(user_data);
std::cout << msg->timestamp << " was received\n";
/*
  // Skip a certain proportion:
  state->frame_counter++;
  if (state->frame_counter % state->frame_skip !=0){
    std::cerr << msg->timestamp << " | INFO: Skipping processing this frame\n";
    return;
  }else{
    std::cerr << ".";
  }   
  
  //std::cout << "im here, image type: " << ((int) msg->image.image_data_format ) << std::endl;
  //std::cout << "compre: " << ((int) msg->depth.compression )  << "depth type: " << ((int) msg->depth.depth_data_format ) << std::endl;
  //std::cout << "image:  width:" << msg->image.width << " height: " << msg->image.height << std::endl;
  //std::cout << "depth:  width:" << msg->depth.width << " height: " << msg->depth.height << std::endl;
  //std::cout << "depth:  bytes:" << msg->depth.depth_data_nbytes << std::endl;
  
  // 1. Publish the Kinect RGB Image:
  state->rgb_image.utime = msg->timestamp;
  state->rgb_image.width = msg->image.width;
  state->rgb_image.height = msg->image.height;
  state->rgb_image.row_stride = 3*msg->image.width; // guess, check me
  state->rgb_image.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB;
  state->rgb_image.nmetadata =0;
  state->rgb_image.metadata = NULL;
  if(msg->image.image_data_format == KINECT_IMAGE_MSG_T_VIDEO_RGB) {
    //std:: cout << "image rgb \n ";
    state->rgb_image.size = msg->image.width*msg->image.height*3;  
    state->rgb_image.data = msg->image.image_data;
    //  memcpy(state->rgb_image.data, msg->image.image_data, 
	//      msg->depth.width * msg->depth.height * 3);
  } else if(msg->image.image_data_format == KINECT_IMAGE_MSG_T_VIDEO_RGB_JPEG) {
    // Passthrough the image data:
    state->rgb_image.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG;
    state->rgb_image.data = msg->image.image_data;
    state->rgb_image.size = msg->image.image_data_nbytes; //msg->image.width*msg->image.height*3;  
    //    jpegijg_decompress_8u_rgb(msg->image.image_data, msg->image.image_data_nbytes,
    //          state->rgb_image.data, msg->image.width, msg->image.height, msg->image.width* 3);
  }
  bot_core_image_t_publish(state->publish_lcm, "KINECT_RGB", &state->rgb_image);
  
  
  // 2. publish the depth image (without manuplication)
  // gives aliased/overlapped image but without processing
  if (1==0){
  state->depth_image.utime = msg->depth.timestamp;
  state->depth_image.width = 2*msg->depth.width;
  state->depth_image.height = msg->depth.height;
  state->depth_image.row_stride = 2*msg->depth.width; // guess, check me
  state->depth_image.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY;
  state->depth_image.nmetadata =0;
  state->depth_image.metadata = NULL;
  //size:  width * height * 2
  state->depth_image.size = msg->depth.depth_data_nbytes;
  state->depth_image.data = msg->depth.depth_data;
  bot_core_image_t_publish(state->publish_lcm, "KINECT_DEPTH", &state->depth_image);
  }
  
  // 3. convert to correct depth information - but will not plot 
  // in spy or camview or mr-viewer but is probably the correct way
  if (1==0){
  state->depth_image.utime = msg->depth.timestamp;
  state->depth_image.width = msg->depth.width;
  state->depth_image.height = msg->depth.height;
  state->depth_image.row_stride = msg->depth.width; // guess, check me
  state->depth_image.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_GRAY16;
  // PIXEL_FORMAT_GRAY
  // BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_GRAY16
  state->depth_image.nmetadata =0;
  state->depth_image.metadata = NULL;
  //size:  width * height * 2
  state->depth_image.size = msg->depth.depth_data_nbytes;
  state->depth_image.data = msg->depth.depth_data;
  bot_core_image_t_publish(state->publish_lcm, "KINECT_DEPTH", &state->depth_image);
  }
  
  // 4. gives a grayscale image of the depth:
  if (1==0){
  state->depth_image.utime = msg->depth.timestamp;
  state->depth_image.width = msg->depth.width;
  state->depth_image.height = msg->depth.height;
  state->depth_image.row_stride = state->depth_image.width; // guess, check me
  state->depth_image.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY;
  state->depth_image.nmetadata =0;
  state->depth_image.metadata = NULL;
  //size:  width * height * 2
  state->depth_image.size = state->depth_image.width*state->depth_image.height;
  state->depth_image.data = (uint8_t *)malloc(state->depth_image.height * state->depth_image.row_stride*sizeof(uint8_t));
  int big_byte, little_byte;
  int max = DEPTH_VAL;
  int min = 800;
  for (int i=0;i<msg->depth.width;i++){ // l to r
   for (int j=0;j<msg->depth.height;j++){ // t to b
      big_byte = (int)msg->depth.depth_data[i*2 + 2*msg->depth.width*j +1]; // more significant byte
      little_byte = (int)msg->depth.depth_data[i*2 + 2*msg->depth.width*j ]; //see significan byte
      little_byte += big_byte*256;
      little_byte = (int)(255 * (little_byte - min) / (max - min));
      state->depth_image.data[i + msg->depth.width*j] = (uint8_t) little_byte;
   }
  }
  bot_core_image_t_publish(state->publish_lcm, "KINECT_DEPTH", &state->depth_image);  
  }

  
  // 5. gives a rgb image of the depth:
  if (1==1){
    const uint16_t* depth = NULL;
    uint8_t* uncompress_buffer;
    if(msg->depth.compression == KINECT_DEPTH_MSG_T_COMPRESSION_ZLIB) {
      unsigned long dlen = msg->depth.uncompressed_size;
      //uint8_t* uncompress_buffer;
      uncompress_buffer = (uint8_t*) realloc(uncompress_buffer, msg->depth.uncompressed_size);
      int status = uncompress(uncompress_buffer, &dlen, 
	      msg->depth.depth_data, msg->depth.depth_data_nbytes);
      if(status != Z_OK) {
	std::cout << "bad\n";
	return;
      }
      depth = (uint16_t*) uncompress_buffer;
    }else if(msg->depth.compression == KINECT_DEPTH_MSG_T_COMPRESSION_NONE) {
      depth = (uint16_t*) msg->depth.depth_data;
    }else{
      std::cout << "unsupported compression\n";
    }
    
    state->depth_image.utime = msg->depth.timestamp;
    state->depth_image.width = msg->depth.width;
    state->depth_image.height = msg->depth.height;
    state->depth_image.row_stride = 3*state->depth_image.width; // guess, check me
    state->depth_image.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB;
    state->depth_image.nmetadata =0;
    state->depth_image.metadata = NULL;
    state->depth_image.size = 3*state->depth_image.height*state->depth_image.width;
    
    int npixels =  state->depth_image.width * state->depth_image.height;
    for (int i=0; i<npixels; i++) {
	    if ( depth[i] >= DEPTH_VAL ) {
	      depth_img[3*i+0] = 0;
	      depth_img[3*i+1] = 0;
	      depth_img[3*i+2] = 0;
	      continue;
	    }
      
    
	      int pval = t_gamma[depth[i]];
	      int lb = pval & 0xff;
	      switch (pval>>8) { // bigger means more range in the distance
		  case 0:
		      depth_img[3*i+0] = 255;
		      depth_img[3*i+1] = 255-lb;
		      depth_img[3*i+2] = 255-lb;
		      break;
		  case 1:
		      depth_img[3*i+0] = 255;
		      depth_img[3*i+1] = lb;
		      depth_img[3*i+2] = 0;
		      break;
		  case 2:
		      depth_img[3*i+0] = 255-lb;
		      depth_img[3*i+1] = 255;
		      depth_img[3*i+2] = 0;
		      break;
		  case 3:
		      depth_img[3*i+0] = 0;
		      depth_img[3*i+1] = 255;
		      depth_img[3*i+2] = lb;
		      break;
		  case 4:
		      depth_img[3*i+0] = 0;
		      depth_img[3*i+1] = 255-lb;
		      depth_img[3*i+2] = 255;
		      break;
		  case 5:
		      depth_img[3*i+0] = 0;
		      depth_img[3*i+1] = 0;
		      depth_img[3*i+2] = 255-lb;
		      break;
		  default:
		      depth_img[3*i+0] = 0;
		      depth_img[3*i+1] = 0;
		      depth_img[3*i+2] = 0;
		      break;
	      }
    }	
    state->depth_image.data = depth_img;
    bot_core_image_t_publish(state->publish_lcm, "KINECT_DEPTH", &state->depth_image);  

    if(msg->depth.compression != KINECT_DEPTH_MSG_T_COMPRESSION_NONE) {
      free(uncompress_buffer); // memory leak bug fixed
    }
  }
  */
  
}


static void
usage(const char* progname)
{
  fprintf(stderr, "Usage: %s [options]\n",      progname);
  exit(1);
}

int main(int argc, char** argv)
{
/*  ConciseArgs parser(argc, argv, "april_tags");
  //string image_channel="IMAGE";
  bool skip_frames=false;
  int frame_skip=1; // ie skip none
  //parser.add(image_channel, "i", "image", "Outgoing image channel");
  parser.add(skip_frames, "s", "skip", "Skip frames to keep up [boolean]");
  parser.add(frame_skip, "d", "drop", "Process 1 of every X frames");
  parser.parse();
  //cout << image_channel << " is image_channel\n";  
  cout << skip_frames << " is skip_frames\n";  
  cout << frame_skip << " is frame_skip\n";  
  */
  
  
  State* state = new State();
  state->publish_lcm = lcm_create(NULL);

  int i;
  for (i=0; i<DEPTH_VAL; i++) {
    float v = i/ ((float) DEPTH_VAL);
    v = powf(v, 3)* 6;
    t_gamma[i] = v*6*256;
  }  
	
  int width = 640;
  int height = 480;
  int npixels = width*height;
  depth_img = (uint8_t*) malloc(npixels*3);	
	
  
  state->subscribe_lcm = state->publish_lcm;
  
  
  state->depth_image.data = (uint8_t *)malloc(3*height* width*sizeof(uint8_t));

  // subscribe to raw kinect data
  kinect_frame_msg_t_subscription_t* sub = kinect_frame_msg_t_subscribe(state->subscribe_lcm, "KINECT_FRAME", on_kinect_frame, state);


  
  
  // setup sigaction();
  struct sigaction new_action;
  new_action.sa_sigaction = sig_action;
  sigemptyset(&new_action.sa_mask);
  new_action.sa_flags = 0;

  sigaction(SIGINT, &new_action, NULL);
  sigaction(SIGTERM, &new_action, NULL);
  sigaction(SIGKILL, &new_action, NULL);
  sigaction(SIGHUP, &new_action, NULL);


  // go!
  while(0 == lcm_handle(state->subscribe_lcm) && !shutdown_flag);


  lcm_destroy(state->subscribe_lcm);
  if(state->subscribe_lcm != state->publish_lcm)
    lcm_destroy(state->publish_lcm);
//  delete state->depth_producer;
  
//  delete state->odom; //TODO: make destructor for state that does all this?


  delete state;

  return 0;
}
