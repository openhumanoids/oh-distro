#include "kinectOpenniLCM.h"
#include <boost/math/constants/constants.hpp>
#include <boost/program_options.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

#include "openni_camera/openni_exception.h"
#include "openni_camera/openni_depth_image.h"
#include "openni_camera/openni_image.h"

#include <bot_core/timestamp.h>

#include <zlib.h>
#include <glib.h>

namespace po = boost::program_options;

KinectOpenniLCM::KinectOpenniLCM(int argc, char **argv)
{
    jpeg_quality = 94;

    double target_rate = INFINITY;
    int c;
    char *lcm_url = NULL;
    int8_t current_image_format;

    int8_t requested_depth_format;
    int8_t current_depth_format;

    int jpeg_quality;
    int user_device_number = 0;

    //int use_zlib;
    int throttle;
    char* msg_channel;

    // command line options - to throtle - to ignore image publish  
    while ((c = getopt(argc, argv, "hd:i:r:jq:zl:n:c:")) >= 0) {
        switch (c) {
        case 'i': //ignore images
            requested_image_format = atoi(optarg);
            break;
        case 'd':
            requested_depth_format = atoi(optarg);
            break;
        case 'j':
            requested_image_format = KINECT_IMAGE_MSG_T_VIDEO_RGB_JPEG;
            printf("Setting JPEG mode\n");
            break;
        case 'q':
            jpeg_quality = atoi(optarg);
            if (jpeg_quality < 0 || jpeg_quality > 100)
                usage(argv[0]);
            break;
        case 'n':
            user_device_number = atoi(optarg);
            printf("attempting to open device %i\n", user_device_number);
            break;
        case 'z':
            use_zlib = 1;
            printf("ZLib compressing depth data\n");
            break;
        case 'r':
            target_rate = strtod(optarg, NULL);
            printf("Target Rate is : %.3f Hz\n", target_rate);
            throttle = 1;
            break;
        case 'l':
            lcm_url = strdup(optarg);
            printf("Using LCM URL \"%s\"\n", lcm_url);
            break;
        case 'c':
            g_free(msg_channel);
            msg_channel = g_strdup(optarg);
            printf("Output on LCM channel: %s\n", msg_channel);
            break;
        case 'h':
        case '?':
            usage(argv[0]);
        }
    }


  std::string deviceId = "#1";
  
  /*throttle = 0;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()
      ("help", "produce help message")
      ( "id", 
        po::value<std::string>(&deviceId)->default_value(std::string("#1")), 
        "index (e.g., #1) or serial of kinect" )
      ( "rate", 
        po::value<double>(&target_rate)->default_value(30.0), 
        "desired publish rate of device" )      
    ;
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  
  if (vm.count("help")) {
    std::cout << desc << std::endl;
    exit(1);
  }

  if (vm.count("rate")) {
      std::cout << "Target Rate : " << target_rate << " Hz" << std::endl;
      }*/

  m_lcm = bot_lcm_get_global(NULL);

  rgb_data = (uint8_t*)calloc(640*480*3, sizeof(uint8_t));                                              
  
  image_buf_size = 640 * 480 * 10;
  if (0 != posix_memalign((void**) &image_buf, 16, image_buf_size)) {
      fprintf(stderr, "Error allocating image buffer\n");
      //return 1;
  }
  
  
  // allocate space for unpacking depth data
  depth_unpack_buf_size = 640 * 480 * sizeof(uint16_t);
  depth_unpack_buf = (uint16_t*) malloc(depth_unpack_buf_size);

  // allocate space for zlib compressing depth data
  depth_compress_buf_size = 640 * 480 * sizeof(int16_t) * 4;
  depth_compress_buf = (uint8_t*) malloc(depth_compress_buf_size);
  depth_data = depth_compress_buf;
  //depth_data = (uint8_t*)calloc(640*480*2, sizeof(uint8_t));
  

  report_rate = rate_new(0.5);
  // throttling
  capture_rate = rate_new(target_rate);

  pthread_create(&work_thread, NULL, status_thread, this);

  SetupDevice(deviceId);

  new_data = false;
}

void KinectOpenniLCM::usage(const char* progname)
{
    fprintf(stderr, "Usage: %s [options]\n"
            "\n"
            "Options:\n"
            "  -r RATE   Throttle publishing to RATE Hz.\n"
            "  -d        Depth mode\n"
            "  -i        Image mode\n"
            "  -j        JPEG-compress RGB images\n"
            "  -q QUAL   JPEG compression quality (0-100, default 94)\n"
            "  -z        ZLib compress depth images\n"
            "  -l URL    Specify LCM URL\n"
            "  -h        This help message\n"
            "  -n dev    Number of the device to open\n"
            "  -c name   LCM channel\n",
            g_path_get_basename(progname));

    fprintf(stderr, "Image mode must be one of:\n"
            "  VIDEO_RGB             = 0\n"
            "  VIDEO_BAYER           = 1\n"
            "  VIDEO_IR_8BIT         = 2\n"
            "  VIDEO_IR_10BIT        = 3\n"
            "  VIDEO_IR_10BIT_PACKED = 4\n"
            "  VIDEO_YUV_RGB         = 5\n"
            "  VIDEO_YUV_RAW         = 6\n"
            "\n"
            "  VIDEO_DISABLED        = -1\n"
            );

    fprintf(stderr, "Depth mode must be one of:\n"
            "  DEPTH_11BIT        = 0\n"
            "  DEPTH_10BIT        = 1\n"
            "  DEPTH_11BIT_PACKED = 2\n"
            "  DEPTH_10BIT_PACKED = 3\n"
            "  DEPTH_REGISTERED   = 4\n"
            "  DEPTH_MM           = 5\n"
            "\n"
            "  DEPTH_DISABLED         =-1\n"
            );
    exit(1);
}

KinectOpenniLCM::~KinectOpenniLCM()
{
  if ( m_device ) {
    m_device->stopDepthStream ();
    m_device->stopImageStream ();
  }
  
  if(capture_rate)
      rate_destroy(capture_rate);
}

KinectOpenniLCM::rate_t* KinectOpenniLCM::rate_new(double target_hz)
{
    rate_t* rt = (rate_t *) calloc(1, sizeof(rate_t));
    rt->target_hz = target_hz;
    rt->tick_count = 0;
    return rt;
}

//pthread - for publishing sensor status 
void * KinectOpenniLCM::status_thread(void *data)
{

    KinectOpenniLCM *self = (KinectOpenniLCM *)data;

    while(1){        
        if(self->report_rate){
            kinect_sensor_status_t msg;
            msg.utime = bot_timestamp_now();
            msg.sensor_name = "kinect"; //maybe use some indexing - to make it unique
            msg.rate = self->capture_rate->current_hz;
            fprintf(stderr, "Freq : %f\n", self->capture_rate->current_hz);
            msg.type = KINECT_SENSOR_STATUS_T_KINECT; //prob need to identify this more - if there are multiple kinects

            kinect_sensor_status_t_publish(self->m_lcm, "SENSOR_STATUS_KINECT", &msg);
            }
        sleep(1);
    }
    
    return 0;
}

void KinectOpenniLCM::rate_destroy(rate_t* rate)
{
    free(rate);
}

int KinectOpenniLCM::rate_check(rate_t* rate)
{
    // check the current time
    int64_t c_utime = bot_timestamp_now();

    // compute the framerate if we were to publish an image
    int64_t dt = c_utime - rate->last_tick;

    double p_framerate = alpha * (1.0 * 1e6 / dt) + (1 - alpha) * rate->current_hz;
    if (p_framerate > rate->target_hz) {
        // if the potential framerate is too high, don't publish, and return 0
        return 0;
    }
    else {
        // otherwise, update current_hz with a exponential moving average, and return 1
        rate->current_hz = p_framerate;
        rate->last_tick = c_utime;
        rate->tick_count++;
        return 1;
    }
}

void KinectOpenniLCM::SetupDevice(const std::string& deviceId)
{
  m_device = boost::shared_ptr<openni_wrapper::OpenNIDevice > ((openni_wrapper::OpenNIDevice*)NULL);

  // Initialize the openni device
  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();

  do {

    driver.updateDeviceList ();

    if (driver.getNumberDevices () == 0) {
      std::cout << "No devices connected.... waiting for devices to be connected" << std::endl;
      sleep(1);
      continue;
    }

    std::cout << boost::format("Number devices connected: %d") % driver.getNumberDevices() << std::endl;
    for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx) {
      std::cout << boost::format("  %u. device on bus %03i:%02i is a %s (%03X) from %s (%03X) with serial id \'%s\'")
    % (deviceIdx+1) % (int)driver.getBus(deviceIdx) % (int)driver.getAddress(deviceIdx)
    % std::string(driver.getProductName(deviceIdx)) % driver.getProductID(deviceIdx) % std::string(driver.getVendorName (deviceIdx))
    % driver.getVendorID(deviceIdx) % std::string(driver.getSerialNumber(deviceIdx)) << std::endl;
    }

    try {
      if (deviceId[0] == '#') {
    unsigned int index = boost::lexical_cast<unsigned int>(deviceId.substr(1));
    std::cout << boost::format("searching for device with index = %d") % index << std::endl;
    m_device = driver.getDeviceByIndex(index - 1);
    break;
      } else {
    std::cout << boost::format("searching for device with serial number = %s") % deviceId << std::endl;
    m_device = driver.getDeviceBySerialNumber(deviceId);
      }
    }
    catch (const openni_wrapper::OpenNIException& exception) {
      if (!m_device) {
    std::cout << boost::format("No matching device found.... waiting for devices. Reason: %s") % exception.what () << std::endl;
        sleep (1);
        continue;
      } else {
    std::cout << boost::format("could not retrieve device. Reason %s") % exception.what () << std::endl;
        exit(-1);
      }
    }
  } while ( !m_device );

  std::cout << boost::format("Opened '%s' on bus %i:%i with serial number '%s'") % m_device->getProductName()
    % (int)m_device->getBus () % (int)m_device->getAddress() % m_device->getSerialNumber() << std::endl;

  m_device->registerImageCallback(&KinectOpenniLCM::ImageCallback, *this);
  m_device->registerDepthCallback(&KinectOpenniLCM::DepthCallback, *this);

  //setting the view point to be the RGB camera 
  m_device->depth_generator_.GetAlternativeViewPointCap().SetViewPoint(m_device->image_generator_);

  m_device->startImageStream ();
  m_device->startDepthStream ();
  startSynchronization ();
}


void KinectOpenniLCM::startSynchronization ()
{
  if (m_device->isSynchronizationSupported () && !m_device->isSynchronized () &&
      m_device->getImageOutputMode ().nFPS == m_device->getDepthOutputMode ().nFPS &&
      m_device->isImageStreamRunning () && m_device->isDepthStreamRunning () )
    m_device->setSynchronization (true);
}

void KinectOpenniLCM::stopSynchronization ()
{
  if (m_device->isSynchronizationSupported () && m_device->isSynchronized ())
    m_device->setSynchronization (false);
}

void KinectOpenniLCM::ImageCallback (boost::shared_ptr<openni_wrapper::Image> image, void* cookie)
{
  int64_t thisTime = bot_timestamp_now();
  int64_t diffTime = thisTime - m_lastImageTime;
  int64_t diffFromDepthTime = thisTime - m_lastDepthTime;


  //std::cout << "got an image     :" << thisTime << ", " << ((float)diffTime/1000000.0f) << ", " << ((float)diffFromDepthTime/1000000.0f) << std::endl;
  m_mutex.lock();

  image->fillRGB(image->getWidth(), image->getHeight(), reinterpret_cast<unsigned char*> (rgb_data), 640*3);
  m_lastImageTime = thisTime;
  
  m_mutex.unlock();
}

void KinectOpenniLCM::DepthCallback (boost::shared_ptr<openni_wrapper::DepthImage> depth_image, void* cookie)
{
  if (rate_check(capture_rate) == 0) return;

  int64_t thisTime = bot_timestamp_now();

  //std::cout << "got a depth image:" << thisTime << ", " << ((float)diffTime/1000000.0f) << ", " << ((float)diffFromImageTime/1000000.0f) << std::endl;

  m_lastDepthTime = thisTime;

  kinect_frame_msg_t msg;
  msg.timestamp = thisTime;
  msg.image.timestamp = m_lastImageTime;
  msg.image.width = 640;
  msg.image.height = 480;
  msg.image.image_data_nbytes = 640*480*3;  
 
  if(requested_image_format == KINECT_IMAGE_MSG_T_VIDEO_RGB_JPEG){
      int compressed_size =  640*480*3;//image_buf_size;

#if USE_JPEG_UTILS_POD
      int compression_status = jpeg_compress_8u_rgb (rgb_data, 640, 480, 640*3,
                                                     image_buf, &compressed_size, jpeg_quality);
#else
      int compression_status = jpegijg_compress_8u_rgb(rgb_data, 640, 480, 640 * 3,
                                                       image_buf, &compressed_size, jpeg_quality);
#endif
      if (0 != compression_status) {
          fprintf(stderr, "JPEG compression failed...\n");
      }
      msg.image.image_data_nbytes = compressed_size;
      msg.image.image_data_format = KINECT_IMAGE_MSG_T_VIDEO_RGB_JPEG;
      msg.image.image_data = image_buf; //rgb_data;
  }
  else{
      msg.image.image_data_format = KINECT_IMAGE_MSG_T_VIDEO_RGB;
      msg.image.image_data = rgb_data;
      msg.image.image_data_nbytes = 640*480*3;
  }

  msg.depth.timestamp = thisTime;
  msg.depth.width = depth_image->getWidth();
  msg.depth.height = depth_image->getHeight();
  //msg.depth.compression = KINECT_DEPTH_MSG_T_COMPRESSION_NONE;
  msg.depth.depth_data_format = KINECT_DEPTH_MSG_T_DEPTH_MM;//KINECT_DEPTH_MSG_T_DEPTH_11BIT;
  
    depth_image->fillDepthImageRaw(msg.depth.width, msg.depth.height, reinterpret_cast<unsigned short*>(depth_unpack_buf), depth_image->getWidth() * sizeof(short));
  
  if(use_zlib == 1){
    int uncompressed_size = depth_image->getHeight() * depth_image->getWidth() * sizeof(short);
    unsigned long compressed_size = depth_compress_buf_size;
    compress2(depth_compress_buf, &compressed_size, (const Bytef*) depth_unpack_buf, uncompressed_size,
                  Z_BEST_SPEED);  
    msg.depth.depth_data_nbytes =(int)compressed_size;
    msg.depth.compression = KINECT_DEPTH_MSG_T_COMPRESSION_ZLIB;
    msg.depth.uncompressed_size = uncompressed_size;
    msg.depth.depth_data = depth_compress_buf;
  }else{
    //assert(msg.depth.depth_data_nbytes <= depth_compress_buf_size);
    //memcpy(state->depth_compress_buf, state->depth_unpack_buf, state->msg.depth.depth_data_nbytes);
    msg.depth.depth_data_nbytes = depth_image->getHeight() * depth_image->getWidth() * sizeof(short);
    msg.depth.compression = KINECT_DEPTH_MSG_T_COMPRESSION_NONE;
    msg.depth.uncompressed_size = msg.depth.depth_data_nbytes;
    msg.depth.depth_data = (uint8_t*)depth_unpack_buf;
  }


  m_mutex.lock();
  msg.image.timestamp = m_lastImageTime;
  kinect_frame_msg_t_publish(m_lcm, "KINECT_FRAME", &msg);
  m_mutex.unlock();
  
  //fprintf(stderr,"j %d %d | z %d %d\n",requested_image_format,msg.image.image_data_nbytes, use_zlib, msg.depth.depth_data_nbytes );
  
}
