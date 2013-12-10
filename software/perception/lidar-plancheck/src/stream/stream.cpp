/// Tool To Stream Point Cloud to iPad or iPhone
// 1 Send a coloured range image lidar sweep here:
//  drc-get-lidar-depth-image 
// 2 Send this to the ipad or iphone
//  drc-stream 111111
// 3 on the kiwiviewer, put in the url of thislaptop: e.g. 128.22.122.12
//  .... after the first pointcloud is received by drc-stream, it should apear on the kiwiviewer
//
//http://pointclouds.org/documentation/tutorials/mobile_streaming.php
//http://pointclouds.org/blog/nvcs/patmarion/all.php
//http://www.kitware.com/blog/home/post/273
//http://www.vtk.org/Wiki/VES/Point_Cloud_Streaming



#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>

#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/bot_core.hpp"
#include <lcmtypes/pointcloud_tools.hpp>
#include <pointcloud_tools/pointcloud_lcm.hpp>

using boost::asio::ip::tcp;


struct PointCloudBuffers
{
  typedef boost::shared_ptr<PointCloudBuffers> Ptr;
  std::vector<short> points;
  std::vector<unsigned char> rgb;
};

void
CopyPointCloudToBuffers (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, PointCloudBuffers& cloud_buffers)
{
  const size_t nr_points = cloud->points.size ();

  cloud_buffers.points.resize (nr_points*3);
  cloud_buffers.rgb.resize (nr_points*3);

  const pcl::PointXYZ  bounds_min (-3.0, -3.0, -3.0);
  const pcl::PointXYZ  bounds_max (3.0, 3.0, 3.0);

  size_t j = 0;
  for (size_t i = 0; i < nr_points; ++i)
  {

    const pcl::PointXYZRGB& point = cloud->points[i];

    if (!pcl_isfinite (point.x) || 
        !pcl_isfinite (point.y) || 
        !pcl_isfinite (point.z))
      continue;

    if (point.x < bounds_min.x ||
        point.y < bounds_min.y ||
        point.z < bounds_min.z ||
        point.x > bounds_max.x ||
        point.y > bounds_max.y ||
        point.z > bounds_max.z)
      continue;

    const int conversion_factor = 500;

    cloud_buffers.points[j*3 + 0] = static_cast<short> (point.x * conversion_factor);
    cloud_buffers.points[j*3 + 1] = static_cast<short> (point.y * conversion_factor);
    cloud_buffers.points[j*3 + 2] = static_cast<short> (point.z * conversion_factor);

    cloud_buffers.rgb[j*3 + 0] = point.r;
    cloud_buffers.rgb[j*3 + 1] = point.g;
    cloud_buffers.rgb[j*3 + 2] = point.b;

    j++;
  }

  cloud_buffers.points.resize (j * 3);
  cloud_buffers.rgb.resize (j * 3);
}


template <typename PointType>
class PCLMobileServer
{
  public:

    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    PCLMobileServer (boost::shared_ptr<lcm::LCM> &lcm_, const std::string& device_id = "", int port = 11111,
                     float leaf_size_x = 0.001, float leaf_size_y = 0.001, float leaf_size_z = 0.001)
    : port_ (port),
      device_id_ (device_id), 
      //viewer_ ("PCL OpenNI Mobile Server"),
      lcm_(lcm_)
    {
      voxel_grid_filter_.setLeafSize (leaf_size_x, leaf_size_y, leaf_size_z);

      //lcm_->subscribe( "SCAN" ,&PCLMobileServer::lidarHandler,this);
      lcm_->subscribe( "RANGE_IMAGE_POINTS" ,&PCLMobileServer::pointcloudHandler,this);
      std::cout <<"subscribe to lidar\n";

    }
    
        
    void pointcloudHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  ptools::pointcloud2_t* msg){
      cout << "got pc\n";
      pointcloud_lcm* pc_lcm_;
      pc_lcm_ = new pointcloud_lcm(lcm_->getUnderlyingLCM() );
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
      pc_lcm_->unpack_pointcloud2(   msg, new_cloud);

      PointCloudBuffers::Ptr new_buffers = PointCloudBuffers::Ptr (new PointCloudBuffers);
      CopyPointCloudToBuffers (new_cloud, *new_buffers);
      boost::mutex::scoped_lock lock (mutex_);
      filtered_cloud_ = new_cloud;
      buffers_ = new_buffers;      
    }    
    
    PointCloudBuffers::Ptr
    getLatestBuffers ()
    {

      boost::mutex::scoped_lock lock (mutex_);
      return (buffers_);
    }

    CloudPtr
    getLatestPointCloud ()
    {
      boost::mutex::scoped_lock lock (mutex_);
      return (filtered_cloud_);
    }

    void
    run ()
    {

      // wait for first cloud
      while (!getLatestPointCloud ())
        boost::this_thread::sleep (boost::posix_time::milliseconds (10));

      boost::asio::io_service io_service;
      tcp::endpoint endpoint (tcp::v4 (), static_cast<unsigned short> (port_));
      tcp::acceptor acceptor (io_service, endpoint);
      tcp::socket socket (io_service);

      std::cout << "Listening on port " << port_ << "..." << std::endl;
      acceptor.accept (socket);

      std::cout << "Client connected." << std::endl;

      double start_time = pcl::getTime ();
      int counter = 0;

      while(1==1){
              // wait for client
        unsigned int nr_points = 0;
        boost::asio::read (socket, boost::asio::buffer (&nr_points, sizeof (nr_points)));

        PointCloudBuffers::Ptr buffers_to_send = getLatestBuffers ();
        nr_points = static_cast<unsigned int> (buffers_to_send->points.size()/3);
        boost::asio::write (socket, boost::asio::buffer (&nr_points, sizeof (nr_points)));

        if (nr_points){
          boost::asio::write (socket, boost::asio::buffer (&buffers_to_send->points.front(), nr_points * 3 * sizeof (short)));
          boost::asio::write (socket, boost::asio::buffer (&buffers_to_send->rgb.front(), nr_points * 3 * sizeof (unsigned char)));
        }

        counter++;
        
        // wait for timer expiry
        boost::asio::io_service service;
        boost::asio::deadline_timer timer(service);
        timer.expires_from_now(boost::posix_time::
                             milliseconds(1000));
        timer.wait();        

        double new_time = pcl::getTime ();
        double elapsed_time = new_time - start_time;
        if (elapsed_time > 1.0){
          double frames_per_second = counter / elapsed_time;
          start_time = new_time;
          counter = 0;
          std::cout << "fps: " << frames_per_second << std::endl;
          std::cout << buffers_to_send->points.size() << " buffer pts headless\n";
        }
      }
    }
      

    int port_;
    std::string device_id_;
    boost::mutex mutex_;

    pcl::VoxelGrid<PointType> voxel_grid_filter_;

    CloudPtr filtered_cloud_;
    PointCloudBuffers::Ptr buffers_;

    boost::shared_ptr<lcm::LCM> lcm_;
};

void
usage (char ** argv)
{
  std::cout << "usage: " << argv[0] << " <options>\n"
            << "where options are:\n"
            << "  -port p :: set the server port (default: 11111)\n"
            << "  -leaf x, y, z  :: set the voxel grid leaf size (default: 0.01)\n";
}


void lcmThread(PCLMobileServer<pcl::PointXYZRGB>& server) { 
  std::cout << "lcmThread started\n";
  while(0 == server.lcm_->handle());
}

void serverThread(PCLMobileServer<pcl::PointXYZRGB>& server) { 
  sleep(1); // ... not necessary just for clarity

  std::cout << "serverThread started\n";
  server.run ();
}


int 
main (int argc, char ** argv)
{
  if (pcl::console::find_argument (argc, argv, "-h") != -1)
  {
    usage (argv);
    return (0);
  }

  int port = 11111;
  double leaf_x = 0.0005, leaf_y = 0.0005, leaf_z = 0.0005;
  std::string device_id = "";

  pcl::console::parse_argument (argc, argv, "-port", port);
  pcl::console::parse_3x_arguments (argc, argv, "-leaf", leaf_x, leaf_y, leaf_z, false);

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  PCL_INFO ("Using %f, %f, %f as a leaf size for VoxelGrid.\n", leaf_x, leaf_y, leaf_z);

  PCLMobileServer<pcl::PointXYZRGB> server (lcm,device_id, port, leaf_x, leaf_y, leaf_z);
    boost::thread_group thread_group;

  thread_group.create_thread(boost::bind(lcmThread, boost::ref( server)));
  thread_group.create_thread(boost::bind(serverThread, boost::ref( server)));
  thread_group.join_all();  
  return (0);
}
