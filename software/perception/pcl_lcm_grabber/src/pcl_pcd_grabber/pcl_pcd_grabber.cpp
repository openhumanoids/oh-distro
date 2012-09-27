#include <pcl/io/pcd_grabber.h>
 #include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#define BOOST_FILESYSTEM_VERSION 2
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

boost::shared_ptr<pcl::PCDGrabber<pcl::PointXYZRGB> > grabber;

 class SimplePCDViewer
 {
   public:
     SimplePCDViewer () : viewer ("PCL PCL Viewer") {}

     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
     {
std::cout << "got cp\n";
       if (!viewer.wasStopped())
         viewer.showCloud (cloud);
     }

     void run ()
     {

std::string path="/home/mfallon/drc/software/perception/pcl_lcm_grabber";
  float frames_per_second = 1; // 0 means only if triggered!
bool repeat=true;

    std::vector<std::string> pcd_files;
//    pcl::console::parse_argument (argc, argv, "-dir", path);
    std::cout << "path: " << path << std::endl;
    if (path != "" && boost::filesystem::exists (path))
    {
      boost::filesystem::directory_iterator end_itr;
      for (boost::filesystem::directory_iterator itr (path); itr != end_itr; ++itr)
      {
        if (!is_directory (itr->status()) && boost::algorithm::to_upper_copy(boost::filesystem::extension (itr->leaf())) == ".PCD" )
        {
          pcd_files.push_back (itr->path ().string());
          std::cout << "added: " << itr->path ().string() << std::endl;
        }
      }
    }
    else
    {
      std::cout << "Neither a pcd file given using the \"-file\" option, nor given a directory containing pcd files using the \"-dir\" option." << std::endl;
    }




       grabber.reset(new pcl::PCDGrabber<pcl::PointXYZRGB> (pcd_files, frames_per_second, repeat));

       boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
         boost::bind (&SimplePCDViewer::cloud_cb_, this, _1);

       grabber->registerCallback (f);

       grabber->start ();

       while (!viewer.wasStopped())
       {
         boost::this_thread::sleep (boost::posix_time::seconds (1));
       }

       grabber->stop ();
     }

     pcl::visualization::CloudViewer viewer;
 };

 int main ()
 {
   SimplePCDViewer v;
   v.run ();
   return 0;
 }
