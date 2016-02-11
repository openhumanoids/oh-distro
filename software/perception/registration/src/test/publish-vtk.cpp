#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <lcmtypes/drc/pointcloud_t.hpp>
#include <lcm/lcm-cpp.hpp>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.vtk output.pcd [options]\n", argv[0]);
  print_info ("where options are: \n");
  print_info ("     -copy_normals 0/1 : set to true (1) or false (0) if the output PointCloud should contain normals or not.\n");
}

template <typename T> void
saveCloud (const std::string &filename, const pcl::PointCloud<T> &cloud)
{
  print_highlight ("Saving "); print_value ("%s ", filename.c_str ()); 
  PCDWriter w;
  w.writeBinaryCompressed (filename, cloud);
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Convert a VTK file to PCD format. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd and .vtk files
  std::vector<int> pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  std::vector<int> vtk_file_indices = parse_file_extension_argument (argc, argv, ".vtk");
  if (pcd_file_indices.size () != 1 || vtk_file_indices.size () != 1)
  {
    print_error ("Need one input VTK file and one output PCD file.\n");
    return (-1);
  }

  // Load the VTK file
  print_highlight ("Loading "); print_value ("%s ", argv[vtk_file_indices[0]]);

  vtkSmartPointer<vtkPolyDataReader> reader = vtkSmartPointer<vtkPolyDataReader>::New ();
  reader->SetFileName (argv[vtk_file_indices[0]]);
  reader->Update ();
  vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput ();


  drc::pointcloud_t msg;
  msg.utime =0;
  msg.seq =0;
  msg.frame_id = "world";
  msg.n_points = cloud.points.size();
  msg.n_channels = 0;
  for (size_t i = 0; i < cloud.points.size(); i++) {
    std::vector<float> point = { cloud.points[i].x, cloud.points[i].y, cloud.points[i].z};
    msg.points.push_back(point);
    std::vector<float> channels = { };
    msg.channels.push_back(channels);
  }
  msg.n_points = msg.points.size();
  msg.n_channels = 0;

  boost::shared_ptr<lcm::LCM> lcm_pub;
  lcm_pub = boost::shared_ptr<lcm::LCM>(new lcm::LCM);
  lcm_pub->publish("POINTCLOUD",&msg);

  std::cout << "Writing to lcm" << std::endl;

  return (0);
}

