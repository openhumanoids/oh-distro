#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

using namespace octomap;
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

  bool copy_normals = false;
  parse_argument (argc, argv, "-copy_normals", copy_normals);
  PCL_INFO ("Copy normals: %s.\n", copy_normals ? "true" : "false");

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  vtkPolyDataToPointCloud (polydata, cloud);
  // Convert to pcd and save
  saveCloud (argv[pcd_file_indices[0]], cloud);

  double res = 0.05;  // create empty tree with resolution 0.05 (different from default 0.1 for test)
  ColorOcTree tree (res);

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    point3d endpoint ((float) cloud.points[i].x, (float) cloud.points[i].y, (float) cloud.points[i].z);
    ColorOcTreeNode* n = tree.updateNode(endpoint, true); 
    n->setColor(cloud.points[i].r,cloud.points[i].g,cloud.points[i].b); // set color to red
  }

 // set inner node colors
  tree.updateInnerOccupancy();

  cout << endl;

  std::string filename ("color_octree.ot");
  std::cout << "Writing color tree to " << filename << std::endl;
  // write color tree
  tree.write(filename);

  return (0);
}

