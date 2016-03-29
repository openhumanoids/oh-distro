#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <ConciseArgs>

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


void saveColorOcTree(pcl::PointCloud<pcl::PointXYZRGB> &cloud, double res){

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

}

void saveOcTree(pcl::PointCloud<pcl::PointXYZRGB> &cloud, double res){

  OcTree tree (res);

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    point3d endpoint ((float) cloud.points[i].x, (float) cloud.points[i].y, (float) cloud.points[i].z);
    tree.updateNode(endpoint, true); 
    //n->setColor(cloud.points[i].r,cloud.points[i].g,cloud.points[i].b); // set color to red
  }

 // set inner node colors
  tree.updateInnerOccupancy();

  cout << endl;

  std::string filename ("octree.bt");
  std::cout << "Writing binary tree to " << filename << std::endl;
  // write color tree
  tree.writeBinary(filename);

}


/* ---[ */
int
main (int argc, char** argv)
{
  std::string vtk_file;
  double res = 0.5;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(vtk_file, "v", "vtk_file","vtk_file");
  opt.add(res, "r", "resolution", "octree resolution");
  opt.parse();

  vtkSmartPointer<vtkPolyDataReader> reader = vtkSmartPointer<vtkPolyDataReader>::New ();
  reader->SetFileName (vtk_file.c_str());
  reader->Update ();
  vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput ();

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  vtkPolyDataToPointCloud (polydata, cloud);

  // Convert and save
  saveCloud ("cloud.pcd", cloud);
  saveColorOcTree(cloud, res);
  saveOcTree(cloud, res);
  return (0);
}