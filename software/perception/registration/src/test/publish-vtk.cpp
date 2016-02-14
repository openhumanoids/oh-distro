#include <pcl/io/vtk_lib_io.h>

#include <lcmtypes/drc/pointcloud_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <ConciseArgs>

int
main (int argc, char** argv)
{
  std::string vtk_file;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(vtk_file, "v", "vtk_file","vtk_file");
  opt.parse();

  vtkSmartPointer<vtkPolyDataReader> reader = vtkSmartPointer<vtkPolyDataReader>::New ();
  reader->SetFileName (vtk_file.c_str());
  reader->Update ();
  vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput ();

  drc::pointcloud_t msg;
  msg.utime =0;
  msg.seq =0;
  msg.frame_id = "world";
  msg.n_points = polydata->GetNumberOfPoints ();
  for (size_t i = 0; i < polydata->GetNumberOfPoints (); i++) {
    double xyz[3];
    polydata->GetPoint (i, xyz);
    std::vector<float> point = { (float) xyz[0], (float) xyz[1], (float) xyz[2]};
    msg.points.push_back(point);
  }
  msg.n_channels = 0;

  boost::shared_ptr<lcm::LCM> lcm_pub;
  lcm_pub = boost::shared_ptr<lcm::LCM>(new lcm::LCM);
  if(!lcm_pub->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  lcm_pub->publish("POINTCLOUD",&msg);
  std::cout << "Writing to lcm" << std::endl;

  return (0);
}

