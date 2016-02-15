#include <pcl/io/vtk_lib_io.h>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <ConciseArgs>

#include <lcmtypes/drc/pointcloud_t.hpp>


float packColor(unsigned char* color) {
    return color[0] + color[1] * 256.0 + color[2] * 256.0 * 256.0;
}

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



  // Set the colors of the pcl::PointCloud (if the pcl::PointCloud supports colors and the input vtkPolyData has colors)
  vtkUnsignedCharArray* colors = vtkUnsignedCharArray::SafeDownCast (polydata->GetPointData ()->GetScalars ());

  if (colors)
  {
    for (size_t i = 0; i < polydata->GetNumberOfPoints (); ++i)
    {
      unsigned char color[3];
      colors->GetTupleValue (i, color);
      float c_float = packColor(color);
      std::vector<float> channels = {c_float};
      msg.channels.push_back(channels);

    }
  msg.channel_names.push_back("rgb_colors");
  msg.n_channels = 1;
  }

//  int32_t n_channels;
//  string channel_names [n_channels];
//  float channels [n_points][n_channels];
//}

  boost::shared_ptr<lcm::LCM> lcm_pub;
  lcm_pub = boost::shared_ptr<lcm::LCM>(new lcm::LCM);
  if(!lcm_pub->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  lcm_pub->publish("POINTCLOUD",&msg);
  std::cout << "Writing to lcm" << std::endl;

  return (0);
}

