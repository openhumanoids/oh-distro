#include <pcl/io/vtk_lib_io.h>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkUnsignedIntArray.h>

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <ConciseArgs>

#include <lcmtypes/bot_core/pointcloud_t.hpp>


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

  bot_core::pointcloud_t msg;
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

  vtkIdType numberOfPointArrays = polydata->GetPointData()->GetNumberOfArrays();

  //http://www.vtk.org/doc/nightly/html/vtkType_8h_source.html
  for(vtkIdType i = 0; i < numberOfPointArrays; i++)
    {
    int dataTypeID = polydata->GetPointData()->GetArray(i)->GetDataType();
    std::string atr_name = polydata->GetPointData()->GetArrayName(i);

    if (dataTypeID == VTK_UNSIGNED_INT)
    {
      vtkUnsignedIntArray* arrayData = vtkUnsignedIntArray::SafeDownCast (polydata->GetPointData ()->GetScalars ( atr_name.c_str() ));
      if (arrayData)
      {
        std::vector<float> arrayDataOutput;
        for (size_t i = 0; i < polydata->GetNumberOfPoints (); ++i)
          arrayDataOutput.push_back( (float) arrayData->GetValue (i));

        msg.channels.push_back(arrayDataOutput);
        msg.channel_names.push_back( atr_name.c_str() );
        msg.n_channels++;
      }
    }
    else if (dataTypeID == VTK_FLOAT)
    {
      vtkFloatArray* arrayData = vtkFloatArray::SafeDownCast (polydata->GetPointData ()->GetScalars ( atr_name.c_str() ));
      if (arrayData)
      {
        std::vector<float> arrayDataOutput;
        for (size_t i = 0; i < polydata->GetNumberOfPoints (); ++i)
          arrayDataOutput.push_back( arrayData->GetValue (i));

        msg.channels.push_back(arrayDataOutput);
        msg.channel_names.push_back( atr_name.c_str() );
        msg.n_channels++;
      }
    }
    else if (dataTypeID == VTK_UNSIGNED_CHAR)
    {
      if (atr_name == "rgb_colors")
      {
        vtkUnsignedCharArray* arrayData = vtkUnsignedCharArray::SafeDownCast (polydata->GetPointData ()->GetArray ("rgb_colors"));
        if (arrayData)
        {
          std::vector<float> arrayDataOutput;
          for (size_t i = 0; i < polydata->GetNumberOfPoints (); ++i)
          {
            unsigned char color[3];
            arrayData->GetTupleValue (i, color);
            float colorPacked = packColor(color);
            arrayDataOutput.push_back(colorPacked);
          }
          msg.channels.push_back(arrayDataOutput);
          msg.channel_names.push_back("rgb_colors");
          msg.n_channels++;
        }
      }
    }
  }

  boost::shared_ptr<lcm::LCM> lcm_pub;
  lcm_pub = boost::shared_ptr<lcm::LCM>(new lcm::LCM);
  if(!lcm_pub->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  lcm_pub->publish("POINTCLOUD",&msg);
  std::cout << "Writing to lcm" << std::endl;

  return (0);
}

