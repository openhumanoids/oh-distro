#include "clouds_io_utils.h"

int savePlanarCloudCSV (const std::string &file_name, const pcl::PCLPointCloud2 &cloud)
{
  if (cloud.data.empty ())
  {
    PCL_ERROR ("[icp_io_utils:savePlanarCloudCSV] Input point cloud has no data!\n");
    return (-1);
  }
 
  // Open file
  std::ofstream fs;
  //fs.precision (precision);
  fs.open (file_name.c_str ());

  unsigned int nr_points  = cloud.width * cloud.height;
  unsigned int point_size = static_cast<unsigned int> (cloud.data.size () / nr_points);

  // Iterate through the points
  for (unsigned int i = 0; i < nr_points; ++i)
  {
    int xy = 0;
    for (size_t d = 0; d < cloud.fields.size (); ++d)
    {
      int count = cloud.fields[d].count;
      if (count == 0)
        count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
      int c = 0;
      if ((cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
       	cloud.fields[d].name == "x" || cloud.fields[d].name == "y"))
      {
        float value;
        memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
        fs << value;
        if (++xy == 2)
          break;
      }
      fs << " , ";
	}
    if (xy != 2)
    {
	  PCL_ERROR ("[icp_io_utils:savePlanarCloudCSV] Input point cloud has no XY data!\n");
	  return (-2);
	}
	fs << std::endl;
  }

  // Close file
  fs.close ();
  return (0); 
}