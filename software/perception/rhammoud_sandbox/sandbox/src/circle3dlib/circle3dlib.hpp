#ifndef CIRCLE3D_
#define CIRCLE3D_

#include <boost/shared_ptr.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


class Circle3D
{
  public:
	
    Circle3D ();

    void setWidth(int width);

    bool doCircle3D(pcl::PointCloud<pcl::PointXYZ> &cloud);
    

  private:

    int width_;
    int height_;
};




#endif
