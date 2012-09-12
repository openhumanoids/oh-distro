#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>

#include "filter_cloudtolaser.hpp"
#include <boost/concept_check.hpp>

FilterCloudToLaser::FilterCloudToLaser() {
  verbose_lcm =0;
  verbose_text =0;
  pose_element_id =-1;
  pose_coll_id =-1;
  max_height = -1000.0; // implausable low height
  max_height = 1000.0; // implausable high height
  
  // from -45->45 in steps of 1 degree
  rad0 = (-M_PI/4); //angle_min
  radstep = (M_PI/180.0); //angle_increment
  radmax = (M_PI/4); //angle_max
  range_max = 30; // dummy holder value
  nranges = std::ceil((radmax - rad0) / radstep);


  // If you change shwill need to ues realloc
  ranges = (float *) malloc(nranges*sizeof(float));
  for (int i=0;i<nranges;i++){
    ranges[i] = range_max;
  }  
}

FilterCloudToLaser::~FilterCloudToLaser () {
//  free(ranges);
}


bool FilterCloudToLaser::doCloud2Laser(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outcloud,bot_core_planar_lidar_t &outlidar){
  //(lcm_t *lcm, LightFilter *lightfilter,PointCloudPtr &cloud,PointCloudPtr &cloud_light_filtered){  
    
  int filter_nan=1; // @param: to filter out the nan points or not
    
  // Duplicate the PC to do light filtering: might be better to just collect the indices and use them
  outcloud->width    = incloud->points.size();
  outcloud->height   = 1;
  outcloud->is_dense = false;
  outcloud->points.resize (outcloud->width * outcloud->height);

  //bot_core_planar_lidar_t outlidar;
  outlidar.utime = 0;
  outlidar.nranges = nranges;
  outlidar.rad0 = rad0;
  outlidar.radstep = radstep;
  outlidar.nintensities =0;
  outlidar.intensities =NULL;

  
  
  int Npts_out=0;
  for(int j3x=0; j3x <incloud->points.size(); j3x++ ) {  //l2r state->width 640
      if (incloud->points[j3x].x < 0){ // remove all points behind camera (null ranges)
	//continue;
      //}else if (v > 473){ // remove a black edge on the bottom of the point cloud:
      }else if (incloud->points[j3x].z > max_height){
	//continue;
      }else if (incloud->points[j3x].z < min_height){
	//continue;
      }else{
	
	bool nan_point =false;
	if (filter_nan==1){
	  if (pcl_isnan (incloud->points[j3x].x)){
	    nan_point=true;
	  }else if(pcl_isnan (incloud->points[j3x].y)){
	    nan_point=true;
	  }else if(pcl_isnan (incloud->points[j3x].z)){	  
	    nan_point=true;
	  }
	}
	
	if (!nan_point){
	  double x =incloud->points[j3x].x;
	  double y =incloud->points[j3x].y;
	  double z =incloud->points[j3x].z;
	  
	  outcloud->points[Npts_out].x =x;
	  outcloud->points[Npts_out].y =y;
	  outcloud->points[Npts_out].z =z;
	  outcloud->points[Npts_out].rgba =incloud->points[j3x].rgba;	  
	  Npts_out++;
	  
	  double angle = atan2(y, x);
	  int index = (angle - rad0) / radstep;
	  double range_sq = y*y+x*x;
	  if (ranges[index] * ranges[index] > range_sq){
	    ranges[index] = sqrt(range_sq);
	  }	  
	}
      }
  }

  outcloud->points.resize (Npts_out);
  outcloud->width    = (Npts_out);  
  
  outlidar.ranges = ranges;
  //bot_core_planar_lidar_t_publish(publish_lcm,"KINECT_LIDAR",&outlidar); 
  //free(ranges);
}
