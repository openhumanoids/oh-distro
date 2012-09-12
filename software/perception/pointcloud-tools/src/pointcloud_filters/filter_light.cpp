#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>

#include "filter_light.hpp"

FilterLight::FilterLight() {
  verbose_lcm =0;
  verbose_text =0;
  pose_element_id =-1;
  pose_coll_id =-1;
  max_range = 0.0; // implausable small filter range
}


bool FilterLight::doLightFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outcloud){
  //(lcm_t *lcm, LightFilter *lightfilter,PointCloudPtr &cloud,PointCloudPtr &cloud_light_filtered){  
    
  int filter_nan=1; // @param: to fiter out the nan points or not
    
  // Duplicate the PC to do light filtering: might be better to just collect the indices and use them
  outcloud->width    = incloud->points.size();
  outcloud->height   = 1;
  outcloud->is_dense = false;
  outcloud->points.resize (outcloud->width * outcloud->height);
  int Nlight=0;

  int decimate_image[] = {1,1};
  if (verbose_text > 0){
    std::cout << "Input to pcd_lightfilter: w: "<< incloud->width << " h " << incloud->height << "\n";
  }
  for(int j3x=0; j3x <incloud->points.size(); j3x++ ) {  //l2r state->width 640
      if (incloud->points[j3x].x > max_range ){ // remove all points 5.5+ m away
      }else if (incloud->points[j3x].x < 0){ // remove all points behind camera (null ranges)
      //}else if (v > 473){ // remove a black edge on the bottom of the point cloud:
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
	  outcloud->points[Nlight].x =incloud->points[j3x].x;
	  outcloud->points[Nlight].y =incloud->points[j3x].y;
	  outcloud->points[Nlight].z =incloud->points[j3x].z;
	  outcloud->points[Nlight].rgba =incloud->points[j3x].rgba;	  
	  Nlight++;
	}
      }
  }

  outcloud->points.resize (Nlight);
  outcloud->width    = (Nlight);  
}