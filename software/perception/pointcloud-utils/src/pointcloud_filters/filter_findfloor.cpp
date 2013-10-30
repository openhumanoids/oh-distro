#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>

#include "filter_findfloor.hpp"


FilterFindfloor::FilterFindfloor () {
  verbose_lcm =0;
  verbose_text =0;
  pose_element_id =-1;
  pose_coll_id =-1;
}


bool FilterFindfloor::getHeightPitchRoll(double height_pitch_roll[]){
  // Take the floor part of the PCD:
  pcl::PointCloud<pcl::PointXYZ> cloudfloor;
  cloudfloor.width    = incloud->points.size();
  cloudfloor.height   = 1;
  cloudfloor.is_dense = false;
  cloudfloor.points.resize (cloudfloor.width * cloudfloor.height);
  // TODO
  // OLD: seem to need a duplicate PC as XYZRGBA cannot be used as ransac input
  // NEW: dec2011: i think this can not be fixed and is not required. mfallon
  
  pcl::PointCloud<pcl::PointXYZRGB> cloudfloorRGB;
  cloudfloorRGB.width    = incloud->points.size();
  cloudfloorRGB.height   = 1;
  cloudfloorRGB.is_dense = false;
  cloudfloorRGB.points.resize (cloudfloorRGB.width * cloudfloorRGB.height);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudfloorRGBptr (new pcl::PointCloud<pcl::PointXYZRGB> (cloudfloorRGB));
  

  if(verbose_text>0){
    std::cout << "No of points into floor filter: " <<  incloud->points.size() << std::endl;
  }
  int Nransac=0;
  for(int j3=0; j3< incloud->points.size(); j3++) { 
   if ((incloud->points[j3].z < -0.75 )&& (incloud->points[j3].x < 5 )) {  // RANSAC head filter
      // -0.75m  for 2011_03_16_cart_stata
      // -1.25m  for 2011_03_14_brookshire
      // either -1.10 -1.00m for me
      cloudfloor.points[Nransac].x =incloud->points[j3].x;
      cloudfloor.points[Nransac].y =incloud->points[j3].y;
      cloudfloor.points[Nransac].z =incloud->points[j3].z;
      //cloudfloor.points[j].rgba =cloud.points[j].rgba;

      cloudfloorRGB.points[Nransac].x =incloud->points[j3].x;
      cloudfloorRGB.points[Nransac].y =incloud->points[j3].y;
      cloudfloorRGB.points[Nransac].z =incloud->points[j3].z;
      //
      cloudfloorRGB.points[Nransac].rgb =incloud->points[j3].rgb;
      //cloudfloorRGB.points[Nransac].rgba =incloud->points[j3].rgba;
      Nransac++;
    }
  }
  cloudfloor.points.resize (Nransac);
  cloudfloor.width    = (Nransac);  
  cloudfloorRGB.points.resize (Nransac);
  cloudfloorRGB.width    = (Nransac);  
  
  if(verbose_text>0){
    std::cout << "No of points into ransac: " << Nransac << std::endl;
  }
  
  pcl::ModelCoefficients coeff;
  pcl::PointIndices inliers;
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01); // was 0.01m
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr (new pcl::PointCloud<pcl::PointXYZ> (cloudfloor));
  seg.setInputCloud (cloudptr);
  seg.segment (inliers, coeff);
  if (inliers.indices.size () == 0)
  {
    printf ("[FilterFindfloor::getHeightPitchRoll] Could not estimate a planar model for the given dataset.\n");
    return -1;
  }
  
  double pitch = atan(coeff.values[0]/coeff.values[2]);
  double roll =- atan(coeff.values[1]/coeff.values[2]);
  double coeff_norm = sqrt(pow(coeff.values[0],2) +
	pow(coeff.values[1],2) + pow(coeff.values[2],2));
  double height = (coeff.values[2]*coeff.values[3]) / coeff_norm;
  // the point directly under the kinect, having corrected the rotation:
  // double sub_pt[3];
  // sub_pt[1]= -coeff.values[0]*coeff.values[3]/pow(coeff_norm,2)
  // sub_pt[2]= -coeff.values[1]*coeff.values[3]/pow(coeff_norm,2)
  // sub_pt[3]= -coeff.values[2]*coeff.values[3]/pow(coeff_norm,2) 

  if (verbose_text>0){
    cout  <<  "\nRANSAC Floor Coefficients: " << coeff.values[0]
      << " " << coeff.values[1] << " "  << coeff.values[2] << " " << coeff.values[3] << endl;
    cout << "Pitch: " << pitch << " (" << (pitch*180/M_PI) << "d). positive nose down\n";
    cout << "Roll : " << roll << " (" << (roll*180/M_PI) << "d). positive right side down\n";
    cout << "Height : " << height << " of device off ground [m]\n";
    cout << "Total points: " << Nransac << ". inliers: " << inliers.indices.size () << endl << endl;
    //std::cout << std::setprecision(20) << "corresponding obj_coll " << state->bot_id
    //  << " and kinect " << state->msg->timestamp << "\n";
  }
  int floor_to_file =0;
  if(floor_to_file==1){
    //pcl::io::savePCDFile ("RANSAC_floorcloud.pcd", cloudfloor, false);
    //std::cout << "about to publish to RANSAC_floorcloud.pcd - " << cloudfloor.points.size () << "pts" << std::endl;
  }
  

  // Filter Ridiculous Values from the Ransac Floor estimator:
  int skip_floor_estimate =0;
  if ((height > 3) || (height < 0.8)){
    skip_floor_estimate =1;
  } 
  if ((pitch  > 10 ) || (height < -10)){
    skip_floor_estimate =1;
  } 
  if ((roll  > 10 ) || (roll < -10)){
    skip_floor_estimate =1;
  }
  if (skip_floor_estimate){
      std::cout << "Ransac has produced ridiculous results, ignore\n";
      height= last_height_pitch_roll[0];
      pitch= last_height_pitch_roll[1];
      roll= last_height_pitch_roll[2];
  }

  
  // use a fixed prior for the DOFs estimated by the kinect: (for testing)
  /*
  int use_fixed_prior=1;
  if (use_fixed_prior){
    roll = -0.5*M_PI/180;// 5*M_PI/180 ;
    pitch = 9.5*M_PI/180;// 5*M_PI/180 ;
    height = 1.2;
  }  
*/


  
  if(verbose_lcm>0){ // important
    // turn inlier points red:
    for (size_t i = 0; i < inliers.indices.size (); ++i){
      size_t this_index = inliers.indices[i];
      unsigned char red[]={255,0,0,0}; // rgba
      unsigned char* rgba_ptr = (unsigned char*)&cloudfloorRGB.points[this_index].rgba;
      (*rgba_ptr) =  red[0];
      (*(rgba_ptr+1)) =  red[1];
      (*(rgba_ptr+2)) =  red[2];
      (*(rgba_ptr+3)) =  red[3];
    }    
    
    int floor_obj_collection;
    int64_t floor_obj_element_id;
    floor_obj_collection= 4;
    floor_obj_element_id = pose_element_id;// bot_timestamp_now();

    /*Ptcoll_cfg floorcoll_cfg;
    floorcoll_cfg.id = 1;
    floorcoll_cfg.name ="Floor RANSAC";  
    floorcoll_cfg.reset=true;
    floorcoll_cfg.type =1;
    floorcoll_cfg.point_lists_id = pose_element_id ;
    floorcoll_cfg.collection = floor_obj_collection;
    floorcoll_cfg.element_id = floor_obj_element_id;
    floorcoll_cfg.npoints = Nransac;
  float colorm_temp0[] ={-1.0,-1.0,-1.0,-1.0};
  floorcoll_cfg.rgba.assign(colorm_temp0,colorm_temp0+4*sizeof(float));
//    floorcoll_cfg.rgba = {-1,-1,-1,-1};
    pcdXYZRGB_to_lcm(publish_lcm,floorcoll_cfg, cloudfloorRGB);  
*/

    if (verbose_lcm > 2){ // unimportant
      // 0 - position (fixed) at 000 with corrected pitch and roll
      vs_obj_collection_t objs;
      objs.id = floor_obj_collection; 
      objs.name = (char*) "Floor Pose"; // "Trajectory";
      objs.type = 1; // a pose
      objs.reset = 0; // true will delete them from the viewer
      objs.nobjs = 1;
      vs_obj_t poses[objs.nobjs];
      poses[0].id = floor_obj_element_id;

      poses[0].x = 0;// state->bot_pos[0] ;
      poses[0].y = 0;// state->bot_pos[1] ;
      poses[0].z = height; // state->bot_pos[2] ;
      poses[0].yaw = 0;// state->bot_rpy[2] ;
      poses[0].pitch = -pitch; //state->bot_rpy[1];
      poses[0].roll = -roll; //state->bot_rpy[0];
      objs.objs = poses;
      vs_obj_collection_t_publish(publish_lcm, "OBJ_COLLECTION", &objs);     

      bot_core_pose_t testdata;
      testdata.utime = pose_element_id;
      testdata.pos[0]=0;
      testdata.pos[1]=0;
      testdata.pos[2]=height;
      Eigen::Quaterniond quat =euler_to_quat(roll,pitch,0); // rpy
      testdata.orientation[0] = quat.w();
      testdata.orientation[1] = quat.x();
      testdata.orientation[2] = quat.y();
      testdata.orientation[3] = quat.z();
//      double temp_rpy[] ={ roll, pitch, 0}; // no yaw estimate
//      bot_roll_pitch_yaw_to_quat(temp_rpy, testdata.orientation) ;
      testdata.vel[0] =0;
      testdata.vel[1] =0;
      testdata.vel[2] =0;
      testdata.rotation_rate[0] =0;
      testdata.rotation_rate[1] =0;
      testdata.rotation_rate[2] =0;
      testdata.accel[0] =0;
      testdata.accel[1] =0;
      testdata.accel[2] =0;
      bot_core_pose_t_publish(publish_lcm,"POSE_FINDGROUND", &testdata);   
    }
  }

  
  
  height_pitch_roll[0] = height;
  height_pitch_roll[1] = pitch;
  height_pitch_roll[2] = roll;
  
  return 1;
}
