#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>

#include "filter_person.hpp"


FilterPerson::FilterPerson () {
  verbose_lcm =0;
  verbose_text =0;
  pose_element_id =-1;
  pose_coll_id =-1;
}


bool FilterPerson::doFilterPerson(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outcloud,double person_mean_out[]){
  int which_floor_pose =1; // using pose at height pitch roll =0
  int kinectcoll_obj_collection;
  int64_t kinectcoll_obj_element_id;  
  if (which_floor_pose==1){
    kinectcoll_obj_collection = pose_coll_id;
    kinectcoll_obj_element_id = pose_element_id;
  }else if (which_floor_pose==2){
    kinectcoll_obj_collection = 3;
    kinectcoll_obj_element_id = pose_element_id;
  }
  
// int floor_obj_collection= 30;
//   int64_t floor_obj_element_id = bot_timestamp_now();
//   vs_obj_collection_t objs;
//   objs.id = floor_obj_collection; 
//   objs.name = "Kinect Pose"; // "Trajectory";
//   objs.type = 1; // a pose
//   objs.reset = 0; // true will delete them from the viewer
//   objs.nobjs = 1;
//   vs_obj_t poses[objs.nobjs];
//   poses[0].id = floor_obj_element_id;
//   poses[0].x = 0;// state->bot_pos[0] ;
//   poses[0].y = 0;// state->bot_pos[1] ;
//   poses[0].z = 0; // state->bot_pos[2] ;
//   poses[0].yaw = 0;// state->bot_rpy[2] ;
//   poses[0].pitch = 0; //state->bot_rpy[1];
//   poses[0].roll = 0; //state->bot_rpy[0];
//   objs.objs = poses;
//   vs_obj_collection_t_publish(state->publish_lcm, "OBJ_COLLECTION", &objs);


  Ptcoll_cfg ptcoll_cfg;
  ptcoll_cfg.reset=true;
  ptcoll_cfg.point_lists_id = kinectcoll_obj_element_id;
  ptcoll_cfg.collection = kinectcoll_obj_collection;
  ptcoll_cfg.element_id = kinectcoll_obj_element_id;
  

  // 2. Build a passthrough filter to remove the person
  pcl::PassThrough<PointT> pass;
  std::cout << "Person Estimate [before]: " << person_mean[0] << ", "
		<< person_mean[1] << ", "
		<< person_mean[2] << "\n";
  
  double *person_mean= person_mean; // x,y,z CoG
  // Size of the person: // x,y,z limits (-/+)
  double person_bb[] = {0.8, 0.8, 1.2}; 


  // 2.1 Take a cloud inside the bounding box:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  pass.setInputCloud (incloud);
  pass.setFilterFieldName ("x"); // distance from camera
  pass.setFilterLimits (person_mean[0] - person_bb[0], person_mean[0] + person_bb[0]);
  pass.filter (*cloud_filtered_ptr);
  
  pass.setInputCloud (cloud_filtered_ptr);
  pass.setFilterFieldName ("y"); // left/right. positive right (i think)
  pass.setFilterLimits (person_mean[1] - person_bb[1], person_mean[1] + person_bb[1]);
  pass.filter (*cloud_filtered_ptr);

  pass.setInputCloud (cloud_filtered_ptr);
  pass.setFilterFieldName ("z"); // height
  pass.setFilterLimits (0.1, person_mean[2] + person_bb[2]); 
  //pass.setFilterLimits (person_mean[2] - person_bb[2], person_mean[2] + person_bb[2]);
  pass.filter (*cloud_filtered_ptr);

  ptcoll_cfg.id = 32;
  ptcoll_cfg.name.assign("Filtered");  
  ptcoll_cfg.type=1;
  ptcoll_cfg.npoints = cloud_filtered_ptr->points.size ();
  float g_temp[] ={0.0,1,0,0};
  ptcoll_cfg.rgba.assign(g_temp,g_temp+4*sizeof(float));
  pcdXYZRGB_to_lcm(publish_lcm,ptcoll_cfg, *cloud_filtered_ptr);  
      
  std::cerr << "Filter Person: PointCloud after filtering has: " << cloud_filtered_ptr->points.size () << " data points." << std::endl;
  //   if (state->verbose_file){
  //   ss.str("");
  //   ss <<  state->file_name.c_str() << "_filtered.pcd";
  //   std::cerr << ss.str() << " written\n";
  //   writer.write (ss.str(), *cloud_filtered_ptr, false);
  //   }
  //   

  // 2.2 Take a cloud outside the bounding box (the remainder)
  // pcl::ConditionAnd<PointT>::Ptr range_cond (new pcl::ConditionAnd<PointT> ());
  // range_cond->addComparison (pcl::FieldComparison<PointT>::Ptr (new pcl::FieldComparison<PointT>("z", pcl::ComparisonOps::LT, 2.5)));
  // range_cond->addComparison (pcl::FieldComparison<PointT>::Ptr (new pcl::FieldComparison<PointT>("z", pcl::ComparisonOps::GT, 0.0)));
  // range_cond->addComparison (pcl::FieldComparison<PointT>::Ptr (new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::GT, -0.5)));
  // range_cond->addComparison (pcl::FieldComparison<PointT>::Ptr (new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::LT, 1.5)));
  // range_cond->addComparison (pcl::FieldComparison<PointT>::Ptr (new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::GT, 4.0)));
  // range_cond->addComparison (pcl::FieldComparison<PointT>::Ptr (new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::LT, 6.0)));
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_remainder_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ConditionOr<PointT>::Ptr range_cond (new pcl::ConditionOr<PointT> ());
  range_cond->addComparison (pcl::FieldComparison<PointT>::Ptr (new pcl::FieldComparison<PointT>("z", pcl::ComparisonOps::GT, person_mean[2] + person_bb[2])));
  range_cond->addComparison (pcl::FieldComparison<PointT>::Ptr (new pcl::FieldComparison<PointT>("z", pcl::ComparisonOps::LT, -0.1)));
  range_cond->addComparison (pcl::FieldComparison<PointT>::Ptr (new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::LT, person_mean[1] - person_bb[1])));
  range_cond->addComparison (pcl::FieldComparison<PointT>::Ptr (new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::GT, person_mean[1] + person_bb[1])));
  range_cond->addComparison (pcl::FieldComparison<PointT>::Ptr (new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::LT, person_mean[0] - person_bb[0])));
  range_cond->addComparison (pcl::FieldComparison<PointT>::Ptr (new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::GT, person_mean[0] + person_bb[0])));
  // Build the filter
  pcl::ConditionalRemoval<PointT> range_filt;
  range_filt.setCondition (range_cond);
  range_filt.setKeepOrganized (false);
  range_filt.setInputCloud(incloud);
  range_filt.filter(*cloud_remainder_ptr);
  
  ptcoll_cfg.id = 33;
  ptcoll_cfg.type=1;
  ptcoll_cfg.name.assign("Filtered Reverse");  
  ptcoll_cfg.npoints = cloud_remainder_ptr->points.size ();
	  float colorm_temp1[] ={0.25,0.25,0.25,0};
	  ptcoll_cfg.rgba.assign(colorm_temp1,colorm_temp1+4*sizeof(float));
//  ptcoll_cfg.rgba ={0.25,0.25,0.25,0};
  ptcoll_cfg.reset = false;
  pcdXYZRGB_to_lcm(publish_lcm,ptcoll_cfg, *cloud_remainder_ptr);    
  

  
  
  
  // REPLACE THIS WITH A CALL TO FILTERPLANES WHEN IT HAS BEEN GENERALIZED BETTER
  // 3. Extract all the planes in the maining cloud (experimental)
  // this should remove the ground plane and any nearby walls
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>) ;
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.02); // 0.01 for table data set
  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  
  int i = 0, nr_points = cloud_filtered_ptr->points.size ();
  // While 50% of the original cloud is still there ... was 30%
  while (cloud_filtered_ptr->points.size () > 0.5 * nr_points)
  {
    if (i >4) {
      std::cerr << "4 is the max number of planes to find, quitting" << std::endl;
     break; 
    }
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered_ptr);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    if (inliers->indices.size () < 800) // stop when the plane is only a few points
    {
      std::cerr << "No remaining planes in this set" << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered_ptr);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    ptcoll_cfg.id =20000+ 1+ i;
    ptcoll_cfg.reset=true;
    ptcoll_cfg.name ="cloud_p ";
    char n_major_char[10];
    sprintf(n_major_char,"%d",i);
    ptcoll_cfg.name.append(n_major_char);
    ptcoll_cfg.npoints =	 cloud_p->points.size();
	  float colorm_temp[] ={0.5,0.5,1.0,0};
	  ptcoll_cfg.rgba.assign(colorm_temp,colorm_temp+4*sizeof(float));
//    ptcoll_cfg.rgba ={0.0,1,1,0};
    pcdXYZRGB_to_lcm(publish_lcm,ptcoll_cfg, *cloud_p);    
    

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_filtered_ptr);

    i++;
  }
  //   ss.str("");
  //   ss <<  file_name.c_str() << "_remainder.pcd";
  //   writer.write<pcl::PointXYZ> (ss.str(), *cloud_filtered_ptr, false);
  ptcoll_cfg.id =20000;
  ptcoll_cfg.reset=false;
  ptcoll_cfg.type=1;
  ptcoll_cfg.name ="cloud_filtered remainder";
  ptcoll_cfg.npoints =	 cloud_filtered_ptr->points.size();
	  float colorm_temp2[] ={0.0,1,1.0,0};
	  ptcoll_cfg.rgba.assign(colorm_temp2,colorm_temp2+4*sizeof(float));
//  ptcoll_cfg.rgba ={0.0,1,1,0};
  pcdXYZRGB_to_lcm(publish_lcm,ptcoll_cfg, *cloud_filtered_ptr);    
  
  
  
  
  // 4. Statistical Outlier Filter (optional)
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud_filtered_ptr);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZRGB>);
  sor.filter (*cloud_filtered2);
  ptcoll_cfg.id = 34;
  ptcoll_cfg.type=1;
  ptcoll_cfg.name.assign("Filtered Outlier-less");  
  ptcoll_cfg.npoints = cloud_filtered2->points.size ();
  float color_temp[] ={0.5,0.5,1.0,0};
  ptcoll_cfg.rgba.assign(color_temp,color_temp+4*sizeof(float));
  //ptcoll_cfg.rgba ={0.5,0.5,1.0,0};
  ptcoll_cfg.reset = true;
  pcdXYZRGB_to_lcm(publish_lcm,ptcoll_cfg, *cloud_filtered2);    
    
  // 5. Find Centre of Mass:
  double com[] ={0,0,0};
  for (int i=0;i< cloud_filtered2->points.size() ;i++){ //
    com[0]+=cloud_filtered2->points[i].x;
    com[1]+=cloud_filtered2->points[i].y;
    com[2]+=cloud_filtered2->points[i].z;
  }
  com[0] = com[0] / cloud_filtered2->points.size();
  com[1] = com[1] / cloud_filtered2->points.size();
  com[2] = com[2] / cloud_filtered2->points.size();
  
  pcl::PointCloud<pcl::PointXYZRGB> cylinder_com;
  cylinder_com.width    = 1;
  cylinder_com.height   = 1;
  cylinder_com.is_dense = false;
  cylinder_com.points.resize (cylinder_com.width * cylinder_com.height);  
  cylinder_com.points[0].x =com[0];
  cylinder_com.points[0].y =com[1];
  cylinder_com.points[0].z =com[2];
  person_mean_out[0] = com[0];
  person_mean_out[1] = com[1];
  person_mean_out[2] = com[2];
  std::cout << "Person Estimate [after]: " << person_mean[0] << ", "
		<< person_mean[1] << ", "
		<< person_mean[2] << "\n";    

  ptcoll_cfg.id = 35;
  ptcoll_cfg.name.assign("Cylinder CoM");  
  ptcoll_cfg.npoints = cylinder_com.points.size ();
	  float colorm_temp3[] ={0.0,0.0,0.0,0};
	  ptcoll_cfg.rgba.assign(colorm_temp3,colorm_temp3+4*sizeof(float));
//  ptcoll_cfg.rgba ={0.0,0,0.0,0};
  ptcoll_cfg.reset = false;
  ptcoll_cfg.type=1;
  pcdXYZRGB_to_lcm(publish_lcm,ptcoll_cfg, cylinder_com); 

  
}
