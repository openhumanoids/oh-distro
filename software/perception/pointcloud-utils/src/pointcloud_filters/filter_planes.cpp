#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>

#include "filter_planes.hpp"
#define DO_TIMING_PROFILE 0


// master color list - taken from collections_renderer.cpp
float colors[] = {
  1.0, 0.0, 0.0, // red
  0.0, 1.0, 0.0, // green
  0.0, 0.0, 1.0, // blue
  1.0, 1.0, 0.0,
  1.0, 0.0, 1.0,
  0.0, 1.0, 1.0,
  0.5, 1.0, 0.0,
  1.0, 0.5, 0.0,
  0.5, 0.0, 1.0,
  1.0, 0.0, 0.5,
  0.0, 0.5, 1.0,
  0.0, 1.0, 0.5,
  1.0, 0.5, 0.5,
  0.5, 1.0, 0.5,
  0.5, 0.5, 1.0,
  0.5, 0.5, 1.0,
  0.5, 1.0, 0.5,
  0.5, 0.5, 1.0
};
const int num_colors = sizeof(colors)/(3*sizeof(float));


FilterPlanes::FilterPlanes () {
  verbose_lcm =3;
  verbose_text =0;//0;
  pose_element_id =-1;
  pose_coll_id =-1;
  
  // works well with kinect data
  distance_threshold_=0.045; 
  stop_proportion_=0.1; //
  stop_cloud_size_ = 100;
  
}


void FilterPlanes::filterPlanes(vector<BasicPlane> &plane_stack){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  if (verbose_text>0){
    cout << "PointCloud before filtering: " << cloud->width * cloud->height << " data points." << std::endl;
  }
  
  #if DO_TIMING_PROFILE
    vector<int64_t> tic_toc;
    tic_toc.push_back(bot_timestamp_now());
  #endif

/*  Ptcoll_cfg ptcoll_cfg;
  ptcoll_cfg.point_lists_id =pose_element_id; //bot_timestamp_now();
  ptcoll_cfg.collection = pose_coll_id;
  ptcoll_cfg.element_id = pose_element_id; */
  
  //1. Downsample the dataset using a leaf size of 1cm
  // this is 99% of the cpu time
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud);
  // for table dataset:  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.setLeafSize (0.05, 0.05, 0.05);
//  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (*cloud_filtered);
  if (verbose_text>0){
    cout << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
  }
  
  #if DO_TIMING_PROFILE
  tic_toc.push_back(bot_timestamp_now());
  #endif
  
  if (verbose_lcm>2){
/*  ptcoll_cfg.id = 200;
  ptcoll_cfg.reset=true;
  ptcoll_cfg.name ="cloud_downsampled";
  ptcoll_cfg.npoints =	 cloud_filtered->points.size();
  float colorm_temp0[] ={-1.0,-1.0,-1.0,-1.0};
  ptcoll_cfg.rgba.assign(colorm_temp0,colorm_temp0+4*sizeof(float));  
  ptcoll_cfg.type =1;
  pcdXYZRGB_to_lcm(publish_lcm,ptcoll_cfg, *cloud_filtered);*/
  }
  
  // 2. Set up the Ransac Plane Fitting Object:
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100); // was 4000
  
  seg.setDistanceThreshold (distance_threshold_); // 0.01 for table data set
  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  
  #if DO_TIMING_PROFILE
  tic_toc.push_back(bot_timestamp_now());
  #endif
  
  int n_major = 0, nr_points = cloud_filtered->points.size ();
  //vector<BasicPlaneX> plane_stack;
  BasicPlane one_plane;
  // Extract the primary planes:
  // major planes are the coarse planes extracted via ransac. they might contain disconnected points
  // these are broken up into minor planes which are portions of major planes
  
  if(verbose_lcm > 2){
    for (int i=0;i<7;i++){
      char n_major_char[10];
      sprintf(n_major_char,"%d",i);
 /*     ptcoll_cfg.id =210+ i+3;
      ptcoll_cfg.reset=true;
      ptcoll_cfg.name =(char*) "cloud_p ";
      ptcoll_cfg.name.append(n_major_char);
      ptcoll_cfg.npoints = 0;
      float colorm_temp0[] ={-1.0,-1.0,-1.0,-1.0};
      ptcoll_cfg.rgba.assign(colorm_temp0,colorm_temp0+4*sizeof(float));  
      ptcoll_cfg.type=1;
      pcdXYZRGB_to_lcm(publish_lcm,ptcoll_cfg, *cloud_filtered);   
*/
    }
    
    for (int i=0;i<7;i++){
/*
      Ptcoll_cfg ptcoll_cfg2;
      // the i below accounts for super planes in the same utime
      ptcoll_cfg2.point_lists_id =pose_element_id; //bot_timestamp_now();
      ptcoll_cfg2.collection = pose_coll_id;
      ptcoll_cfg2.element_id = pose_element_id;    
      if (i==0){
        ptcoll_cfg2.reset=true;
      }else{
        ptcoll_cfg2.reset=false;
      }
      ptcoll_cfg2.id =500;
      ptcoll_cfg2.name.assign("Grown Stack Final");  
      ptcoll_cfg2.npoints = 0;
      ptcoll_cfg2.type =1;      
      pcdXYZRGB_to_lcm(publish_lcm,ptcoll_cfg2, *cloud_filtered);  
*/
    }       

    // TODO: this will rest this object. need to add publish of reset
    // at start of this block and set rese ttofal
    for (int i=0;i<7;i++){
/*
      Ptcoll_cfg ptcoll_cfg2;
      // the i below accounts for super planes in the same utime
      ptcoll_cfg2.point_lists_id =pose_element_id; //bot_timestamp_now();
      ptcoll_cfg2.collection = pose_coll_id;
      ptcoll_cfg2.element_id = pose_element_id;
      if (i==0){
        ptcoll_cfg2.reset=true;
      }else{
        ptcoll_cfg2.reset=false;
      }
      ptcoll_cfg2.id =501;
      ptcoll_cfg2.name.assign("Grown Stack Final Hull");  
      ptcoll_cfg2.npoints = 0;
      ptcoll_cfg2.type =3;
      pcdXYZRGB_to_lcm(publish_lcm,ptcoll_cfg2, *cloud_filtered);  
*/
    }       
  }

  #if DO_TIMING_PROFILE
  tic_toc.push_back(bot_timestamp_now());
  #endif
  
  while (cloud_filtered->points.size () > stop_proportion_ * nr_points) { 
    // While XX% of the original cloud is still there
    char n_major_char[10];
    sprintf(n_major_char,"%d",n_major);
    
    if (n_major >6) {
      if (verbose_text >0){
	std::cout << n_major << " is the max number of planes to find, quitting" << std::endl;
      }
      break; 
    }

    //a Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }
    
 
    if (inliers->indices.size () <  stop_cloud_size_) // stop when the plane is only a few points
    {
      //std::cerr << "No remaining planes in this set" << std::endl;
      break;
    }    
    
    //b Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    if (verbose_text>0){
      std::cout << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
    }
    
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_p);
    sor.setMeanK (30);
    sor.setStddevMulThresh (0.5); //was 1.0
    sor.filter (*cloud_p);

    if(verbose_lcm > 2){
/*
      ptcoll_cfg.id =210+ n_major+3;
      ptcoll_cfg.reset=true;
      ptcoll_cfg.name ="cloud_p ";
      ptcoll_cfg.name.append(n_major_char);
      ptcoll_cfg.npoints = cloud_p->points.size();
      float colorm_temp0[] ={-1.0,-1.0,-1.0,-1.0};
      ptcoll_cfg.rgba.assign(colorm_temp0,colorm_temp0+4*sizeof(float));  
      ptcoll_cfg.type=1;
      pcdXYZRGB_to_lcm(publish_lcm,ptcoll_cfg, *cloud_p); 
*/   
    }

    vector<BasicPlane> grown_stack;
    vector<BasicPlane> * grown_stack_ptr;
    grown_stack_ptr = &grown_stack;
    
    GrowCloud grow;
    grow.setInputCloud(cloud_p);
    grow.setMinCloudSize(stop_cloud_size_); // useing stop cloud size here too
    grow.setLCM(publish_lcm);
    grow.doGrowCloud(*grown_stack_ptr);
    
    if (verbose_text>0){
      cout << "grow_cloud new found " << grown_stack.size() << " seperate clouds\n";
    }
	
    // Spit raw clouds out to LCM:
    if(verbose_lcm > 2){
      for (int i=0;i<grown_stack.size();i++){
/*
	Ptcoll_cfg ptcoll_cfg2;
	ptcoll_cfg2.reset=false;
	// the i below accounts for super planes in the same utime
	ptcoll_cfg2.point_lists_id =10*n_major + i; //filterplanes->pose_element_id;
	ptcoll_cfg2.collection = pose_coll_id;
	ptcoll_cfg2.element_id = pose_element_id;    

	ptcoll_cfg2.id =500;
	ptcoll_cfg2.name.assign("Grown Stack Final");  
	ptcoll_cfg2.npoints = grown_stack[i].cloud.points.size ();
	ptcoll_cfg2.type =1;
	
	int id =  plane_stack.size() + i; //   10*n_major + i;
	float colorm_temp0[] ={colors[3*(id%num_colors)],colors[3*(id%num_colors)+1],colors[3*(id%num_colors)+2] ,0.0};
	ptcoll_cfg2.rgba.assign(colorm_temp0,colorm_temp0+4*sizeof(float));  
	pcdXYZRGB_to_lcm(publish_lcm,ptcoll_cfg2, grown_stack[i].cloud);  
*/
      }  
    }
    
    BasicPlane one_plane;
    int n_minor=0;
    BOOST_FOREACH( BasicPlane one_plane, grown_stack ){
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
      
      // c Project the model inliers (seems to be necessary to fitting convex hull
      pcl::ProjectInliers<pcl::PointXYZRGB> proj;
      proj.setModelType (pcl::SACMODEL_PLANE);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB> ());
      *temp = (one_plane.cloud);
      proj.setInputCloud (temp);
      proj.setModelCoefficients (coefficients);
      proj.filter (*cloud_projected);
      
      std::vector <pcl::Vertices> vertices;
      if (1==1){ // convex:
        pcl::ConvexHull<pcl::PointXYZRGB> chull;
        chull.setInputCloud (cloud_projected);
        chull.setDimension(2);
        chull.reconstruct (*cloud_hull,vertices);
      }else { // concave
        pcl::ConcaveHull<pcl::PointXYZRGB> chull;
        chull.setInputCloud (cloud_projected);
        chull.setKeepInformation(true);
        chull.setAlpha(0.5);  
        // for arch way:
        // 1.1 too few
        // 0.7 a little to few but much better
        chull.reconstruct (*cloud_hull,vertices);
      }
      
      //std::cout << "Hull has: " << cloud_hull->points.size () << " vertices." << std::endl;
      if (cloud_hull->points.size () ==0){
        cout <<"ERROR: CONVEX HULL HAS NO POINTS! - NEED TO RESOLVE THIS\n"; 
      }
      
      
      // d.2 Find the mean colour of the hull:
      int rgba[]={0,0,0,0};
      for(int i=0;i<temp->points.size();i++){
	int rgba_one = *reinterpret_cast<int*>(&temp->points[i].rgba);
	rgba[3] += ((rgba_one >> 24) & 0xff);
	rgba[2] += ((rgba_one >> 16) & 0xff);
	rgba[1] += ((rgba_one >> 8) & 0xff);
	rgba[0] += (rgba_one & 0xff);      
      }
      double scale = ((double) temp->points.size());
      rgba[3] =(int)  round(((double) rgba[3]) / scale);
      rgba[2] =(int)  round(((double) rgba[2]) / scale);
      rgba[1] =(int)  round(((double) rgba[1]) / scale);
      rgba[0] =(int)  round(((double) rgba[0]) / scale);
      
      // Write the plane to file for experimentation:
      //stringstream oss;
      //oss << ptcoll_cfg.element_id <<"_" << ptcoll_cfg.name << ".pcd";
      //writer.write (oss.str(), *this_hull, false);
      for(int i=0;i<cloud_hull->points.size();i++){
	unsigned char* rgba_ptr = (unsigned char*)&cloud_hull->points[i].rgba;
	(*rgba_ptr) =  rgba[0]  ;
	(*(rgba_ptr+1)) = rgba[1];
	(*(rgba_ptr+2)) = rgba[2];
	(*(rgba_ptr+3)) = rgba[3];	
	
      }
      
      (one_plane.coeffs) = *coefficients;
      one_plane.cloud = *cloud_hull;
      one_plane.utime = pose_element_id;
      one_plane.major = n_major;
      one_plane.minor = n_minor;
      one_plane.n_source_points = cloud_projected->points.size();
      
      
      plane_stack.push_back(one_plane);      
      n_minor++;
  
    //  int stop;
    //  cout << "int to cont: ";
    //  cin >> stop;      
    }
    
    
    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_filtered);
    n_major++;
  }
  
  #if DO_TIMING_PROFILE
  tic_toc.push_back(bot_timestamp_now());
  #endif
  
  if(verbose_lcm > 2){
    // Spit raw clouds out to LCM:
    for (int i=0;i<plane_stack.size();i++){
/*      Ptcoll_cfg ptcoll_cfg2;
      if (i==0){
        ptcoll_cfg2.reset=true;
      }else{
        ptcoll_cfg2.reset=false;
      }
      // the i below accounts for super planes in the same utime
      ptcoll_cfg2.point_lists_id =i; //filterplanes->pose_element_id;
      ptcoll_cfg2.collection = pose_coll_id;
      ptcoll_cfg2.element_id = pose_element_id;   
      ptcoll_cfg2.id =501;
      ptcoll_cfg2.name.assign("Grown Stack Final Hull");  
      ptcoll_cfg2.npoints = plane_stack[i].cloud.points.size ();
      ptcoll_cfg2.type =3;
      
      int id =i;// 10*n_major + i;
      float colorm_temp0[] ={colors[3*(id%num_colors)],colors[3*(id%num_colors)+1],colors[3*(id%num_colors)+2] ,0.0};
      ptcoll_cfg2.rgba.assign(colorm_temp0,colorm_temp0+4*sizeof(float));  
      pcdXYZRGB_to_lcm(publish_lcm,ptcoll_cfg2, plane_stack[i].cloud);        
*/
    }

/*    ptcoll_cfg.id = 201;
    ptcoll_cfg.reset=true;
    ptcoll_cfg.type= 1;
    ptcoll_cfg.name ="cloud_filtered remainder";
    ptcoll_cfg.npoints =	 cloud_filtered->points.size();
    pcdXYZRGB_to_lcm(publish_lcm,ptcoll_cfg, *cloud_filtered);    
*/
  }
  
  if (verbose_text>0){
    std::cout << "[IN ] number of planes extracted: " << plane_stack.size() << "\n";
  }
  
  #if DO_TIMING_PROFILE
    tic_toc.push_back(bot_timestamp_now());
    display_tic_toc(tic_toc,"filter_planes");
  #endif  
  
}
