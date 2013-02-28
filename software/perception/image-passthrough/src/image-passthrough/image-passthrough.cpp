#include "image-passthrough.hpp"
#include <boost/assign/std/vector.hpp>

using namespace boost::assign; // bring 'operator+=()' into scope

#define LINK_OFFSET 128
#define DO_TIMING_PROFILE FALSE

SimExample::SimExample(int argc, char** argv,
	int height,int width, boost::shared_ptr<lcm::LCM> &lcm_,
        int output_color_mode_):
        height_(height), width_(width), lcm_(lcm_), output_color_mode_(output_color_mode_){

  initializeGL (argc, argv);
  
  // 1. construct member elements:
  camera_ = Camera::Ptr (new Camera ());
  scene_ = Scene::Ptr (new Scene ());

  //rl_ = RangeLikelihoodGLSL::Ptr(new RangeLikelihoodGLSL(1, 1, height, width, scene_, 0));
  rl_ = RangeLikelihood::Ptr (new RangeLikelihood (1, 1, height, width, scene_));
  // rl_ = RangeLikelihood::Ptr(new RangeLikelihood(10, 10, 96, 96, scene_));
  // rl_ = RangeLikelihood::Ptr(new RangeLikelihood(1, 1, height_, width_, scene_));

  // Actually corresponds to default parameters:
  rl_->setCameraIntrinsicsParameters (width_,height_, 576.09757860,
            576.09757860, 321.06398107, 242.97676897);
  rl_->setCameraDepthLimits (0.05, 20.0); // kinect = 0.7, 20.0
  rl_->setComputeOnCPU (false);
  rl_->setSumOnCPU (true);
  rl_->setUseColor (true);  
  
  if (1==0){ // set world at launch:
  // 2. read mesh and setup model:
  std::cout << "About to read: " << argv[2] << std::endl;
  pcl::PolygonMesh combined_mesh;	// (new pcl::PolygonMesh);
  pcl::io::loadPolygonFile (argv[2], combined_mesh);
  pcl::PolygonMesh::Ptr combined_mesh_ptr_temp (new pcl::PolygonMesh (combined_mesh));
  combined_mesh_ptr_ = combined_mesh_ptr_temp;
  
  // Not sure if PolygonMesh assumes triangles if to, TODO: Ask a developer
  PolygonMeshModel::Ptr model = PolygonMeshModel::Ptr (new PolygonMeshModel (GL_POLYGON, combined_mesh_ptr_));
  scene_->add (model);
  
  std::cout << "Just read " << argv[2] << std::endl;
  std::cout << combined_mesh.polygons.size () << " polygons and "
	    << combined_mesh.cloud.data.size () << " triangles\n";  
  }
	    
  // works well for MIT CSAIL model 3rd floor:
  //camera_->set(4.04454, 44.9377, 1.1, 0.0, 0.0, -2.00352);

  // works well for MIT CSAIL model 2nd floor:
//  camera_->set (27.4503, 37.383, 4.30908, 0.0, 0.0654498, -2.25802);

  // works for small files:
  //camera_->set(-5.0, 0.0, 1.0, 0.0, 0.0, 0.0);
  camera_->set(0.471703, 1.59862, 3.10937, 0, 0.418879, -12.2129);
  camera_->setPitch(0.418879); // not sure why this is here:
  
  for (int i=0; i<2048; i++)
  {
    float v = i/2048.0;
    v = powf(v, 3)* 6;
    t_gamma[i] = v*6*256;
  }  
  
  
  // Duplicates the list in collections renderer:
  colors_+= 
      51, 160, 44,  //0
      166, 206, 227,
      178, 223, 138,//6
      31, 120, 180,
      251, 154, 153,// 12
      227, 26, 28,
      253, 191, 111,// 18
      106, 61, 154,
      255, 127, 0, // 24
      202, 178, 214,
      255,   0,   0, // red // 30
        0, 255,   0, // green
        0,   0, 255, // blue// 36
      255, 255,   0,
      255,   0, 255, // 42
        0, 255, 255,
      127, 255,   0,
      255, 127,   0,
      127,   0, 255,
      255,   0, 127,
        0, 127, 255,
        0, 255, 127,
      255, 127, 127,
      127, 255, 127,
      127, 127, 255,
      127, 127, 255,
      127, 255, 127,
      127, 127, 255;  
  
}

// display_tic_toc: a helper function which accepts a set of 
// timestamps and displays the elapsed time between them as 
// a fraction and time used [for profiling]
void display_tic_toc(std::vector<int64_t> &tic_toc,const std::string &fun_name){
  int tic_toc_size = tic_toc.size();
  
  double percent_tic_toc_last = 0;
  double dtime = ((double) (tic_toc[tic_toc_size-1] - tic_toc[0])/1000000);
  cout << "fraction_" << fun_name << ",";  
  for (int i=0; i < tic_toc_size;i++){
    double percent_tic_toc = (double) (tic_toc[i] - tic_toc[0])/(tic_toc[tic_toc_size-1] - tic_toc[0]);
    cout <<  percent_tic_toc - percent_tic_toc_last << ", ";
    percent_tic_toc_last = percent_tic_toc;
  }
  cout << "\ntime_" << fun_name << ",";
  double time_tic_toc_last = 0;
  for (int i=0; i < tic_toc_size;i++){
    double percent_tic_toc = (double) (tic_toc[i] - tic_toc[0])/(tic_toc[tic_toc_size-1] - tic_toc[0]);
    cout <<  percent_tic_toc*dtime - time_tic_toc_last << ", ";
    time_tic_toc_last = percent_tic_toc*dtime;
  }
  cout << "\ntotal_time_" << fun_name << " " << dtime << "\n";  
  
}


void SimExample::setCameraIntrinsicsParameters (int camera_width_in, int camera_height_in, 
        float camera_fx_in, float camera_fy_in,
        float camera_cx_in, float camera_cy_in){
  height_ =camera_height_in;
  width_ =camera_width_in;
  rl_->setCameraIntrinsicsParameters (camera_width_in, camera_height_in, 
        camera_fx_in, camera_fy_in,
        camera_cx_in, camera_cy_in);
}



Eigen::Isometry3f SimExample::isometryDoubleToFloat(Eigen::Isometry3d pose_in){
  Eigen::Quaterniond r(pose_in.rotation());
  Eigen::Quaternionf rf(r.w() , r.x() , r.y() , r.z() );
  //Eigen::Quaternionf rf(r.x() , r.y() , r.y() , r.z() );

  Eigen::Isometry3f pose_out;
  pose_out.setIdentity();
  pose_out.translation()  << pose_in.translation().x() , pose_in.translation().y() , pose_in.translation().z();
  pose_out.rotate(rf);

  return pose_out;
}

void 
SimExample::initializeGL (int argc, char** argv)
{
  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);// was GLUT_RGBA
  glutInitWindowPosition (10, 10);
  glutInitWindowSize (10, 10);
  //glutInitWindowSize (window_width_, window_height_);
  glutCreateWindow ("OpenGL range likelihood");

  GLenum err = glewInit ();
  if (GLEW_OK != err)
  {
    std::cerr << "Error: " << glewGetErrorString (err) << std::endl;
    exit (-1);
  }

  std::cout << "Status: Using GLEW " << glewGetString (GLEW_VERSION) << std::endl;
  if (glewIsSupported ("GL_VERSION_2_0"))
    std::cout << "OpenGL 2.0 supported" << std::endl;
  else
  {
    std::cerr << "Error: OpenGL 2.0 not supported" << std::endl;
    exit(1);
  }
  
  std::cout << "GL_MAX_VIEWPORTS: " << GL_MAX_VIEWPORTS << std::endl;
  const GLubyte* version = glGetString (GL_VERSION);
  std::cout << "OpenGL Version: " << version << std::endl;  
}

pcl::PolygonMesh::Ptr getPolygonMesh(std::string filename){
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFile(  filename    ,mesh);
  pcl::PolygonMesh::Ptr mesh_ptr (new pcl::PolygonMesh(mesh));
  cout << "read in :" << filename << "\n"; 
  //state->model = mesh_ptr;  
  return mesh_ptr;
}


void SimExample::setPolygonMeshColor( pcl::PolygonMesh::Ptr &mesh, int r,int g, int b ){
  pcl::PointCloud<pcl::PointXYZRGB> mesh_cloud_1st;  
  pcl::fromROSMsg(mesh->cloud, mesh_cloud_1st);

  for (size_t i=0;i< mesh_cloud_1st.points.size() ; i++){
    mesh_cloud_1st.points[i].r = r;
    mesh_cloud_1st.points[i].g = g;
    mesh_cloud_1st.points[i].b = b;
  }
      
  // transform
  pcl::toROSMsg (mesh_cloud_1st, mesh->cloud);      
}

bool SimExample::mergePolygonMesh(pcl::PolygonMesh::Ptr &meshA, pcl::PolygonMesh::Ptr meshB){
  pcl::PointCloud<pcl::PointXYZRGB> cloudA;  
  // HACKY BUG FIX: 
  // issue: if meshA->cloud contains no data, then it contains no cloud.fields
  //        so it will complain and give a warning when we try to copy to cloudA
  //        Failed to find match for field 'x'.
  //        Failed to find match for field 'y'.
  //        Failed to find match for field 'z'.
  //        Failed to find match for field 'rgb'.  
  // Instead dont try to copy if empty...
  if ( meshA->cloud.fields.size()  !=0){
    pcl::fromROSMsg(meshA->cloud, cloudA);
  }
  
  int original_size = cloudA.points.size() ;

  //cout << original_size << " is the cloud before (insize) size\n";
  //cout <<  meshA->polygons.size () << "polygons before\n";
  
  int N_polygonsB = meshB->polygons.size ();
  pcl::PointCloud<pcl::PointXYZRGB> cloudB;  
  pcl::fromROSMsg(meshB->cloud, cloudB);
  Eigen::Vector4f tmp;
  for(size_t i=0; i< N_polygonsB; i++){ // each triangle/polygon
    pcl::Vertices apoly_in = meshB->polygons[i];//[i];
    int N_points = apoly_in.vertices.size ();
    for(size_t j=0; j< N_points; j++){ // each point
      // increment the vertex numbers by the size of the original clouds
      apoly_in.vertices[j] += original_size; 
    }
    meshA->polygons.push_back(apoly_in);
  } 
  
  cloudA += cloudB;
  
  pcl::toROSMsg (cloudA, meshA->cloud);

  //cout <<  meshA->polygons.size () << "polygons after\n";
  //cout << cloudA.points.size() << " is the cloud inside size\n";
  
  return true;
}


// cat meshB onto combined_mesh_ptr_
void SimExample::mergePolygonMeshToCombinedMesh( pcl::PolygonMesh::Ptr meshB){
  mergePolygonMesh(combined_mesh_ptr_,meshB);  
}



void SimExample::setPolygonMeshs (std::vector< std::string > link_names_in,
                                  std::vector< std::string > file_paths_in){
  
  for(size_t i=0; i < link_names_in.size() ; i++){ 
    PolygonMeshStruct mesh_struct;
    mesh_struct.link_name = link_names_in[i];
    mesh_struct.file_path = file_paths_in[i];
    mesh_struct.polygon_mesh = getPolygonMesh(mesh_struct.file_path);          
    
    
    if(output_color_mode_==0){
      // Set the mesh to a false color:
      int j =i%(colors_.size()/3);
      setPolygonMeshColor(mesh_struct.polygon_mesh, colors_[j*3], colors_[j*3+1], colors_[j*3+2] );
    }else if(output_color_mode_==1){
      setPolygonMeshColor(mesh_struct.polygon_mesh, LINK_OFFSET + (int) i ,
                          0,0 );
    }else{      
      setPolygonMeshColor(mesh_struct.polygon_mesh, 255,0,0 ); // last two are not used
    }
    
    polymesh_map_.insert(make_pair( link_names_in[i] , mesh_struct));
  }
    
  std::cout << polymesh_map_.size() << " meshes stored:\n";
  typedef std::map<std::string, PolygonMeshStruct > mesh_map_it;
  for(mesh_map_it::const_iterator it =  polymesh_map_.begin(); it!= polymesh_map_.end(); it++){
    std::cout << it->first<< " link \t [...]"
        << it->second.file_path.substr (  it->second.file_path.size() - 30 ) << " file_path \t " 
        << it->second.polygon_mesh->polygons.size() << " polygons\n";
  }  
}



// same as bot_timestamp_now():
int64_t _timestamp_now()
{
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

void
SimExample::createScene (std::vector<std::string> object_names, 
                         std::vector<Eigen::Isometry3d> object_tfs){
  #if DO_TIMING_PROFILE
    std::vector<int64_t> tic_toc;
    tic_toc.push_back(_timestamp_now());
  #endif

  scene_->clear();
  
  #if DO_TIMING_PROFILE
    tic_toc.push_back(_timestamp_now());
  #endif
    
  pcl::PolygonMesh combined_mesh;
  pcl::PolygonMesh::Ptr combined_mesh_ptr_temp(new pcl::PolygonMesh(combined_mesh));
  combined_mesh_ptr_ = combined_mesh_ptr_temp;
  for (size_t i=0; i < object_names.size() ; i++ ) { 
    // skip the head and hokuyo_link (which cannot be seen and are large meshes
    if ( object_names[i].compare("head") == 0){ 
      continue; 
    }else if ( object_names[i].compare("hokuyo_link") == 0){
      continue; 
    }
      
    
    std::map<std::string, PolygonMeshStruct >::iterator it;
    it=polymesh_map_.find(  object_names[i] );
    if (it == polymesh_map_.end() ){ // the element has a visual link but is not a mesh ... skip for now
      //std::cout <<  object_names[i] << " link not found ====================\n"; 
      continue;
    }
    
    pcl::PolygonMesh::Ptr mesh_ptr_1st_transformed(new pcl::PolygonMesh( *(polymesh_map_.find( object_names[i] )->second.polygon_mesh)  ));
    pcl::PointCloud<pcl::PointXYZRGB> mesh_cloud_1st;  
    pcl::fromROSMsg(mesh_ptr_1st_transformed->cloud, mesh_cloud_1st);

    // transform
    Eigen::Isometry3f pose_f_1st = isometryDoubleToFloat( object_tfs[i] );
    Eigen::Quaternionf pose_quat_1st(pose_f_1st.rotation());
    pcl::transformPointCloud (mesh_cloud_1st, mesh_cloud_1st, pose_f_1st.translation(), pose_quat_1st);  
    pcl::toROSMsg (mesh_cloud_1st, mesh_ptr_1st_transformed->cloud);  
    
    mergePolygonMesh(combined_mesh_ptr_,mesh_ptr_1st_transformed);
  }  
  
  #if DO_TIMING_PROFILE
    tic_toc.push_back(_timestamp_now());
    display_tic_toc(tic_toc,"createScene");
  #endif

}    
    
void
SimExample::addScene (){    
  // NB: using triangles would make things much quicker....
  //  TriangleMeshModel::Ptr combined_model = TriangleMeshModel::Ptr (new TriangleMeshModel (combined_mesh_ptr_));
  PolygonMeshModel::Ptr combined_model = PolygonMeshModel::Ptr (new PolygonMeshModel (GL_POLYGON, combined_mesh_ptr_));
  scene_->add(combined_model);   
}



void
SimExample::doSim (Eigen::Isometry3d pose_in)
{
  // No reference image - but this is kept for compatability with range_test_v2:
  float* reference = new float[rl_->getRowHeight() * rl_->getColWidth()];
  const float* depth_buffer = rl_->getDepthBuffer();
  // Copy one image from our last as a reference.
  for (int i=0, n=0; i<rl_->getRowHeight(); ++i)
  {
    for (int j=0; j<rl_->getColWidth(); ++j)
    {
      reference[n++] = depth_buffer[i*rl_->getWidth() + j];
    }
  }

  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > poses;
  std::vector<float> scores;
  int n = 1;
  poses.push_back (pose_in);
  // additional views:
  //poses.push_back (pose_in);
  //poses.push_back (pose_in);
  //poses.push_back (pose_in);
  rl_->computeLikelihoods (reference, poses, scores);
  /*
  std::cout << "camera: " << camera_->getX ()
       << " " << camera_->getY ()
       << " " << camera_->getZ ()
       << " " << camera_->getRoll ()
       << " " << camera_->getPitch ()
       << " " << camera_->getYaw ()
       << std::endl;
       */
       
  delete [] reference;
}




void
SimExample::write_score_image(const float* score_buffer, std::string fname)
{
  int npixels = rl_->getWidth() * rl_->getHeight();
  uint8_t* score_img = new uint8_t[npixels * 3];

  float min_score = score_buffer[0];
  float max_score = score_buffer[0];
  for (int i=1; i<npixels; i++)
  {
    if (score_buffer[i] < min_score) min_score = score_buffer[i];
    if (score_buffer[i] > max_score) max_score = score_buffer[i];
  }

  for (int y = 0; y <  height_; ++y)
  {
    for (int x = 0; x < width_; ++x)
    {
      int i = y*width_ + x ;
      int i_in= (height_-1 -y) *width_ + x ; // flip up

      float d = (score_buffer[i_in]-min_score)/(max_score-min_score);
      score_img[3*i+0] = 0;
      score_img[3*i+1] = d*255;
      score_img[3*i+2] = 0;
    }
  }

  // Write to file:
  IplImage *cv_ipl = cvCreateImage( cvSize(width_ ,height_), 8, 3);
  cv::Mat cv_mat(cv_ipl);
  cv_mat.data = score_img;
  cv::imwrite(fname, cv_mat);     
  
  delete [] score_img;
}

void
SimExample::write_depth_image(const float* depth_buffer, std::string fname)
{
  int npixels = rl_->getWidth() * rl_->getHeight();
  uint8_t* depth_img = new uint8_t[npixels * 3];

  float min_depth = depth_buffer[0];
  float max_depth = depth_buffer[0];
  for (int i=1; i<npixels; i++)
  {
    if (depth_buffer[i] < min_depth) min_depth = depth_buffer[i];
    if (depth_buffer[i] > max_depth) max_depth = depth_buffer[i];
  }

  for (int y = 0; y <  rl_->getHeight(); ++y)
  {
    for (int x = 0; x < rl_->getWidth(); ++x)
    {
      int i= y*rl_->getWidth() + x ;
      int i_in= (rl_->getHeight()-1 -y) *rl_->getWidth() + x ; // flip up down
    
    
      float zn = 0.7;
      float zf = 20.0;
      float d = depth_buffer[i_in];
      float z = -zf*zn/((zf-zn)*(d - zf/(zf-zn)));
      float b = 0.075;
      float f = 580.0;
      uint16_t kd = static_cast<uint16_t>(1090 - b*f/z*8);
      if (kd < 0) kd = 0;
      else if (kd>2047) kd = 2047;

      int pval = t_gamma[kd];
      int lb = pval & 0xff;
      switch (pval>>8) {
	case 0:
	    depth_img[3*i+2] = 255;
	    depth_img[3*i+1] = 255-lb;
	    depth_img[3*i+0] = 255-lb;
	    break;
	case 1:
	    depth_img[3*i+2] = 255;
	    depth_img[3*i+1] = lb;
	    depth_img[3*i+0] = 0;
	    break;
	case 2:
	    depth_img[3*i+2] = 255-lb;
	    depth_img[3*i+1] = 255;
	    depth_img[3*i+0] = 0;
	    break;
	case 3:
	    depth_img[3*i+2] = 0;
	    depth_img[3*i+1] = 255;
	    depth_img[3*i+0] = lb;
	    break;
	case 4:
	    depth_img[3*i+2] = 0;
	    depth_img[3*i+1] = 255-lb;
	    depth_img[3*i+0] = 255;
	    break;
	case 5:
	    depth_img[3*i+2] = 0;
	    depth_img[3*i+1] = 0;
	    depth_img[3*i+0] = 255-lb;
	    break;
	default:
	    depth_img[3*i+2] = 0;
	    depth_img[3*i+1] = 0;
	    depth_img[3*i+0] = 0;
	    break;
      }
    }
  }

  std::string channel = fname + "_DEPTH";
  int n_colors = 3;
  int isize =rl_->getWidth()*rl_->getHeight();
  
  if (1==1){  
    bot_core_image_t image;
    image.utime =0;
    image.width =rl_->getWidth();
    image.height=rl_->getHeight();
    image.row_stride =n_colors*rl_->getWidth();
    image.pixelformat =BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB;
    image.size =n_colors*isize;
    image.data = depth_img;
    image.nmetadata =0;
    image.metadata = NULL;
    bot_core_image_t_publish( lcm_->getUnderlyingLCM(), channel.c_str(), &image);  
  
  } else {  
    bot_core::image_t lcm_img;
    lcm_img.utime =0;//current_utime;
    lcm_img.width =rl_->getWidth();
    lcm_img.height =rl_->getHeight();
    lcm_img.nmetadata =0;
    lcm_img.row_stride=n_colors*rl_->getWidth();
    lcm_img.pixelformat =bot_core::image_t::PIXEL_FORMAT_RGB;
    lcm_img.size =n_colors*isize;
    //copy(msg->data.begin(), msg->data.end(), singleimage_data);
    //lcm_img.data = *depth_img;//.assign(singleimage_data, singleimage_data + ( n_colors*isize));

    lcm_img.data.assign(depth_img, depth_img + ( n_colors*isize));
    lcm_->publish(channel.c_str(), &lcm_img);
  }  
  

  // Write to file: (with rgb flipped)
  //IplImage *cv_ipl = cvCreateImage( cvSize(width_ ,height_), 8, 3);
  //cv::Mat cv_mat(cv_ipl);
  //cv_mat.data = depth_img;
  //cv::imwrite(fname, cv_mat);     
  
  delete [] depth_img;
}


void
SimExample::write_depth_image_uint(const float* depth_buffer, std::string fname)
{
  int npixels = rl_->getWidth() * rl_->getHeight();
  unsigned short * depth_img = new unsigned short[npixels ];

  float min_depth = depth_buffer[0];
  float max_depth = depth_buffer[0];
  for (int i=1; i<npixels; i++)
  {
    if (depth_buffer[i] < min_depth) min_depth = depth_buffer[i];
    if (depth_buffer[i] > max_depth) max_depth = depth_buffer[i];
  }

  for (int y = 0; y <  height_; ++y)
  {
    for (int x = 0; x < width_; ++x)
    {
      int i= y*width_ + x ;
      int i_in= (height_-1 -y) *width_ + x ; // flip up down
    
      float zn = 0.7;
      float zf = 20.0;
      float d = depth_buffer[i_in];
      
      unsigned short z_new = (unsigned short)  floor( 1000*( -zf*zn/((zf-zn)*(d - zf/(zf-zn)))));
      if (z_new < 0) z_new = 0;
      else if (z_new>65535) z_new = 65535;
      
      if ( z_new < 18000){
	  cout << z_new << " " << d << " " << x << "\n";  
      }
	
      float z = 1000*( -zf*zn/((zf-zn)*(d - zf/(zf-zn))));
      float b = 0.075;
      float f = 580.0;
      uint16_t kd = static_cast<uint16_t>(1090 - b*f/z*8);
      if (kd < 0) kd = 0;
      else if (kd>2047) kd = 2047;

      int pval = t_gamma[kd];
      int lb = pval & 0xff;
      depth_img[i] = z_new;
    }
  }

  // Write to file:
  IplImage *cv_ipl = cvCreateImage( cvSize(width_ ,height_), IPL_DEPTH_16U, 1);
  cv::Mat cv_mat(cv_ipl);
  cv_mat.data =(uchar *) depth_img;
  cv::imwrite(fname, cv_mat);     
  
  delete [] depth_img;
}


void
SimExample::write_rgb_image(const uint8_t* rgb_buffer, std::string fname)
{
  bool do_timing=false;
  std::vector<int64_t> tic_toc;
  if (do_timing){
    tic_toc.push_back(_timestamp_now());
  }
  
  int npixels = rl_->getWidth() * rl_->getHeight();
  int n_colors;
  int8_t pixelformat;
  uint8_t* img_buff;
  if (output_color_mode_==0){
    n_colors = 3;
    img_buff = new uint8_t[npixels * n_colors];
    for (int y = 0; y <  rl_->getHeight(); ++y)
    {
      for (int x = 0; x < rl_->getWidth(); ++x)
      {
        int px= y*rl_->getWidth() + x ;
        int px_in= (rl_->getHeight()-1 -y) *rl_->getWidth() + x ; // flip up down
        img_buff [3* (px) +0] = rgb_buffer[3*px_in+0];
        img_buff [3* (px) +1] = rgb_buffer[3*px_in+1];
        img_buff [3* (px) +2] = rgb_buffer[3*px_in+2];      
      }
    }
    
  }else{
    n_colors=1;
    img_buff = new uint8_t[npixels * n_colors];
    for (int y = 0; y <  rl_->getHeight(); ++y)
    {
      for (int x = 0; x < rl_->getWidth(); ++x)
      {
        int px= y*rl_->getWidth() + x ;
        int px_in= (rl_->getHeight()-1 -y) *rl_->getWidth() + x ; // flip up down
        img_buff [1* (px) +0] = rgb_buffer[3*px_in+0]; // only use the red value
      }
    }
  }
  
  
  
  
  if (do_timing==1){
    tic_toc.push_back(_timestamp_now());
  }
  
  std::string channel = fname + "_COLOR";
  int isize =rl_->getWidth()*rl_->getHeight();

  if (1==1){ 
    bot_core_image_t image;
    image.utime =0;
    image.width =rl_->getWidth();
    image.height=rl_->getHeight();
    image.row_stride =n_colors*rl_->getWidth();
    if (n_colors==1){
      image.pixelformat =BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY;
    }else{
      image.pixelformat =BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB;
    }
    image.size =n_colors*isize;
    image.data = img_buff;
    image.nmetadata =0;
    image.metadata = NULL;
    bot_core_image_t_publish( lcm_->getUnderlyingLCM(), channel.c_str(), &image);  
  
  } else {  
    bot_core::image_t lcm_img;
    lcm_img.utime =0;//current_utime;
    lcm_img.width =rl_->getWidth();
    lcm_img.height =rl_->getHeight();
    lcm_img.nmetadata =0;
    lcm_img.row_stride=n_colors*rl_->getWidth();
    if (n_colors==1){
      lcm_img.pixelformat =bot_core::image_t::PIXEL_FORMAT_GRAY;
    }else{
      lcm_img.pixelformat =bot_core::image_t::PIXEL_FORMAT_RGB;
    }
    lcm_img.size =n_colors*isize;
    //copy(msg->data.begin(), msg->data.end(), singleimage_data);
    //lcm_img.data = *depth_img;//.assign(singleimage_data, singleimage_data + ( n_colors*isize));

    lcm_img.data.assign(img_buff, img_buff + ( n_colors*isize));
    lcm_->publish(channel.c_str(), &lcm_img);
  }

  if (do_timing==1){
    tic_toc.push_back(_timestamp_now());
    display_tic_toc(tic_toc,"write_rgb_image");
  }
  
  

  // Write to file: (with rgb flipped)
  //IplImage *cv_ipl = cvCreateImage( cvSize(rl_->getWidth() ,rl_->getHeight()), 8, 3);
  //cv::Mat cv_mat(cv_ipl);
  //cv_mat.data = img_buff ;
  //cv::imwrite(fname, cv_mat);     
  
  delete [] img_buff;
}


