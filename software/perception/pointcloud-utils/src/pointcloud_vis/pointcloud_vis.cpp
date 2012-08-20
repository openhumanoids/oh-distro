#include <iostream>

#include <lcm/lcm.h>

#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>

#include "pointcloud_vis.hpp"

#include "visualization/collections.hpp"

//#include "jpeg-utils.h"
//#include "jpeg-utils-ijg.c"
//#include "jpeg-utils-ijg.h"

#include <zlib.h>

#define PCL_VERBOSITY_LEVEL L_ERROR

using namespace std;


//from hordur:
//time x y z qx qy qz qw - all floating points
void read_poses_csv(std::string poses_files, std::vector<Isometry3d_Time>& poses){
  int counter=0;
  string line;
  ifstream myfile (poses_files.c_str());
  if (myfile.is_open()){
    while ( myfile.good() ){
      getline (myfile,line);
      if (line.size() > 4){
	Isometry3d_Time apose;
	double quat[4];
	double pos[3];
	
	//cout << line << "\n";
	//double dtime;
	int64_t dtime;
 	sscanf(line.c_str(),"%lld,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
           &dtime,
           &pos[0],&pos[1],&pos[2],
   	   &quat[1],&quat[2],&quat[3],&quat[0]); // NBNBN: note the order
// 	cout << dtime << " "
// 	<< pos[0] << " "
// 	<< pos[1] << " "
// 	<< pos[2]<< " "
// 	<< quat[0]<< " "
// 	<< quat[1]<< " "
// 	<< quat[2]<< " "
// 	<< quat[3]<< "\n";
	
	apose.utime = (int64_t) (dtime);
//  apose.utime = (int64_t) (dtime*1E6);
// 	cout << apose.utime << "\n\n\n";
	
	Eigen::Quaterniond quat2(quat[0],quat[1],quat[2],quat[3]);
	
	Eigen::Isometry3d odom_dpose;
	apose.p.setIdentity();
	apose.p.translation()  << pos[0] ,pos[1],pos[2];
	apose.p.rotate(quat2);	
	counter++;
 	poses.push_back(apose);
      }
    }
    myfile.close();
  } else{
    printf( "Unable to open poses file\n%s",poses_files.c_str());
    return;
  }    
}



bool ObjU_to_lcm(lcm_t *lcm, Objq_coll_cfg objq_coll_cfg,vector<ObjQ> objq_coll){
    vs_obj_collection_t objs;
    objs.id = objq_coll_cfg.id ; // was: this_id;
    objs.name = (char*)   objq_coll_cfg.name.c_str();
    objs.type = objq_coll_cfg.type;
    objs.reset = objq_coll_cfg.reset;
    objs.nobjs = objq_coll_cfg.nobjs;
    
    vs_obj_t all_objs[objq_coll_cfg.nobjs];
    for (int i = 0; i < objq_coll_cfg.nobjs; i++) {
        all_objs[i].id = objq_coll[i].utime;
        all_objs[i].x =objq_coll[i].p.trans_vec[0];
        all_objs[i].y =objq_coll[i].p.trans_vec[1];
        all_objs[i].z =objq_coll[i].p.trans_vec[2];
	double temp_rpy[3];
	bot_quat_to_roll_pitch_yaw(objq_coll[i].p.rot_quat,temp_rpy);
        all_objs[i].yaw =temp_rpy[2];
        all_objs[i].pitch = temp_rpy[1];
        all_objs[i].roll = temp_rpy[0];
    }
    objs.objs = all_objs;
    vs_obj_collection_t_publish(lcm, "OBJ_COLLECTION", &objs);  
}

void savePLYFile(pcl::PolygonMesh::Ptr model,string fname){
  pcl::PointCloud<pcl::PointXYZRGB> bigcloud;  
  pcl::fromROSMsg(model->cloud, bigcloud);
  Eigen::Vector4f tmp;
  
  FILE * fid;
  int n;
  char name [100];
  fid = fopen (fname.c_str(),"w");

  fprintf (fid, "ply\n");
  fprintf (fid, "format ascii 1.0\n");
  fprintf (fid, "comment native PCL export\n");
  fprintf(fid,"element vertex %d\n", bigcloud.points.size());
  fprintf(fid,"property float x\n");
  fprintf(fid,"property float y\n");
  fprintf(fid,"property float z\n");
  fprintf(fid,"property uchar red\n");
  fprintf(fid,"property uchar green\n");
  fprintf(fid,"property uchar blue\n");
  fprintf(fid,"property uchar alpha\n");

  fprintf(fid,"element face %d\n", model->polygons.size());
  fprintf(fid,"property list uchar int vertex_indices\n");
  fprintf(fid,"end_header\n");
   
  for(size_t i=0; i<bigcloud.size() ; i++){ // each triangle/polygon
    tmp = bigcloud.points[i].getVector4fMap();
    
    stringstream ss;// (stringstream::in | stringstream::out);
    ss << tmp(0) << " "
       << tmp(1) << " "
       << tmp(2) << " "
       << (int) bigcloud.points[i].r<< " "
       << (int)  bigcloud.points[i].g << " "
       << (int)  bigcloud.points[i].b << " 255";
      
    //cout << ss.str() << "\n";
    fprintf(fid,"%s\n", ss.str().c_str());
  }     
  
  for(size_t i=0; i< model->polygons.size (); i++){ // each triangle/polygon
    pcl::Vertices apoly_in = model->polygons[i];
    stringstream ss;// (stringstream::in | stringstream::out);
    int nvert= apoly_in.vertices.size ();
    ss << nvert ;
    for(size_t j=0; j< apoly_in.vertices.size (); j++){ // each point 
      ss << " " << apoly_in.vertices[j] ;
    }
    //cout << ss.str() << "\n";
    fprintf(fid,"%s\n", ss.str().c_str());
  }
  fclose(fid);
}


void remove_colored_polygons(pcl::PolygonMesh::Ptr meshin_ptr,vector<int> &color ){
  int N_polygons = meshin_ptr->polygons.size ();
  pcl::PointCloud<pcl::PointXYZRGB> bigcloud;  
  pcl::fromROSMsg(meshin_ptr->cloud, bigcloud);
  
  vector<int> polygon_indices;
  
  for(size_t i=0; i< N_polygons; i++){ // each triangle/polygon
    pcl::Vertices apoly_in = meshin_ptr->polygons[i];
    int N_points = apoly_in.vertices.size ();
    {
      size_t j=0;//for the 1st point in the polygon
      uint32_t pt = apoly_in.vertices[j];
      int in_mesh = (int)i; // inside 3D box

      vector <float> this_color;
      this_color.push_back( bigcloud.points[pt].r);
      this_color.push_back( bigcloud.points[pt].g);
      this_color.push_back( bigcloud.points[pt].b);
      bool match=false;
      if (color[0] == this_color[0]){
      if (color[1] == this_color[1]){
      if (color[2] == this_color[2]){
	match=true;
      }
      }
      }
      if(!match){
	polygon_indices.push_back(in_mesh);
      }

    }
  }
  
  vector <pcl::Vertices> new_verts;
  for(size_t i=0; i< polygon_indices.size(); i++){ // each triangle/polygon
    pcl::Vertices apoly_in = meshin_ptr->polygons[polygon_indices[i]];
    new_verts.push_back(apoly_in);
  }
  meshin_ptr->polygons.clear();
  meshin_ptr->polygons = new_verts;
}



void get_MeshInBoxIndices(pcl::PolygonMesh::Ptr meshin_ptr,
		   vector<double> &center, vector<double> &dgrid,
		   vector<int> &polygon_in_box_indices){
  int N_polygons = meshin_ptr->polygons.size ();
  pcl::PointCloud<pcl::PointXYZRGB> bigcloud;  
  pcl::fromROSMsg(meshin_ptr->cloud, bigcloud);
  Eigen::Vector4f tmp;
  for(size_t i=0; i< N_polygons; i++){ // each triangle/polygon
    pcl::Vertices apoly_in = meshin_ptr->polygons[i];
    int N_points = apoly_in.vertices.size ();
    bool add =0;
    for(size_t j=0; j< N_points; j++){ // each point
      uint32_t pt = apoly_in.vertices[j];
      tmp = bigcloud.points[pt].getVector4fMap(); // xyz 
      // are any points of the polygon in the box, if so add them to the new model
      if ((tmp(0) > center[0] - dgrid[0])&& (tmp(0) < center[0] + dgrid[0])) // widtin x
	if ((tmp(1) > center[1] - dgrid[1])&& (tmp(1) < center[1] + dgrid[1])) // within y
	  if ((tmp(2) > center[2] - dgrid[2])&& (tmp(2) < center[2] + dgrid[2])){ // within z
	     int in_mesh = (int)i; // inside 3D box
	     polygon_in_box_indices.push_back(in_mesh);
	     break;
	  }
    }
  }
  cout << polygon_in_box_indices.size() << " is the polygon_in_box_indices size\n";
}


void get_MeshInCircleIndices(pcl::PolygonMesh::Ptr meshin_ptr,
		   vector<double> &center, double radius,
		   vector<int> &polygon_in_circle_indices){
  int N_polygons = meshin_ptr->polygons.size ();
  pcl::PointCloud<pcl::PointXYZRGB> bigcloud;  
  pcl::fromROSMsg(meshin_ptr->cloud, bigcloud);
  Eigen::Vector4f tmp;
  for(size_t i=0; i< N_polygons; i++){ // each triangle/polygon
    pcl::Vertices apoly_in = meshin_ptr->polygons[i];
    int N_points = apoly_in.vertices.size ();
    bool add =0;
    for(size_t j=0; j< N_points; j++){ // each point
      uint32_t pt = apoly_in.vertices[j];
      tmp = bigcloud.points[pt].getVector4fMap(); // xyz 
      // are any points of the polygon in the circle, if so add them to the new model
      // currently only in 2d
      
      double dist2d = sqrt(pow(tmp(0) - center[0],2) + pow(tmp(1) - center[1],2));
      if (dist2d < radius){
	int in_mesh = (int)i; // inside 3D box
	polygon_in_circle_indices.push_back(in_mesh);
	break;
      }
    }
  }
  cout << polygon_in_circle_indices.size() << " is the polygon_in_circle_indices size\n";
}



void get_MeshInBox(pcl::PolygonMesh::Ptr meshin_ptr,
		   vector<double> &center, vector<double> &dgrid,
		   pcl::PolygonMesh::Ptr &minimesh_ptr){
  int N_polygons = meshin_ptr->polygons.size ();
  pcl::PointCloud<pcl::PointXYZRGB> bigcloud;  
  pcl::fromROSMsg(meshin_ptr->cloud, bigcloud);
  Eigen::Vector4f tmp;
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr minicloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  int N_cloud_pts=0;
  for(size_t i=0; i< N_polygons; i++){ // each triangle/polygon
    pcl::Vertices apoly_in = meshin_ptr->polygons[i];
    int N_points = apoly_in.vertices.size ();
    
    bool add =0;
    for(size_t j=0; j< N_points; j++){ // each point
      uint32_t pt = apoly_in.vertices[j];
      tmp = bigcloud.points[pt].getVector4fMap(); // xyz 
      
      // are any points of the polygon in the box, add them to the new model
      if ((tmp(0) > center[0] - dgrid[0])&& (tmp(0) < center[0] + dgrid[0])) // widtin x
	if ((tmp(1) > center[1] - dgrid[1])&& (tmp(1) < center[1] + dgrid[1])) // within y
	  if ((tmp(2) > center[2] - dgrid[2])&& (tmp(2) < center[2] + dgrid[2])) // within z
	     add=1;	   // inside 3D box
	     
      if (add)
	break;
    }
    
    if (add){
      pcl::Vertices apoly_out;
      for(size_t j=0; j< N_points; j++){ // each point
	uint32_t pt = apoly_in.vertices[j];
	tmp = bigcloud.points[pt].getVector4fMap(); // xyz 
	pcl::PointXYZRGB a_pt= bigcloud.points[pt];
	minicloud->points.push_back(a_pt);
	apoly_out.vertices.push_back(N_cloud_pts);
	N_cloud_pts++;
      }
      minimesh_ptr->polygons.push_back(apoly_out);
    }
  } 
  minicloud->width    = N_cloud_pts;
  minicloud->height   = 1;
  minicloud->is_dense = true;
  minicloud->points.resize (minicloud->width * minicloud->height); // is this necessary?
  cout << minicloud->points.size() << " is the cloud size\n";
  cout << minimesh_ptr->polygons.size() << " is the polygon size\n";
  pcl::toROSMsg (*minicloud, minimesh_ptr->cloud);
}



bool merge_PolygonMesh(pcl::PolygonMesh::Ptr &meshA, pcl::PolygonMesh::Ptr meshB){

  pcl::PointCloud<pcl::PointXYZRGB> cloudA;  
  pcl::fromROSMsg(meshA->cloud, cloudA);
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

bool PolygonMesh_to_lcm(lcm_t *lcm, Ptcoll_cfg ptcoll_cfg,pcl::PolygonMesh::Ptr mesh,
			bool clip_z , double clip_height,
			bool sendSubset, const vector<int> &SubsetIndicies){
  
  // skip_above_z doesnt display the z dimension
  //TODO: have general method for skipping outside x,y,z +/-
  int N_polygons;
  if (!sendSubset){
    N_polygons = mesh->polygons.size ();
  }else{
    N_polygons = SubsetIndicies.size ();
  }
  
  vs_point3d_list_collection_t point_lists;
  point_lists.id = ptcoll_cfg.id;
  point_lists.name = (char *)ptcoll_cfg.name.c_str(); // Use channel name?
  point_lists.type = ptcoll_cfg.type; // collection of POINTS

  point_lists.reset = ptcoll_cfg.reset;
  point_lists.nlists = N_polygons; // number of seperate sets of points  
  vs_point3d_list_t point_list[N_polygons];  
  
  pcl::PointCloud<pcl::PointXYZRGB> newcloud;  
  pcl::fromROSMsg(mesh->cloud, newcloud);
  Eigen::Vector4f tmp;
  for(size_t i=0; i< N_polygons; i++){ // each triangle/polygon
    size_t k;
    if (!sendSubset){ // send all of the mesh
      k = i;
    }else{ // only send some of it:
      k =  SubsetIndicies[i];
    }

    pcl::Vertices apoly_in = mesh->polygons[k];//[i];
    int N_points = apoly_in.vertices.size ();

    vs_point3d_list_t* points = &(point_list[i]); //[i]);
    points->nnormals = 0;
    points->normals = NULL;
    points->npointids = 0;
    points->pointids = NULL;
    
    points->ncolors = N_points;
    vs_color_t* colors = new vs_color_t[N_points];
    points->npoints = N_points;
    vs_point3d_t* entries = new vs_point3d_t[N_points];
    
    bool send_this = false;
    
    points->id = i; // ... still i - not k
    points->collection = ptcoll_cfg.collection;//PoseCollID;//collection.objectCollectionId();
    points->element_id = ptcoll_cfg.element_id;//PoseID;
    for(size_t j=0; j< N_points; j++){ // each point
      uint32_t pt = apoly_in.vertices[j];
      tmp = newcloud.points[pt].getVector4fMap();
      entries[j].x =(float) tmp(0);
      entries[j].y =(float) tmp(1);
      entries[j].z =(float) tmp(2);
      // r,g,b: input is ints 0->255, opengl wants floats 0->1
      colors[j].r = (float) newcloud.points[pt].r/255.0; // Red  
      colors[j].g = (float) newcloud.points[pt].g/255.0; // Green
      colors[j].b = (float) newcloud.points[pt].b/255.0; // Blue
      
      if (!send_this){ // this is bad coding, improve me...
	if (tmp(2) < clip_height){ // transmit if at least one z point is below thresh
	  send_this=true;
	}
      }
    }
    if (clip_z){
      if (!send_this){ // sends an empty list
	points->ncolors =0;
	points->npoints =0;
      }
    }
    points->points = entries;
    points->colors = colors;
  } 
  point_lists.point_lists = point_list;
  vs_point3d_list_collection_t_publish(lcm,"POINTS_COLLECTION",&point_lists);
  
  //TODO I don't think i am doing memory management properly!!!
  //  delete colors;
  for (int i=0;i<point_lists.nlists;i++) {
      delete [] point_lists.point_lists[i].points;
      delete [] point_lists.point_lists[i].colors;
  }   
  
  return 1;
}



bool pcdXYZRGB_to_lcm(lcm_t *lcm, Ptcoll_cfg ptcoll_cfg,pcl::PointCloud<pcl::PointXYZRGB> &cloud){
   
  vs_point3d_list_collection_t plist_coll;
  plist_coll.id = ptcoll_cfg.id;
  plist_coll.name =(char*)   ptcoll_cfg.name.c_str();
  plist_coll.type =ptcoll_cfg.type; // collection of points
  plist_coll.reset = ptcoll_cfg.reset;
  plist_coll.nlists = 1; // number of seperate sets of points
  vs_point3d_list_t plist[plist_coll.nlists];
  
  // loop here for many lists
  vs_point3d_list_t* this_plist = &(plist[0]);
  // 3.0: header
  this_plist->id =ptcoll_cfg.point_lists_id; //bot_timestamp_now();
  this_plist->collection = ptcoll_cfg.collection;
  this_plist->element_id = ptcoll_cfg.element_id;
  // 3.1: points/entries (rename) 
  vs_point3d_t* points = new vs_point3d_t[ptcoll_cfg.npoints];
  this_plist->npoints = ptcoll_cfg.npoints;
  // 3.2: colors:
  vs_color_t* colors = new vs_color_t[ptcoll_cfg.npoints];
  //points->colors = NULL;
  this_plist->ncolors = ptcoll_cfg.npoints;
  // 3.3: normals:
  this_plist->nnormals = 0;
  this_plist->normals = NULL;   
  // 3.4: point ids:
  this_plist->npointids = ptcoll_cfg.npoints;
  int64_t* pointsids= new int64_t[ptcoll_cfg.npoints];
  
  float rgba[4];
  for(int j=0; j<ptcoll_cfg.npoints; j++) {  //Nransac
      if (ptcoll_cfg.rgba[0] <-0.5){ // if rgba value is negative use it
	// PARTICAL FIX: now using .r.g.b values
	//int rgba_one = *reinterpret_cast<int*>(&cloud.points[j].rgba);
	//rgba[3] =((float) ((rgba_one >> 24) & 0xff))/255.0;
	//rgba[2] =((float) ((rgba_one >> 16) & 0xff))/255.0;
	//rgba[1] =((float) ((rgba_one >> 8) & 0xff))/255.0;
	//rgba[0] =((float) (rgba_one & 0xff) )/255.0;      
	
	rgba[2] = cloud.points[j].b/255.0;
	rgba[1] = cloud.points[j].g/255.0;
	rgba[0] = cloud.points[j].r/255.0;
      }else{ // use the rgba value
	//rgba[3] = ptcoll_cfg.rgba[3];
	rgba[2] = ptcoll_cfg.rgba[2];
	rgba[1] = ptcoll_cfg.rgba[1];
	rgba[0] = ptcoll_cfg.rgba[0];
	
      }
      
      colors[j].r =  rgba[0]; // points_collection values range 0-1
      colors[j].g =  rgba[1];
      colors[j].b = rgba[2];
      points[j].x = cloud.points[j].x;
      points[j].y = cloud.points[j].y;
      points[j].z = cloud.points[j].z;
      pointsids[j] = ptcoll_cfg.element_id+j; //bot_timestamp_now();
  }   
 
  this_plist->colors = colors;
  this_plist->points = points;
  this_plist->pointids = pointsids;
  plist_coll.point_lists = plist;
  vs_point3d_list_collection_t_publish(lcm,"POINTS_COLLECTION",&plist_coll);  
  
  
  delete pointsids;
  delete colors;
  delete points;
}

bool pcdXYZ_to_lcm(lcm_t *lcm, Ptcoll_cfg ptcoll_cfg,pcl::PointCloud<pcl::PointXYZ> &cloud){
   
  vs_point3d_list_collection_t plist_coll;
  plist_coll.id = ptcoll_cfg.id;
  plist_coll.name =(char*)   ptcoll_cfg.name.c_str();
  plist_coll.type =ptcoll_cfg.type; // collection of points
  plist_coll.reset = ptcoll_cfg.reset;
  plist_coll.nlists = 1; // number of seperate sets of points
  vs_point3d_list_t plist[plist_coll.nlists];
  
  // loop here for many lists
  vs_point3d_list_t* this_plist = &(plist[0]);
  // 3.0: header
  this_plist->id = ptcoll_cfg.point_lists_id;//bot_timestamp_now();
  this_plist->collection = ptcoll_cfg.collection;
  this_plist->element_id = ptcoll_cfg.element_id;
  // 3.1: points/entries (rename) 
  vs_point3d_t* points = new vs_point3d_t[ptcoll_cfg.npoints];
  this_plist->npoints = ptcoll_cfg.npoints;
  // 3.2: colors:
  vs_color_t* colors = new vs_color_t[ptcoll_cfg.npoints];
  //points->colors = NULL;
  this_plist->ncolors = ptcoll_cfg.npoints;
  // 3.3: normals:
  this_plist->nnormals = 0;
  this_plist->normals = NULL;   
  // 3.4: point ids:
  this_plist->npointids = ptcoll_cfg.npoints;
  int64_t* pointsids= new int64_t[ptcoll_cfg.npoints];
  
  int rgba[4];
  for(int j=0; j<ptcoll_cfg.npoints; j++) {  //Nransac
      colors[j].r = ptcoll_cfg.rgba[0]; // 0-1
      colors[j].g = ptcoll_cfg.rgba[1];
      colors[j].b = ptcoll_cfg.rgba[2];
      points[j].x = cloud.points[j].x;
      points[j].y = cloud.points[j].y;
      points[j].z = cloud.points[j].z;
      pointsids[j] = ptcoll_cfg.element_id + j;
  }   
  this_plist->colors = colors;
  this_plist->points = points;
  this_plist->pointids = pointsids;
  plist_coll.point_lists = plist;
  vs_point3d_list_collection_t_publish(lcm,"POINTS_COLLECTION",&plist_coll);  
  
  
  delete pointsids;
  delete colors;
  delete points;
}



// display_tic_toc: a helper function which accepts a set of 
// timestamps and displays the elapsed time between them as 
// a fraction and time used [for profiling]
void display_tic_toc(vector<int64_t> &tic_toc,const string &fun_name){
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





// Copied from kinect-lcm
static inline void
_matrix_vector_multiply_3x4_4d (const double m[12], const double v[4],
        double result[3])
{
    result[0] = m[0]*v[0] + m[1]*v[1] + m[2] *v[2] + m[3] *v[3];
    result[1] = m[4]*v[0] + m[5]*v[1] + m[6] *v[2] + m[7] *v[3];
    result[2] = m[8]*v[0] + m[9]*v[1] + m[10]*v[2] + m[11]*v[3];
}




void do_unpack_openni_ros_frame(void *user_data,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud){
//  unpack->kcal = state->kcal;
//  unpack->rgb_data=  state->rgb_data;
//  unpack->msg =(kinect_frame_msg_t*) msg;  
//  do_unpack_kinect_frame(unpack,cloud);
  UnpackOpenniROS* self = static_cast<UnpackOpenniROS*>(user_data);

  
  // 1. Unpack /*&*/ Decompress the data:
  // 1.1 RGB:
  // TODO check width, height
  if(self->msg->image.image_data_format == OPENNI_IMAGE_MSG_T_VIDEO_RGB) {
      memcpy(self->rgb_data, self->msg->image.image_data, 
	      self->msg->depth.width * self->msg->depth.height * 3);
  } else if(self->msg->image.image_data_format == OPENNI_IMAGE_MSG_T_VIDEO_RGB_JPEG) {
    jpegijg_decompress_8u_rgb(self->msg->image.image_data, self->msg->image.image_data_nbytes,
            self->rgb_data, self->msg->image.width, self->msg->image.height, self->msg->image.width* 3);
  }
  //cout << (int) self->msg->depth.compression << "\n";
  
  /// 1.2. DEPTH:
  uint8_t* uncompress_buffer;
  int uncompress_buffer_size;
  const uint8_t* depth_data = self->msg->depth.depth_data;
  float* disparity_array = (float*) malloc(self->msg->depth.width * self->msg->depth.height * sizeof(float)); 
  // 1.2.1 De-compress if necessary:
  if(self->msg->depth.compression != OPENNI_DEPTH_MSG_T_COMPRESSION_NONE) {
    //std:: cout << "compression \n ";
    if(self->msg->depth.uncompressed_size > uncompress_buffer_size) {
      uncompress_buffer_size = self->msg->depth.uncompressed_size;
      uncompress_buffer = (uint8_t*) realloc(uncompress_buffer, uncompress_buffer_size);
    }
    unsigned long dlen = self->msg->depth.uncompressed_size;
    int status = uncompress(uncompress_buffer, &dlen, 
	      self->msg->depth.depth_data, self->msg->depth.depth_data_nbytes);
    if(status != Z_OK) {
      return;
    }
    depth_data = uncompress_buffer;
  }else{
    //std:: cout << "no compression \n ";
  }

  // 1.2.2 unpack raw byte data into float values in mm
  int npixels = self->msg->depth.width * self->msg->depth.height;
  //float* val = reinterpret_cast<float*>( self->msg->depth.depth_data );
  const float* val = reinterpret_cast<const float*>( depth_data );
  for(int i=0; i<npixels; i++) {
    //cout << i << " i\n";
    disparity_array[i] = val[i];
  }

  /// 2 Calculate transformation matrices:
  double depth_to_rgb_uvd[12];
  double depth_to_depth_xyz[16];
  kinect_calib_get_depth_uvd_to_rgb_uvw_3x4(self->kcal, depth_to_rgb_uvd);
  kinect_calib_get_depth_uvd_to_depth_xyz_4x4(self->kcal, depth_to_depth_xyz);
  double depth_to_depth_xyz_trans[16];
  //_matrix_transpose_4x4d(depth_to_depth_xyz, depth_to_depth_xyz_trans);
  bot_matrix_transpose_4x4d(depth_to_depth_xyz, depth_to_depth_xyz_trans);

  cloud->width    = (self->msg->depth.width/self->decimate_image[0]) ;
  cloud->height   = (self->msg->depth.height/self->decimate_image[1]);
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);
  double xyzw[4];
  int j=0;
  int j2=0;
  for(int v=0; v<self->msg->depth.height; v=v+ self->decimate_image[1]) { // t2b self->height 480
    for(int u=0; u<self->msg->depth.width; u=u+self->decimate_image[0] ) {  //l2r self->width 640
      // 3.4.1 compute distorted pixel coordinates
//      uint16_t disparity = 
      double uvd_depth[4] = { u, v, disparity_array[v*self->msg->depth.width+u], 1 };
      double uvd_rgb[3];
      _matrix_vector_multiply_3x4_4d(depth_to_rgb_uvd, uvd_depth, uvd_rgb);
      double uv_rect[2] = {
	  uvd_rgb[0] / uvd_rgb[2],
	  uvd_rgb[1] / uvd_rgb[2]
      };
      double uv_dist[2];

      kinect_calib_distort_rgb_uv(self->kcal, uv_rect, uv_dist);
      int u_rgb = uv_dist[0] + 0.5;
      int v_rgb = uv_dist[1] + 0.5;
      uint8_t r, g, b;
  //    if(u_rgb >= msg->depth.width || u_rgb < 0 || v_rgb >= msg->depth.height || v_rgb < 0) {
//	  r = g = b = 0;
  //    } else {
	  r = self->rgb_data[v_rgb*self->msg->depth.width*3 + u_rgb*3 + 0];
	  g = self->rgb_data[v_rgb*self->msg->depth.width*3 + u_rgb*3 + 1];
	  b = self->rgb_data[v_rgb*self->msg->depth.width*3 + u_rgb*3 + 2];
    //  }
      
      
   //   r=g=b=0;
	  r = self->rgb_data[v*self->msg->depth.width*3 + u*3 + 0];
	  g = self->rgb_data[v*self->msg->depth.width*3 + u*3 + 1];
	  b = self->rgb_data[v*self->msg->depth.width*3 + u*3 + 2];

	  
     // colors[j].r = ((float) r)/255.0; // points_collection values range 0-1
     // colors[j].g = ((float) g)/255.0;
     // colors[j].b = ((float) b)/255.0;
      // printf("col: %f,%f,%f\n",colors[j].r,colors[j].g,colors[j].b);
      
      // 3.4.2 find the xyz location of the points:
      //_matrix_multiply(depth_to_depth_xyz, 4, 4, 
      //      uvd_depth, 4, 1, xyzw);
      bot_matrix_multiply(depth_to_depth_xyz, 4, 4, 
	      uvd_depth, 4, 1, xyzw);      

      // printf("a: %d,%d,%f,%f,%f,%f,%f\n",u,v,(double)disparity,xyzw[0],xyzw[1],xyzw[2],xyzw[3]);
      // TODO: look up a more accurate transformation between vehicle pose and camera pose
      // this is sufficient for now:
  
        double constant = 1.0f / self->kcal->intrinsics_rgb.fx ;
        //(cloud_msg->width);
          //float constant = 1.0f / device_->getImageFocalLength (cloud_msg->width);
      
	double disparity_d = disparity_array[v*self->msg->depth.width+u];
	
//        entries[j].y = - (((double) u)- 319.50)*disparity_d*constant;
 //	entries[j].z = - (((double) v)- 239.50)*disparity_d*constant;
   //     entries[j].x = disparity_d;// -xyzw[1]/xyzw[3];
      
      
//      entries[j].y = -xyzw[0]/xyzw[3];
//      entries[j].z = -xyzw[1]/xyzw[3];
//      entries[j].x = xyzw[2]/xyzw[3];
      
      // 3.4.3 do some light filtering on the points:
      // NB: these points are still published though...
//       if (self->do_light_filtering){      
// 	if (entries[j].x > 5.5){ // remove all points 5.5+ m away
// 	  entries[j].x=0;
// 	  entries[j].y=0;
// 	  entries[j].z=0;
// 	}else if (entries[j].x < 0){ // remove all points behind camera (null ranges)
// 	  entries[j].x=0;
// 	  entries[j].y=0;
// 	  entries[j].z=0;
// 	}
//       }
// 

//       cloud->points[j2].y =  -xyzw[0]/xyzw[3];//y right+ (check)
//       cloud->points[j2].z = -xyzw[1]/xyzw[3];//z up+
//       cloud->points[j2].x = xyzw[2]/xyzw[3]; //x forward+
	
	
      // 10x pre-decimated version - for ros tool chain:
      //cloud->points[j2].y =  - (((double) u)- 31.950)*10*disparity_d*constant;
      //cloud->points[j2].z = - (((double) v)- 23.950)*10*disparity_d*constant;

      // Original not-pre decimated verison:
      cloud->points[j2].y =  - (((double) u)- 319.50)*disparity_d*constant;
      cloud->points[j2].z = - (((double) v)- 239.50)*disparity_d*constant;
      
      cloud->points[j2].x = disparity_d;// -xyzw[1]/xyzw[3];

      unsigned char* rgba_ptr = (unsigned char*)&cloud->points[j2].rgba;
      (*rgba_ptr) =  r;     // actually blue
      (*(rgba_ptr+1)) =  g; // actually green
      (*(rgba_ptr+2)) =  b; // actually blue
      (*(rgba_ptr+3)) =  0;
      j2++;



     // pointids[j] = bot_timestamp_now();
     // j++;
      // xyzw[0] = xyzw[0]/xyzw[3];
      // xyzw[1] = xyzw[1]/xyzw[3];
      // xyzw[2] = xyzw[2]/xyzw[3];
      // printf("b: %d,%d,%f,%f,%f\n",u,v,xyzw[0],xyzw[1],xyzw[2]);
    }
  }   
//   points->colors = colors;
//   points->points = entries;
//   points->pointids = pointids;
//   point_lists.point_lists = point_list;
//   vs_point3d_list_collection_t_publish(self->publish_lcm,"POINTS_COLLECTION",&point_lists);  
//  
// 	  cout << "\n";
//   
//   cout << points->npointids << "\n";
// 

  free(disparity_array);

}



void do_unpack_kinect_frame(void *user_data,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud){
  UnpackFreenectLCM* state = static_cast<UnpackFreenectLCM*>(user_data);
  
  // 1. Unpack & Decompress the data:
  // 1.1 RGB:
  // TODO check width, height
  if(state->msg->image.image_data_format == KINECT_IMAGE_MSG_T_VIDEO_RGB) {
      memcpy(state->rgb_data, state->msg->image.image_data, 
	      state->msg->depth.width * state->msg->depth.height * 3);
  } else if(state->msg->image.image_data_format == KINECT_IMAGE_MSG_T_VIDEO_RGB_JPEG) {
    jpegijg_decompress_8u_rgb(state->msg->image.image_data, state->msg->image.image_data_nbytes,
            state->rgb_data, state->msg->image.width, state->msg->image.height, state->msg->image.width* 3);
  }

  /// 1.2. DEPTH:
  uint8_t* uncompress_buffer;
  int uncompress_buffer_size;
  const uint8_t* depth_data = state->msg->depth.depth_data;
  uint16_t* disparity_array = (uint16_t*) malloc(state->msg->depth.width * state->msg->depth.height * sizeof(uint16_t)); 
  // 1.2.1 De-compress if necessary:
  if(state->msg->depth.compression != KINECT_DEPTH_MSG_T_COMPRESSION_NONE) {
    //std:: cout << "compression \n ";
    if(state->msg->depth.uncompressed_size > uncompress_buffer_size) {
      uncompress_buffer_size = state->msg->depth.uncompressed_size;
      uncompress_buffer = (uint8_t*) realloc(uncompress_buffer, uncompress_buffer_size);
    }
    unsigned long dlen = state->msg->depth.uncompressed_size;
    int status = uncompress(uncompress_buffer, &dlen, 
	      state->msg->depth.depth_data, state->msg->depth.depth_data_nbytes);
    if(status != Z_OK) {
      return;
    }
    depth_data = uncompress_buffer;
  }else{
    //std:: cout << "no compression \n ";
  }
  
  // 1.2.2 unpack raw byte data into int values
  int npixels = state->msg->depth.width * state->msg->depth.height;
  switch(state->msg->depth.depth_data_format) {
    case KINECT_DEPTH_MSG_T_DEPTH_11BIT:
      if(G_BYTE_ORDER == G_LITTLE_ENDIAN) {
	int16_t* rdd = (int16_t*) depth_data;
	int i;
	for(i=0; i<npixels; i++) {
	  int d = rdd[i];
	  disparity_array[i] = d;
	}
      } else {
	fprintf(stderr, "Big endian systems not supported\n");
      }
      break;
    case KINECT_DEPTH_MSG_T_DEPTH_10BIT:
      fprintf(stderr, "10-bit depth data not supported\n");
      break;
    default:
      break;
  }  

  /// 2 Calculate transformation matrices:
  double depth_to_rgb_uvd[12];
  double depth_to_depth_xyz[16];
  kinect_calib_get_depth_uvd_to_rgb_uvw_3x4(state->kcal, depth_to_rgb_uvd);
  kinect_calib_get_depth_uvd_to_depth_xyz_4x4(state->kcal, depth_to_depth_xyz);
  double depth_to_depth_xyz_trans[16];
  //_matrix_transpose_4x4d(depth_to_depth_xyz, depth_to_depth_xyz_trans);
  bot_matrix_transpose_4x4d(depth_to_depth_xyz, depth_to_depth_xyz_trans);
    
  // 3 for each depth point find the corresponding xyz and then RGB
  //   then put into PCL structure
  cloud->width    = (state->msg->depth.width/state->decimate_image[0]) ;
  cloud->height   = (state->msg->depth.height/state->decimate_image[1]);
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);
  double xyzw2[4];
  int j2=0;
  // NB: the order of these loop was changed... aug 2011. important
  for(int v=0; v<state->msg->depth.height; v=v+ state->decimate_image[1]) { // t2b state->height 480
     for(int u=0; u<state->msg->depth.width; u=u+state->decimate_image[0] ) {  //l2r state->width 640
      // 3.4.1 compute distorted pixel coordinates
      uint16_t disparity = disparity_array[v*state->msg->depth.width+u];
      double uvd_depth[4] = { u, v, disparity, 1 };
      double uvd_rgb[3];
      _matrix_vector_multiply_3x4_4d(depth_to_rgb_uvd, uvd_depth, uvd_rgb);
      double uv_rect[2] = {
	  uvd_rgb[0] / uvd_rgb[2],
	  uvd_rgb[1] / uvd_rgb[2]
      };
      double uv_dist[2];
      kinect_calib_distort_rgb_uv(state->kcal, uv_rect, uv_dist);
      int u_rgb = uv_dist[0] + 0.5;
      int v_rgb = uv_dist[1] + 0.5;
      uint8_t r, g, b;
      if(u_rgb >= state->msg->depth.width || u_rgb < 0 || v_rgb >= state->msg->depth.height || v_rgb < 0) {
	  r = g = b = 0;
      } else {
	  r = state->rgb_data[v_rgb*state->msg->depth.width*3 + u_rgb*3 + 0];
	  g = state->rgb_data[v_rgb*state->msg->depth.width*3 + u_rgb*3 + 1];
	  b = state->rgb_data[v_rgb*state->msg->depth.width*3 + u_rgb*3 + 2];
      }
      // 3.4.2 find the xyz location of the points:
      bot_matrix_multiply(depth_to_depth_xyz, 4, 4, 
	      uvd_depth, 4, 1, xyzw2);

      cloud->points[j2].y = -xyzw2[0]/xyzw2[3];//y right+ (check)
      cloud->points[j2].z = -xyzw2[1]/xyzw2[3];//z up+
      cloud->points[j2].x = xyzw2[2]/xyzw2[3]; //x forward+
      unsigned char* rgba_ptr = (unsigned char*)&cloud->points[j2].rgba;
      (*rgba_ptr) =  r;
      (*(rgba_ptr+1)) =  g;
      (*(rgba_ptr+2)) =  b;
      (*(rgba_ptr+3)) =  0;
      j2++;
    }
  }     
  
  free(disparity_array);
  if(state->msg->depth.compression != KINECT_DEPTH_MSG_T_COMPRESSION_NONE) {
    free(uncompress_buffer); // memory leak bug fixed
  }
}


/*
bool read_and_project_submaps(std::string file_path,std::string file_name_in, BasicPlane &one_plane, BasicPlane &one_plane000  ){
  pcl::PCDReader reader;

  // 1. read in a set of CONCAVE polygons
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

  string full_filename = file_path;
  full_filename.append(file_name_in);
  reader.read (full_filename, *cloud);
  //std::cerr << pcd_files[i] << " has: " << cloud.points.size () << " data points." << std::endl;
  
  one_plane.name.assign(file_name_in);

  sscanf(file_name_in.c_str(),"submap_%d_cloud_%d.pcd",&one_plane.major,&one_plane.minor);
  
  
//   while(file_name_in.find("_")!=string::npos){
//   file_name_in.replace(file_name_in.find("_"),1," ");
//   }
//   file_name_in.replace(file_name_in.find(".pcd"),4,"    ");
//   string temp_str;
//   istringstream iss4 (file_name_in,istringstream::in);
//   //iss4>>one_plane.utime>>temp_str>>temp_str;

  std::cout << "INFO: reading cloud: " << one_plane.name 
    << " | " << one_plane.major << " | " << one_plane.minor<<"\n";
  
  
   
  // 2. Fit a plane to the points using RANSAC, extract plane coeffs:
  pcl::ModelCoefficients::Ptr other_coeffs (new pcl::ModelCoefficients() );
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices() );
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setInputCloud (cloud);
  seg.setOptimizeCoefficients (true); // Optional
  seg.setModelType (pcl::SACMODEL_PLANE); // Mandatory
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.05); // 0.01 for table data set
  seg.segment (*inliers, *other_coeffs);    
  
  // 3. Project the model inliers (seems to be necessary for fitting convex hull)
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud);
  proj.setModelCoefficients (other_coeffs);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>());
  proj.filter (*cloud_projected);    
  one_plane.coeffs = (*other_coeffs);      
  
  // PCL bug: Colour Information Lost in reconstruction, manually put it in
  // i think this bug is fixed in the more recent versions of PCL
  for (size_t i=0;i<cloud_projected->size();i++){
    cloud_projected->points[i].rgba  = cloud->points[0].rgba;
  }
  compute3DCentroid (*cloud_projected,one_plane.centroid);
  one_plane.cloud = (*cloud_projected);
  

  // 4. Get the transform for the cloud to 0,0,0 and rotate onto x,y axis (from concave_hull.cpp in PCL)
  one_plane.covariance_matrix;
  computeCovarianceMatrix (one_plane.cloud, one_plane.centroid, one_plane.covariance_matrix);
  EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
  EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
  pcl::eigen33 (one_plane.covariance_matrix, eigen_vectors, eigen_values);

  one_plane.transform000.setIdentity ();
  if (eigen_values[0] / eigen_values[2] < 1.0e-5){
    //we have points laying on a plane, using 2d convex hull
    //compute transformation bring eigen_vectors.col(i) to z-axis
    eigen_vectors.col (2) = eigen_vectors.col (0).cross (eigen_vectors.col (1));
    eigen_vectors.col (1) = eigen_vectors.col (2).cross (eigen_vectors.col (0));
    one_plane.transform000 (0, 2) = eigen_vectors (0, 0);
    one_plane.transform000 (1, 2) = eigen_vectors (1, 0);
    one_plane.transform000 (2, 2) = eigen_vectors (2, 0);
    one_plane.transform000 (0, 1) = eigen_vectors (0, 1);
    one_plane.transform000 (1, 1) = eigen_vectors (1, 1);
    one_plane.transform000 (2, 1) = eigen_vectors (2, 1);
    one_plane.transform000 (0, 0) = eigen_vectors (0, 2);
    one_plane.transform000 (1, 0) = eigen_vectors (1, 2);
    one_plane.transform000 (2, 0) = eigen_vectors (2, 2);
    one_plane.transform000 = one_plane.transform000.inverse ();
  }else{
    one_plane.transform000.setIdentity ();
  }  
  
  
  // 5. Get (and keep) the plane transformed back to 000:
  one_plane000 = one_plane;
  pcl::demeanPointCloud (one_plane000.cloud, one_plane000.centroid, one_plane000.cloud);
  pcl::transformPointCloud (one_plane000.cloud, one_plane000.cloud, one_plane000.transform000);  
}*/
