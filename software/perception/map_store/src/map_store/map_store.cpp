// Listen to map_store_POINTS files:
// - store them in a local object
// - maintain a list and broadcast the list
// - support queries, which sent a specific cloud back
// - listen for 



// LCM example program for interacting  with PCL data
// In this case it pushes the data back out to LCM in
// a different message type for which the "collections"
// renderer can view it in the LCM viewer
// mfallon aug 2012
#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <lcmtypes/visualization.h>

#include <path_util/path_util.h>

#include "map_store.hpp"
#include <boost/shared_ptr.hpp>


using namespace std;
using namespace pcl;


typedef boost::shared_ptr<otdf::ModelInterface> otdfPtr;


/////////////////////////////////////

map_store::map_store(lcm_t* publish_lcm):
          publish_lcm_(publish_lcm) {




  pc_vis_ = new pointcloud_vis(publish_lcm_);
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1000,"Pose - Dump",5,0) );
  float colors_b[] ={0.0,0.0,1.0};
  vector <float> colors_v;
  colors_v.assign(colors_b,colors_b+4*sizeof(float));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1001,"Cloud - Dump"     ,1,0, 1000,1,colors_v));

  //
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1100,"Pose - Current",5,0) );
  float colors_r[] ={1.0,0.0,0.0};
  colors_v.assign(colors_r,colors_r+4*sizeof(float));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1101,"Cloud - Current"     ,1,0, 1100,1,colors_v));



  // LCM:
  lcm_t* subscribe_lcm_ = publish_lcm_;
  drc_pointcloud2_t_subscribe(subscribe_lcm_, "LOCAL_MAP_POINTS",
      pointcloud_handler_aux, this);

  drc_localize_reinitialize_cmd_t_subscribe(subscribe_lcm_, "SEG_REQUEST",
      seg_request_handler_aux, this);

  drc_localize_reinitialize_cmd_t_subscribe(subscribe_lcm_, "SEG_UPDATE",
      seg_update_handler_aux, this);

  drc_localize_reinitialize_cmd_t_subscribe(subscribe_lcm_, "DUMP_MAPS",
      dump_maps_handler_aux, this);

  drc_localize_reinitialize_cmd_t_subscribe(subscribe_lcm_, "CURRENT_MAP_ID",
      current_map_handler_aux, this);
}

void map_store::initialize(){

}



void map_store::publish_affordance_collection(LocalMap m){

  cout <<m.map_id  << " | " << m.utime << "| " << m.cloud->size() << " cloud size\n";

  // Send a pose
  drc_affordance_collection_t affs_coll;
  affs_coll.map_id = m.map_id;
  affs_coll.name = (char*)  "homers_desk";
  affs_coll.map_utime = m.utime;
  affs_coll.naffs = m.objects.size();
  drc_affordance_t affs[affs_coll.naffs];

  cout << m.objects.size() << " Objects\n";
  for (size_t i=0;i <m.objects.size(); i++){

    affs[i].map_utime = m.utime;
    affs[i].map_id = m.map_id;
    affs[i].name = (char*) m.object_names[i].c_str();
    affs[i].otdf_id = m.object_ids[i];

    otdfPtr object= m.objects[i];
    std::map<std::string, double> param_map = object->params_map_; // all parameters defined in the object template
    std::map<std::string, double>::iterator it;
    cout << "params.size() is " << (int) param_map.size() << endl;

    affs[i].nparams=param_map.size();
    double* params = new double[affs[i].nparams];
    char** param_names = new char*[affs[i].nparams];
    int j;
    for (it=param_map.begin(),j=0  ; it != param_map.end(); it++ , j++){
      cout << (*it).first << " => " << (*it).second << " | ";
      param_names[j] = (char*) (*it).first.c_str();
      params[j] = (*it).second;
    }
    affs[i].params = params;
    affs[i].param_names = param_names;


    affs[i].nstates=0;
    affs[i].states=NULL;
    affs[i].state_names=NULL;

    affs[i].nptids=0;
    affs[i].ptids=NULL;

    cout << endl;
  }


  affs_coll.affs = affs;
  drc_affordance_collection_t_publish(publish_lcm_, "AFFORDANCE_COLLECTION", &affs_coll);
  cout << "affs gone\n";


}

void map_store::publish_local_map(unsigned int map_id){

  cout << "publish_local_map called\n";
  LocalMap m = maps[map_id];
  publish_affordance_collection(m);


  cout <<m.map_id  << " | " << m.utime << "| " << m.cloud->size() << " cloud size\n";

  Eigen::Isometry3d offset_pose = m.base_pose;
  offset_pose.translation() << m.base_pose.translation().x(),
      m.base_pose.translation().y(),
      m.base_pose.translation().z();

  Isometry3dTime offset_poseT = Isometry3dTime(m.utime, offset_pose);

  pc_vis_->pose_to_lcm_from_list(1100, offset_poseT);
  pc_vis_->ptcld_to_lcm_from_list(1101, *(m.cloud), offset_poseT.utime, offset_poseT.utime);

  cout << m.objects.size() << " Objects\n";
  for (size_t i=0;i <m.objects.size(); i++){
    otdfPtr object= m.objects[i];
    std::map<std::string, double> params = object->params_map_; // all parameters defined in the object template
    std::map<std::string, double>::iterator it;

    cout << "params.size() is " << (int) params.size() << endl;

    for (it=params.begin() ; it != params.end(); it++ ){
      cout << (*it).first << " => " << (*it).second << " | ";
    }
    cout << endl;
  }

}


void map_store::dump_maps(DumpCode code, double x_offset){

  if (code == DUMP_SCREEN){
   cout << "dump screen set\n";
  }

  cout << "dump maps blah\n";
  for (size_t i=0;i<maps.size() ; i++){
     LocalMap m = maps[i];
    cout <<m.map_id  << " | " << m.utime << "| " << m.cloud->size() << " cloud size\n";

    Eigen::Isometry3d offset_pose = m.base_pose;
    offset_pose.translation() << m.base_pose.translation().x() + x_offset*i,
               m.base_pose.translation().y()+ x_offset,
               m.base_pose.translation().z();

    Isometry3dTime offset_poseT = Isometry3dTime(m.utime, offset_pose);

    pc_vis_->pose_to_lcm_from_list(1000, offset_poseT);
    pc_vis_->ptcld_to_lcm_from_list(1001, *(m.cloud), offset_poseT.utime, offset_poseT.utime);


    cout << m.objects.size() << " Objects\n";
    for (size_t i=0;i <m.objects.size(); i++){
      otdfPtr object= m.objects[i];
      std::map<std::string, double> params = object->params_map_; // all parameters defined in the object template
      std::map<std::string, double>::iterator it;

      cout << "params.size() is " << (int) params.size() << endl;


      for (it=params.begin() ; it != params.end(); it++ ){
        cout << (*it).first << " => " << (*it).second << " | ";
      }
      cout << endl;
    }
  }
}

void map_store::dump_maps_handler(const drc_localize_reinitialize_cmd_t *msg){
  cout << "dump maps requested\n";

  dump_maps(DUMP_SCREEN,40.0);

  // Sent a specific map to main gui
  publish_local_map(0);
}

void map_store::current_map_handler(const drc_localize_reinitialize_cmd_t *msg){
  cout << "map\n";

  // contains which map to visualise
  // ... transmit the specified map to the segmentation gui (continously @ 1Hz in a different thread)
}



void map_store::seg_request_handler(const drc_localize_reinitialize_cmd_t *msg){
  cout << "segmentation map requested\n";

  // ... transmit the map to the segmentation gui
}


void map_store::seg_update_handler(const drc_localize_reinitialize_cmd_t *msg){
  cout << "received segmentation\n";

  // ... log the segmentation
}



void create_otdf_object_instance (otdfPtr &object, string otdf_name)//(RendererOtdf *self)
{

// std::string filename = self->otdf_filenames[self->otdf_id]+".otdf";
// std::transform(filename.begin(), filename.end(), filename.begin(), ::tolower);
  string otdf_models_path = std::string(getModelsPath()) + "/otdf/"; // getModelsPath gives /drc/software/build/

  std::string  filepath =  otdf_models_path+ otdf_name +".otdf";//(*self->otdf_dir_name_ptr)+filename;

 std::cout << "instantiating " << filepath << std::endl;
 std::string xml_string;
 std::fstream xml_file(filepath.c_str(), std::fstream::in);
  while ( xml_file.good() )
  {
    std::string line;
    std::getline( xml_file, line);
    xml_string += (line + "\n");
  }
  xml_file.close();
  object = otdf::parseOTDF(xml_string);
}


void map_store::pointcloud_handler(const drc_pointcloud2_t *msg){

  cout << "got ptd handler\n";
  // Create new local map object, store cloud

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pc_lcm_->unpack_pointcloud2( (const ptools_pointcloud2_t*)   msg, cloud);

  LocalMap map;
  //Eigen::Isometry3d base_pose;
  map.map_id =maps.size(); // use a counter instead
  map.utime =maps.size();// msg->utime; // use time later..

  map.base_pose.setIdentity();
  map.cloud =cloud;

  otdfPtr object( new otdf::ModelInterface);
  create_otdf_object_instance(object,"cylinder");
  object->setParam("x", 0.66); // just for adding data
  object->setParam("y", -0.31);
  object->setParam("z", 0.45);
  object->setParam("roll", 0.0);
  object->setParam("pitch", 0.0);
  object->setParam("yaw", 0.0);
  object->setParam("radius", 0.16);
  object->setParam("length", 0.41);
  object->setParam("mass", 1.00);
  object->update();
  map.objects.push_back(object);
  map.object_ids.push_back(DRC_AFFORDANCE_T_CYLINDER);
  map.object_names.push_back("uraniumrod");

  otdfPtr object2( new otdf::ModelInterface);
  create_otdf_object_instance(object2,"lever");
  object2->setParam("x", maps.size()+1);
  object2->setParam("y", 17);
  object2->setParam("z", 5);
  object2->update();
  map.objects.push_back(object2);
  map.object_ids.push_back(DRC_AFFORDANCE_T_LEVER);
  map.object_names.push_back("savereactor");

  maps.push_back(map);

}

int
main(int argc, char ** argv)
{
  lcm_t * lcm;
  lcm = lcm_create(NULL);
  map_store app(lcm);


  while(1)
    lcm_handle(lcm);

  lcm_destroy(lcm);
  return 0;
}

