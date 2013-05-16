
#include <iostream>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

#include "AffordanceCollectionListener.hpp"

#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/io/pcd_io.h>
#include <drc_utils/PointConvert.h>

#include <affordance/AffordanceUtils.hpp>

using namespace std;
using namespace boost;
using namespace visualization_utils;
using namespace collision;
using namespace drc::PointConvert;

namespace renderer_affordances
{

  //==================constructor / destructor

//   AffordanceCollectionListener::AffordanceCollectionListener(boost::shared_ptr<lcm::LCM> &lcm, RendererAffordances* affordance_renderer):
//     _lcm(lcm),
//     _parent_affordance_renderer(affordance_renderer)
  AffordanceCollectionListener::AffordanceCollectionListener( RendererAffordances* affordance_renderer):
    _parent_affordance_renderer(affordance_renderer)
  {
    _lcm = affordance_renderer->lcm;
    
 
    //lcm ok?
    if(!_lcm->good())
      {
	      cerr << "\nLCM Not Good: Robot State Handler" << endl;
	      return;
      }


    _lcm->subscribe("AFFORDANCE_COLLECTION", &AffordanceCollectionListener::handleAffordanceCollectionMsg, this); 
    _lcm->subscribe("AFFORDANCE", &AffordanceCollectionListener::handleAffordanceMsg, this); 
    _lcm->subscribe("AFFORDANCE_PLUS_COLLECTION", &AffordanceCollectionListener::handleAffordancePlusCollectionMsg, this); 
    _lcm->subscribe("AFFORDANCE_PLUS", &AffordanceCollectionListener::handleAffordancePlusMsg, this); 

  }
  


  AffordanceCollectionListener::~AffordanceCollectionListener() {}

 // =======================================================================  
 //  message callbacks
 // =======================================================================   

// checks to see the affordances are in the parent renderer object list. If it is pre-existing 
// it is updated and if its new a new instance is created.
void AffordanceCollectionListener::handleAffordanceCollectionMsg(const lcm::ReceiveBuffer* rbuf,
						 const string& chan, 
						 const drc::affordance_collection_t* msg)						 
  {
  

  
    //cout << "Ok!: " << oss.str() << endl;
    for (size_t i=0; i< (size_t)msg->naffs; i++)
    {
      const drc::affordance_t aff = msg->affs[i];
      
      std::stringstream oss;
      oss << aff.otdf_type << "_"<< aff.uid; 
        
      typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
      object_instance_map_type_::iterator it = _parent_affordance_renderer->instantiated_objects.find(oss.str());
       
       if (it!=_parent_affordance_renderer->instantiated_objects.end()) {
          //exists so update
          //cout <<"updated_otdf_object_instance: "<< aff.otdf_type <<  " " <<  aff.uid << endl;
          update_object_instance(aff);
       }      
       else {
          // add new otdf instance
          std::string filename = aff.otdf_type;//get_filename(aff.otdf_id);
          cout <<"add new otdf instance: "<< aff.otdf_type << "_"<< aff.uid << ", of template :" << filename << endl;
          //std::transform(filename.begin(), filename.end(), filename.begin(), ::tolower);
          add_new_otdf_object_instance(filename, aff);
       }
    }

  } // end handleMessage

  void AffordanceCollectionListener::handleAffordancePlusCollectionMsg(const lcm::ReceiveBuffer* rbuf,
                                                                   const string& chan, 
                                                                   const drc::affordance_plus_collection_t* msg)						 
  {
    // call handleAffordancePlusMsg for each affordance in collection
    set<pair<string,int> > currentAffs;
    for (size_t i=0; i< (size_t)msg->naffs; i++) {
      const drc::affordance_plus_t& aff = msg->affs_plus[i];
      handleAffordancePlusMsg(rbuf, chan, &aff);
      currentAffs.insert(make_pair(aff.aff.otdf_type, aff.aff.uid));
    }

    // delete any affordance not in message
    if(!_parent_affordance_renderer->debugMode){
      std::map<std::string, OtdfInstanceStruc>::iterator it;
      std::map<std::string, OtdfInstanceStruc>& objs = _parent_affordance_renderer->instantiated_objects;
      for(it = objs.begin(); it!=objs.end(); ){
        // if not in collection, erase
        if(currentAffs.find(make_pair(it->second.otdf_type, it->second.uid)) == currentAffs.end()) {
          // clear selection if selected
          const char *instance_name;
          instance_name = bot_gtk_param_widget_get_enum_str( _parent_affordance_renderer->pw, PARAM_OTDF_INSTANCE_SELECT );
          if(instance_name && _parent_affordance_renderer->object_selection==string(instance_name)){
            _parent_affordance_renderer->link_selection = " ";
            _parent_affordance_renderer->object_selection = " ";
          }  
          // clear sticky hands if selected  
          typedef map<string, StickyHandStruc > sticky_hands_map_type_;
          sticky_hands_map_type_::iterator hand_it = _parent_affordance_renderer->sticky_hands.begin();
          while (hand_it!=_parent_affordance_renderer->sticky_hands.end()) {
            string hand_name = string(hand_it->second.object_name);
            if (hand_name == string(instance_name)) {
              if(_parent_affordance_renderer->stickyhand_selection==hand_it->first)
                 _parent_affordance_renderer->stickyhand_selection = " ";
              _parent_affordance_renderer->sticky_hands.erase(hand_it++);
            } else hand_it++;
          } 
          // erase object
          objs.erase(it++);
          // trigger redraw
          bot_viewer_request_redraw(_parent_affordance_renderer->viewer);
        } else ++it;   // otherwise go to next object
      }
    }
  }

   // =======================================================================  
   
  void AffordanceCollectionListener::handleAffordanceMsg(const lcm::ReceiveBuffer* rbuf, const string& channel, 
					       const  drc::affordance_t* msg) 
  {
    //cout << "Received affordance: " << msg->otdf_type << " with uid: " << msg->uid << endl;
       std::stringstream oss;
       oss << msg->otdf_type << "_"<< msg->uid; 
    
       typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
       object_instance_map_type_::iterator it = _parent_affordance_renderer->instantiated_objects.find(oss.str());
       if (it!=_parent_affordance_renderer->instantiated_objects.end()) {
          //exists so update
         //cout <<"update_otdf_object_instance" << endl;
         update_object_instance((*msg));
       }      
       else{
          // add new otdf instance
          cout <<"add new otdf instance" << endl;
          std::string filename = msg->otdf_type;//get_filename(msg->otdf_id);
          add_new_otdf_object_instance(filename, (*msg));
       }
       
  } 

  void AffordanceCollectionListener::handleAffordancePlusMsg(const lcm::ReceiveBuffer* rbuf, const string& channel, 
                                                             const  drc::affordance_plus_t* msg)
  {
    // handle affordance_t within affordance_plus_t message
    handleAffordanceMsg(rbuf,channel,&msg->aff);

    //////////////////////////
    // handle plus features

    // retrieve otdf object associated with current message
    std::stringstream oss;
    oss << msg->aff.otdf_type << "_"<< msg->aff.uid;
    typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator it = _parent_affordance_renderer->instantiated_objects.find(oss.str());
    if (it!=_parent_affordance_renderer->instantiated_objects.end()) {

      // if no modelfile, copy points and triangles
      if(msg->aff.modelfile.empty()){
        // copy triangles and points
        convertVec3(msg->points, it->second._gl_object->points);
        convertVec3(msg->triangles, it->second._gl_object->triangles);
      }

      // copy bounding box
      convert3(msg->aff.bounding_xyz, it->second.boundingBoxXYZ);
      convert3(msg->aff.bounding_rpy, it->second.boundingBoxRPY);
      convert3(msg->aff.bounding_lwh, it->second.boundingBoxLWH);

      // if modelfile has changed, read in file
      if(msg->aff.modelfile != it->second.modelfile){
        // read in model
        vector<vector<float> > points;
        vector<vector<int> > triangles;
        string file = getenv("HOME") + string("/drc/software/models/otdf/") + msg->aff.modelfile;
        AffordanceUtils affutils;
        affutils.getModelAsLists(file, points, triangles);
        
        convertVec3(points, it->second._gl_object->points);
        convertVec3(triangles, it->second._gl_object->triangles);

        it->second.modelfile = msg->aff.modelfile;
      }

      // find links of type DynamicMesh and copy points and triangles into it.
      /*
      std::vector<boost::shared_ptr<otdf::Link> > links;
      it->second._otdf_instance->getLinks(links);
      for(int i=0; i<links.size(); i++){
        if(links[i]->visual && links[i]->visual->geometry){
          //cout << links[i]->visual->geometry->type << endl;
          boost::shared_ptr<otdf::DynamicMesh> dmesh = 
            dynamic_pointer_cast<otdf::DynamicMesh>(links[i]->visual->geometry);
          
          if(dmesh){
            dmesh->points = msg->points;
            dmesh->triangles = msg->triangles;
          }
        }
      }
      */
      
    } else{
      // object should always exist, so we should never get here
      cout << "*** ERROR handleAffordancePlusMsg: shouldn't get here\n";
    }

  }

 // =======================================================================  
 //  Utils
 // =======================================================================   

 /*std::string AffordanceCollectionListener::get_filename(int32_t otdf_id)
 {
       
  std::string filename;
          
  //NOTE: Have to manually list all the otdf templates, not ideal. 
  //Much better if otdf_id is a string instead of a enumerated type.
  if(otdf_id==drc::affordance_t::CYLINDER){
   // check if cylinder exists in _parent_affordance_renderer->otdf_filenames
   filename = "cylinder";
  }
  else if(otdf_id==drc::affordance_t::LEVER)
  {
   filename = "lever";
  }
  else if(otdf_id==drc::affordance_t::SPHERE)
  {
    filename = "sphere";  
  }
    
  return filename;
 }*/
 // =======================================================================
void AffordanceCollectionListener::add_new_otdf_object_instance (std::string &filename, const drc::affordance_t &aff)
{
 
  std::string xml_string;
  if(!otdf::get_xml_string_from_file(filename, xml_string)){
    return; // file extraction failed
  }
  
  OtdfInstanceStruc instance_struc;
  instance_struc._otdf_instance = otdf::parseOTDF(xml_string);
  if (!instance_struc._otdf_instance){
    std::cerr << "ERROR: Model Parsing of " << filename << " the xml failed" << std::endl;
  }
  
  //instance_struc._otdf_instance->name_ = aff.name;

  // set non-standard params from affordance message
  for (size_t i=0; i < (size_t)aff.nparams; i++)
    {   
      instance_struc._otdf_instance->setParam(aff.param_names[i],aff.params[i]);   
    }

  
  // set standard params from affordance message
  instance_struc._otdf_instance->setParam("x",aff.origin_xyz[0]);
  instance_struc._otdf_instance->setParam("y",aff.origin_xyz[1]);
  instance_struc._otdf_instance->setParam("z",aff.origin_xyz[2]);
  instance_struc._otdf_instance->setParam("roll", aff.origin_rpy[0]);
  instance_struc._otdf_instance->setParam("pitch",aff.origin_rpy[1]);
  instance_struc._otdf_instance->setParam("yaw",  aff.origin_rpy[2]);

  instance_struc._otdf_instance->update();
  
  //TODO: set All JointStates too.
  
  // create a KDL tree parser from OTDF instance, without having to convert to urdf.
  // otdf can contain some elements that are not part of urdf. e.g. TORUS, DYNAMIC_MESH (They are handled as special cases)  
  std::string _urdf_xml_string = otdf::convertObjectInstanceToCompliantURDFstring(instance_struc._otdf_instance);
  
  /*std::map<std::string, int >::iterator it;
  it= _parent_affordance_renderer->instance_cnt.find(filename);
  it->second = it->second + 1;*/
  std::stringstream oss;
  oss << aff.otdf_type << "_"<< aff.uid;
  instance_struc.uid = aff.uid;
  instance_struc.otdf_type = aff.otdf_type;//new string(aff.otdf_type);
  instance_struc._collision_detector.reset();
  instance_struc._collision_detector = shared_ptr<Collision_Detector>(new Collision_Detector());
  instance_struc._gl_object = shared_ptr<InteractableGlKinematicBody>(new InteractableGlKinematicBody(instance_struc._otdf_instance,instance_struc._collision_detector,true,oss.str()));
  instance_struc._gl_object->set_state(instance_struc._otdf_instance);

  //boost::shared_ptr<const otdf::BaseEntity> base = instance_struc._otdf_instance->getRoot();
  //boost::shared_ptr<otdf::Link> link = dynamic_pointer_cast<otdf::Link>(instance_struc._otdf_instance->getRoot());

  //->visual->geometry;


  //_parent_affordance_renderer->instantiated_objects.insert(std::make_pair(oss.str(), instance_struc));
  _parent_affordance_renderer->instantiated_objects.insert(std::make_pair(oss.str(), instance_struc));
  // Update params 
  
  //Request redraw
  bot_viewer_request_redraw(_parent_affordance_renderer->viewer);
} 


// =======================================================================  
void AffordanceCollectionListener::update_object_instance (const drc::affordance_t &aff)
{

  std::stringstream oss;
  oss << aff.otdf_type << "_"<< aff.uid; 

  typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
  object_instance_map_type_::iterator it = _parent_affordance_renderer->instantiated_objects.find(oss.str());

  // set non-standard params from affordance message
   for (size_t i=0; i < (size_t)aff.nparams; i++)
   {   
       it->second._otdf_instance->setParam(aff.param_names[i],aff.params[i]);   
   }

  // set standard params from affordance message
  it->second._otdf_instance->setParam("x",aff.origin_xyz[0]);
  it->second._otdf_instance->setParam("y",aff.origin_xyz[1]);
  it->second._otdf_instance->setParam("z",aff.origin_xyz[2]);
  it->second._otdf_instance->setParam("roll", aff.origin_rpy[0]);
  it->second._otdf_instance->setParam("pitch",aff.origin_rpy[1]);
  it->second._otdf_instance->setParam("yaw",  aff.origin_rpy[2]);
       
    //TODO: set All JointStates too.

    it->second._otdf_instance->update();
    if(it->second.otdf_instance_viz_object_sync)
      it->second._gl_object->set_state(it->second._otdf_instance);  
 



      //Request redraw
    bot_viewer_request_redraw(_parent_affordance_renderer->viewer);
 }
 
// =======================================================================


} //namespace affordance_renderer

