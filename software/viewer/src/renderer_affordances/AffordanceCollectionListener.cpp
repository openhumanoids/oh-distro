
#include <iostream>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

#include "AffordanceCollectionListener.hpp"

using namespace std;
using namespace boost;
using namespace visualization_utils;
using namespace collision;

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
    for (size_t i=0; i< (size_t)msg->naffs; i++) {
      const drc::affordance_plus_t& aff = msg->affs_plus[i];
      handleAffordancePlusMsg(rbuf, chan, &aff);
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

      // copy triangles
      it->second.triangles.resize(msg->triangles.size());
      for(size_t i=0;i<msg->triangles.size();i++){
        it->second.triangles[i][0] = msg->triangles[i][0];
        it->second.triangles[i][1] = msg->triangles[i][1];
        it->second.triangles[i][2] = msg->triangles[i][2];
      }
      // copy points
      it->second.points.resize(msg->points.size());
      for(size_t i=0;i<msg->points.size();i++){
        it->second.points[i][0] = msg->points[i][0];
        it->second.points[i][1] = msg->points[i][1];
        it->second.points[i][2] = msg->points[i][2];
      }

      // copy bounding box
      it->second.boundingBoxXYZ[0] = msg->aff.bounding_xyz[0];
      it->second.boundingBoxXYZ[1] = msg->aff.bounding_xyz[1];
      it->second.boundingBoxXYZ[2] = msg->aff.bounding_xyz[2];
      it->second.boundingBoxRPY[0] = msg->aff.bounding_rpy[0];
      it->second.boundingBoxRPY[1] = msg->aff.bounding_rpy[1];
      it->second.boundingBoxRPY[2] = msg->aff.bounding_rpy[2];
      it->second.boundingBoxLWH[0] = msg->aff.bounding_lwh[0];
      it->second.boundingBoxLWH[1] = msg->aff.bounding_lwh[1];
      it->second.boundingBoxLWH[2] = msg->aff.bounding_lwh[2];

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
  
  // set standard params from affordance message
  instance_struc._otdf_instance->setParam("x",aff.origin_xyz[0]);
  instance_struc._otdf_instance->setParam("y",aff.origin_xyz[1]);
  instance_struc._otdf_instance->setParam("z",aff.origin_xyz[2]);
  instance_struc._otdf_instance->setParam("roll", aff.origin_rpy[0]);
  instance_struc._otdf_instance->setParam("pitch",aff.origin_rpy[1]);
  instance_struc._otdf_instance->setParam("yaw",  aff.origin_rpy[2]);

  // set non-standard params from affordance message
  for (size_t i=0; i < (size_t)aff.nparams; i++)
    {   
      instance_struc._otdf_instance->setParam(aff.param_names[i],aff.params[i]);   
    }
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

  // set standard params from affordance message
  it->second._otdf_instance->setParam("x",aff.origin_xyz[0]);
  it->second._otdf_instance->setParam("y",aff.origin_xyz[1]);
  it->second._otdf_instance->setParam("z",aff.origin_xyz[2]);
  it->second._otdf_instance->setParam("roll", aff.origin_rpy[0]);
  it->second._otdf_instance->setParam("pitch",aff.origin_rpy[1]);
  it->second._otdf_instance->setParam("yaw",  aff.origin_rpy[2]);
       
  // set non-standard params from affordance message
   for (size_t i=0; i < (size_t)aff.nparams; i++)
   {   
       it->second._otdf_instance->setParam(aff.param_names[i],aff.params[i]);   
   }
    //TODO: set All JointStates too.

    it->second._otdf_instance->update();
    it->second._gl_object->set_state(it->second._otdf_instance);  
 



      //Request redraw
    bot_viewer_request_redraw(_parent_affordance_renderer->viewer);
 }
 
// =======================================================================


} //namespace affordance_renderer

