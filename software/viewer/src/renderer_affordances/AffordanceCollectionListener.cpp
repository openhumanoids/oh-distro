
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
    //cout << "Ok!: " << msg->name << endl;
    for (size_t i=0; i< (size_t)msg->naffs; i++)
    {
      const drc::affordance_t aff = msg->affs[i];
        
      typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
      object_instance_map_type_::iterator it = _parent_affordance_renderer->instantiated_objects.find(aff.name);
       
       if (it!=_parent_affordance_renderer->instantiated_objects.end()) {
          //exists so update
          // cout <<"updated_otdf_object_instance: "<< aff.name << endl;
          update_object_instance(aff);
       }      
       else {
          // add new otdf instance
          std::string filename = get_filename(aff.otdf_id);
          cout <<"add new otdf instance: "<< aff.name << ", of template :" << filename << endl;
          //std::transform(filename.begin(), filename.end(), filename.begin(), ::tolower);
          add_new_otdf_object_instance(filename, aff);
       }
    }

  } // end handleMessage

   // =======================================================================  
   
  void AffordanceCollectionListener::handleAffordanceMsg(const lcm::ReceiveBuffer* rbuf, const string& channel, 
					       const  drc::affordance_t* msg) 
  {
       cout << "Received affordance: " << msg->name << endl;
       
       typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
       object_instance_map_type_::iterator it = _parent_affordance_renderer->instantiated_objects.find(msg->name);
       if (it!=_parent_affordance_renderer->instantiated_objects.end()) {
          //exists so update
         cout <<"update_otdf_object_instance" << endl;
         update_object_instance((*msg));
       }      
       else{
          // add new otdf instance
          cout <<"add new otdf instance" << endl;
          std::string filename = get_filename(msg->otdf_id);
          add_new_otdf_object_instance(filename, (*msg));
       }
       
  } 

 // =======================================================================  
 //  Utils
 // =======================================================================   

 std::string AffordanceCollectionListener::get_filename(int32_t otdf_id)
 {
   enum{CYLINDER, LEVER};
       
  std::string filename;
          
  //NOTE: Have to manually list all the otdf templates, not ideal. 
  //Much better if otdf_id is a string instead of a enumerated type.
  if(otdf_id==CYLINDER){
   // check if cylinder exists in _parent_affordance_renderer->otdf_filenames
   filename = "cylinder";
  }
  else if(otdf_id==LEVER)
  {
   filename = "lever";
  }
    
  return filename;
 }
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
  
   //set All Params
   for (size_t i=0; i < (size_t)aff.nparams; i++)
   {   
       instance_struc._otdf_instance->setParam(aff.param_names[i],aff.params[i]);   
   }
   instance_struc._otdf_instance->update();
  
  //TODO: set All JointStates too.
  
  // TODO: create a KDL tree parser from OTDF instance, without having to convert to urdf.
  // otdf can contain some elements that are not part of urdf. e.g. TORUS  
  std::string _urdf_xml_string = otdf::convertObjectInstanceToCompliantURDFstring(instance_struc._otdf_instance);
  
  std::map<std::string, int >::iterator it;
  it= _parent_affordance_renderer->instance_cnt.find(filename);
  it->second = it->second + 1;
  std::stringstream oss;
  oss << filename << "_"<< it->second; //TODO: eventually use object_id? 
  
  instance_struc._collision_detector.reset();
  instance_struc._collision_detector = shared_ptr<Collision_Detector>(new Collision_Detector());
  instance_struc._gl_object = shared_ptr<InteractableGlKinematicBody>(new InteractableGlKinematicBody(instance_struc._otdf_instance,instance_struc._collision_detector,true,aff.name));
  instance_struc._gl_object->set_state(instance_struc._otdf_instance);


  //_parent_affordance_renderer->instantiated_objects.insert(std::make_pair(oss.str(), instance_struc));
  _parent_affordance_renderer->instantiated_objects.insert(std::make_pair(aff.name, instance_struc));
  // Update params 
  
  //Request redraw
  bot_viewer_request_redraw(_parent_affordance_renderer->viewer);
} 


// =======================================================================  
void AffordanceCollectionListener::update_object_instance (const drc::affordance_t &aff)
{
  typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
       object_instance_map_type_::iterator it = _parent_affordance_renderer->instantiated_objects.find(aff.name);
       
   //set All Params
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

