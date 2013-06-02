#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include "CandidateFootStepSeedManager.hpp"
#include <algorithm>

using namespace std;
using namespace boost;
using namespace visualization_utils;
using namespace collision;

namespace renderer_affordances 
{
    //==================constructor / destructor

    CandidateFootStepSeedManager::CandidateFootStepSeedManager(RendererAffordances* affordance_renderer):
        _parent_renderer(affordance_renderer),_foot_urdfs_found(false)
    {
 
        _lcm = affordance_renderer->lcm; 
    
        //lcm ok?
        if(!_lcm->good()) {
            cerr << "\nLCM Not Good: Candidate FootStep Seed Manager" << endl;
            return;
        }
    
        _T_groundframe_bodyframe_left = KDL::Frame::Identity();
        _T_groundframe_bodyframe_right = KDL::Frame::Identity();
    
        _left_foot_name ="l_foot";
        _right_foot_name = "r_foot";
        if(!load_foot_urdfs()){
            cerr << "##### ERROR: #####" <<  " sticky_foot urdfs not found in CandidateFootStepSeedManager.cpp. Disabling CandidateFootStepSeed Manager in renderer_affordances. Please update your models folder" << endl;    
            return;
        }
        _foot_urdfs_found= true;
        _base_gl_foot_left =  shared_ptr<GlKinematicBody>(new GlKinematicBody(_left_urdf_xml_string));     
        _base_gl_foot_right =  shared_ptr<GlKinematicBody>(new GlKinematicBody(_right_urdf_xml_string));    
    
        std::map<std::string, double> jointpos_in;
        jointpos_in =  _base_gl_foot_left->_current_jointpos;
        _base_gl_foot_left->set_state(_base_gl_foot_left->_T_world_body,jointpos_in); // set to initialized values.
        jointpos_in.clear();
        jointpos_in =  _base_gl_foot_right->_current_jointpos;
        _base_gl_foot_right->set_state(_base_gl_foot_left->_T_world_body,jointpos_in); // set to initialized values.  
    
    
        Eigen::Vector3f whole_body_span;
        Eigen::Vector3f offset;
        MeshStruct mesh_struct;
        bool val;
        _base_gl_foot_left->get_whole_body_span_dims(whole_body_span,offset);
    
        val =_base_gl_foot_left->get_mesh_struct("l_talus_0", mesh_struct);
        _T_groundframe_bodyframe_left.p[2] = whole_body_span[2]-0.5*(mesh_struct.span_z);    
    
        _base_gl_foot_right->get_whole_body_span_dims(whole_body_span,offset);
        val = _base_gl_foot_right->get_mesh_struct("r_talus_0", mesh_struct); 
        _T_groundframe_bodyframe_right.p[2] = whole_body_span[2]-0.5*(mesh_struct.span_z);      

    }
 
    CandidateFootStepSeedManager::~CandidateFootStepSeedManager() {

    }


    //------------------------------------------------------------------------------ 

    bool  CandidateFootStepSeedManager::load_foot_urdfs()
    {   

        string urdf_models_path = string(getModelsPath()) + "/mit_gazebo_models/mit_robot_feet/"; // getModelsPath gives /drc/software/build/models/
        cout << "searching for foot urdf files in: "<< (urdf_models_path) << endl;
        vector<string> urdf_files = vector<string>();
        int res = get_URDF_filenames_from_dir(urdf_models_path, urdf_files);  
  
     
        if(res==0)  //urdf found
            cout << "found " << urdf_files.size() << " " << urdf_files[0] << " " << urdf_files[1] << " files"<< endl;
        else{
            cerr << "ERROR: no urdf files found in: "<< (urdf_models_path) << endl;
            return false;
        } 
        
        std::vector<std::string>::const_iterator found;
        found = std::find (urdf_files.begin(), urdf_files.end(), _left_foot_name); 
        if(found !=  urdf_files.end()) {
            unsigned int index = found - urdf_files.begin();
            std::stringstream oss;
            oss << urdf_models_path << urdf_files[index] << ".urdf" ;
            get_xmlstring_from_file(oss.str(), _left_urdf_xml_string);
        }
        else {
            cerr <<"ERROR: " << _left_foot_name  << ".urdf not found"<< endl;
            return false;
        }

        found = std::find (urdf_files.begin(), urdf_files.end(), _right_foot_name);  
        if(found !=  urdf_files.end()) {
            unsigned int index = found - urdf_files.begin();
            std::stringstream oss;
            oss << urdf_models_path << urdf_files[index] << ".urdf" ;
            get_xmlstring_from_file(oss.str(), _right_urdf_xml_string);
        }
        else {
            cerr <<"ERROR:" << _right_foot_name  << ".urdf not found"<< endl;
            return false;
        }
        return true;
    }
  
  
    //-------------------------------------------------------------------------------------------

    void CandidateFootStepSeedManager::add_or_update_sticky_foot(int uid,int foot_type, string& object_name, string& geometry_name, 
                                                                 KDL::Frame &T_world_foot, vector<string> &joint_names,vector<double> &joint_positions)		
    {
    
        if(!_foot_urdfs_found) {
            cerr << "##### ERROR: #####" <<  " sticky_foot urdfs not found in CandidateFootStepSeedManager.cpp. Cannot spawn sticky feet" << endl;    
            return;    
        }
        
        string  unique_foot_name;
        std::stringstream oss;
        if(foot_type==0)
            oss << object_name <<"_"<< geometry_name << "_lfootstep_" << uid;
        else
            oss << object_name <<"_"<< geometry_name << "_rfootstep_" << uid;
        unique_foot_name = oss.str(); 
  
        typedef std::map<std::string,StickyFootStruc> sticky_feet_map_type;
        sticky_feet_map_type::iterator it = _parent_renderer->sticky_feet.find(unique_foot_name);
        if(it ==_parent_renderer->sticky_feet.end() ) { // not in cache
     
            StickyFootStruc sticky_foot_struc;
            sticky_foot_struc.object_name = object_name;//.c_str() ;
            sticky_foot_struc.geometry_name = geometry_name;//.c_str(); if created here as na c str it goes out of scope when called from top level functions, hence using string.
            sticky_foot_struc.uid = uid; 
            sticky_foot_struc._collision_detector.reset();
            // should we have a global collision detector and add and remove objects to it as we create and delete objects and foots. We would have to manually add and remove multiple links to the collision detector.??
            // Each foot has its own collision detector for now.
            sticky_foot_struc._collision_detector = shared_ptr<Collision_Detector>(new Collision_Detector()); 
            sticky_foot_struc.foot_type=foot_type;
      
            if(foot_type==0)// LEFT
                    sticky_foot_struc._gl_foot = shared_ptr<InteractableGlKinematicBody>(new InteractableGlKinematicBody((*_base_gl_foot_left),sticky_foot_struc._collision_detector,true,unique_foot_name));
            else if(foot_type==1)// RIGHT
                    sticky_foot_struc._gl_foot = shared_ptr<InteractableGlKinematicBody>(new InteractableGlKinematicBody((*_base_gl_foot_right),sticky_foot_struc._collision_detector,true,unique_foot_name));
      
            drc::joint_angles_t posture_msg;
            posture_msg.num_joints = joint_names.size();
            posture_msg.joint_name = joint_names;
            posture_msg.joint_position = joint_positions;
      
            sticky_foot_struc._gl_foot->set_state(T_world_foot, posture_msg);
            sticky_foot_struc.foot_type = foot_type;
            sticky_foot_struc.T_geometry_foot = T_world_foot;

            sticky_foot_struc.joint_name = joint_names;
            sticky_foot_struc.joint_position = joint_positions;
            _parent_renderer->sticky_feet.insert(make_pair(unique_foot_name, sticky_foot_struc));
        }
        else {
            drc::joint_angles_t posture_msg;
            posture_msg.num_joints= joint_names.size();
            posture_msg.joint_name= joint_names;
            posture_msg.joint_position= joint_positions;

            it->second._gl_foot->set_state(T_world_foot, posture_msg);
            it->second.T_geometry_foot = T_world_foot; 
            it->second.joint_position = joint_positions;
        }
    }
    


} //end namespace


