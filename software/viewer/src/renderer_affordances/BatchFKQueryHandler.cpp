#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include <algorithm>
#include "BatchFKQueryHandler.hpp"

using namespace std;
using namespace otdf;
using namespace KDL;
using namespace boost;
using namespace visualization_utils;

namespace renderer_affordances
{


  //==================constructor / destructor

  BatchFKQueryHandler::BatchFKQueryHandler(boost::shared_ptr<otdf::ModelInterface> otdf_instance, int num_of_incs):_num_of_incs(num_of_incs)
  {
    _otdf_instance = otdf::duplicateOTDFInstance(otdf_instance); // create a local placeholder
    
    for(size_t i =0;i< (size_t)_num_of_incs;i++)
    {
      shared_ptr<GlKinematicBody> new_frame;
      new_frame =  shared_ptr<GlKinematicBody>(new GlKinematicBody(_otdf_instance));
      new_frame->set_state(_otdf_instance);
      _frame_list.push_back(new_frame);
    }
  }
 
  BatchFKQueryHandler::~BatchFKQueryHandler() {

  }

 
  
  void BatchFKQueryHandler::doBatchFK(std::string &dof_name,double dof_min,double dof_max, double num_of_incs)
  {
    // call _otdf_instance.setJointState,create and store new _gl_kin_bodies for reach increment
    // enables you to do multiple queries of different link frames, for one fk solve.
    // But you trade off memory.

    //_frame_list.clear();
    
    _dof_names.clear();
    _dof_min.clear();
    _dof_max.clear();
    _num_of_incs= num_of_incs;
    _dof_names.push_back(dof_name);
    _dof_min.push_back(dof_min);
    _dof_max.push_back(dof_max);

    Eigen::VectorXf dof_positions;
    dof_positions.setLinSpaced(num_of_incs,dof_min,dof_max);
    double dof_vel= 0;
    for(size_t i =0;i< (size_t)num_of_incs;i++)
    {
       if(i > _num_of_incs-1)
       {
        shared_ptr<GlKinematicBody> new_frame;
        _otdf_instance->setJointState(dof_name,dof_positions[i],dof_vel);
        new_frame =  shared_ptr<GlKinematicBody>(new GlKinematicBody(_otdf_instance));
        //new_frame->set_state(_otdf_instance);
        _frame_list.push_back(new_frame);
        _num_of_incs++;
        }
       _frame_list[i]->set_state(_otdf_instance);
    }
  }
  

  void BatchFKQueryHandler::doBatchFK(std::vector<std::string> &dof_names,std::vector<double> &dof_min,std::vector<double> &dof_max, double num_of_incs)
  {
    _frame_list.clear();
    _dof_names.clear();
    _dof_min.clear();
    _dof_max.clear();
    _num_of_incs= num_of_incs;

    
    double dof_vel= 0;
    
    int num_of_dofs = dof_names.size();
    std::vector<Eigen::VectorXf> dof_positions_list; 
     for(size_t k =0;k<(size_t)num_of_dofs;k++)
     {
      Eigen::VectorXf dof_positions;
      dof_positions.setLinSpaced(num_of_incs,dof_min[k],dof_max[k]);
      dof_positions_list.push_back(dof_positions);  
      
      // cache to later retrieve a parent dof for any sticky hand or foot
      _dof_names.push_back(dof_names[k]);
      _dof_min.push_back(dof_min[k]);
      _dof_max.push_back(dof_max[k]);
     }
     
    for(size_t i =0;i<num_of_incs;i++)
    {
      shared_ptr<GlKinematicBody> new_frame;
       for(size_t k =0;k<(size_t)num_of_dofs;k++)
       {
          Eigen::VectorXf dof_positions = dof_positions_list[k];
          _otdf_instance->setJointState(dof_names[k],dof_positions[i],dof_vel);
       }
      new_frame =  shared_ptr<GlKinematicBody>(new GlKinematicBody(_otdf_instance));
      new_frame->set_state(_otdf_instance);
      _frame_list.push_back(new_frame);
      _frame_list[i]->set_state(_otdf_instance);
    } 

  }

  
  void BatchFKQueryHandler::getLinkFrames(std::string &link_geometry_name, std::vector<KDL::Frame> &link_frames)
  {
    link_frames.clear();
    KDL::Frame T_world_link;
    for(size_t i = 0; i < _frame_list.size(); i++) 
    { 
        _frame_list[i]->get_link_geometry_frame(link_geometry_name,T_world_link);
        link_frames.push_back(T_world_link);
    }
  }
  
  
  bool BatchFKQueryHandler::getAssociatedDoFNameAndVal(std::string &link_geometry_name,std::string &dof_name, std::vector<double> &dof_values)
  {
  
    std::string link_name;
    _frame_list[0]->get_associated_link_name(link_geometry_name,link_name);

    dof_values.clear();
    std::map<std::string, boost::shared_ptr<Link> >::const_iterator it = _otdf_instance->links_.find(link_name);
    if(it==_otdf_instance->links_.end()){
      cerr << link_name << " not found in otdf instance in BatchFKQueryHandler::getAssociatedDoFNameAndVal \n";
      return false;
    }
    std::vector<std::string>::const_iterator found;
    found = std::find(_dof_names.begin(), _dof_names.end(), it->second->parent_joint->name); 
    if(found != _dof_names.end()) {
        unsigned int index = found - _dof_names.begin();
        dof_name = _dof_names[index];
        Eigen::VectorXf dof_positions;
        dof_positions.setLinSpaced(_num_of_incs,_dof_min[index],_dof_max[index]);
        for(size_t k =0;k<(size_t)_num_of_incs;k++) {
            dof_values.push_back(dof_positions[k]);
        }
    }
    else {
        //cerr <<"ERROR: parent dof for link: "<< link_name << "  " << it->second->parent_joint->name  << " was never set in BatchFK"<< endl;
        return false;
    }    
    
    dof_name = it->second->parent_joint->name;
    return true;
  }
    
//------------------------------------------------------------------------------ 

  


} //end namespace


