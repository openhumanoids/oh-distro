#ifndef BATCHFKQUERYHANDLER_HPP
#define BATCHFKQUERYHANDLER_HPP

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <string> 
#include <vector>
#include <math.h>
#include <fstream>
#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <errno.h>
#include <dirent.h>
#include <boost/function.hpp>
#include <map>
#include <otdf_parser/otdf_parser.h>
#include <visualization_utils/GlKinematicBody.hpp>
#include <visualization_utils/InteractableGlKinematicBody.hpp>
#include <visualization_utils/angles.hpp>
#include <visualization_utils/eigen_kdl_conversions.hpp>
#include <visualization_utils/file_access_utils.hpp>
#include "lcmtypes/bot_core.hpp"
#include <Eigen/Dense>

namespace renderer_affordances
{

//  Usage
//  ---------------
//  BatchFKQueryHandler dofRangeFkQueryHandler(_otdf_instance);
//  batckfksolver.doBatchFK("steering_joint",-1,1,10);
//  std::vector<KDL::Frame> link_frames;
//  batckfksolver.getLinkFrames("cylinder_1",link_frames);
//        (or)
//  BatchFKQueryHandler dofRangeFkQueryHandler(_otdf_instance);
//  std::vector<std::string> dof_names;
//  std::vector<double> dof_min,dof_max;
//  dof_names.push_back("steering_joint");dof_names.push_back("gas_pedal_joint");
//  dof_min.push_back(-1);dof_min.push_back(0);
//  dof_max.push_back(1);dof_max.push_back(0.05);
//  int num_of_increments = 10;
//  batckfksolver.doBatchFK(dof_names,dof_min,dof_max,num_of_increments);
//  std::vector<KDL::Frame> link_frames;
//  batckfksolver.getLinkFrames("cylinder_1",link_frames);

  class BatchFKQueryHandler 
  {
    //--------fields
  private:
    boost::shared_ptr<otdf::ModelInterface> _otdf_instance; 
    std::vector< boost::shared_ptr<visualization_utils::GlKinematicBody> >  _frame_list;
    
    std::vector<std::string> _dof_names;
    std::vector<double> _dof_min;
    std::vector<double> _dof_max;
    int _num_of_incs;
   
    //----------------constructor/destructor
  public:
    BatchFKQueryHandler(boost::shared_ptr<otdf::ModelInterface> otdf_instance, int num_of_incs);
    ~BatchFKQueryHandler();
    
   void doBatchFK(std::string &dof_name,double dof_min,double dof_max, double num_of_incs);
   void doBatchFK(std::vector<std::string> &dof_names,std::vector<double> &dof_min,std::vector<double> &dof_max, double num_of_incs);
   void getLinkFrames(std::string &link_name, std::vector<KDL::Frame> &link_frames);
   bool getAssociatedDoFNameAndVal(std::string &link_name, std::string &dof_name, std::vector<double> &dof_values);
  }; //class BatchFKQueryHandler


} //end namespace


#endif //BATCHFKQUERYHANDLER_HPP
