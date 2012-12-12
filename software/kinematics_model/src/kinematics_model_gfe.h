#ifndef KINEMATICS_MODEL_KINEMATICS_MODEL_GFE_H
#define KINEMATICS_MODEL_KINEMATICS_MODEL_GFE_H

#include <iostream>
#include <string>

#include <boost/shared_ptr.hpp>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>

namespace kinematics_model {
  class Kinematics_Model_GFE {
  public:
    Kinematics_Model_GFE();
    Kinematics_Model_GFE( std::string urdfFilename );
    Kinematics_Model_GFE( const Kinematics_Model_GFE& other );
    ~Kinematics_Model_GFE();

    bool load_urdf( std::string urdf );
    void set( drc::robot_state_t& robotState );  

    std::string urdf_filename( void )const;
    const urdf::Model& model( void )const;
    const KDL::Tree& tree( void )const;
    KDL::Frame link( std::string linkID )const;

  protected:
    std::string _urdf_filename;
    urdf::Model _model;
    KDL::Tree   _tree;
    KDL::TreeFkSolverPos_recursive * _fk_solver;
    KDL::Frame    _world_to_body;
    KDL::JntArray _joint_angles;

  private:
  
  };
  std::ostream& operator<<( std::ostream& out, const Kinematics_Model_GFE& other );
}

#endif /* KINEMATICS_MODEL_KINEMATICS_MODEL_GFE */
