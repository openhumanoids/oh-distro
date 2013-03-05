#ifndef KINEMATICS_MODEL_KINEMATICS_MODEL_URDF_H
#define KINEMATICS_MODEL_KINEMATICS_MODEL_URDF_H

#include <iostream>
#include <string>

#include <boost/shared_ptr.hpp>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>

namespace kinematics {
  class Kinematics_Model_URDF {
  public:
    Kinematics_Model_URDF();
    Kinematics_Model_URDF( std::string xmlString );
    Kinematics_Model_URDF( const Kinematics_Model_URDF& other );
    ~Kinematics_Model_URDF();
    
    bool load_xml_string( std::string xmlString ); 
    bool load_urdf( std::string urdf );

    const urdf::Model& model( void )const;
    const KDL::Tree& tree( void )const;
    KDL::Frame link( std::string linkName )const;

  protected:
    void _update_joint( std::map< std::string, double >& jointAngles, KDL::Frame& parentFrame, const KDL::SegmentMap::const_iterator jointIterator );

    urdf::Model _model;
    KDL::Tree   _tree;
    KDL::TreeFkSolverPos_recursive * _fk_solver;
    KDL::Frame    _world_to_body;
    std::map< std::string, KDL::Frame > _joint_frames;
  
  private:
  
  };
  std::ostream& operator<<( std::ostream& out, const Kinematics_Model_URDF& other );
}

#endif /* KINEMATICS_MODEL_KINEMATICS_MODEL_URDF */
