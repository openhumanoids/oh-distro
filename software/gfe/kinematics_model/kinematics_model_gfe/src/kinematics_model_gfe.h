#ifndef KINEMATICS_MODEL_KINEMATICS_MODEL_GFE_H
#define KINEMATICS_MODEL_KINEMATICS_MODEL_GFE_H

#include <iostream>
#include <string>

#include <boost/shared_ptr.hpp>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <state/state_gfe.h>

namespace kinematics {
  class Kinematics_Model_GFE {
  public:
    Kinematics_Model_GFE( double eps = 0.001, unsigned int maxIterations = 500 );
    Kinematics_Model_GFE( const std::string& xmlString, double eps = 0.001, unsigned int maxIterations = 500 );
    Kinematics_Model_GFE( const Kinematics_Model_GFE& other );
    ~Kinematics_Model_GFE();
    
    bool inverse_kinematics_left_arm( const state::State_GFE& robotState, const KDL::Frame& pelvisToHandPose, state::State_GFE& solution ); 
    bool inverse_kinematics_right_arm( const state::State_GFE& robotState, const KDL::Frame& pelvisToHandPose, state::State_GFE& solution );
    bool inverse_kinematics_left_leg( const state::State_GFE& robotState, const KDL::Frame& pelvisToFootPose, state::State_GFE& solution ); 
    bool inverse_kinematics_right_leg( const state::State_GFE& robotState, const KDL::Frame& pelvisToFootPose, state::State_GFE& solution );

    static std::string urdf_filename_to_xml_string( std::string urdfFilename );

    bool load_xml_string( std::string xmlString, double eps, unsigned int maxIterations ); 
    bool load_urdf( std::string urdf, double eps, unsigned int maxIterations );
    void set( const drc::robot_state_t& robotState );  
    void set( state::State_GFE& stateGFE );

    const urdf::Model& model( void )const;
    const KDL::Tree& tree( void )const;
    KDL::Frame link( std::string linkName )const;
    std::map< std::string, KDL::Frame > link_frames( void )const;
    const KDL::JntArray& min_joint_limits_left_arm( void )const;
    const KDL::JntArray& max_joint_limits_left_arm( void )const;
    const KDL::JntArray& min_joint_limits_right_arm( void )const;
    const KDL::JntArray& max_joint_limits_right_arm( void )const;
    const KDL::JntArray& min_joint_limits_left_leg( void )const;
    const KDL::JntArray& max_joint_limits_left_leg( void )const;
    const KDL::JntArray& min_joint_limits_right_leg( void )const;
    const KDL::JntArray& max_joint_limits_right_leg( void )const;
    const KDL::Chain& left_arm_chain( void )const;
    const KDL::Chain& right_arm_chain( void )const;
    const KDL::Chain& left_leg_chain( void )const;
    const KDL::Chain& right_leg_chain( void )const;
    

  protected:
    void _update_joint( std::map< std::string, double >& jointAngles, KDL::Frame& parentFrame, const KDL::SegmentMap::const_iterator jointIterator );

    urdf::Model _model;
    KDL::Tree   _tree;
    KDL::TreeFkSolverPos_recursive * _fk_solver;
    KDL::ChainIkSolverPos_LMA * _iksolverpos_left_arm;
    KDL::ChainIkSolverPos_LMA * _iksolverpos_right_arm;
    KDL::ChainIkSolverPos_LMA * _iksolverpos_left_leg;
    KDL::ChainIkSolverPos_LMA * _iksolverpos_right_leg;
    KDL::Frame    _world_to_body;
    std::map< std::string, KDL::Frame > _link_frames;
    KDL::Chain _left_arm_chain;
    KDL::Chain _right_arm_chain;
    KDL::Chain _left_leg_chain;
    KDL::Chain _right_leg_chain;
    KDL::JntArray _min_joint_limits_left_arm;
    KDL::JntArray _max_joint_limits_left_arm;
    KDL::JntArray _min_joint_limits_right_arm;
    KDL::JntArray _max_joint_limits_right_arm;
    KDL::JntArray _min_joint_limits_left_leg;
    KDL::JntArray _max_joint_limits_left_leg;
    KDL::JntArray _min_joint_limits_right_leg;
    KDL::JntArray _max_joint_limits_right_leg;
  
  private:
  
  };
  std::ostream& operator<<( std::ostream& out, const Kinematics_Model_GFE& other );
}

#endif /* KINEMATICS_MODEL_KINEMATICS_MODEL_GFE */
