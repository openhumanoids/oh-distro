#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include <iostream>

#include <lcmtypes/visualization.h>

#include <path_util/path_util.h>

#include "constraint_app.hpp"
#include <boost/shared_ptr.hpp>

constraint_app::constraint_app(lcm_t* publish_lcm) :
  publish_lcm_(publish_lcm) 
{
  // LCM:
  lcm_t* subscribe_lcm_ = publish_lcm_;

  drc_affordance_collection_t_subscribe(subscribe_lcm_, "AFFORDANCE_COLLECTION",
					affordance_collection_handler_aux, this);

  drc_affordance_track_collection_t_subscribe(subscribe_lcm_, "AFFORDANCE_TRACK_COLLECTION",
					      affordance_track_collection_handler_aux, this);
}

void constraint_app::affordance_collection_handler(const drc_affordance_collection_t *msg)
{
  std::cout << "got a new affordance collection" << std::endl;

  for (size_t i=0; i< (size_t)msg->naffs; i++) {
    const drc_affordance_t aff(msg->affs[i]);
    const std::string filename(aff.otdf_type);

    // load the OTDF file
    std::string xml_string;
    if ( !otdf::get_xml_string_from_file(filename, xml_string) ) {
      std::cerr << "unable to get_xml_string_from_file " << filename << std::endl;
      continue; // file extraction failed
    }

    //convert OTDF to URDF
    boost::shared_ptr<otdf::ModelInterface> otdf_instance(otdf::parseOTDF(xml_string));
    std::string urdf_xml_string(otdf::convertObjectInstanceToCompliantURDFstring(otdf_instance));

    // Parse KDL tree
    KDL::Tree tree;
    if ( !kdl_parser::treeFromString(urdf_xml_string,tree) ) {
      std::cerr << "ERROR: Failed to extract kdl tree from "  << otdf_instance->name_ 
		<< " otdf object instance "<< std::endl; 
    }

    std::cout << "created KDL object with " << tree.getNrOfJoints() << " joints and " 
	      << tree.getNrOfSegments() << " segments" << std::endl;
    print_kdl_tree(tree, tree.getRootSegment()->second, 0);
  }

}

void constraint_app::print_kdl_tree(const KDL::Tree& tree, 
				    const KDL::TreeElement& segment,
				    unsigned int depth) 
{
  for ( unsigned int i = 0; i < depth; i++ ) {
    std::cout << " ";
  }
  std::cout << segment.segment.getName() 
	    << ", joint " << segment.segment.getJoint().getName() 
	    << " of type " << segment.segment.getJoint().getTypeName()
	    << std::endl;
  for ( unsigned int i = 0; i < segment.children.size(); i++ ) {
    print_kdl_tree(tree, segment.children[i]->second, depth+1);
  }
  
}

void constraint_app::affordance_track_collection_handler(const drc_affordance_track_collection_t *msg)
{
  std::cout << "got a new track collection" << std::endl;

  std::cout << "sent a new (empty) estimate of joint angles" << std::endl;
  drc_affordance_joint_angles_t aja;
  aja.utime = msg->utime;
  aja.uid = msg->uid;
  drc_affordance_joint_angles_t_publish(publish_lcm_, "AFFORDANCE_JOINT_ANGLES", &aja);
}

int main(int argc, char ** argv)
{
  lcm_t * lcm;
  lcm = lcm_create(NULL);
  constraint_app app(lcm);

  while(1)
    lcm_handle(lcm);

  lcm_destroy(lcm);
  return 0;
}

