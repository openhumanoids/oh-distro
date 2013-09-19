#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include <iostream>
#include <lcmtypes/visualization.h>
#include <path_util/path_util.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/format.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <lcm/lcm-cpp.hpp>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <forward_kinematics/treefksolverposfull_recursive.hpp>
#include <otdf_parser/otdf_parser.h>
#include <otdf_parser/otdf_urdf_converter.h>
#include "mytreefksolverpos_recursive.hpp"
#include <bot_lcmgl_client/lcmgl.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <lcmtypes/bot_param/update_t.hpp>
#include <math.h>

boost::mutex idMutex;
boost::condition_variable idCondition;
boost::mutex readyMutex;
boost::condition_variable readyCondition;
volatile bool idValid = false;
volatile bool matlabReady = false;
volatile int affordance_id = 0;
KDL::Tree tree;

void print_kdl_tree(const KDL::Tree& tree, 
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

void loadTree(const std::string& filename)
{
  // load the OTDF file
  std::string xml_string;
  if ( !otdf::get_xml_string_from_file(filename, xml_string) ) {
    std::cerr << "unable to get_xml_string_from_file " << filename << std::endl;
    exit(-1);
  }

  //convert OTDF to URDF
  boost::shared_ptr<otdf::ModelInterface> otdf_instance(otdf::parseOTDF(xml_string));
  std::string urdf_xml_string(otdf::convertObjectInstanceToCompliantURDFstring(otdf_instance));
  
  // Parse KDL tree
  if ( !kdl_parser::treeFromString(urdf_xml_string,tree) ) {
    std::cerr << "ERROR: Failed to extract kdl tree from "  << otdf_instance->name_ 
	      << " otdf object instance "<< std::endl; 
  }

  std::cout << "created KDL object with " << tree.getNrOfJoints() << " joints and " 
	    << tree.getNrOfSegments() << " segments" << std::endl;
  print_kdl_tree(tree, tree.getRootSegment()->second, 0);
}

void runPopulate(lcm::LCM* lcm)
{
  bot_lcmgl_t* lcmgl = bot_lcmgl_init(lcm->getUnderlyingLCM(), "ca_test");

  std::string otdf_name("fixed_valve_task_wall3");

  loadTree(otdf_name);
  MyTreeFkSolverPos_recursive fk(tree);

  drc::affordance_plus_t affordance_plus;
  drc::affordance_plus_t affordance_plus_truth;
  drc::affordance_t& affordance(affordance_plus.aff);
  affordance.utime = 0;
  affordance.otdf_type = otdf_name;
  affordance.friendly_name = "test";
  affordance.uid = affordance_id;
  affordance.aff_store_control = drc::affordance_t::NEW;
  affordance.origin_xyz[0] = 0.0;
  affordance.origin_xyz[1] = 0.0;
  affordance.origin_xyz[2] = 0.0;
  affordance.origin_rpy[0] = 3.14;
  affordance.origin_rpy[1] = 0.0;
  affordance.origin_rpy[2] = 0.0;
  affordance.nparams = 0;

  affordance.nstates = tree.getNrOfJoints();
  affordance.states.resize(affordance.nstates);
  affordance.state_names.resize(affordance.nstates);
  KDL::JntArray joints(tree.getNrOfJoints());
  for ( int i = 0; i < tree.getNrOfJoints(); i++ ) {
    affordance.states[i] = 0.0;
    affordance.state_names[i] = boost::str(boost::format("base_joint%i") % i);
    joints(i) = affordance.states[i];
  }

  affordance.bounding_xyz[0] = 0.0;
  affordance.bounding_xyz[1] = 0.0;
  affordance.bounding_xyz[2] = 0.0;
  affordance.bounding_rpy[0] = 0.0;
  affordance.bounding_rpy[1] = 0.0;
  affordance.bounding_rpy[2] = 0.0;
  affordance.bounding_lwh[0] = 0.0;
  affordance.bounding_lwh[1] = 0.0;
  affordance.bounding_lwh[2] = 0.0;
  affordance_plus.npoints = 0;
  affordance_plus.ntriangles = 0;

  //wait for matlab to be ready
  {
    boost::mutex::scoped_lock lock(readyMutex);
    while ( !matlabReady ) {
      std::cout << "waiting on matlab to be ready" << std::endl;
      boost::system_time timeout = boost::get_system_time() + 
	boost::posix_time::milliseconds(1000);
      readyCondition.timed_wait(lock, timeout);
    }
  }

  // publish to the affordance server
  std::cout << "publishing AFFORDANCE_FIT for id=" << affordance_plus.aff.uid << std::endl;
  lcm->publish("AFFORDANCE_FIT", &affordance_plus);  //this will draw it in the viewer for debugging

  //wait for the next response message and retrieve the uid
  {
    boost::mutex::scoped_lock lock(idMutex);
    while ( !idValid ) {
      std::cout << "waiting on lcm message from affordance server" << std::endl;
      boost::system_time timeout = boost::get_system_time() + 
	boost::posix_time::milliseconds(1000);
      idCondition.timed_wait(lock, timeout);
    }
  }

  //start using this uid
  affordance.uid = affordance_id;
  std::cout << "publishing CAM_AFFORDANCE_FIT for id=" << affordance_plus.aff.uid << std::endl;
  lcm->publish("CAM_AFFORDANCE_FIT", &affordance_plus); //this will initialize our tracker with the fit


  int ring_segs[3] = {0, 6, 12};
  KDL::Frame base_expressedIn_world;
  base_expressedIn_world.p[0] = affordance.origin_xyz[0];
  base_expressedIn_world.p[1] = affordance.origin_xyz[1];
  base_expressedIn_world.p[2] = affordance.origin_xyz[2];
  base_expressedIn_world.M = KDL::Rotation::RPY(affordance.origin_rpy[0], affordance.origin_rpy[1], affordance.origin_rpy[2]);

  affordance_plus_truth = affordance_plus;
  affordance_plus_truth.aff.uid++;
  affordance_plus_truth.aff.aff_store_control = drc::affordance_t::NEW;
  affordance_plus_truth.aff.origin_xyz[2] += 1.0;

  for ( int i = 0; i < 20; i ++ ) {
    
    std::cout << "publishing" << std::endl;

    affordance.aff_store_control = drc::affordance_t::UPDATE;
    for ( int j = 0; j < tree.getNrOfJoints(); j++ ) {
      affordance.states[j] = fmod(i / 10.0 + j, 2.0 * M_PI);
      if ( affordance.states[j] > M_PI ) {
	affordance.states[j] = affordance.states[j] - 2.0 * M_PI;
      }
      joints(j) = affordance.states[j];
    }

    std::cout << "publishing ground truth " << affordance_plus_truth.aff.uid << std::endl;
    affordance_plus_truth.aff.states = affordance.states;
    lcm->publish("AFFORDANCE_FIT", &affordance_plus_truth);
    affordance_plus_truth.aff.aff_store_control = drc::affordance_t::UPDATE;

    MyTreeFkSolverPos_recursive::SegmentToPoseMap map;
    int fkret = fk.JntToCart(joints, map);
    if ( fkret < 0 ) {
      std::cout << "error while getting forward kinematics: " << fkret << std::endl;
      continue;
    }

    bot_lcmgl_enable(lcmgl, GL_DEPTH_TEST);
    bot_lcmgl_color3f(lcmgl, 1, 0, 0);
    bot_lcmgl_point_size(lcmgl, 3);
    bot_lcmgl_begin(lcmgl, LCMGL_LINES);

    drc::affordance_track_collection_t tracks;
    tracks.utime = 0;
    tracks.uid = affordance.uid;

    for ( MyTreeFkSolverPos_recursive::SegmentToPoseMap::const_iterator iter = map.begin();
	  iter != map.end(); ++iter ) {
      
      for ( int j = 0; j < (otdf_name == "fixed_valve_task_wall" ? 2 : 3); j++ ) {
	for ( int s = 0; s < 3; s++ ) {
	  //std::string seg( (j==0) ? boost::str(boost::format("RING_%i") % ring_segs[s] ) : 
	  //		   boost::str(boost::format("RING%i_%i") % (j+1) % ring_segs[s]) );
	  std::string spoke(boost::str(boost::format("Spokes%i_%i") % j % s));
	  if ( spoke == iter->first ) {

	    const KDL::Frame& pt_expressedIn_base(iter->second);
	    double xyzrpw[6] = { pt_expressedIn_base.p[0], pt_expressedIn_base.p[1], pt_expressedIn_base.p[2], 0.0, 0.0, 0.0 };
	    pt_expressedIn_base.M.GetRPY(xyzrpw[3], xyzrpw[4], xyzrpw[5]);
	    KDL::Frame pt_expressedIn_world = base_expressedIn_world * pt_expressedIn_base;
	    KDL::Frame pt2_expressedIn_pt;
	    pt2_expressedIn_pt.p[0] = 0.3;
	    KDL::Frame pt3_expressedIn_pt;
	    pt3_expressedIn_pt.p[2] = 0.3;
	    KDL::Frame pt2_expressedIn_world = base_expressedIn_world * pt_expressedIn_base * pt2_expressedIn_pt;
	    KDL::Frame pt3_expressedIn_world = base_expressedIn_world * pt_expressedIn_base * pt3_expressedIn_pt;
	    std::cout << "   " << iter->first << ", " << joints(j) << std::endl;	    
	   
	    bot_lcmgl_vertex3f(lcmgl, pt_expressedIn_world.p[0], pt_expressedIn_world.p[1], pt_expressedIn_world.p[2]); 
	    bot_lcmgl_vertex3f(lcmgl, pt2_expressedIn_world.p[0], pt2_expressedIn_world.p[1], pt2_expressedIn_world.p[2]); 

	    bot_lcmgl_vertex3f(lcmgl, pt_expressedIn_world.p[0], pt_expressedIn_world.p[1], pt_expressedIn_world.p[2]); 
	    bot_lcmgl_vertex3f(lcmgl, pt3_expressedIn_world.p[0], pt3_expressedIn_world.p[1], pt3_expressedIn_world.p[2]); 

	    drc::affordance_track_t track;
	    track.segment = iter->first;
	    track.position.x = pt_expressedIn_world.p[0];
	    track.position.y = pt_expressedIn_world.p[1];
	    track.position.z = pt_expressedIn_world.p[2];
	    track.id = 0;

	    tracks.tracks.push_back(track);
	  }
	}
      }
    }

    tracks.ntracks = tracks.tracks.size();
    lcm->publish("AFFORDANCE_DETECTIONS", &tracks);

    bot_lcmgl_end(lcmgl);
    bot_lcmgl_switch_buffer(lcmgl);

    boost::this_thread::sleep(boost::posix_time::seconds(2));
  }

  std::cout << "done publishing" << std::endl;
  bot_lcmgl_destroy(lcmgl);

  exit(0);
}

class dummy {
public:
void handleAffordanceCollection(const lcm::ReceiveBuffer* rbuf, 
				const std::string& channel,
				const drc::affordance_plus_collection_t *affordance_plus_col)
{
  boost::mutex::scoped_lock lock(idMutex);

  std::cout << "got a collection" << std::endl;

  if ( idValid ) {
    std::cout << "ignoring collection" << std::endl;
    return;
  }

  if ( affordance_plus_col->naffs == 0 ) return;
  
  for ( int i = 0; i < affordance_plus_col->naffs; i++ ) {
    if ( affordance_plus_col->affs_plus[i].aff.friendly_name == "test" ) {
      int this_uid = affordance_plus_col->affs_plus[i].aff.uid;
      if ( this_uid >= affordance_id ) {
	affordance_id = this_uid;
	std::cout << " " << affordance_id << std::endl;
      }
    }
  }

  idValid = true;
  idCondition.notify_all();
}

void handleTrackPFStatus(const lcm::ReceiveBuffer* rbuf, 
			 const std::string& channel,
			 const bot_param::update_t *update)
{
  if ( update->params == "READY" ) {
    boost::mutex::scoped_lock lock(readyMutex);
    matlabReady = true;
    readyCondition.notify_all();
  }
}
};

int main(int argc, char ** argv)
{
  lcm::LCM lcm;
  dummy d;

  lcm.subscribe("AFFORDANCE_PLUS_COLLECTION", &dummy::handleAffordanceCollection, &d);
  lcm.subscribe("TRACK_PF_STATUS", &dummy::handleTrackPFStatus, &d);

  std::cout << "starting..." << std::endl;
  boost::thread populateThread = boost::thread(runPopulate, &lcm);

  while (0 == lcm.handle()) {


  }

  return 0;
}
