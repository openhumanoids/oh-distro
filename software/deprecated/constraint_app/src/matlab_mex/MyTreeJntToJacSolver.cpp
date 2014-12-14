#include "MyTreeJntToJacSolver.hpp"
#include <iostream>
#include <kdl/kinfam_io.hpp>
#include <kdl/treejnttojacsolver.hpp>

using namespace KDL;

MyTreeJntToJacSolver::MyTreeJntToJacSolver(const Tree& tree_in) :
    tree(tree_in) {
}

MyTreeJntToJacSolver::~MyTreeJntToJacSolver() {
}

int MyTreeJntToJacSolver::JntToJac(const JntArray& q_in, Jacobian& jac, const std::string& segmentname) {
    //First we check all the sizes:
    if (q_in.rows() != tree.getNrOfJoints() || jac.columns() != tree.getNrOfJoints())
        return -1;
    
    //Lets search the tree-element
    SegmentMap::const_iterator it = tree.getSegments().find(segmentname);

    //If segmentname is not inside the tree, back out:
    if (it == tree.getSegments().end())
        return -2;
    
    //Let's make the jacobian zero:
    SetToZero(jac);
    
    SegmentMap::const_iterator root = tree.getRootSegment();

    Frame T_total = Frame::Identity();
    Frame T_local;
    //Lets recursively iterate until we are in the root segment
    while (it != root) {
        //get the corresponding q_nr for this TreeElement:
        unsigned int q_nr = it->second.q_nr;

	//get the pose of the segment:
	std::string thisSegmentName(it->second.segment.getName());
	if ( thisSegmentName == "segment_roll" ) {
	  //compute the T_local all the way from segment_x
	  SegmentMap::const_iterator it_temp = it;

	  while ( it_temp != root ) { 
	    unsigned int q_nr_temp = it_temp->second.q_nr;
	    Frame T_local_temp = it_temp->second.segment.pose(q_in(q_nr_temp));
	    T_local = T_local_temp * T_local;
	    if ( it_temp->second.segment.getName() == "segment_x" ) break;
	    it_temp = it_temp->second.parent;
	  }
	  if ( it_temp == root ) return -3;
	  T_total = T_local * T_total;

	} else if ( thisSegmentName == "segment_pitch" || 
		    thisSegmentName == "segment_yaw" ||
		    thisSegmentName == "segment_z" ||
		    thisSegmentName == "segment_y" ||
		    thisSegmentName == "segment_x" ) {
	  //do nothing because we've all these joints take place at the same point
	} else {
	  // this is just a normal joint
	  T_local = it->second.segment.pose(q_in(q_nr));
	  T_total = T_local * T_total;
	}	         
        //Frame T_local = it->second.segment.pose(q_in(q_nr));
        //calculate new T_end:
        //T_total = T_local * T_total;
        
        //get the twist of the segment:
        if (it->second.segment.getJoint().getType() != Joint::None) {
            Twist t_local = it->second.segment.twist(q_in(q_nr), 1.0);
            //transform the endpoint of the local twist to the global endpoint:
            t_local = t_local.RefPoint(T_total.p - T_local.p);
            //transform the base of the twist to the endpoint
            t_local = T_total.M.Inverse(t_local);
            //store the twist in the jacobian:
            jac.setColumn(q_nr,t_local);
        }//endif
        //goto the parent
        it = it->second.parent;
    }//endwhile
    //Change the base of the complete jacobian from the endpoint to the base
    changeBase(jac, T_total.M, jac);
    
    return 0;
    
}//end JntToJac


