/*
 * AffordanceDrawingArea.h
 *
 *  Created on: Oct 5, 2012
 *      Author: ppetrova
 */

#ifndef AFFORDANCEDRAWINGAREA_H_
#define AFFORDANCEDRAWINGAREA_H_

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
#include <string>

#include <errno.h>
#include <dirent.h>

#include <lcm/lcm_coretypes.h>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <boost/shared_ptr.hpp>

#include <path_util/path_util.h>

#include <bot_vis/bot_vis.h>
#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>

#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <forward_kinematics/treefksolverposfull_recursive.hpp>
#include <otdf_parser/otdf_parser.h>
#include <otdf_parser/otdf_urdf_converter.h>

#include "GlDrawingArea.h"
#include "otdf_parser/otdf_parser.h"

namespace gui {

class AffordanceDrawingArea : public GlDrawingArea {
public:
	AffordanceDrawingArea(boost::shared_ptr<otdf::ModelInterface> otdfObject, boost::shared_ptr<lcm::LCM> &lcm);
	//	AffordanceDrawingArea(const AffordanceDrawingArea& other);
	virtual ~AffordanceDrawingArea();
	void on_param_change(std::string parameter_name, double delta);
protected:
	bool on_expose_event(GdkEventExpose* event);
private:
	void run_fk_and_gen_link_shapes_and_tfs ();
	void update_OtdfInstance ();
	void handleAffordanceParameterMsg(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const drc::affordance_parameter_t* msg);
	boost::shared_ptr<otdf::ModelInterface> _otdfObject;
	boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> _fksolver;
	std::vector<boost::shared_ptr<otdf::Geometry> > _link_shapes;
	std::vector<drc::link_transform_t> _link_tfs;
};
}

#endif /* AFFORDANCEDRAWINGAREA_H_ */
