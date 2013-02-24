#include "ManipulationRelation.h"
#include "PointContactRelation.h"

using namespace action_authoring;
using namespace affordance;

ManipulationRelation::
ManipulationRelation(AffConstPtr affordance,
                     ManipulatorStateConstPtr manipulator,
                     RelationStatePtr relationState)
    : _affordance(affordance),
      _manipulator(manipulator),
      _relationState(relationState)
{

}

drc::contact_goal_t ManipulationRelation::toLCM()
{
	printf("creating LCM message\n");
	drc::contact_goal_t msg;
	//TODO change contact type based on affordance
	msg.contact_type = 0;
	msg.object_1_name = _manipulator->getName();
	msg.object_1_contact_grp = _manipulator->getSelectedContactGroupIndex();
	if (_relationState->getRelationType() == RelationState::POINT_CONTACT)
	{
		Eigen::Vector3f v = boost::dynamic_pointer_cast<PointContactRelation>(_relationState)->getPoint2();
	    drc::vector_3d_t pt;
   	 	pt.x = v.x();
    	pt.y = v.y();
    	pt.z = v.z();
		msg.ground_plane_pt = pt;
	}
	return msg;
}