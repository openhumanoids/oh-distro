#include "ManipulationConstraint.h"
#include "PointContactRelation.h"

using namespace action_authoring;
using namespace affordance;
using namespace boost;

ManipulationConstraint::
ManipulationConstraint(AffConstPtr affordance,
                       ManipulatorStateConstPtr manipulator,
                       RelationStatePtr relationState
    )
    : _affordance(affordance),
      _manipulator(manipulator),
      _relationState(relationState)
{
	_timeLowerBound = 0.0;
	_timeUpperBound = 0.0;
}

ManipulationConstraint::
ManipulationConstraint(AffConstPtr affordance,
                       ManipulatorStateConstPtr manipulator,
                       RelationStatePtr relationState,
                       double timeLowerBound,
                       double timeUpperBound
    )
    : _affordance(affordance),
      _manipulator(manipulator),
      _relationState(relationState)
{
	_timeLowerBound = timeLowerBound;
	_timeUpperBound = timeUpperBound;
}

drc::contact_goal_t ManipulationConstraint::toLCM()
{
	printf("creating LCM message\n");
	drc::contact_goal_t msg;
    msg.utime = 0.0;

    //TODO change contact type based on affordance
	msg.contact_type = 0;
    printf("flag0\n");
	msg.object_1_name = _manipulator->getLinkName();
    printf("flag0.5\n");
	msg.object_1_contact_grp = _manipulator->getContactGroupName();
  
    //TODO remove hardcodes here!
    msg.contact_type = msg.ON_GROUND_PLANE;
    msg.lower_bound_completion_time = _timeLowerBound;
    msg.upper_bound_completion_time = _timeUpperBound;
  
	if (_relationState->getRelationType() == RelationState::POINT_CONTACT)
	{
        printf("flag1\n");
        PointContactRelationPtr p = (boost::static_pointer_cast<PointContactRelation>(_relationState));
		Eigen::Vector3f v = p->getPoint2();
        printf("flag2\n");

        msg.ground_plane_pt_radius = p->getTolerance();
        msg.x_relation = p->getXInequality();
        msg.y_relation = p->getYInequality();
        msg.z_relation = p->getZInequality();

        drc::vector_3d_t pt;
        pt.x = v.x();
        pt.y = v.y();
        pt.z = v.z();
        msg.ground_plane_pt = pt;
    }

	return msg;
}
