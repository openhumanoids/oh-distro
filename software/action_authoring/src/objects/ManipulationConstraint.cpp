#include "ManipulationConstraint.h"
#include "PointContactRelation.h"

using namespace action_authoring;
using namespace affordance;
using namespace boost;

ManipulationConstraint::
ManipulationConstraint(GlobalUID affordanceUID,
                       GlobalUID manipulatorUID,
                       AffordanceManipMap *amMap,
                       RelationStatePtr relationState
    )
  :     
  _amMap(amMap),
  _relationState(relationState)
{
  setAffordance(affordanceUID);
  setManipulator(manipulatorUID);
  
  _timeLowerBound = 0.0;
  _timeUpperBound = 0.0;
}

ManipulationConstraint::
ManipulationConstraint(GlobalUID affordanceUID,
                       GlobalUID manipulatorUID,
                       AffordanceManipMap *amMap,
                       RelationStatePtr relationState,
                       double timeLowerBound,
                       double timeUpperBound)
  :       _amMap(amMap),
          _relationState(relationState)
{
  setAffordance(affordanceUID);
  setManipulator(manipulatorUID);

  _timeLowerBound = timeLowerBound;
  _timeUpperBound = timeUpperBound;
}


void ManipulationConstraint::setAffordance(const affordance::GlobalUID &affordanceUID)
    {
      _affordanceUID = shared_ptr<GlobalUID>(new GlobalUID(affordanceUID));
    }

void ManipulationConstraint::setManipulator(const affordance::GlobalUID &manipulatorUID)
{
  _manipulatorUID = shared_ptr<GlobalUID>(new GlobalUID(manipulatorUID));
}


drc::contact_goal_t ManipulationConstraint::toLCM()
{
	printf("creating LCM message\n");
	drc::contact_goal_t msg;
    msg.utime = 0.0;

    //TODO change contact type based on affordance
	msg.contact_type = 0;
    printf("flag0\n");
	msg.object_1_name = getManipulator()->getLinkName();
    printf("flag0.5\n");
	msg.object_1_contact_grp = getManipulator()->getContactGroupName();
  
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

        msg.target_pt_radius = p->getTolerance();
        msg.x_relation = p->getXInequality();
        msg.y_relation = p->getYInequality();
        msg.z_relation = p->getZInequality();

        drc::vector_3d_t pt;
        pt.x = v.x();
        pt.y = v.y();
        pt.z = v.z();
        msg.target_pt = pt;
    }

	return msg;
}
