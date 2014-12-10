#include "ManipulationConstraint.h"
#include "PointContactRelation.h"

using namespace action_authoring;
using namespace affordance;
using namespace boost;
using namespace std;

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
    switch(_relationState->getRelationType())
      {
      case RelationState::POINT_CONTACT:
        msg.contact_type = drc::contact_goal_t::ON_GROUND_PLANE;
        break;
      case RelationState::NOT_IN_CONTACT:
        msg.contact_type = drc::contact_goal_t::NOT_IN_CONTACT;
        break;
      default:
        cout << "\n\n\n relation state = " << _relationState->getRelationType() << endl;
        throw std::runtime_error("ManipulationConstraint: unhandled relation type for to lcm");
      }
    
    
    msg.lower_bound_completion_time = _timeLowerBound;
    msg.upper_bound_completion_time = _timeUpperBound;
  
	if (_relationState->getRelationType() == RelationState::POINT_CONTACT)
	{
        printf("flag1\n");
        PointContactRelationPtr p = (boost::static_pointer_cast<PointContactRelation>(_relationState));
		Eigen::Vector3f v = p->getPoint2();
        printf("flag2\n");

        msg.x_offset = p->getXOffset();
        msg.y_offset = p->getYOffset();
        msg.z_offset = p->getZOffset();
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
