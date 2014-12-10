#ifndef AFFORDANCE_MANIP_MAP
#define AFFORDANCE_MANIP_MAP

namespace action_authoring
{

/**Interface for mapping from globaluid --> affordance or manipulator*/
class AffordanceManipMap
{
    //----------Enumerations
public:
  virtual affordance::AffConstPtr getAffordance(const affordance::GlobalUID &affordanceUID) const = 0;
 
  virtual affordance::ManipulatorStateConstPtr getManipulator(const affordance::GlobalUID &manipulatorUID) const = 0;

}; //class AffordanceManipMap


} //namespace action_authoring


#endif //AFFORDANCE_MANIP_MAP
