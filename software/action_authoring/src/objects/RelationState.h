#ifndef RELATION_STATE_H
#define RELATION_STATE_H

#include "boost/shared_ptr.hpp"

namespace action_authoring
{

/**todo: add comment*/
class RelationState
{
  
  //----------Enumerations
 public:
    typedef enum {
      UNDEFINED,
      GRASP,
      FORCE_CLOSURE,
      OFFSET
    } RelationType;


    //------------fields
 private:
    RelationType _relationType;

    //------------Constructor--------
 public:
    RelationState(const RelationState::RelationType &relationType);

  //---------------Accessors
  RelationType getRelationType() const { return _relationType; };

  //mutators
  
}; //class RelationState
 
 typedef boost::shared_ptr<RelationState> RelationStatePtr;
 typedef boost::shared_ptr<const RelationState> RelationStateConstPtr;

} //namespace action_authoring


#endif //RELATION_STATE_H
