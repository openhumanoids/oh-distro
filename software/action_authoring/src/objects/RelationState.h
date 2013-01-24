#ifndef RELATION_STATE_H
#define RELATION_STATE_H

#include <boost/shared_ptr.hpp>
#include <algorithm>
#include <vector>
#include <string>

namespace action_authoring
{

/**todo: add comment*/
class RelationState
{
  
  //----------Enumerations
 public:
//------------------------------------------------------------------------
// WARNING
// if you edit this enum, edit the string definitions in the constructor
//------------------------------------------------------------------------
    typedef enum {
      UNDEFINED,
      GRASP,
      FORCE_CLOSURE,
      OFFSET
    } RelationType;

// TODO make const
    std::vector<std::string> RelationTypeNames;
    RelationType RelationTypeFromName(std::string);

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
