#ifndef RELATION_STATE_H
#define RELATION_STATE_H

#include <boost/unordered_map.hpp>
#include <boost/shared_ptr.hpp>
#include <algorithm>
#include <vector>
#include <string>

namespace action_authoring
{

/**todo: add comment
 Immutable class indicating how 2 objects are relatined*/
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

    //static init helpers
    static boost::unordered_map<std::string, RelationType> makeRNameMap();

    /**maps from string name for RelationType to the value*/
    const static boost::unordered_map<std::string, RelationType> rNameToValue;
    
    //------------fields
 private:
    const RelationType _relationType;

    //------------Constructor--------
 public:
    RelationState(const RelationState::RelationType &relationType);

  //---------------Accessors
    RelationType getRelationType() const;

}; //class RelationState

 std::ostream& operator<<(std::ostream& out, const RelationState& other);
 
 typedef boost::shared_ptr<RelationState> RelationStatePtr;
 typedef boost::shared_ptr<const RelationState> RelationStateConstPtr;
 
} //namespace action_authoring


#endif //RELATION_STATE_H
