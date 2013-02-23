#ifndef RELATION_STATE_H
#define RELATION_STATE_H

#include <boost/bimap.hpp>
#include <boost/shared_ptr.hpp>
#include <algorithm>
#include <vector>
#include <string>

namespace action_authoring
{

/**todo: add comment
 Immutable class indicating how 2 objects are related */
class RelationState
{

    //----------Enumerations
public:
    //------------------------------------------------------------------------
    // WARNING
    // if you edit this enum, edit the string definitions in the constructor
    //------------------------------------------------------------------------
    typedef enum
    {
        UNDEFINED,
        GRASP,
        FORCE_CLOSURE,
        POINT_CONTACT,
        OFFSET,
        RELATION_TYPE_LENGTH,
    } RelationType;

    typedef boost::bimap<std::string, RelationType> bm_type;

    //static init helpers
    static bm_type makeRNameMap();

    /**maps from string name for RelationType to the value*/
    const static bm_type rNameToValue;

    //------------fields
private:
    const RelationType _relationType;

    //------------Constructor--------
public:
    RelationState(const RelationState::RelationType &relationType);

    //---------------Accessors
    RelationType getRelationType() const;
    virtual std::string getState() const; // returns a user-friendly string that explains the state of relation
    virtual std::string getPrompt() const; // prompts the user for the next field to set to complete the relation

}; //class RelationState

std::ostream &operator<<(std::ostream &out, const RelationState &other);

typedef boost::shared_ptr<RelationState> RelationStatePtr;
typedef boost::shared_ptr<const RelationState> RelationStateConstPtr;

} //namespace action_authoring


#endif //RELATION_STATE_H
