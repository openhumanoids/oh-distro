#include "RelationState.h"
using namespace action_authoring;
using namespace boost;
using namespace std;

unordered_map<string, RelationState::RelationType> RelationState::makeRNameMap()
{
  unordered_map<string, RelationState::RelationType> m;
  m["UNDEFINED"]         = UNDEFINED;
  m["GRASP"]             = GRASP;
  m["FORCE_CLOSURE"]     = FORCE_CLOSURE;
  m["OFFSET"]            = OFFSET;
  m["POINT_CONTACT"]     = POINT_CONTACT;
  return m;
}


const unordered_map<string, RelationState::RelationType> rNameToValue = RelationState::makeRNameMap();

RelationState::RelationState(const RelationState::RelationType &relationType) 
  : _relationType(relationType)
{
}

RelationState::RelationType RelationState::getRelationType() const
{
  return _relationType;
}


ostream& operator<<(ostream &out, const RelationState &other)
{
  out << other.getRelationType();
  return out;
}

std::string RelationState::getState() const {
    return "";
}

std::string RelationState::getPrompt() const {
    return "";
}
