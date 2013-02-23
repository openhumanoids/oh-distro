#include "RelationState.h"
using namespace action_authoring;
using namespace boost;
using namespace std;

typedef boost::bimap<std::string, RelationState::RelationType> bm_type;

bm_type RelationState::makeRNameMap()
{
    // The ORDER is significant. Make sure the order matches the header declaration.
    bm_type m;
    m.insert(bm_type::value_type("UNDEFINED", UNDEFINED));
    m.insert(bm_type::value_type("GRASP", GRASP));
    m.insert(bm_type::value_type("FORCE_CLOSURE", FORCE_CLOSURE));
    m.insert(bm_type::value_type("OFFSET", OFFSET));
    m.insert(bm_type::value_type("POINT_CONTACT", POINT_CONTACT));
    return m;
}


const bm_type RelationState::rNameToValue = RelationState::makeRNameMap();

RelationState::RelationState(const RelationState::RelationType &relationType)
    : _relationType(relationType)
{
    
}

RelationState::RelationType RelationState::getRelationType() const
{
    return _relationType;
}


/*
ostream &operator<<(ostream &out, const RelationState &other)
{
    out << other.getRelationType();
    return out;
}
*/

std::string RelationState::getState() const
{
    return "";
}

std::string RelationState::getPrompt() const
{
    return "";
}
