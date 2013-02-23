#include "OffsetRelation.h"
#include <sstream>

using namespace action_authoring;

OffsetRelation::OffsetRelation() : RelationState(RelationState::OFFSET)
{
    
}

std::string
OffsetRelation::
getPrompt() const
{
    return "demo offset relation prompt; TODO";
}

std::string
OffsetRelation::
getState() const
{
    std::stringstream ss;
    return ss.str();
}
