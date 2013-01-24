#include "RelationState.h"

using namespace action_authoring;

RelationState::RelationState(const RelationState::RelationType &relationType) {
    _relationType = relationType;

    // todo make const
    RelationTypeNames.push_back("UNDEFINED");
    RelationTypeNames.push_back("GRASP");
    RelationTypeNames.push_back("FORCE_CLOSURE");
    RelationTypeNames.push_back("OFFSET");
}

RelationState::RelationType RelationState::RelationTypeFromName(std::string name) {
    std::vector<std::string>::iterator it = std::find(RelationTypeNames.begin(), RelationTypeNames.end(), name);
    if (it != RelationTypeNames.end()) {
	return (RelationType)(it - RelationTypeNames.begin());
    }
}
