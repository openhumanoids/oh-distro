 #ifndef DATABASE_MANAGER_H
#define DATABASE_MANAGER_H

#include "action_authoring/ConstraintMacro.h"
#include "action_authoring/ManipulationRelation.h"
#include "affordance/AffordanceState.h"
#include <vector>
#include <queue>
#include <set>
#include <map>
#include <libxml/parser.h>
#include <libxml/tree.h>

using namespace affordance;
using namespace std;

namespace action_authoring 
{
    struct ObjectToStorageUIDMappings
    {
        map<AffConstPtr, string> affordanceToStorageUID;
        map<ConstraintMacroPtr, string> constraintMacroToStorageUID;
        map<ManipulatorStateConstPtr, string> manipulatorStateToStorageUID;
        map<RelationStateConstPtr, string> relationStateToStorageUID;
        map<ManRelPtr, string> manipulationRelationToStorageUID;
        map<AtomicConstraintConstPtr, string> atomicConstraintToStorageUID;
    };

    struct StorageUIDToObjectMappings
    {
        map<string, AffConstPtr> StorageUIDToAffordance;
        map<string, ConstraintMacroPtr> StorageUIDToConstraintMacro;
        map<string, ManipulatorStateConstPtr> StorageUIDToManipulatorState;
        map<string, RelationStatePtr> StorageUIDToRelationState;
        map<string, ManRelPtr> StorageUIDToManipulationRelation;
        map<string, AtomicConstraintPtr> StorageUIDToAtomicConstraint;
    };

  /**todo comment*/
  class DatabaseManager 
  {
  private:
    static string getStorageUID(AffConstPtr affordance, ObjectToStorageUIDMappings &mappings);
    static string getStorageUID(ConstraintMacroPtr constraint, ObjectToStorageUIDMappings &mappings);
    static string getStorageUID(RelationStateConstPtr relationState, ObjectToStorageUIDMappings &mappings);
    static string getStorageUID(ManRelPtr manipulationRelation, ObjectToStorageUIDMappings &mappings);
    static string getStorageUID(ManipulatorStateConstPtr manipulator, ObjectToStorageUIDMappings &mappings);
    static string getStorageUID(AtomicConstraintConstPtr atomicConstraint, ObjectToStorageUIDMappings &mappings);

    //todo : in the cpp file, add doxygen comments for each of these methods
    static void addAffordanceStateToNode(AffConstPtr affordance, xmlNodePtr node, ObjectToStorageUIDMappings &mappings);
    static void addManipulatorStateToNode(ManipulatorStateConstPtr manipulator, xmlNodePtr node, ObjectToStorageUIDMappings &mappings);
    static void addRelationStateToNode(RelationStateConstPtr relationState, xmlNodePtr node, ObjectToStorageUIDMappings &mappings);
    static void addAtomicConstraintToNode(AtomicConstraintConstPtr atomicConstraint, xmlNodePtr node, ObjectToStorageUIDMappings &mappings);
    static void addConstraintMacroToNode(ConstraintMacroPtr constraint, xmlNodePtr node, ObjectToStorageUIDMappings &mappings);
    static void postOrderAddConstraintMacroToQueue(ConstraintMacroPtr constraint, queue<ConstraintMacroPtr> &q, set<ConstraintMacroPtr> &done);
    
    static void parseTree(xmlDocPtr doc, xmlNode* xmlnode, StorageUIDToObjectMappings &mappings);

  public:
    //Writing data to a file
    //todo : add (in the cpp file) doxygen style comments 
    static void store(const string &filename, vector<AffConstPtr> &affordanceList, vector<ConstraintMacroPtr> &constraintList);
    
    //Reading data from a file
    //todo : add (in the cpp file) doxygen style comments
    static void retrieve(const string &filename, vector<AffConstPtr> &affordanceList, vector<ConstraintMacroPtr> &constraintList);

  }; //class DatabaseManager
} //namespace action_authoring

#endif //DATABASE_MANAGER_H
