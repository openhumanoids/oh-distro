 #ifndef DATABASE_MANAGER_H
#define DATABASE_MANAGER_H

#include "action_authoring/ConstraintMacro.h"
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
    struct ObjectToGUIDMappings
    {
        map<AffConstPtr, string> affordanceToGUID;
        map<ConstraintMacroPtr, string> constraintMacroToGUID;
        map<ManipulatorStateConstPtr, string> manipulatorStateToGUID;
        map<RelationStateConstPtr, string> relationStateToGUID;
        map<ManRelPtr, string> manipulationRelationToGUID;
        map<AtomicConstraintConstPtr, string> atomicConstraintToGUID;
    };

    struct GUIDToObjectMappings
    {
        map<string, AffConstPtr> GUIDToAffordance;
        map<string, ConstraintMacroPtr> GUIDToConstraintMacro;
        map<string, ManipulatorStateConstPtr> GUIDToManipulatorState;
        map<string, RelationStateConstPtr> GUIDToRelationState;
        map<string, ManRelPtr> GUIDToManipulationRelation;
        map<string, AtomicConstraintConstPtr> GUIDToAtomicConstraint;
    };

  /**todo comment*/
  class DatabaseManager 
  {
  private:
    static string getGUID(AffConstPtr affordance, ObjectToGUIDMappings &mappings);
    static string getGUID(ConstraintMacroPtr constraint, ObjectToGUIDMappings &mappings);
    static string getGUID(RelationStateConstPtr relationState, ObjectToGUIDMappings &mappings);
    static string getGUID(ManRelPtr manipulationRelation, ObjectToGUIDMappings &mappings);
    static string getGUID(ManipulatorStateConstPtr manipulator, ObjectToGUIDMappings &mappings);
    static string getGUID(AtomicConstraintConstPtr atomicConstraint, ObjectToGUIDMappings &mappings);

    //todo : in the cpp file, add doxygen comments for each of these methods
    static void addAffordanceStateToNode(AffConstPtr affordance, xmlNodePtr node, ObjectToGUIDMappings &mappings);
    static void addManipulatorStateToNode(ManipulatorStateConstPtr manipulator, xmlNodePtr node, ObjectToGUIDMappings &mappings);
    static void addRelationStateToNode(RelationStateConstPtr relationState, xmlNodePtr node, ObjectToGUIDMappings &mappings);
    static void addAtomicConstraintToNode(AtomicConstraintConstPtr atomicConstraint, xmlNodePtr node, ObjectToGUIDMappings &mappings);
    static void addManipulationRelationToNode(ManRelPtr relation, xmlNodePtr node, ObjectToGUIDMappings &mappings);
    static void addConstraintMacroToNode(ConstraintMacroPtr constraint, xmlNodePtr node, ObjectToGUIDMappings &mappings);
    static void postOrderAddConstraintMacroToQueue(ConstraintMacroPtr constraint, queue<ConstraintMacroPtr> &q, set<ConstraintMacroPtr> &done);
    
    static void parseTree(xmlDocPtr doc, xmlNode* xmlnode, GUIDToObjectMappings &mappings);

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
