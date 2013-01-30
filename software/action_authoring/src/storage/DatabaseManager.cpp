/*
Created Jan 2013 by cdolan

The Database Manager class is responsible for handling all file i/o
that stores persistent state in the action authoring gui.
This includes platonic affordancesm, atomic constraints, and higher level plans

*/

#include "DatabaseManager.h"
#include "action_authoring/AtomicConstraint.h"
#include <sstream>
#include <iostream>
#include <time.h>

using namespace action_authoring;
using namespace affordance;
using namespace std;

#define DEBUG 0

#ifdef DEBUG
#define DEBUG_TEST 1
#else
#define DEBUG_TEST 0
#endif

#define debug_print(fmt, ...) do { if (DEBUG) fprintf(stderr, fmt, __VA_ARGS__); } while (0)

//shared defines
#define CONSTRAINT_STORE_NODE "constraint-store"
#define NAME_NODE "name"
#define UID_NODE "uid"

//SequentialConstraintMacro defines
#define SEQUENTIAL_CONSTRAINT_MACRO_NODE "sequential-constraint-macro"
#define CHILD_CONSTRAINT_MACRO_UID_NODE "child-constraint-macro-uid"

//AtomicConstraintMacro defines
#define ATOMIC_CONSTRAINT_MACRO_NODE "atomic-constraint-macro"
#define RELATION_UID_NODE "relation-uid"
#define TIME_LOWER_BOUND_NODE "time-lower-bound"
#define TIME_UPPER_BOUND_NODE "time-upper-bound"

//RelationState defines
#define RELATION_STATE_NODE "relation-state"
#define RELATION_STATE_TYPE_NODE "relation-state-type"

//ManipulationRelation defines
#define MANIPULATION_RELATION_NODE "mainpulation-relation"
#define AFFORDANCE_STATE_UID_NODE "affordance-state-uid"
#define MANIPULATOR_STATE_UID_NODE "manipulator-state-uid"
#define RELATION_STATE_UID_NODE "relation-state-uid"

//ManipulatorState defines
#define MANIPULATOR_STATE_NODE "manipulator-state"
#define GUID_PART_1_NODE "guid-part-1"
#define GUID_PART_2_NODE "guid-part-2"

//AffordanceState defines
#define AFFORDANCE_STATE_NODE "affordance-state"

/***********************************
            Utility Methods
************************************/

string generateRandomIdString() {
  string result;
  int len = 16;
  char alphanum[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
  for (int i = 0; i < len; ++i) {
    result += alphanum[rand() % (sizeof(alphanum) - 1)];
  }
  return result;
}

bool isInMapping(AffConstPtr affordance, ObjectToStorageUIDMappings &mappings) {
  return mappings.affordanceToStorageUID.isKey(affordance);
}

//Looks up the uid of an affordance or assigns one if none exists
string DatabaseManager::getStorageUID(AffConstPtr affordance, ObjectToStorageUIDMappings &mappings) {
  string uid;
  if (mappings.affordanceToStorageUID.isKey(affordance)) {
      uid = mappings.affordanceToStorageUID[affordance];
      debug_print("Affordance %s uid found, it is '%s'\n", affordance->getName().c_str(), uid.c_str());
  }
  else {
      uid = generateRandomIdString();
      mappings.affordanceToStorageUID[affordance] = uid;
      debug_print("Affordance '%s' uid assigned, it is '%s'\n", affordance->getName().c_str(), uid.c_str());
  }
  return uid;
}

string DatabaseManager::getStorageUID(RelationStateConstPtr relationState, ObjectToStorageUIDMappings &mappings) {
  string uid;
  if (mappings.relationStateToStorageUID.isKey(relationState)) {
    uid = mappings.relationStateToStorageUID[relationState];
  }
  else {
    uid = generateRandomIdString();
    mappings.relationStateToStorageUID[relationState] = uid;
  }
  return uid;
}

//Looks up the uid of an affordance or assigns one if none exists
string DatabaseManager::getStorageUID(ConstraintMacroPtr constraint, ObjectToStorageUIDMappings &mappings) {
  string uid;
  if (mappings.constraintMacroToStorageUID.isKey(constraint)) {
    uid = mappings.constraintMacroToStorageUID[constraint];
  }
  else {
    uid = generateRandomIdString();
    mappings.constraintMacroToStorageUID[constraint] = uid;
  }
  return uid;
}

string DatabaseManager::getStorageUID(ManRelPtr manipulationRelation, ObjectToStorageUIDMappings &mappings) {
  string uid;
  if (mappings.manipulationRelationToStorageUID.isKey(manipulationRelation)) {
    uid = mappings.manipulationRelationToStorageUID[manipulationRelation];
  }
  else {
    uid = generateRandomIdString();
    mappings.manipulationRelationToStorageUID[manipulationRelation] = uid;
  }
  return uid;
}

string DatabaseManager::getStorageUID(ManipulatorStateConstPtr manipulator, ObjectToStorageUIDMappings &mappings) {
  string uid;
  if (mappings.manipulatorStateToStorageUID.isKey(manipulator)) {
    uid = mappings.manipulatorStateToStorageUID[manipulator];
  }
  else {
    uid = generateRandomIdString();
    mappings.manipulatorStateToStorageUID[manipulator] = uid;
  }
  return uid;
}

string DatabaseManager::getStorageUID(AtomicConstraintConstPtr atomicConstraint, ObjectToStorageUIDMappings &mappings) {
  string uid;
  if (mappings.atomicConstraintToStorageUID.isKey(atomicConstraint)) {
    uid = mappings.atomicConstraintToStorageUID[atomicConstraint];
  }
  else {
    uid = generateRandomIdString();
    mappings.atomicConstraintToStorageUID[atomicConstraint] = uid;
  }
  return uid;
}

xmlNodePtr makeChild(xmlNodePtr parent, const string &name, const string &content) {
  return xmlNewChild(parent, NULL, BAD_CAST name.c_str(), BAD_CAST content.c_str());
}

xmlNodePtr makeChild(xmlNodePtr parent, const string &name, const int &content) {
  stringstream ss;
  ss << content;
  return makeChild(parent, name, ss.str());
}

xmlNodePtr makeChild(xmlNodePtr parent, const string &name, const double &content) {
  stringstream ss;
  ss << content;
  return makeChild(parent, name, ss.str());
}

xmlNodePtr makeChild(xmlNodePtr parent, const string &name) {
  return xmlNewChild(parent, NULL, BAD_CAST name.c_str(), NULL);
}

/***********************************
            Writing To File
************************************/

//Converts an affordance into an XML Node, and adds it as a child to another node
void DatabaseManager::addAffordanceStateToNode(AffConstPtr affordance, xmlNodePtr node, ObjectToStorageUIDMappings &mappings) {
  xmlNodePtr affordanceNode = makeChild(node, AFFORDANCE_STATE_NODE);
  makeChild(affordanceNode, UID_NODE, getStorageUID(affordance, mappings));
  makeChild(affordanceNode, NAME_NODE, affordance->getName());
  makeChild(affordanceNode, "contents", "LCM serialized affordance");
}

void DatabaseManager::addManipulatorStateToNode(ManipulatorStateConstPtr manipulator, xmlNodePtr node, ObjectToStorageUIDMappings &mappings) {
  xmlNodePtr manipulatorNode = makeChild(node, MANIPULATOR_STATE_NODE);
  makeChild(manipulatorNode, NAME_NODE, manipulator->getName());
  makeChild(manipulatorNode, UID_NODE, getStorageUID(manipulator, mappings));
  makeChild(manipulatorNode, GUID_PART_1_NODE, manipulator->getGlobalUniqueId().first);
  makeChild(manipulatorNode, GUID_PART_2_NODE, manipulator->getGlobalUniqueId().second);
}

void DatabaseManager::addRelationStateToNode(RelationStateConstPtr relationState, xmlNodePtr node, ObjectToStorageUIDMappings &mappings) {
  xmlNodePtr relationStateNode = makeChild(node, RELATION_STATE_NODE);
  makeChild(relationStateNode, UID_NODE, getStorageUID(relationState, mappings));
  makeChild(relationStateNode, RELATION_STATE_TYPE_NODE, "foo bar");//relationState->getRelationType())
}

void DatabaseManager::addAtomicConstraintToNode(AtomicConstraintConstPtr atom, xmlNodePtr node, ObjectToStorageUIDMappings &mappings) {
  addManipulatorStateToNode(atom->getManipulator(), node, mappings);

  const ManipulationRelation *underlying = dynamic_cast<const ManipulationRelation*>(atom.get());
  if (underlying == NULL)
    throw NotImplementedException("todo: handle atomic constraints that aren't manipulation relations");

  if (!isInMapping(atom->getAffordance(), mappings)) {
    addAffordanceStateToNode(atom->getAffordance(), node, mappings);
  }

  addRelationStateToNode(underlying->getRelationState(), node, mappings);

  xmlNodePtr relationNode = makeChild(node, MANIPULATION_RELATION_NODE);
  makeChild(relationNode, UID_NODE, getStorageUID(atom, mappings));
  makeChild(relationNode, AFFORDANCE_STATE_UID_NODE, getStorageUID(atom->getAffordance(), mappings));
  makeChild(relationNode, MANIPULATOR_STATE_UID_NODE, getStorageUID(atom->getManipulator(), mappings));
  makeChild(relationNode, RELATION_STATE_UID_NODE, getStorageUID(underlying->getRelationState(), mappings));
}

//Converts a constraint into an XML Node, and adds it as a child to another node
void DatabaseManager::addConstraintMacroToNode(ConstraintMacroPtr constraint, xmlNodePtr node, ObjectToStorageUIDMappings &mappings) {
  //Atomic ConstraintMacro conversion

  if (constraint->getConstraintMacroType() == ConstraintMacro::ATOMIC) {
    addAtomicConstraintToNode(constraint->getAtomicConstraint(), node, mappings);

    xmlNodePtr constraintNode = makeChild(node, ATOMIC_CONSTRAINT_MACRO_NODE);
    makeChild(constraintNode, UID_NODE, getStorageUID(constraint, mappings));
    makeChild(constraintNode, NAME_NODE, constraint->getName());
    makeChild(constraintNode, RELATION_UID_NODE, getStorageUID(constraint->getAtomicConstraint(), mappings));
    makeChild(constraintNode, TIME_LOWER_BOUND_NODE, constraint->getTimeLowerBound());
    makeChild(constraintNode, TIME_UPPER_BOUND_NODE, constraint->getTimeUpperBound());
  }
  //Seqeuntial ConstraintMacro conversion
  else if ( constraint->getConstraintMacroType() == ConstraintMacro::SEQUENTIAL ) {
    xmlNodePtr constraintNode = makeChild(node, SEQUENTIAL_CONSTRAINT_MACRO_NODE);
    makeChild(constraintNode, UID_NODE, getStorageUID(constraint, mappings));
    makeChild(constraintNode, NAME_NODE, constraint->getName());

    std::vector<ConstraintMacroPtr> constraintList;
    constraint->getConstraintMacros(constraintList);

    for ( int i = 0; i < (int)constraintList.size(); i++ ) {
      string uid = getStorageUID(constraintList[i], mappings);
      makeChild(constraintNode, CHILD_CONSTRAINT_MACRO_UID_NODE, uid);
    }
  }
  else {
    printf("Don't know how to parse constraint '%s'. Ignoring.\n", 
	   constraint->getName().c_str()); 
  }
}

//Storing algorithm preparation - ensures all nodes efficent storage via uid references to children 
void DatabaseManager::postOrderAddConstraintMacroToQueue(ConstraintMacroPtr constraint, 
							                                           std::queue<ConstraintMacroPtr> &q, 
							                                           std::set<ConstraintMacroPtr> &done) {
  if (constraint->getConstraintMacroType() != ConstraintMacro::ATOMIC) {
    std::vector<ConstraintMacroPtr> constraintList;
    constraint->getConstraintMacros(constraintList);
    for( int i = 0; i < (int)constraintList.size(); i++ ) {
      postOrderAddConstraintMacroToQueue(constraintList[i], q, done);
    }
  }
  if (done.count(constraint) == 0) {
      q.push(constraint);
      done.insert(constraint);
    }
}

//main API call for storing all objects
void DatabaseManager::store(const std::string &filename, std::vector<AffConstPtr> &affordanceList, std::vector<ConstraintMacroPtr> &constraintList) {
	//printf("beginning to store\n");

  srand(time(0));

  xmlDocPtr doc = NULL;
	xmlNodePtr node = NULL;

  //setup a new xml document and get the root node
	doc = xmlNewDoc(BAD_CAST "1.0");
	node = xmlNewNode(NULL, BAD_CAST CONSTRAINT_STORE_NODE);
	xmlDocSetRootElement(doc, node);

  ObjectToStorageUIDMappings mappings;

  //printf("got root, beginning to store affordance list\n");
  //add each affordance to the xml tree
	for(int i = 0; i < (int)affordanceList.size(); i++) {
	  	addAffordanceStateToNode(affordanceList[i], node, mappings);
	}

  // printf("done storing affordances \n");

	std::queue<ConstraintMacroPtr> constraintQueue;
	std::set<ConstraintMacroPtr> processedConstraintMacros;
  // printf("constraintList has %i items\n", (int)constraintList.size());

  //Use a postorder traversal of the constraint trees to ensure child constraints
  //are stored before the constraints that depend on them
	for(int i = 0; i < (int)constraintList.size(); i++) {
		postOrderAddConstraintMacroToQueue(constraintList[i], constraintQueue, processedConstraintMacros);
	}	
  // printf("constraintQueue has %i items\n", (int)constraintQueue.size());

  //add each constraint to the xml tree
	while (!constraintQueue.empty()) {
	  addConstraintMacroToNode(constraintQueue.front(), node, mappings);
	  constraintQueue.pop();
	}

  //finally, save the file
	xmlSaveFormatFileEnc(filename.c_str(), doc, "UTF-8", 1);
	xmlFreeDoc(doc);
	xmlCleanupParser();
}

/***********************************
          Reading From File
************************************/

//Utiliity function that checks equality of xmlNode name to given name
bool nodeIs(xmlNode* node, const char* name) {
  return xmlStrncmp(node->name, BAD_CAST name, 100) == 0;
}

//Utility function that gets value of an xml node
char* value(xmlDocPtr doc, xmlNode* node) {
  return (char*)xmlNodeListGetString(doc, node->xmlChildrenNode, 1);
}

int intvalue(xmlDocPtr doc, xmlNode* node) {
  return (int)(*value(doc, node));
}

double doublevalue(xmlDocPtr doc, xmlNode* node) {
  return (double) atof(value(doc, node));
}

void printAffordanceMap(map<string, AffConstPtr> &affordances) {
  printf("Affordance Map:\n");
  for(map<string, AffConstPtr>::iterator iter = affordances.begin(); iter != affordances.end(); ++iter)
  {
    printf("key: %s, name:%s\n", iter->first.c_str(), iter->second->getName().c_str());
  }
}

void printManipulatorStateMap(StorageUIDToObjectMappings mappings) {
  vector<ManipulatorStateConstPtr> manipulatorStates = mappings.StorageUIDToManipulatorState.getOrderedValues();
  vector<string> manipulatorStateIds = mappings.StorageUIDToManipulatorState.getOrderedKeys();
  printf("Manipulator Map:\n");
  for(int i = 0; i < (int) manipulatorStates.size(); i++)
  {
    printf("key: %s, name:%s\n", manipulatorStateIds[i].c_str(), manipulatorStates[i]->getName().c_str());
  }
}

//Creates an Affordance object from an XML node and stores it in a map of uids to affordances
void deserializeAffordanceState(xmlDocPtr doc, xmlNode* node, StorageUIDToObjectMappings &mappings) {
  //printf("\nserializing affordance\n");
  //printAffordanceMap(mappings.StorageUIDToAffordance);
  xmlNode* current_node = NULL;
  string name;
  string uid;

  for (current_node = node->children; current_node; current_node = current_node->next) {
    if (current_node->type == XML_ELEMENT_NODE) {
      if(nodeIs(current_node, UID_NODE)) {
        uid = value(doc, current_node);
      }
      else if(nodeIs(current_node, NAME_NODE)) {
        name = value(doc, current_node);
      }
      else {
        printf("WARNING: Ignored unknown entry '%s' while deserializing Affordance.\n", current_node->name);
      }
    }
  }
  AffConstPtr affordance (new AffordanceState(name));
  mappings.StorageUIDToAffordance[uid] = affordance;
}

void deserializeManipulatorState(xmlDocPtr doc, xmlNode* node, StorageUIDToObjectMappings &mappings) {
  xmlNode* current_node = NULL;
  string name;
  string uid;
  int guidpt1;
  int guidpt2;

  for (current_node = node->children; current_node; current_node = current_node->next) {
    if (current_node->type == XML_ELEMENT_NODE) {
      if(nodeIs(current_node, UID_NODE)) {
        uid = value(doc, current_node);
      }
      else if(nodeIs(current_node, NAME_NODE)) {
        name = value(doc, current_node);
      }
      else if(nodeIs(current_node, GUID_PART_1_NODE)) {
        guidpt1 = intvalue(doc, current_node);
      }
      else if(nodeIs(current_node, GUID_PART_2_NODE)) {
        guidpt2 = intvalue(doc, current_node);
      }
      else {
        printf("WARNING: Ignored unknown entry '%s' while deserializing ManipulatorState.\n", current_node->name);
      }
    }
  }
  ManipulatorStateConstPtr manipulator (new ManipulatorState(name, GlobalUID(guidpt1, guidpt2)));
  mappings.StorageUIDToManipulatorState[uid] = manipulator;
}

void deserializeRelationState(xmlDocPtr doc, xmlNode* node, StorageUIDToObjectMappings &mappings) {
  xmlNode* current_node = NULL;
  string name;
  string uid;

  for (current_node = node->children; current_node; current_node = current_node->next) {
    if (current_node->type == XML_ELEMENT_NODE) {
      if(nodeIs(current_node, UID_NODE)) {
        uid = value(doc, current_node);
      }
      else if(nodeIs(current_node, RELATION_STATE_TYPE_NODE)) {
        printf("deserializeRelationState TODO: deserialize type\n");
      }
      else {
        printf("WARNING: Ignored unknown entry '%s' while deserializing RelationState.\n", current_node->name);
      }
    }
  }
  
  //TODO deserialize type
  RelationStatePtr relationState (new RelationState(RelationState::UNDEFINED));
  mappings.StorageUIDToRelationState[uid] = relationState;
}

void deserializeManipulationRelation(xmlDocPtr doc, xmlNode* node, StorageUIDToObjectMappings &mappings) {
  //printf("\nserializing affordance\n");
  xmlNode* current_node = NULL;
  string name;
  string uid;
  string affordanceStateUID;
  string manipulatorStateUID;
  string relationStateUID;

  for (current_node = node->children; current_node; current_node = current_node->next) {
    if (current_node->type == XML_ELEMENT_NODE) {
      if(nodeIs(current_node, UID_NODE)) {
        uid = value(doc, current_node);
      }
      else if (nodeIs(current_node, AFFORDANCE_STATE_UID_NODE)) {
        affordanceStateUID = value(doc, current_node);
      }
      else if (nodeIs(current_node, MANIPULATOR_STATE_UID_NODE)) {
        manipulatorStateUID = value(doc, current_node);
      }
      else if (nodeIs(current_node, RELATION_STATE_UID_NODE)) {
        relationStateUID = value(doc, current_node);
      }
      else {
        printf("WARNING: Ignored unknown entry '%s' while deserializing ManipulationRelation\n", current_node->name);
      }
    }
  }

  AffConstPtr affordance  = mappings.StorageUIDToAffordance[affordanceStateUID];
  assert(affordance != NULL);
  ManipulatorStateConstPtr manipulatorState = mappings.StorageUIDToManipulatorState[manipulatorStateUID];
  assert(manipulatorState != NULL);
  RelationStatePtr relationState = mappings.StorageUIDToRelationState[relationStateUID];
  assert(relationState != NULL);

  ManRelPtr manipulationRelation (new ManipulationRelation(affordance, manipulatorState, relationState));
  mappings.StorageUIDToManipulationRelation[uid] = manipulationRelation;
}

//Creates an Atomic ConstraintMacro object from an XML node and stores it in a map of uids to constraints
void deserializeAtomicConstraintMacro(xmlDocPtr doc, xmlNode* node, StorageUIDToObjectMappings &mappings) {
  xmlNode* current_node = NULL;
  string uid;
  string name;
  double timeLowerBound;
  double timeUpperBound;
  string atomicConstraintUID;

  for (current_node = node->children; current_node; current_node = current_node->next) {
    if (current_node->type == XML_ELEMENT_NODE) {
      if (nodeIs(current_node, UID_NODE)) {
        uid = value(doc, current_node);
      }
      else if (nodeIs(current_node, NAME_NODE)) {
        name = value(doc, current_node);
      }
      else if (nodeIs(current_node, TIME_LOWER_BOUND_NODE)) {
        timeLowerBound = doublevalue(doc, current_node);
      }
      else if (nodeIs(current_node, TIME_UPPER_BOUND_NODE)) {
        timeUpperBound = doublevalue(doc, current_node);
      }
      else if (nodeIs(current_node, RELATION_UID_NODE)) {
        atomicConstraintUID = value(doc, current_node);
      }

      else {
        printf("WARNING: Ignored unknown entry '%s' while parsing AtomicConstraintMacro.\n", current_node->name);
      }
    }
  }

  //TODO switch lookup for different relations
  AtomicConstraintPtr atomicConstraint = mappings.StorageUIDToManipulationRelation[atomicConstraintUID];
  assert(atomicConstraint != NULL);
  ConstraintMacroPtr constraintMacro (new ConstraintMacro(name, atomicConstraint));
  constraintMacro->setTimeLowerBound(timeLowerBound);
  constraintMacro->setTimeUpperBound(timeUpperBound);
  mappings.StorageUIDToConstraintMacro[uid] = constraintMacro;
}

//Creates a Sequential ConstraintMacro object from an XML node and stores in an a map of uids to constraints
void deserializeSequentialConstraintMacro(xmlDocPtr doc, xmlNode* node, StorageUIDToObjectMappings &mappings) {
  xmlNode* current_node = NULL;
  string name;
  string uid;
  std::vector<ConstraintMacroPtr> childConstraintMacros;

  for (current_node = node->children; current_node; current_node = current_node->next) {
    if (current_node->type == XML_ELEMENT_NODE) {
      if (nodeIs(current_node, CHILD_CONSTRAINT_MACRO_UID_NODE)) {
        string childUID = value(doc, current_node);
        ConstraintMacroPtr childConstraintMacro = mappings.StorageUIDToConstraintMacro[childUID];
        childConstraintMacros.push_back(childConstraintMacro);
      }
      else if (nodeIs(current_node, UID_NODE)) {
        uid = value(doc, current_node);
      }
      else if (nodeIs(current_node, NAME_NODE)) {
        name = value(doc, current_node);
      }
      else {
        printf("WARNING: Ignored unknown entry '%s' while parsing SequentialConstraintMacro.\n", current_node->name);
      }
    }
  }

  ///TODO FIX ME!!!!!!!!!!!!!!!!!!!!!!!!
  //I SHOULD NOT BE ConstraintMacro::SEQUENTIAL!!!!!!!!!!!!!
  printf("deserializeSequentialConstraintMacro TODO: deserialized type.\n");
  ConstraintMacroPtr constraintMacro (new ConstraintMacro(name, ConstraintMacro::SEQUENTIAL));
  for (int i = 0; i < (int)childConstraintMacros.size(); i++ ) {
    constraintMacro->appendConstraintMacro(childConstraintMacros[i]);
  }
  mappings.StorageUIDToConstraintMacro[uid] = constraintMacro;
}

//Dispatch the proper methods to create objects from XML nodes based on node name
void DatabaseManager::parseTree(xmlDocPtr doc, xmlNode* xmlnode, StorageUIDToObjectMappings &mappings) {
  
  xmlNode* current_node = NULL;

  for (current_node = xmlnode; current_node; current_node = current_node->next) {
    if (current_node->type == XML_ELEMENT_NODE) {
      if(nodeIs(current_node, AFFORDANCE_STATE_NODE)) {
        // debug_print("deserialize affordance-state called\n");
        deserializeAffordanceState(doc, current_node, mappings);
      }
      else if (nodeIs(current_node, ATOMIC_CONSTRAINT_MACRO_NODE)) {
        // debug_print("deserialize atomic constraint macro called\n");
        deserializeAtomicConstraintMacro(doc, current_node, mappings);
      }
      else if (nodeIs(current_node, SEQUENTIAL_CONSTRAINT_MACRO_NODE)){
        // debug_print("deserialize sequential called\n");
        deserializeSequentialConstraintMacro(doc, current_node, mappings);
      }
      else if (nodeIs(current_node, MANIPULATOR_STATE_NODE)) {
        // debug_print("deserialize manipulator-state called\n");
        deserializeManipulatorState(doc, current_node, mappings);
      }
      else if (nodeIs(current_node, RELATION_STATE_NODE)) {
        // debug_print("deserialize relation-state called\n");
        deserializeRelationState(doc, current_node, mappings);
      }
      else if (nodeIs(current_node, MANIPULATION_RELATION_NODE)) {
        // debug_print("deserialize manipulation-relation called\n");
        deserializeManipulationRelation(doc, current_node, mappings);
      }
      else {
        //parse the child nodes
        printf("Dont's know how to parse node '%s'. Ignoring.\n", current_node->name);          
      }
      // debug_print("done processing node\n");
    }
  }
}

//Accessor methods
void DatabaseManager::retrieve(const string &filename, vector<AffConstPtr> &affordanceList, vector<ConstraintMacroPtr> &constraintList) {
  xmlDocPtr doc;
  doc = xmlReadFile(filename.c_str(), NULL, 0);
  if (doc == NULL) {
    fprintf(stderr, "Failed to parse %s\n", filename.c_str());
    exit(1);
  }

  xmlNode* root = NULL;
  root = xmlDocGetRootElement(doc);

  xmlNode* current_node;
  for (current_node = root; current_node; current_node = current_node->next) {
    if (current_node->type == XML_ELEMENT_NODE) {
      if (!nodeIs(current_node, CONSTRAINT_STORE_NODE)) {
        printf("xml file of type '%s' is not valid. Must be '%s' to parse.\n", current_node->name, CONSTRAINT_STORE_NODE);
        return;
      }
    }  
  }

  StorageUIDToObjectMappings mappings;

  parseTree(doc, root->children, mappings);

  debug_print("Retrieved %i affordanceState items.\n", (int) mappings.StorageUIDToAffordance.size());  
  debug_print("Retrieved %i manipulationState items.\n", (int) mappings.StorageUIDToManipulatorState.size());  
  debug_print("Retrieved %i relationState items.\n", (int) mappings.StorageUIDToRelationState.size());
  debug_print("Retrieved %i manipulationRelation items.\n", (int) mappings.StorageUIDToManipulationRelation.size());  
  debug_print("Retrieved %i atomicConstraint items.\n", (int) mappings.StorageUIDToAtomicConstraint.size());  
  debug_print("Retrieved %i ConstraintMacro items.\n", (int) mappings.StorageUIDToConstraintMacro.size());  

  vector<AffConstPtr> affordances =  mappings.StorageUIDToAffordance.getOrderedValues();
  for (int i = 0; i < (int)affordances.size(); i++) {
    if (affordances[i] != NULL) {
      affordanceList.push_back(affordances[i]);
    }
    else {
      fprintf(stderr, "Error restoring affordance. An entry in the affordance map was NULL.\n");
    }
  }

  vector<ConstraintMacroPtr> constraints = mappings.StorageUIDToConstraintMacro.getOrderedValues();
  for (int i = 0; i < (int)constraints.size(); i++){
    if (constraints[i] != NULL) {
      constraintList.push_back(constraints[i]);
    }
    else {
      fprintf(stderr, "Error restoring constraint. An entry in the constraint map was NULL.\n");
    }
  }

  xmlFreeDoc(doc);
  xmlCleanupParser();
}

