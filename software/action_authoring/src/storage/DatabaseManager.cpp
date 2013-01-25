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

//Looks up the guid of an affordance or assigns one if none exists
string DatabaseManager::getGUID(AffConstPtr affordance, ObjectToGUIDMappings &mappings) {
  string guid;
  if (mappings.affordanceToGUID.find(affordance) != mappings.affordanceToGUID.end()) {
      guid = mappings.affordanceToGUID[affordance];
  }
  else {
      guid = generateRandomIdString();
      mappings.affordanceToGUID[affordance] = guid;
  }
  return guid;
}

string DatabaseManager::getGUID(RelationStateConstPtr relationState, ObjectToGUIDMappings &mappings) {
  string guid;
  if (mappings.relationStateToGUID.find(relationState) != mappings.relationStateToGUID.end()) {
    guid = mappings.relationStateToGUID[relationState];
  }
  else {
    guid = generateRandomIdString();
    mappings.relationStateToGUID[relationState] = guid;
  }
  return guid;
}

//Looks up the guid of an affordance or assigns one if none exists
string DatabaseManager::getGUID(ConstraintMacroPtr constraint, ObjectToGUIDMappings &mappings) {
  string guid;
  if (mappings.constraintMacroToGUID.find(constraint) != mappings.constraintMacroToGUID.end()) {
    guid = mappings.constraintMacroToGUID[constraint];
  }
  else {
    guid = generateRandomIdString();
    mappings.constraintMacroToGUID[constraint] = guid;
  }
  return guid;
}

string DatabaseManager::getGUID(ManRelPtr manipulationRelation, ObjectToGUIDMappings &mappings) {
  string guid;
  if (mappings.manipulationRelationToGUID.find(manipulationRelation) != mappings.manipulationRelationToGUID.end()) {
    guid = mappings.manipulationRelationToGUID[manipulationRelation];
  }
  else {
    guid = generateRandomIdString();
    mappings.manipulationRelationToGUID[manipulationRelation] = guid;
  }
  return guid;
}

string DatabaseManager::getGUID(ManipulatorStateConstPtr manipulator, ObjectToGUIDMappings &mappings) {
  string guid;
  if (mappings.manipulatorStateToGUID.find(manipulator) != mappings.manipulatorStateToGUID.end()) {
    guid = mappings.manipulatorStateToGUID[manipulator];
  }
  else {
    guid = generateRandomIdString();
    mappings.manipulatorStateToGUID[manipulator] = guid;
  }
  return guid;
}

string DatabaseManager::getGUID(AtomicConstraintConstPtr atomicConstraint, ObjectToGUIDMappings &mappings) {
  string guid;
  if (mappings.atomicConstraintToGUID.find(atomicConstraint) != mappings.atomicConstraintToGUID.end()) {
    guid = mappings.atomicConstraintToGUID[atomicConstraint];
  }
  else {
    guid = generateRandomIdString();
    mappings.atomicConstraintToGUID[atomicConstraint] = guid;
  }
  return guid;
}

xmlNodePtr makeChild(xmlNodePtr parent, const string &name, const string &content) {
  return xmlNewChild(parent, NULL, BAD_CAST name.c_str(), BAD_CAST content.c_str());
}

xmlNodePtr makeChild(xmlNodePtr parent, const string &name) {
  return xmlNewChild(parent, NULL, BAD_CAST name.c_str(), NULL);
}

/***********************************
            Writing To File
************************************/

//Converts an affordance into an XML Node, and adds it as a child to another node
void DatabaseManager::addAffordanceStateToNode(AffConstPtr affordance, xmlNodePtr node, ObjectToGUIDMappings &mappings) {
  xmlNodePtr affordanceNode = makeChild(node, "affordance-state");
  makeChild(affordanceNode,"uid", getGUID(affordance, mappings));
  makeChild(affordanceNode, "name", affordance->getName());
  makeChild(affordanceNode, "contents", "LCM serialized affordance");
}

void DatabaseManager::addManipulatorStateToNode(ManipulatorStateConstPtr manipulator, xmlNodePtr node, ObjectToGUIDMappings &mappings) {
  xmlNodePtr manipulatorNode = makeChild(node, "manipulator-state");
  makeChild(manipulatorNode, "name", manipulator->getName());
  makeChild(manipulatorNode, "uid", getGUID(manipulator, mappings));
}

void DatabaseManager::addRelationStateToNode(RelationStateConstPtr relationState, xmlNodePtr node, ObjectToGUIDMappings &mappings) {
  xmlNodePtr relationStateNode = makeChild(node, "relation-state");
  makeChild(relationStateNode, "uid", getGUID(relationState, mappings));
  makeChild(relationStateNode, "type", "foo bar");//relationState->getRelationType())
}

void DatabaseManager::addManipulationRelationToNode(ManRelPtr relation, xmlNodePtr node, ObjectToGUIDMappings &mappings) {
  addManipulatorStateToNode(relation->getManipulator(), node, mappings);
  addRelationStateToNode(relation->getRelationState(), node, mappings);

  xmlNodePtr relationNode = makeChild(node, "manipulation-relation");
  makeChild(relationNode, "uid", getGUID(relation, mappings));
  makeChild(relationNode, "affordance-state-uid", getGUID(relation->getAffordance(), mappings));
  makeChild(relationNode, "manipulator-state-uid", getGUID(relation->getManipulator(), mappings));
  makeChild(relationNode, "relation-state-uid", getGUID(relation->getRelationState(), mappings));
}

void DatabaseManager::addAtomicConstraintToNode(AtomicConstraintConstPtr atomicConstraint, xmlNodePtr node, ObjectToGUIDMappings &mappings) {
  addManipulationRelationToNode(atomicConstraint->getRelation(), node, mappings);

  xmlNodePtr atomicConstraintNode = makeChild(node, "atomic-constraint");
  makeChild(atomicConstraintNode, "uid", getGUID(atomicConstraint, mappings));
  makeChild(atomicConstraintNode, "manipulation-relation-uid", getGUID(atomicConstraint->getRelation(), mappings));
}

//Converts a constraint into an XML Node, and adds it as a child to another node
void DatabaseManager::addConstraintMacroToNode(ConstraintMacroPtr constraint, xmlNodePtr node, ObjectToGUIDMappings &mappings) {
  //Atomic ConstraintMacro conversion

  if (constraint->getConstraintMacroType() == ConstraintMacro::ATOMIC) {
    addAtomicConstraintToNode(constraint->getAtomicConstraint(), node, mappings);

    xmlNodePtr constraintNode = makeChild(node, "atomic-constraint-macro");
    makeChild(constraintNode, "uid", getGUID(constraint, mappings));
    makeChild(constraintNode, "name", constraint->getName());
    makeChild(constraintNode, "constraint", getGUID(constraint->getAtomicConstraint(), mappings));
  }
  //Seqeuntial ConstraintMacro conversion
  else if ( constraint->getConstraintMacroType() == ConstraintMacro::SEQUENTIAL ) {
    xmlNodePtr constraintNode = makeChild(node, "sequential-constraint-macro");
    makeChild(constraintNode, "uid", getGUID(constraint, mappings));
    makeChild(constraintNode, "name", constraint->getName());

    std::vector<ConstraintMacroPtr> constraintList;
    constraint->getConstraintMacros(constraintList);

    for ( int i = 0; i < constraintList.size(); i++ ) {
      string guid = getGUID(constraintList[i], mappings);
      makeChild(constraintNode, "child", guid);
    }
  }
  else {
    printf("Don't know how to parse constraint '%s'. Ignoring.\n", 
	   constraint->getName().c_str()); 
  }
}

//TODO
//Storing algorithm preparation - ensures all nodes efficent storage via uid references to children 
void DatabaseManager::postOrderAddConstraintMacroToQueue(ConstraintMacroPtr constraint, 
							                                           std::queue<ConstraintMacroPtr> &q, 
							                                           std::set<ConstraintMacroPtr> &done) {
  if (constraint->getConstraintMacroType() != ConstraintMacro::ATOMIC) {
    std::vector<ConstraintMacroPtr> constraintList;
    constraint->getConstraintMacros(constraintList);
    for( int i = 0; i < constraintList.size(); i++ ) {
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
	node = xmlNewNode(NULL, BAD_CAST "constraintstore");
	xmlDocSetRootElement(doc, node);

  ObjectToGUIDMappings mappings;

  //printf("got root, beginning to store affordance list\n");
  //add each affordance to the xml tree
	for(int i = 0; i < affordanceList.size(); i++) {
	  	addAffordanceStateToNode(affordanceList[i], node, mappings);
	}

  // printf("done storing affordances \n");

	std::queue<ConstraintMacroPtr> constraintQueue;
	std::set<ConstraintMacroPtr> processedConstraintMacros;
  // printf("constraintList has %i items\n", (int)constraintList.size());

  //Use a postorder traversal of the constraint trees to ensure child constraints
  //are stored before the constraints that depend on them
	for(int i = 0; i < constraintList.size(); i++) {
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
bool nodeNameIs(xmlNode* node, const char* name) {
  return xmlStrncmp(node->name, BAD_CAST name, 100) == 0;
}

//Utility function that gets value of an xml node
char* value(xmlDocPtr doc, xmlNode* node) {
  return (char*)xmlNodeListGetString(doc, node->xmlChildrenNode, 1);
}

void printAffordanceMap(map<string, AffConstPtr> &affordances) {
  printf("Affordance Map:\n");
  for(map<string, AffConstPtr>::iterator iter = affordances.begin(); iter != affordances.end(); ++iter)
  {
    printf("key: %s, name:%s\n", iter->first.c_str(), iter->second->getName().c_str());
  }
}

void printManipulatorStateMap(GUIDToObjectMappings mappings) {
  map<string, ManipulatorStateConstPtr> &manipulatorStates = mappings.GUIDToManipulatorState;
  printf("Manipulator Map:\n");
  for(map<string, ManipulatorStateConstPtr>::iterator iter = manipulatorStates.begin(); iter != manipulatorStates.end(); ++iter)
  {
    printf("key: %s, name:%s\n", iter->first.c_str(), iter->second->getName().c_str());
  }
}

//Creates an Affordance object from an XML node and stores it in a map of uids to affordances
void deserializeAffordanceState(xmlDocPtr doc, xmlNode* node, GUIDToObjectMappings &mappings) {
  //printf("\nserializing affordance\n");
  xmlNode* current_node = NULL;
  string name;
  string guid;

  for (current_node = node->children; current_node; current_node = current_node->next) {
    if (current_node->type == XML_ELEMENT_NODE) {
      if(nodeNameIs(current_node, "uid")) {
        guid = value(doc, current_node);
      }
      else if(nodeNameIs(current_node, "name")) {
        name = value(doc, current_node);
      }
      else {
        printf("WARNING: Ignored unknown entry '%s' while deserializing Affordance.\n", current_node->name);
      }
    }
  }
  AffConstPtr affordance (new AffordanceState(name));
  mappings.GUIDToAffordance[guid] = affordance;
}

void deserializeManipulatorState(xmlDocPtr doc, xmlNode* node, GUIDToObjectMappings &mappings) {
  //printf("\nserializing affordance\n");
  xmlNode* current_node = NULL;
  string name;
  string guid;

  for (current_node = node->children; current_node; current_node = current_node->next) {
    if (current_node->type == XML_ELEMENT_NODE) {
      if(nodeNameIs(current_node, "uid")) {
        guid = value(doc, current_node);
      }
      else if(nodeNameIs(current_node, "name")) {
        name = value(doc, current_node);
      }
      else {
        printf("WARNING: Ignored unknown entry '%s' while deserializing ManipulatorState.\n", current_node->name);
      }
    }
  }
  ManipulatorStateConstPtr manipulator (new ManipulatorState(name));
  mappings.GUIDToManipulatorState[guid] = manipulator;
}

void deserializeRelationState(xmlDocPtr doc, xmlNode* node, GUIDToObjectMappings &mappings) {
  xmlNode* current_node = NULL;
  string name;
  string guid;

  for (current_node = node->children; current_node; current_node = current_node->next) {
    if (current_node->type == XML_ELEMENT_NODE) {
      if(nodeNameIs(current_node, "uid")) {
        guid = value(doc, current_node);
      }
      else if(nodeNameIs(current_node, "type")) {
        printf("deserializeRelationState TODO: deserialize type\n");
      }
      else {
        printf("WARNING: Ignored unknown entry '%s' while deserializing RelationState.\n", current_node->name);
      }
    }
  }
  
  //TODO deserialize type
  RelationStatePtr relationState (new RelationState(RelationState::UNDEFINED));
  mappings.GUIDToRelationState[guid] = relationState;
}

void deserializeManipulationRelation(xmlDocPtr doc, xmlNode* node, GUIDToObjectMappings &mappings) {
  //printf("\nserializing affordance\n");
  xmlNode* current_node = NULL;
  string name;
  string guid;
  string affordanceStateGUID;
  string manipulatorStateGUID;
  string relationStateGUID;

  for (current_node = node->children; current_node; current_node = current_node->next) {
    if (current_node->type == XML_ELEMENT_NODE) {
      if(nodeNameIs(current_node, "uid")) {
        guid = value(doc, current_node);
      }
      else if (nodeNameIs(current_node, "affordance-state-uid")) {
        affordanceStateGUID = value(doc, current_node);
      }
      else if (nodeNameIs(current_node, "manipulator-state-uid")) {
        manipulatorStateGUID = value(doc, current_node);
      }
      else if (nodeNameIs(current_node, "relation-state-uid")) {
        relationStateGUID = value(doc, current_node);
      }
      else {
        printf("WARNING: Ignored unknown entry '%s' while deserializing ManipulationRelation\n", current_node->name);
      }
    }
  }

  AffConstPtr affordance  = mappings.GUIDToAffordance[affordanceStateGUID];
  ManipulatorStateConstPtr manipulatorState = mappings.GUIDToManipulatorState[manipulatorStateGUID];
  RelationStateConstPtr relationState = mappings.GUIDToRelationState[relationStateGUID];

  ManRelPtr manipulationRelation (new ManipulationRelation(affordance, manipulatorState, relationState));
  mappings.GUIDToManipulationRelation[guid] = manipulationRelation;
}

void deserializeAtomicConstraint(xmlDocPtr doc, xmlNode* node, GUIDToObjectMappings &mappings) {
  //printf("\nserializing affordance\n");
  xmlNode* current_node = NULL;
  string name;
  string guid;
  string manipulationRelationGUID;

  for (current_node = node->children; current_node; current_node = current_node->next) {
    if (current_node->type == XML_ELEMENT_NODE) {
      if(nodeNameIs(current_node, "uid")) {
        guid = value(doc, current_node);
      }
      else if (nodeNameIs(current_node, "manipulation-relation-uid")) {
        manipulationRelationGUID = value(doc, current_node);
      }
      else {
        printf("WARNING: Ignored unknown entry '%s' while deserializing AtomicConstraint\n", current_node->name);
      }
    }
  }

  ManRelPtr manipulationRelation  = mappings.GUIDToManipulationRelation[manipulationRelationGUID];
  
  AtomicConstraintPtr atomicConstraint (new AtomicConstraint(manipulationRelation));

  mappings.GUIDToAtomicConstraint[guid] = atomicConstraint;
}

//Creates an Atomic ConstraintMacro object from an XML node and stores it in a map of uids to constraints
void deserializeAtomicConstraintMacro(xmlDocPtr doc, xmlNode* node, GUIDToObjectMappings &mappings) {
  xmlNode* current_node = NULL;
  string guid;
  string name;
  string atomicConstraintGUID;

  for (current_node = node->children; current_node; current_node = current_node->next) {
    if (current_node->type == XML_ELEMENT_NODE) {
      if (nodeNameIs(current_node, "uid")) {
        guid = value(doc, current_node);
      }
      else if (nodeNameIs(current_node, "name")) {
        name = value(doc, current_node);
      }
      else if (nodeNameIs(current_node, "constraint")) {
        atomicConstraintGUID = value(doc, current_node);
      }
      else {
        printf("WARNING: Ignored unknown entry '%s' while parsing AtomicConstraintMacro.\n", current_node->name);
      }
    }
  }

  AtomicConstraintConstPtr atomicConstraint = mappings.GUIDToAtomicConstraint[atomicConstraintGUID];

  ConstraintMacroPtr constraintMacro (new ConstraintMacro(name, atomicConstraint));
  mappings.GUIDToConstraintMacro[guid] = constraintMacro;
}

//Creates a Sequential ConstraintMacro object from an XML node and stores in an a map of uids to constraints
void deserializeSequentialConstraintMacro(xmlDocPtr doc, xmlNode* node, GUIDToObjectMappings &mappings) {
  xmlNode* current_node = NULL;
  string name;
  string guid;
  std::vector<ConstraintMacroPtr> childConstraintMacros;

  for (current_node = node->children; current_node; current_node = current_node->next) {
    if (current_node->type == XML_ELEMENT_NODE) {
      if (nodeNameIs(current_node, "child")) {
        string childGUID = value(doc, current_node);
        ConstraintMacroPtr childConstraintMacro = mappings.GUIDToConstraintMacro[childGUID];
        childConstraintMacros.push_back(childConstraintMacro);
      }
      else if (nodeNameIs(current_node, "uid")) {
        guid = value(doc, current_node);
      }
      else if (nodeNameIs(current_node, "name")) {
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
  for (int i = 0; i < childConstraintMacros.size(); i++ ) {
    constraintMacro->appendConstraintMacro(childConstraintMacros[i]);
  }
  mappings.GUIDToConstraintMacro[guid] = constraintMacro;
}

//Dispatch the proper methods to create objects from XML nodes based on node name
void DatabaseManager::parseTree(xmlDocPtr doc, xmlNode* xmlnode, GUIDToObjectMappings &mappings) {
  
  xmlNode* current_node = NULL;

  for (current_node = xmlnode; current_node; current_node = current_node->next) {
    if (current_node->type == XML_ELEMENT_NODE) {
      if(nodeNameIs(current_node, "affordance-state")) {
        //printf("deserialize affordance-state called\n");
        deserializeAffordanceState(doc, current_node, mappings);
      }
      else if (nodeNameIs(current_node, "atomic-constraint")) {
        //printf("deserialize atomic constraint called\n");
        deserializeAtomicConstraint(doc, current_node, mappings);
      }
      else if (nodeNameIs(current_node, "atomic-constraint-macro")) {
        //printf("deserialize atomic constraint macro called\n");
        deserializeAtomicConstraintMacro(doc, current_node, mappings);
      }
      else if (nodeNameIs(current_node, "sequential-constraint-macro")){
        //printf("deserialize sequential called\n");
        deserializeSequentialConstraintMacro(doc, current_node, mappings);
      }
      else if (nodeNameIs(current_node, "manipulator-state")) {
        //printf("deserialize manipulator-state called\n");
        deserializeManipulatorState(doc, current_node, mappings);
      }
      else if (nodeNameIs(current_node, "relation-state")) {
        //printf("deserialize relation-state called\n");
        deserializeRelationState(doc, current_node, mappings);
      }
      else if (nodeNameIs(current_node, "manipulation-relation")) {
        //printf("deserialize manipulation-relation called\n");
        deserializeManipulationRelation(doc, current_node, mappings);
      }
      else {
        //parse the child nodes
        printf("Dont's know how to parse node '%s'. Ignoring.\n", current_node->name);          
      }
    }
  }
}

//Accessor methods
void DatabaseManager::retrieve(const string &filename, vector<AffConstPtr> &affordanceList, vector<ConstraintMacroPtr> &constraintList) {
  // printf("Called retrieve\n");

  xmlDocPtr doc;
  doc = xmlReadFile(filename.c_str(), NULL, 0);
  if (doc == NULL) {
    fprintf(stderr, "Failed to parse %s\n", filename.c_str());
    exit(1);
  }

  // printf("done reading file. Parsing...\n");

  xmlNode* root = NULL;
  root = xmlDocGetRootElement(doc);

  xmlNode* current_node;
  for (current_node = root; current_node; current_node = current_node->next) {
    if (current_node->type == XML_ELEMENT_NODE) {
      if (!nodeNameIs(current_node, "constraintstore")) {
        printf("xml file of type '%s' is not valid. Must be 'constraintstore' to parse.\n", current_node->name);
        return;
      }
    }  
  }

  //Root element is "constraintstore" type check goes here

  GUIDToObjectMappings mappings;

  parseTree(doc, root->children, mappings);

  // printAffordanceMap(mappings.GUIDToAffordance);
  // printManipulatorStateMap(mappings);
  // printf("There are %i affordanceState items.\n", (int) mappings.GUIDToAffordance.size());  
  // printf("There are %i manipulationState items.\n", (int) mappings.GUIDToManipulatorState.size());  
  // printf("There are %i relationState items.\n", (int) mappings.GUIDToRelationState.size());
  // printf("There are %i manipulationRelation items.\n", (int) mappings.GUIDToManipulationRelation.size());  
  // printf("There are %i atomicConstraint items.\n", (int) mappings.GUIDToAtomicConstraint.size());  
  // printf("There are %i ConstraintMacro items.\n", (int) mappings.GUIDToConstraintMacro.size());  

  for(map<string, AffConstPtr>::iterator iter = mappings.GUIDToAffordance.begin(); iter != mappings.GUIDToAffordance.end(); ++iter)
  {
    affordanceList.push_back(iter->second);
  }

  for(map<string, ConstraintMacroPtr>::iterator iter = mappings.GUIDToConstraintMacro.begin(); iter != mappings.GUIDToConstraintMacro.end(); ++iter)
  {
    constraintList.push_back(iter->second);
  }

  xmlFreeDoc(doc);
  xmlCleanupParser();
}

