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

//Looks up the uid of an affordance or assigns one if none exists
string DatabaseManager::getStorageUID(AffConstPtr affordance, ObjectToStorageUIDMappings &mappings) {
  string uid;
  if (mappings.affordanceToStorageUID.find(affordance) != mappings.affordanceToStorageUID.end()) {
      uid = mappings.affordanceToStorageUID[affordance];
  }
  else {
      uid = generateRandomIdString();
      mappings.affordanceToStorageUID[affordance] = uid;
  }
  return uid;
}

string DatabaseManager::getStorageUID(RelationStateConstPtr relationState, ObjectToStorageUIDMappings &mappings) {
  string uid;
  if (mappings.relationStateToStorageUID.find(relationState) != mappings.relationStateToStorageUID.end()) {
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
  if (mappings.constraintMacroToStorageUID.find(constraint) != mappings.constraintMacroToStorageUID.end()) {
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
  if (mappings.manipulationRelationToStorageUID.find(manipulationRelation) != mappings.manipulationRelationToStorageUID.end()) {
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
  if (mappings.manipulatorStateToStorageUID.find(manipulator) != mappings.manipulatorStateToStorageUID.end()) {
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
  if (mappings.atomicConstraintToStorageUID.find(atomicConstraint) != mappings.atomicConstraintToStorageUID.end()) {
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

xmlNodePtr makeChild(xmlNodePtr parent, const string &name) {
  return xmlNewChild(parent, NULL, BAD_CAST name.c_str(), NULL);
}

/***********************************
            Writing To File
************************************/

//Converts an affordance into an XML Node, and adds it as a child to another node
void DatabaseManager::addAffordanceStateToNode(AffConstPtr affordance, xmlNodePtr node, ObjectToStorageUIDMappings &mappings) {
  xmlNodePtr affordanceNode = makeChild(node, "affordance-state");
  makeChild(affordanceNode,"uid", getStorageUID(affordance, mappings));
  makeChild(affordanceNode, "name", affordance->getName());
  makeChild(affordanceNode, "contents", "LCM serialized affordance");
}

void DatabaseManager::addManipulatorStateToNode(ManipulatorStateConstPtr manipulator, xmlNodePtr node, ObjectToStorageUIDMappings &mappings) {
  xmlNodePtr manipulatorNode = makeChild(node, "manipulator-state");
  makeChild(manipulatorNode, "name", manipulator->getName());
  makeChild(manipulatorNode, "uid", getStorageUID(manipulator, mappings));
  makeChild(manipulatorNode, "guidpt1", manipulator->getGlobalUniqueId().first);
  makeChild(manipulatorNode, "guidpt2", manipulator->getGlobalUniqueId().second);
}

void DatabaseManager::addRelationStateToNode(RelationStateConstPtr relationState, xmlNodePtr node, ObjectToStorageUIDMappings &mappings) {
  xmlNodePtr relationStateNode = makeChild(node, "relation-state");
  makeChild(relationStateNode, "uid", getStorageUID(relationState, mappings));
  makeChild(relationStateNode, "type", "foo bar");//relationState->getRelationType())
}

void DatabaseManager::addAtomicConstraintToNode(AtomicConstraintConstPtr atom, xmlNodePtr node, ObjectToStorageUIDMappings &mappings) {
  addManipulatorStateToNode(atom->getManipulator(), node, mappings);

  const ManipulationRelation *underlying = dynamic_cast<const ManipulationRelation*>(atom.get());
  if (underlying == NULL)
    throw NotImplementedException("todo: handle atomic constraints that aren't manipulation relations");

  addRelationStateToNode(underlying->getRelationState(), node, mappings);

  xmlNodePtr relationNode = makeChild(node, "manipulation-relation");
  makeChild(relationNode, "uid", getStorageUID(atom, mappings));
  makeChild(relationNode, "affordance-state-uid", getStorageUID(atom->getAffordance(), mappings));
  makeChild(relationNode, "manipulator-state-uid", getStorageUID(atom->getManipulator(), mappings));
  makeChild(relationNode, "relation-state-uid", getStorageUID(underlying->getRelationState(), mappings));
}

//Converts a constraint into an XML Node, and adds it as a child to another node
void DatabaseManager::addConstraintMacroToNode(ConstraintMacroPtr constraint, xmlNodePtr node, ObjectToStorageUIDMappings &mappings) {
  //Atomic ConstraintMacro conversion

  if (constraint->getConstraintMacroType() == ConstraintMacro::ATOMIC) {
    addAtomicConstraintToNode(constraint->getAtomicConstraint(), node, mappings);

    xmlNodePtr constraintNode = makeChild(node, "atomic-constraint-macro");
    makeChild(constraintNode, "uid", getStorageUID(constraint, mappings));
    makeChild(constraintNode, "name", constraint->getName());
    makeChild(constraintNode, "constraint", getStorageUID(constraint->getAtomicConstraint(), mappings));
  }
  //Seqeuntial ConstraintMacro conversion
  else if ( constraint->getConstraintMacroType() == ConstraintMacro::SEQUENTIAL ) {
    xmlNodePtr constraintNode = makeChild(node, "sequential-constraint-macro");
    makeChild(constraintNode, "uid", getStorageUID(constraint, mappings));
    makeChild(constraintNode, "name", constraint->getName());

    std::vector<ConstraintMacroPtr> constraintList;
    constraint->getConstraintMacros(constraintList);

    for ( int i = 0; i < (int)constraintList.size(); i++ ) {
      string uid = getStorageUID(constraintList[i], mappings);
      makeChild(constraintNode, "child", uid);
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
	node = xmlNewNode(NULL, BAD_CAST "constraintstore");
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
bool nodeNameIs(xmlNode* node, const char* name) {
  return xmlStrncmp(node->name, BAD_CAST name, 100) == 0;
}

//Utility function that gets value of an xml node
char* value(xmlDocPtr doc, xmlNode* node) {
  return (char*)xmlNodeListGetString(doc, node->xmlChildrenNode, 1);
}

int intvalue(xmlDocPtr doc, xmlNode* node) {
  return (int)(*value(doc, node));
}

void printAffordanceMap(map<string, AffConstPtr> &affordances) {
  printf("Affordance Map:\n");
  for(map<string, AffConstPtr>::iterator iter = affordances.begin(); iter != affordances.end(); ++iter)
  {
    printf("key: %s, name:%s\n", iter->first.c_str(), iter->second->getName().c_str());
  }
}

void printManipulatorStateMap(StorageUIDToObjectMappings mappings) {
  map<string, ManipulatorStateConstPtr> &manipulatorStates = mappings.StorageUIDToManipulatorState;
  printf("Manipulator Map:\n");
  for(map<string, ManipulatorStateConstPtr>::iterator iter = manipulatorStates.begin(); iter != manipulatorStates.end(); ++iter)
  {
    printf("key: %s, name:%s\n", iter->first.c_str(), iter->second->getName().c_str());
  }
}

//Creates an Affordance object from an XML node and stores it in a map of uids to affordances
void deserializeAffordanceState(xmlDocPtr doc, xmlNode* node, StorageUIDToObjectMappings &mappings) {
  //printf("\nserializing affordance\n");
  xmlNode* current_node = NULL;
  string name;
  string uid;

  for (current_node = node->children; current_node; current_node = current_node->next) {
    if (current_node->type == XML_ELEMENT_NODE) {
      if(nodeNameIs(current_node, "uid")) {
        uid = value(doc, current_node);
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
  mappings.StorageUIDToAffordance[uid] = affordance;
}

void deserializeManipulatorState(xmlDocPtr doc, xmlNode* node, StorageUIDToObjectMappings &mappings) {
  //printf("\nserializing affordance\n");
  xmlNode* current_node = NULL;
  string name;
  string uid;
  int guidpt1;
  int guidpt2;

  for (current_node = node->children; current_node; current_node = current_node->next) {
    if (current_node->type == XML_ELEMENT_NODE) {
      if(nodeNameIs(current_node, "uid")) {
        uid = value(doc, current_node);
      }
      else if(nodeNameIs(current_node, "name")) {
        name = value(doc, current_node);
      }
      else if(nodeNameIs(current_node, "guidpt1")) {
        guidpt1 = intvalue(doc, current_node);
      }
      else if(nodeNameIs(current_node, "guidpt2")) {
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
      if(nodeNameIs(current_node, "uid")) {
        uid = value(doc, current_node);
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
      if(nodeNameIs(current_node, "uid")) {
        uid = value(doc, current_node);
      }
      else if (nodeNameIs(current_node, "affordance-state-uid")) {
        affordanceStateUID = value(doc, current_node);
      }
      else if (nodeNameIs(current_node, "manipulator-state-uid")) {
        manipulatorStateUID = value(doc, current_node);
      }
      else if (nodeNameIs(current_node, "relation-state-uid")) {
        relationStateUID = value(doc, current_node);
      }
      else {
        printf("WARNING: Ignored unknown entry '%s' while deserializing ManipulationRelation\n", current_node->name);
      }
    }
  }

  AffConstPtr affordance  = mappings.StorageUIDToAffordance[affordanceStateUID];
  ManipulatorStateConstPtr manipulatorState = mappings.StorageUIDToManipulatorState[manipulatorStateUID];
  RelationStatePtr relationState = mappings.StorageUIDToRelationState[relationStateUID];

  ManRelPtr manipulationRelation (new ManipulationRelation(affordance, manipulatorState, relationState));
  mappings.StorageUIDToManipulationRelation[uid] = manipulationRelation;
}

/*
void deserializeAtomicConstraint(xmlDocPtr doc, xmlNode* node, StorageUIDToObjectMappings &mappings) {
  //printf("\nserializing affordance\n");
  xmlNode* current_node = NULL;
  string name;
  string uid;
  string manipulationRelationUID;

  for (current_node = node->children; current_node; current_node = current_node->next) {
    if (current_node->type == XML_ELEMENT_NODE) {
      if(nodeNameIs(current_node, "uid")) {
        uid = value(doc, current_node);
      }
      else if (nodeNameIs(current_node, "manipulation-relation-uid")) {
        manipulationRelationUID = value(doc, current_node);
      }
      else {
        printf("WARNING: Ignored unknown entry '%s' while deserializing AtomicConstraint\n", current_node->name);
      }
    }
  }

  ManRelPtr manipulationRelation  = mappings.StorageUIDToManipulationRelation[manipulationRelationUID];
  
  AtomicConstraintPtr atomicConstraint (manipulationRelation);

  mappings.StorageUIDToAtomicConstraint[uid] = atomicConstraint;
}*/

//Creates an Atomic ConstraintMacro object from an XML node and stores it in a map of uids to constraints
void deserializeAtomicConstraintMacro(xmlDocPtr doc, xmlNode* node, StorageUIDToObjectMappings &mappings) {
  xmlNode* current_node = NULL;
  string uid;
  string name;
  string atomicConstraintUID;

  for (current_node = node->children; current_node; current_node = current_node->next) {
    if (current_node->type == XML_ELEMENT_NODE) {
      if (nodeNameIs(current_node, "uid")) {
        uid = value(doc, current_node);
      }
      else if (nodeNameIs(current_node, "name")) {
        name = value(doc, current_node);
      }
      else if (nodeNameIs(current_node, "constraint")) {
        atomicConstraintUID = value(doc, current_node);
      }
      else {
        printf("WARNING: Ignored unknown entry '%s' while parsing AtomicConstraintMacro.\n", current_node->name);
      }
    }
  }

  //TODO switch lookup for different relations
  AtomicConstraintPtr atomicConstraint = mappings.StorageUIDToManipulationRelation[atomicConstraintUID];

  ConstraintMacroPtr constraintMacro (new ConstraintMacro(name, atomicConstraint));
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
      if (nodeNameIs(current_node, "child")) {
        string childUID = value(doc, current_node);
        ConstraintMacroPtr childConstraintMacro = mappings.StorageUIDToConstraintMacro[childUID];
        childConstraintMacros.push_back(childConstraintMacro);
      }
      else if (nodeNameIs(current_node, "uid")) {
        uid = value(doc, current_node);
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
      if(nodeNameIs(current_node, "affordance-state")) {
        //printf("deserialize affordance-state called\n");
        deserializeAffordanceState(doc, current_node, mappings);
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

  StorageUIDToObjectMappings mappings;

  parseTree(doc, root->children, mappings);

  // printAffordanceMap(mappings.StorageUIDToAffordance);
  // printManipulatorStateMap(mappings);
  // printf("There are %i affordanceState items.\n", (int) mappings.StorageUIDToAffordance.size());  
  // printf("There are %i manipulationState items.\n", (int) mappings.StorageUIDToManipulatorState.size());  
  // printf("There are %i relationState items.\n", (int) mappings.StorageUIDToRelationState.size());
  // printf("There are %i manipulationRelation items.\n", (int) mappings.StorageUIDToManipulationRelation.size());  
  // printf("There are %i atomicConstraint items.\n", (int) mappings.StorageUIDToAtomicConstraint.size());  
  // printf("There are %i ConstraintMacro items.\n", (int) mappings.StorageUIDToConstraintMacro.size());  

  for(map<string, AffConstPtr>::iterator iter = mappings.StorageUIDToAffordance.begin(); iter != mappings.StorageUIDToAffordance.end(); ++iter)
  {
    affordanceList.push_back(iter->second);
  }

  for(map<string, ConstraintMacroPtr>::iterator iter = mappings.StorageUIDToConstraintMacro.begin(); iter != mappings.StorageUIDToConstraintMacro.end(); ++iter)
  {
    constraintList.push_back(iter->second);
  }

  xmlFreeDoc(doc);
  xmlCleanupParser();
}

