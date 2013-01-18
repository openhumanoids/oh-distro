/*
Created Jan 2013 by cdolan

The Database Manager class is responsible for handling all file i/o
that stores persistent state in the action authoring gui.
This includes platonic affordancesm, atomic constraints, and higher level plans

Example usage:

//Instantiation
DatabaseManager* db = new DatabaseManager("constraints.xml");
  
//changing the file name
db->setFilename("someotherfilename.xml");

//storing state state, pass in all the data at once
db->store(affordanceList, constraintList);

//to retrieve the data, you must first call parseFile()!
db->parseFile();

//then to get the objects back, use db->get<YourTypeHere>()
std::vector<Affordance*> revivedAffordances = db->getAffordances();
std::vector<Constraint*> revivedConstraints = db->getConstraints();
*/

#include "DatabaseManager.h"
#include "AffordanceRelation.h"
#include <sstream>

using namespace action_authoring;

DatabaseManager::DatabaseManager(const char* filename) {
	m_filename = filename;
  m_guidCounter = 1;
}

/***********************************
            Utility Methods
************************************/

/*sets the name of the xml file that that constraint store will be written to*/
void DatabaseManager::setFilename(const char* filename) {
	m_filename = filename;
}

//Converts an integer uid into a string for writing to a file
std::string DatabaseManager::intToString(int i) {
  std::stringstream ss;
  ss << i;
  std::string result = ss.str();
  //printf("converting int %i to string %s\n", i, result.c_str());
  return result;
}

//Looks up the guid of an affordance or assigns one if none exists
int DatabaseManager::getGUID(Affordance* affordance){
  int guid = m_affordanceToGUID[affordance];
  if (guid <=  0) {
    guid = m_guidCounter;
    m_guidCounter += 1;
    m_affordanceToGUID[affordance] = guid;
  }
  //printf("guid for %s is %i\n", affordance->getName().c_str(), guid);
  return guid;
}

//Looks up the guid of an affordance or assigns one if none exists
int DatabaseManager::getGUID(Constraint* constraint){
  int guid = m_constraintToGUID[constraint];
  if (guid <= 0) {
    guid = m_guidCounter;
    m_guidCounter += 1;
    m_constraintToGUID[constraint] = guid;
  }
  //printf("guid for %s is %i\n", constraint->getName(), guid);
  return guid;
}



/***********************************
            Writing To File
************************************/

//Converts an affordance into an XML Node, and adds it as a child to another node
void DatabaseManager::addAffordanceToNode(Affordance* affordance, xmlNodePtr node) {
  //printf("storing affordance %s\n", affordance->getName().c_str());
  xmlNodePtr affordanceNode = xmlNewChild(node, NULL, BAD_CAST "affordance", NULL);
  std::string uidStr = intToString(getGUID(affordance));
  xmlNewChild(affordanceNode, NULL, BAD_CAST "uid", BAD_CAST uidStr.c_str());
  xmlNewChild(affordanceNode, NULL, BAD_CAST "name", BAD_CAST affordance->getName().c_str());
}

//Converts a constraint into an XML Node, and adds it as a child to another node
void DatabaseManager::addConstraintToNode(Constraint* constraint, xmlNodePtr node) {
  //Atomic Constraint conversion

  if (constraint->getConstraintType() == Constraint::ATOMIC) {
    //ATOMIC Constraint tag
    xmlNodePtr constraintNode = xmlNewChild(node, NULL, BAD_CAST "atomic", NULL);

    //Atomic Constraint Info
    std::string uidStr = intToString(getGUID(constraint));
    xmlNewChild(constraintNode, NULL, BAD_CAST "uid", BAD_CAST uidStr.c_str());
    xmlNewChild(constraintNode, NULL, BAD_CAST "name", BAD_CAST constraint->getName());

    //Affordance 1 Reference
    std::string aff1UidStr = intToString(getGUID(constraint->getAffordanceRelation()->getAffordance1()));
    xmlNewChild(constraintNode, NULL, BAD_CAST "aff1", BAD_CAST aff1UidStr.c_str());

    //Affordance 2 Reference
    std::string aff2UidStr = intToString(getGUID(constraint->getAffordanceRelation()->getAffordance2()));
    xmlNewChild(constraintNode, NULL, BAD_CAST "aff2", BAD_CAST aff2UidStr.c_str());
  }
  //Seqeuntial Constraint conversion
  else if ( constraint->getConstraintType() == Constraint::SEQUENTIAL ) {
    xmlNodePtr constraintNode = xmlNewChild(node, NULL, BAD_CAST "sequential", NULL);

    std::string uidStr = intToString(getGUID(constraint));
    xmlNewChild(constraintNode, NULL, BAD_CAST "uid", BAD_CAST uidStr.c_str());
    xmlNewChild(constraintNode, NULL, BAD_CAST "name", BAD_CAST constraint->getName());

    std::vector <Constraint*>constraintList = constraint->getConstraints();
    
    std::string childUidStr;
    for ( int i = 0; i < constraintList.size(); i++ ) {
      childUidStr = intToString(getGUID(constraintList[i]));
      xmlNewChild(constraintNode, NULL, BAD_CAST "child", BAD_CAST childUidStr.c_str());
    }
  }
  else {
    printf("Don't know how to parse constraint '%s'. Ignoring.\n", constraint->getName()); 
  }
}

//TODO
//Storing algorithm preparation - ensures all nodes efficent storage via uid references to children 
void DatabaseManager::postOrderAddConstraintToQueue(Constraint* constraint, std::queue<Constraint*>* q, std::set<Constraint*>* done) {
    if (constraint->getConstraintType() != Constraint::ATOMIC) {
      std::vector<Constraint*> constraintList = constraint->getConstraints();
      for( int i = 0; i < constraintList.size(); i++ ) {
        postOrderAddConstraintToQueue(constraintList[i], q, done);
      }
    }
    if (done->count(constraint) == 0) {
      q->push(constraint);
      done->insert(constraint);
    }
}

//main API call for storing all objects
void DatabaseManager::store(std::vector<Affordance*> affordanceList, std::vector<Constraint*> constraintList) {
	//printf("beginning to store\n");
  xmlDocPtr doc = NULL;
	xmlNodePtr node = NULL;

  //setup a new xml document and get the root node
	doc = xmlNewDoc(BAD_CAST "1.0");
	node = xmlNewNode(NULL, BAD_CAST "constraintstore");
	xmlDocSetRootElement(doc, node);

  //printf("got root, beginning to store affordance list\n");
  //add each affordance to the xml tree
	for(int i = 0; i < affordanceList.size(); i++) {
	  	addAffordanceToNode(affordanceList[i], node);
	}

  //printf("done storing affordances \n");
	std::queue<Constraint*>* constraintQueue = new std::queue<Constraint*>();
	std::set<Constraint*>* processedConstraints = new std::set<Constraint*>();

  //Use a postorder traversal of the constraint trees to ensure child constraints
  //are stored before the constraints that depend on them
	for(int i = 0; i < constraintList.size(); i++) {
		postOrderAddConstraintToQueue(constraintList[i], constraintQueue, processedConstraints);
	}	

  //add each constraint to the xml tree
	while (!constraintQueue->empty()){
	  addConstraintToNode(constraintQueue->front(), node);
	  constraintQueue->pop();
	}

  //finally, save the file
	xmlSaveFormatFileEnc(m_filename, doc, "UTF-8", 1);
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

void printAffordanceMap(std::map<int, Affordance*>* affordances) {
  printf("Affordance Map:\n");
  for(std::map<int, Affordance*>::iterator iter = affordances->begin(); iter != affordances->end(); ++iter)
  {
    printf("key: %i, name:%s\n", iter->first, iter->second->getName().c_str());
  }
}

//Creates an Affordance object from an XML node and stores it in a map of uids to affordances
Affordance* serializeAffordance(xmlDocPtr doc, xmlNode* node, std::map<int, Affordance*>* affordances) {
  //printf("\nserializing affordance\n");
  xmlNode* current_node = NULL;
  char* name;
  int uid;

  for (current_node = node->children; current_node; current_node = current_node->next) {
    if (current_node->type == XML_ELEMENT_NODE) {
      if(nodeNameIs(current_node, "uid")) {
        uid = atoi(value(doc, current_node));
      }
      else if(nodeNameIs(current_node, "name")) {
        name = value(doc, current_node);
      }
      else {
        printf("WARNING: Ignored unknown entry '%s' while parsing Affordance.\n", current_node->name);
      }
    }
  }

/*
  printf("\tname: %s\n", name);
  printf("\tuid: %i\n", uid);
*/
  Affordance* affordance = new Affordance(name);
  (*affordances)[uid] = affordance;
  return affordance;
}

//Creates an Atomic Constraint object from an XML node and stores it in a map of uids to constraints
Constraint* serializeAtomicConstraint(xmlDocPtr doc, xmlNode* node, std::map<int, Affordance*>* affordances, std::map<int, Constraint*>* constraints) {
  //printf("\nserializing atomic constraint\n");
  xmlNode* current_node = NULL;
  char* name;
  int uid;
  int aff1uid;
  int aff2uid;

  for (current_node = node->children; current_node; current_node = current_node->next) {
    if (current_node->type == XML_ELEMENT_NODE) {
      if (nodeNameIs(current_node, "aff1")) {
        aff1uid = atoi(value(doc, current_node));
      }
      else if(nodeNameIs(current_node, "aff2")) {
        aff2uid = atoi(value(doc, current_node));
      }
      else if (nodeNameIs(current_node, "uid")) {
        uid = atoi(value(doc, current_node));
      }
      else if (nodeNameIs(current_node, "name")) {
        name = value(doc, current_node);
      }
      else {
        printf("WARNING: Ignored unknown entry '%s' while parsing Atomic Constraint.\n", current_node->name);
      }
    }
  }

/*
  printf("\tname: %s\n", name);
  printf("\tuid: %i\n", uid);
  printf("\taff1uid: %i\n", aff1uid);
  printf("\taff2uid: %i\n", aff2uid);
*/

  Affordance* aff1 = (*affordances).find(aff1uid)->second;
  Affordance* aff2 = (*affordances).find(aff2uid)->second;

  //printAffordanceMap(affordances);
  //printf("aff1name: %s\n", aff1->getName().c_str());

  ///TODO FIX ME!!!!!!!!!!!!!!!!!!!!!!!!
  //I SHOULD NOT BE Constraint::TANGENT!!!!!!!!!!!!!
  AffordanceRelation* affordanceRelation = new AffordanceRelation(aff1, aff2, AffordanceRelation::TANGENT);
  Constraint* constraint = new Constraint(name, affordanceRelation);

  (*constraints)[uid] = constraint;
  return constraint;
}

//Creates a Sequential Constraint object from an XML node and stores in an a map of uids to constraints
Constraint* serializeSequentialConstraint(xmlDocPtr doc, xmlNode* node, std::map<int, Affordance*>* affordances, std::map<int, Constraint*>* constraints) {
  //printf("\nserializing sequential constraint\n");
  xmlNode* current_node = NULL;
  char* name;
  int uid;
  std::vector<Constraint*> childConstraints;
  std::vector<int> childuids;

  for (current_node = node->children; current_node; current_node = current_node->next) {
    if (current_node->type == XML_ELEMENT_NODE) {
      if (nodeNameIs(current_node, "child")) {
        //get the child uid
        int childuid = atoi(value(doc, current_node));
        //look up the corresponding constraint in the map
        Constraint* childConstraint = (*constraints).find(childuid)->second;
        childConstraints.push_back(childConstraint);
        childuids.push_back(childuid);
      }
      else if (nodeNameIs(current_node, "uid")) {
        uid = atoi(value(doc, current_node));
      }
      else if (nodeNameIs(current_node, "name")) {
        name = value(doc, current_node);
      }
      else {
        printf("WARNING: Ignored unknown entry '%s' while parsing Constraint.\n", current_node->name);
      }
    }
  }

/*
  printf("\tname: %s\n", name);
  printf("\tuid: %i\n", uid);
  for (int i = 0; i < childuids.size(); i++ ) {
      printf("\tchild %i: %i\n", i, childuids[i]);
  }
*/

  ///TODO FIX ME!!!!!!!!!!!!!!!!!!!!!!!!
  //I SHOULD NOT BE Constraint::SEQUENTIAL!!!!!!!!!!!!!
  Constraint* constraint = new Constraint(name, Constraint::SEQUENTIAL);
  for (int i = 0; i < childConstraints.size(); i++ ) {
    constraint->addConstraint(childConstraints[i]);
  }

  (*constraints)[uid] = constraint;
  return constraint;
}

//Dispatch the proper methods to create objects from XML nodes based on node name
void DatabaseManager::parseTreeHelper(xmlDocPtr doc, xmlNode* xmlnode, std::map<int,  Affordance*>* affordances, std::map<int, Constraint*>* constraints) {
  xmlNode* current_node = NULL;

  for (current_node = xmlnode; current_node; current_node = current_node->next) {
      if (current_node->type == XML_ELEMENT_NODE) {
        if(nodeNameIs(current_node, "affordance")) {
          serializeAffordance(doc, current_node, affordances);
        }
        else if (nodeNameIs(current_node, "atomic")) {
          serializeAtomicConstraint(doc, current_node, affordances, constraints);
        }
        else if (nodeNameIs(current_node, "sequential")){
          serializeSequentialConstraint(doc, current_node, affordances, constraints);
        }
        else {
          //parse the child nodes
          parseTreeHelper(doc, current_node->children, affordances, constraints);
        }
      }
  }
}

//Nice wrapper API call to parse the entire xml tree from a root node into Constraints and Affordances 
void DatabaseManager::parseTree(xmlDocPtr doc, xmlNode* root) {
  //printf("calling helper\n");
  std::map<int, Constraint*>* constraints = new std::map<int, Constraint*>();
  std::map<int, Affordance*>* affordances = new std::map<int, Affordance*>();
  parseTreeHelper(doc, root, affordances, constraints);
  
  /*
  printf("affordance map has %lu entries\n", affordances->size());
  printf("constraint map has %lu entries\n", constraints->size());

 for(std::map<int,Constraint*>::iterator ii=constraints->begin(); ii!=constraints->end(); ++ii) {
    printf("name: %s\n", (*ii).second->getName());
  }

  for(std::map<int,Affordance*>::iterator ii=affordances->begin(); ii!=affordances->end(); ++ii) {
    printf("name: %s\n", (*ii).second->getName().c_str());
  }

  for(std::map<int, Constraint*>::iterator iter = constraints->begin(); iter != constraints->end(); ++iter)
  {
    printf("key: %i, name:%s\n", iter->first, iter->second->getName());
  }

  printf("affordance map has %i entries\n", affordances->size());
  printf("constraint map has %i entries\n", constraints->size());
 */

  //Clear and update the affordance and constraint lists
  m_affordanceList.clear();
  m_constraintList.clear();
  
  for(std::map<int,Affordance*>::iterator ii=affordances->begin(); ii!=affordances->end(); ++ii) {
    m_affordanceList.push_back((*ii).second);
  }

  for(std::map<int,Constraint*>::iterator ii=constraints->begin(); ii!=constraints->end(); ++ii) {
    m_constraintList.push_back((*ii).second);
  }
}

//Wrapper call that kicks off the parsing of the file specified my m_filename
void DatabaseManager::parseFile(){
  xmlDocPtr readDoc;
  readDoc = xmlReadFile(m_filename, NULL, 0);
  if (readDoc == NULL) {
    fprintf(stderr, "Failed to parse %s\n", m_filename);
    exit(1);
  }

  //printf("done reading file. Parsing...\n");

  xmlNode* root_element = NULL;
  root_element = xmlDocGetRootElement(readDoc);
  parseTree(readDoc, root_element);

  xmlFreeDoc(readDoc);
  xmlCleanupParser();
}



//Accessor methods
std::vector<Affordance*> DatabaseManager::getAffordances() {
	return m_affordanceList;
}

std::vector<Constraint*> DatabaseManager::getConstraints() {
	return m_constraintList;
}