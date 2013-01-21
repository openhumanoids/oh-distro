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
std::vector<ConstraintMacro*> revivedConstraintMacros = db->getConstraintMacros();
*/

#include "DatabaseManager.h"
#include "AtomicConstraint.h"
#include <sstream>

using namespace action_authoring;
using namespace affordance;
using namespace std;

DatabaseManager::DatabaseManager(const string &filename) 
{
  //todo : use initializer list
  _filename = filename;
  _guidCounter = 1;
}

/***********************************
            Utility Methods
************************************/

/*sets the name of the xml file that that constraint store will be written to*/
void DatabaseManager::setFilename(const std::string &filename) 
{
	_filename = filename;
}

//Converts an integer uid into a string for writing to a file
std::string DatabaseManager::intToString(const int &i)
{
  std::stringstream ss;
  ss << i;
  return ss.str();
}

//Looks up the guid of an affordance or assigns one if none exists
int DatabaseManager::getGUID(AffConstPtr affordance)
{
  int guid = _affordanceToGUID[affordance];
  if (guid <=  0) 
    {
      guid = _guidCounter++;
      _affordanceToGUID[affordance] = guid;
    }
  return guid;
}

//Looks up the guid of an affordance or assigns one if none exists
int DatabaseManager::getGUID(ConstraintMacroConstPtr constraint){
  int guid = _constraintToGUID[constraint];
  if (guid <= 0) {
    guid = _guidCounter++;
    _constraintToGUID[constraint] = guid;
  }
  return guid;
}



/***********************************
            Writing To File
************************************/

//Converts an affordance into an XML Node, and adds it as a child to another node
void DatabaseManager::addAffordanceToNode(AffConstPtr affordance, xmlNodePtr node) {
  //printf("storing affordance %s\n", affordance->getName().c_str());
  xmlNodePtr affordanceNode = xmlNewChild(node, NULL, BAD_CAST "affordance", NULL);
  std::string uidStr = intToString(getGUID(affordance));
  xmlNewChild(affordanceNode, NULL, BAD_CAST "uid", BAD_CAST uidStr.c_str());
  xmlNewChild(affordanceNode, NULL, BAD_CAST "name", BAD_CAST affordance->getName().c_str());
}

//Converts a constraint into an XML Node, and adds it as a child to another node
void DatabaseManager::addConstraintMacroToNode(ConstraintMacroConstPtr constraint, xmlNodePtr node) {
  //Atomic ConstraintMacro conversion

  if (constraint->getConstraintMacroType() == ConstraintMacro::ATOMIC) {
    //ATOMIC ConstraintMacro tag
    xmlNodePtr constraintNode = xmlNewChild(node, NULL, BAD_CAST "atomic", NULL);

    //Atomic ConstraintMacro Info
    std::string uidStr = intToString(getGUID(constraint));
    xmlNewChild(constraintNode, NULL, BAD_CAST "uid", BAD_CAST uidStr.c_str());
    xmlNewChild(constraintNode, NULL, BAD_CAST "name", 
		BAD_CAST constraint->getName().c_str());

    //Affordance 1 Reference
    std::string aff1UidStr = intToString(getGUID(constraint->getAtomicConstraint()->getAffordance1()));
    xmlNewChild(constraintNode, NULL, BAD_CAST "aff1", BAD_CAST aff1UidStr.c_str());

    //Affordance 2 Reference
    std::string aff2UidStr = intToString(getGUID(constraint->getAtomicConstraint()->getAffordance2()));
    xmlNewChild(constraintNode, NULL, BAD_CAST "aff2", BAD_CAST aff2UidStr.c_str());
  }
  //Seqeuntial ConstraintMacro conversion
  else if ( constraint->getConstraintMacroType() == ConstraintMacro::SEQUENTIAL ) {
    xmlNodePtr constraintNode = xmlNewChild(node, NULL, BAD_CAST "sequential", NULL);

    std::string uidStr = intToString(getGUID(constraint));
    xmlNewChild(constraintNode, NULL, BAD_CAST "uid", BAD_CAST uidStr.c_str());
    xmlNewChild(constraintNode, NULL, BAD_CAST "name", 
		BAD_CAST constraint->getName().c_str());

    std::vector <ConstraintMacroPtr>constraintList;
    constraint->getConstraintMacros(constraintList);
    
    std::string childUidStr;
    for ( int i = 0; i < constraintList.size(); i++ ) {
      childUidStr = intToString(getGUID(constraintList[i]));
      xmlNewChild(constraintNode, NULL, BAD_CAST "child", BAD_CAST childUidStr.c_str());
    }
  }
  else {
    printf("Don't know how to parse constraint '%s'. Ignoring.\n", 
	   constraint->getName().c_str()); 
  }
}

//TODO
//Storing algorithm preparation - ensures all nodes efficent storage via uid references to children 
void DatabaseManager::postOrderAddConstraintMacroToQueue(ConstraintMacroConstPtr constraint, 
							 std::queue<ConstraintMacroConstPtr> *q, 
							 std::set<ConstraintMacroConstPtr> *done) {
  if (constraint->getConstraintMacroType() != ConstraintMacro::ATOMIC) 
    {
      std::vector<ConstraintMacroPtr> constraintList;
      constraint->getConstraintMacros(constraintList);
      for( int i = 0; i < constraintList.size(); i++ ) 
	  {
	    postOrderAddConstraintMacroToQueue(constraintList[i], q, done);
	  }
    }
  if (done->count(constraint) == 0) 
    {
      q->push(constraint);
      done->insert(constraint);
    }
}

//main API call for storing all objects
void DatabaseManager::store(const std::vector<AffConstPtr> &affordanceList, const std::vector<ConstraintMacroConstPtr> &constraintList) {
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
	std::queue<ConstraintMacroConstPtr>* constraintQueue = new std::queue<ConstraintMacroConstPtr>();
	std::set<ConstraintMacroConstPtr>* processedConstraintMacros = new std::set<ConstraintMacroConstPtr>();

  //Use a postorder traversal of the constraint trees to ensure child constraints
  //are stored before the constraints that depend on them
	for(int i = 0; i < constraintList.size(); i++) {
		postOrderAddConstraintMacroToQueue(constraintList[i], constraintQueue, processedConstraintMacros);
	}	

  //add each constraint to the xml tree
	while (!constraintQueue->empty()){
	  addConstraintMacroToNode(constraintQueue->front(), node);
	  constraintQueue->pop();
	}

  //finally, save the file
	xmlSaveFormatFileEnc(_filename.c_str(), doc, "UTF-8", 1);
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

void printAffordanceMap(std::map<int, AffConstPtr>* affordances) {
  printf("Affordance Map:\n");
  for(std::map<int, AffConstPtr>::iterator iter = affordances->begin(); iter != affordances->end(); ++iter)
  {
    printf("key: %i, name:%s\n", iter->first, iter->second->getName().c_str());
  }
}

//Creates an Affordance object from an XML node and stores it in a map of uids to affordances
AffPtr serializeAffordance(xmlDocPtr doc, xmlNode* node, 
				std::map<int, AffPtr>* affordances) {
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


  AffPtr affordance(new AffordanceState(name));
  (*affordances)[uid] = affordance;
  return affordance;
}

//Creates an Atomic ConstraintMacro object from an XML node and stores it in a map of uids to constraints
ConstraintMacroPtr serializeAtomicConstraintMacro(xmlDocPtr doc, xmlNode* node, std::map<int, AffPtr>* affordances, std::map<int, ConstraintMacroPtr>* constraints) {
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
        printf("WARNING: Ignored unknown entry '%s' while parsing Atomic ConstraintMacro.\n", current_node->name);
      }
    }
  }

/*
  printf("\tname: %s\n", name);
  printf("\tuid: %i\n", uid);
  printf("\taff1uid: %i\n", aff1uid);
  printf("\taff2uid: %i\n", aff2uid);
*/

  AffPtr aff1 = (*affordances).find(aff1uid)->second;
  AffPtr aff2 = (*affordances).find(aff2uid)->second;

  //printAffordanceMap(affordances);
  //printf("aff1name: %s\n", aff1->getName().c_str());

  ///TODO FIX ME!!!!!!!!!!!!!!!!!!!!!!!!
  //I SHOULD NOT BE ConstraintMacro::TANGENT!!!!!!!!!!!!!
  AtomicConstraintPtr atomicConstraint(new AtomicConstraint(aff1, aff2, AtomicConstraint::TANGENT));
  ConstraintMacroPtr constraint(new ConstraintMacro(name, atomicConstraint));

  (*constraints)[uid] = constraint;
  return constraint;
}

//Creates a Sequential ConstraintMacro object from an XML node and stores in an a map of uids to constraints
ConstraintMacroPtr serializeSequentialConstraintMacro(xmlDocPtr doc, xmlNode* node, 
						 std::map<int, AffPtr>* affordances, 
						 std::map<int, ConstraintMacroPtr>* constraints) 
{
  xmlNode* current_node = NULL;
  char* name;
  int uid;
  std::vector<ConstraintMacroPtr> childConstraintMacros;
  std::vector<int> childuids;

  for (current_node = node->children; current_node; current_node = current_node->next) {
    if (current_node->type == XML_ELEMENT_NODE) {
      if (nodeNameIs(current_node, "child")) {
        //get the child uid
        int childuid = atoi(value(doc, current_node));
        //look up the corresponding constraint in the map
        ConstraintMacroPtr childConstraintMacro = (*constraints).find(childuid)->second;
        childConstraintMacros.push_back(childConstraintMacro);
        childuids.push_back(childuid);
      }
      else if (nodeNameIs(current_node, "uid")) {
        uid = atoi(value(doc, current_node));
      }
      else if (nodeNameIs(current_node, "name")) {
        name = value(doc, current_node);
      }
      else {
        printf("WARNING: Ignored unknown entry '%s' while parsing ConstraintMacro.\n", current_node->name);
      }
    }
  }


  ///TODO FIX ME!!!!!!!!!!!!!!!!!!!!!!!!
  //I SHOULD NOT BE ConstraintMacro::SEQUENTIAL!!!!!!!!!!!!!
  ConstraintMacroPtr constraint(new ConstraintMacro(name, ConstraintMacro::SEQUENTIAL));
  for (int i = 0; i < childConstraintMacros.size(); i++ ) {
    constraint->addConstraintMacro(childConstraintMacros[i]);
  }

  (*constraints)[uid] = constraint;
  return constraint;
}

//Dispatch the proper methods to create objects from XML nodes based on node name
void DatabaseManager::parseTreeHelper(xmlDocPtr doc, xmlNode* xmlnode, std::map<int,  AffPtr>* affordances, std::map<int, ConstraintMacroPtr>* constraints) {
  xmlNode* current_node = NULL;

  for (current_node = xmlnode; current_node; current_node = current_node->next) {
      if (current_node->type == XML_ELEMENT_NODE) {
        if(nodeNameIs(current_node, "affordance")) {
          serializeAffordance(doc, current_node, affordances);
        }
        else if (nodeNameIs(current_node, "atomic")) {
          serializeAtomicConstraintMacro(doc, current_node, affordances, constraints);
        }
        else if (nodeNameIs(current_node, "sequential")){
          serializeSequentialConstraintMacro(doc, current_node, affordances, constraints);
        }
        else {
          //parse the child nodes
          parseTreeHelper(doc, current_node->children, affordances, constraints);
        }
      }
  }
}

//Nice wrapper API call to parse the entire xml tree from a root node into ConstraintMacros and Affordances 
void DatabaseManager::parseTree(xmlDocPtr doc, xmlNode* root) {
  //printf("calling helper\n");
  std::map<int, ConstraintMacroPtr>* constraints = new std::map<int, ConstraintMacroPtr>();
  std::map<int, AffPtr>* affordances = new std::map<int, AffPtr>();
  parseTreeHelper(doc, root, affordances, constraints);
  

  //Clear and update the affordance and constraint lists
  _affordanceList.clear();
  _constraintList.clear();
  
  for(std::map<int,AffPtr>::iterator ii=affordances->begin(); ii!=affordances->end(); ++ii) {
    _affordanceList.push_back((*ii).second);
  }

  for(std::map<int,ConstraintMacroPtr>::iterator ii=constraints->begin(); ii!=constraints->end(); ++ii) {
    _constraintList.push_back((*ii).second);
  }
}

//Wrapper call that kicks off the parsing of the file specified my _filename
void DatabaseManager::parseFile(){
  xmlDocPtr readDoc;
  readDoc = xmlReadFile(_filename.c_str(), NULL, 0);
  if (readDoc == NULL) {
    fprintf(stderr, "Failed to parse %s\n", _filename.c_str());
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
void DatabaseManager::getAffordances(vector<AffPtr> &affordances) 
{
  affordances.clear();
  affordances.insert(affordances.end(),
		     _affordanceList.begin(),
		     _affordanceList.end());
}

void DatabaseManager::getConstraintMacros(vector<ConstraintMacroPtr> &constraints) 
{
  constraints.clear();
  constraints.insert(constraints.end(),
		     _constraintList.begin(),
		     _constraintList.end());
}
