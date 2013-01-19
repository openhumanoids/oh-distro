#ifndef DATABASE_MANAGER_H
#define DATABASE_MANAGER_H

#include "Constraint.h"
#include "affordance/AffordanceState.h"
#include <vector>
#include <queue>
#include <set>
#include <map>
#include <libxml/parser.h>
#include <libxml/tree.h>

namespace action_authoring 
{
  /**todo comment*/
  class DatabaseManager 
  {
  private:

    //member variables
    //todo : could you have boost::unordered_map instead of map?  (which would be more efficient)
    std::string _filename;
    int _guidCounter;
    std::map<affordance::AffConstPtr, int> _affordanceToGUID; //todo: comment like "maps from { } --> {}
    std::map<ConstraintConstPtr, int> _constraintToGUID; //todo : see above
    std::vector<affordance::AffConstPtr> _affordanceList; 
    std::vector<ConstraintConstPtr> _constraintList;
    
    //writing to file helper function
    //todo : in the cpp file, add doxygen comments for each of these methods
    static std::string intToString(const int &i);
    void addAffordanceToNode(affordance::AffConstPtr affordance, xmlNodePtr node);
    void addConstraintToNode(ConstraintConstPtr constraint, xmlNodePtr node);
    void postOrderAddConstraintToQueue(ConstraintConstPtr constraint, 
				       std::queue<ConstraintConstPtr>* q, 
				       std::set<ConstraintConstPtr>* done);
    int getGUID(ConstraintConstPtr constraint);
    int getGUID(affordance::AffConstPtr affordance);
    
    //reading from file helper functions
    //todo: add comments in the cpp file for parseTreeHelper and parseTree
    //doxygen style.  describe the arguments and functionality of each method
    void parseTreeHelper(xmlDocPtr doc, xmlNode *xmlnode, 
			 std::map<int,  affordance::AffConstPtr> *affordances,
			 std::map<int, ConstraintConstPtr> *constraints);
    void parseTree(xmlDocPtr doc, xmlNode *root);
    
  public:
    //constructor
    DatabaseManager(const std::string &filename);
    
    //mutators
    void setFilename(const std::string & filename);
    
    //Writing data to a file
    //todo : add (in the cpp file) doxygen style comments 
    void store(const std::vector<affordance::AffConstPtr> &affordanceList, 
	       const std::vector<ConstraintConstPtr> &constrainList);
    
    //Reading data from a file
    //todo : add (in the cpp file) doxygen style comments
    void parseFile();
    void getAffordances(std::vector<affordance::AffConstPtr> &affordacnes) const;
    void getConstraints(std::vector<ConstraintConstPtr> &constraints) const;
  }; //class DatabaseManager
} //namespace action_authoring

#endif //DATABASE_MANAGER_H
