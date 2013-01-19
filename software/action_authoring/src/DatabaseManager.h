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
  protected:
    //member variables
    std::string _filename;
    int _guidCounter;
    std::map<affordance::AffConstPtr, int> _affordanceToGUID;
    std::map<ConstraintConstPtr, int> _constraintToGUID;
    std::vector<affordance::AffConstPtr> _affordanceList;
    std::vector<ConstraintConstPtr> _constraintList;
    
    //writing to file helper functions
    static std::string intToString(const int &i);
    void addAffordanceToNode(affordance::AffConstPtr affordance, xmlNodePtr node);
    void addConstraintToNode(ConstraintConstPtr constraint, xmlNodePtr node);
    void postOrderAddConstraintToQueue(ConstraintConstPtr constraint, const std::queue<ConstraintConstPtr>& q, const std::set<ConstraintConstPtr>& done);
    int getGUID(ConstraintConstPtr constraint);
    int getGUID(affordance::AffConstPtr affordance);
    
    //reading from file helper functions
    void parseTreeHelper(xmlDocPtr doc, const xmlNode &xmlnode, 
			 const std::map<int,  affordance::AffConstPtr> &affordances,
			 const std::map<int, ConstraintConstPtr>& constraints);
    void parseTree(xmlDocPtr doc, const xmlNode &root);
    
  public:
    //constructor
    DatabaseManager(const std::string &filename);
    
    //mutators
    void setFilename(const std::string & filename);
    
    //Writing data to a file
    void store(const std::vector<affordance::AffConstPtr> &affordanceList, 
	       const std::vector<ConstraintConstPtr> &constrainList);
    
    //Reading data from a file
    void parseFile();
    void getAffordances(std::vector<affordance::AffConstPtr> &affordacnes) const;
    void getConstraints(std::vector<ConstraintConstPtr> &constraints) const;
  }; //class DatabaseManager
} //namespace action_authoring

#endif //DATABASE_MANAGER_H
