#include <stdio.h>
#include <sstream>
#include <queue>
#include <set>
#include <map>
#include "action_authoring/Constraint.h"
#include "action_authoring/DatabaseManager.h"

using namespace action_authoring;
using namespace std;
using namespace boost;
using namespace affordance;

void tabprintf(std::string string, int num_tabs) {
  for ( int i = 0; i < num_tabs; i++ ) {
    printf("\t");
  }
  printf("%s\n", string.c_str());
}

void printConstraint(ConstraintConstPtr constraint, int num_tabs=0) {
  std::string constraintString;
  switch (constraint->getConstraintType()) {
  case Constraint::ATOMIC:
    constraintString = "ATOMIC ";
    break;
  case Constraint::SEQUENTIAL:
    constraintString = "SEQUENTIAL ";
    break;
  default:
    constraintString = "UNKNOWN ";
    break;
  }

  tabprintf(constraintString += constraint->getName(), num_tabs);

  if (constraint->getConstraintType() == Constraint::ATOMIC) {
    std::string affordanceRelationString;
    switch (constraint->getAffordanceRelation()->getRelationType()) {
    case AffordanceRelation::TANGENT:
      affordanceRelationString = "TANGENT";
      break;
    case AffordanceRelation::NORMAL:
      affordanceRelationString = "NORMAL";
      break;
    default:
      affordanceRelationString = "UNKNOWN";
      break;
    }
    
    tabprintf(affordanceRelationString, num_tabs + 1);
    tabprintf(constraint->getAffordanceRelation()->getAffordance1()->getName(), num_tabs + 1);
    tabprintf(constraint->getAffordanceRelation()->getAffordance2()->getName(), num_tabs + 1);
  }
  else {
    vector<ConstraintConstPtr> constraints;
    constraint->getConstraints(constraints);
    for (int i = 0; i < constraints.size(); i++ ) {
      printConstraint(constraints[i], num_tabs + 1);
    }
  }
}

int main() {

  AffConstPtr rhand  (new AffordanceState("Right Hand"));
  AffConstPtr lhand  (new AffordanceState("Left Hand"));
  AffConstPtr rfoot (new AffordanceState("Right Foot"));
  AffConstPtr lfoot (new AffordanceState("Left Foot"));
  AffConstPtr wheel (new AffordanceState("Steering Wheel"));
  AffConstPtr gas   (new AffordanceState("Gas Pedal"));
  AffConstPtr brake (new AffordanceState("Brake Pedal"));
  std::vector<AffConstPtr> affordanceList;
  affordanceList.push_back(rhand);
  affordanceList.push_back(lhand);
  affordanceList.push_back(rfoot);
  affordanceList.push_back(lfoot);
  affordanceList.push_back(wheel);
  affordanceList.push_back(gas);
  affordanceList.push_back(brake);


  AffRelationConstPtr rfoot_gas_relation (new AffordanceRelation(rfoot, gas, AffordanceRelation::NORMAL));
  AffRelationConstPtr lfoot_brake_relation(new AffordanceRelation(lfoot, brake, AffordanceRelation::TANGENT));
  AffRelationConstPtr rhand_wheel_relation(new AffordanceRelation(rhand, wheel, AffordanceRelation::TANGENT));
  AffRelationConstPtr lhand_wheel_relation(new AffordanceRelation(lhand, wheel, AffordanceRelation::TANGENT));

  ConstraintConstPtr rfoot_gas  (new Constraint("Right Foot to Gas Pedal", rfoot_gas_relation));
  ConstraintConstPtr lfoot_brake (new Constraint("Left Foot to Brake Pedal", lfoot_brake_relation));                                                                                            
  ConstraintConstPtr rhand_wheel (new Constraint("Right Hand To Wheel", rhand_wheel_relation));
  ConstraintConstPtr lhand_wheel (new Constraint("Left Hand To Wheel", lhand_wheel_relation));
  std::vector<ConstraintConstPtr> constraintList;
  constraintList.push_back(rfoot_gas);
  constraintList.push_back(lfoot_brake);
  constraintList.push_back(rhand_wheel);
  constraintList.push_back(lhand_wheel);

  ConstraintPtr pedals  (new Constraint("Pedals", Constraint::SEQUENTIAL));
  ConstraintPtr hands   (new Constraint("Hands", Constraint::SEQUENTIAL));
  ConstraintPtr ingress (new Constraint("Ingress", Constraint::SEQUENTIAL));
  
  pedals->addConstraint(rfoot_gas);
  pedals->addConstraint(lfoot_brake);
  hands->addConstraint(rhand_wheel);
  hands->addConstraint(lhand_wheel);

  ingress->addConstraint(pedals);
  ingress->addConstraint(hands);

  constraintList.push_back(pedals);
  constraintList.push_back(hands);
  constraintList.push_back(ingress);

  //print the top level constraint
  printf("\nThis is a constraint created in the function\n");
  printConstraint(ingress);


  //To instantiate the databse manager, you must give it a file name
  shared_ptr<DatabaseManager> db (new DatabaseManager("constraints.xml"));
  
  //to change the file name later, use the setFilename method
  //db->setFilename("someotherfilename.xml");

  //to store state, use the db->store method and pass in all the data at once
  db->store(affordanceList, constraintList);

  //to retrieve the data, you must first call parseFile()!
  //this parses the current filename that the db has been given
  //at initialization or by the setFilename method
  db->parseFile();

  //to get the objects back, use db->get<YourTypeHere>()
  std::vector<AffConstPtr> revivedAffordances;
  db->getAffordances(revivedAffordances);
  std::vector<ConstraintConstPtr> revivedConstraints;
  db->getConstraints(revivedConstraints);

  //this is an example of how to iterate an print the names of the retrieved data items
  /*
  for (int i = 0; i < revivedAffordances.size(); i++ ){
    printf("name: %s\n", revivedAffordances[i]->getName().c_str());
  }

  for (int i = 0; i < revivedConstraints.size(); i++ ){
    printf("name: %s\n", revivedConstraints[i]->getName());
  }
  */

  //print the same constraint as created before, but this time using the data from the file 
  printf("\nThis is the same constraint, written to and then reconstructed from a file:\n");
  printConstraint(revivedConstraints[6]);

  return(0);
}
