#include <stdio.h>
#include <sstream>
#include <queue>
#include <set>
#include <map>
#include "action_authoring/ConstraintMacro.h"
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

void printConstraintMacro(ConstraintMacroPtr constraint, int num_tabs=0) {
  std::string constraintString;
  switch (constraint->getConstraintMacroType()) {
  case ConstraintMacro::ATOMIC:
    constraintString = "ATOMIC ";
    break;
  case ConstraintMacro::SEQUENTIAL:
    constraintString = "SEQUENTIAL ";
    break;
  default:
    constraintString = "UNKNOWN ";
    break;
  }

  tabprintf(constraintString += constraint->getName(), num_tabs);

  if (constraint->getConstraintMacroType() == ConstraintMacro::ATOMIC) {
    std::string atomicConstraintString;
    switch (constraint->getAtomicConstraint()->getRelationType()) {
    case AtomicConstraint::TANGENT:
      atomicConstraintString = "TANGENT";
      break;
    case AtomicConstraint::NORMAL:
      atomicConstraintString = "NORMAL";
      break;
    default:
      atomicConstraintString = "UNKNOWN";
      break;
    }
    
    tabprintf(atomicConstraintString, num_tabs + 1);
    tabprintf(constraint->getAtomicConstraint()->getAffordance1()->getName(), num_tabs + 1);
    tabprintf(constraint->getAtomicConstraint()->getAffordance2()->getName(), num_tabs + 1);
  }
  else {
    vector<ConstraintMacroPtr> constraints;
    constraint->getConstraintMacros(constraints);
    for (int i = 0; i < constraints.size(); i++ ) {
      printConstraintMacro(constraints[i], num_tabs + 1);
    }
  }
}

int main() {

  AffPtr rhand  (new AffordanceState("Right Hand"));
  AffPtr lhand  (new AffordanceState("Left Hand"));
  AffPtr rfoot (new AffordanceState("Right Foot"));
  AffPtr lfoot (new AffordanceState("Left Foot"));
  AffPtr wheel (new AffordanceState("Steering Wheel"));
  AffPtr gas   (new AffordanceState("Gas Pedal"));
  AffPtr brake (new AffordanceState("Brake Pedal"));
  std::vector<AffConstPtr> affordanceList;
  affordanceList.push_back(rhand);
  affordanceList.push_back(lhand);
  affordanceList.push_back(rfoot);
  affordanceList.push_back(lfoot);
  affordanceList.push_back(wheel);
  affordanceList.push_back(gas);
  affordanceList.push_back(brake);

  AtomicConstraintPtr rfoot_gas_relation (new AtomicConstraint(rfoot, gas, AtomicConstraint::NORMAL));
  AtomicConstraintPtr lfoot_brake_relation(new AtomicConstraint(lfoot, brake, AtomicConstraint::TANGENT));
  AtomicConstraintPtr rhand_wheel_relation(new AtomicConstraint(rhand, wheel, AtomicConstraint::TANGENT));
  AtomicConstraintPtr lhand_wheel_relation(new AtomicConstraint(lhand, wheel, AtomicConstraint::TANGENT));

  ConstraintMacroPtr rfoot_gas  (new ConstraintMacro("Right Foot to Gas Pedal", rfoot_gas_relation));
  ConstraintMacroPtr lfoot_brake (new ConstraintMacro("Left Foot to Brake Pedal", lfoot_brake_relation));                                                                                            
  ConstraintMacroPtr rhand_wheel (new ConstraintMacro("Right Hand To Wheel", rhand_wheel_relation));
  ConstraintMacroPtr lhand_wheel (new ConstraintMacro("Left Hand To Wheel", lhand_wheel_relation));
  std::vector<ConstraintMacroConstPtr> constraintList;
  constraintList.push_back(rfoot_gas);
  constraintList.push_back(lfoot_brake);
  constraintList.push_back(rhand_wheel);
  constraintList.push_back(lhand_wheel);

  ConstraintMacroPtr pedals  (new ConstraintMacro("Pedals", ConstraintMacro::SEQUENTIAL));
  ConstraintMacroPtr hands   (new ConstraintMacro("Hands", ConstraintMacro::SEQUENTIAL));
  ConstraintMacroPtr ingress (new ConstraintMacro("Ingress", ConstraintMacro::SEQUENTIAL));
  
  pedals->addConstraintMacro(rfoot_gas);
  pedals->addConstraintMacro(lfoot_brake);
  hands->addConstraintMacro(rhand_wheel);
  hands->addConstraintMacro(lhand_wheel);

  ingress->addConstraintMacro(pedals);
  ingress->addConstraintMacro(hands);

  constraintList.push_back(pedals);
  constraintList.push_back(hands);
  constraintList.push_back(ingress);

  //print the top level constraint
  printf("\nThis is a constraint created in the function\n");
  printConstraintMacro(ingress);


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
  std::vector<AffPtr> revivedAffordances;
  db->getAffordances(revivedAffordances);
  std::vector<ConstraintMacroPtr> revivedConstraintMacros;
  db->getConstraintMacros(revivedConstraintMacros);

  //this is an example of how to iterate an print the names of the retrieved data items
  /*
  for (int i = 0; i < revivedAffordances.size(); i++ ){
    printf("name: %s\n", revivedAffordances[i]->getName().c_str());
  }

  for (int i = 0; i < revivedConstraintMacros.size(); i++ ){
    printf("name: %s\n", revivedConstraintMacros[i]->getName());
  }
  */

  //print the same constraint as created before, but this time using the data from the file 
  printf("\nThis is the same constraint, written to and then reconstructed from a file:\n");
  printConstraintMacro(revivedConstraintMacros[6]);

  return(0);
}
