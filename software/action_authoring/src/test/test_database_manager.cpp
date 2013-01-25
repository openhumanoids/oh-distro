#include <stdio.h>
#include <iostream>
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

void printManipulationRelation(ManRelPtr manipulationRelation, int num_tabs) {
    std::string relationStateString;
    if (manipulationRelation->getRelationState()->getRelationType() == RelationState::UNDEFINED )
      relationStateString = "Undefined Type";
    else {
      relationStateString = "could not find match for relation type";
    }
    tabprintf("Manipulation Relation - " + relationStateString, num_tabs);
    tabprintf(manipulationRelation->getManipulator()->getName(), num_tabs);
    tabprintf(manipulationRelation->getAffordance()->getName(), num_tabs);
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
    printManipulationRelation(constraint->getAtomicConstraint()->getRelation(), num_tabs + 1);
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

  ManipulatorStateConstPtr rhand (new ManipulatorState("Right Hand"));
  ManipulatorStateConstPtr lhand (new ManipulatorState("Left Hand"));
  ManipulatorStateConstPtr rfoot (new ManipulatorState("Right Foot"));
  ManipulatorStateConstPtr lfoot (new ManipulatorState("Left Foot"));

  AffConstPtr wheel (new AffordanceState("Steering Wheel"));
  AffConstPtr gas   (new AffordanceState("Gas Pedal"));
  AffConstPtr brake (new AffordanceState("Brake Pedal"));

  vector<AffConstPtr> affordanceList;
  affordanceList.push_back(wheel);
  affordanceList.push_back(gas);
  affordanceList.push_back(brake);

  RelationStatePtr relstate (new RelationState(RelationState::UNDEFINED));

  ManRelPtr rfoot_gas_relation (new ManipulationRelation(gas, rfoot, relstate));
  ManRelPtr lfoot_brake_relation(new ManipulationRelation(brake, lfoot, relstate));
  ManRelPtr rhand_wheel_relation(new ManipulationRelation(wheel, rhand, relstate));
  ManRelPtr lhand_wheel_relation(new ManipulationRelation(wheel, lhand, relstate));
  
  ConstraintMacroPtr rfoot_gas  (new ConstraintMacro("Right Foot to Gas Pedal", (AtomicConstraintPtr)new AtomicConstraint(rfoot_gas_relation)));
  ConstraintMacroPtr lfoot_brake (new ConstraintMacro("Left Foot to Brake Pedal", (AtomicConstraintPtr)new AtomicConstraint(lfoot_brake_relation))); 
  ConstraintMacroPtr rhand_wheel (new ConstraintMacro("Right Hand To Wheel", (AtomicConstraintPtr)new AtomicConstraint(rhand_wheel_relation)));
  ConstraintMacroPtr lhand_wheel (new ConstraintMacro("Left Hand To Wheel", (AtomicConstraintPtr)new AtomicConstraint(lhand_wheel_relation)));

  ConstraintMacroPtr hands (new ConstraintMacro("Hands", ConstraintMacro::SEQUENTIAL));
  hands->appendConstraintMacro(rhand_wheel);
  hands->appendConstraintMacro(lhand_wheel);

  ConstraintMacroPtr feet (new ConstraintMacro("Feet", ConstraintMacro::SEQUENTIAL));
  feet->appendConstraintMacro(rfoot_gas);
  feet->appendConstraintMacro(lfoot_brake);

  ConstraintMacroPtr ingress (new ConstraintMacro("Ingress", ConstraintMacro::SEQUENTIAL));
  ingress->appendConstraintMacro(hands);
  ingress->appendConstraintMacro(feet);

  std::vector<ConstraintMacroPtr> constraintList;
  constraintList.push_back(ingress);


  //print the top level constraint
  printf("\nThis is a constraint created in the function\n");
  printConstraintMacro(ingress);

  //to store state, use the db->store method and pass in all the data at once
  DatabaseManager::store("constraints.xml", affordanceList, constraintList);

  printf("\nDone storing.\n");
  printf("\nRetrieving from file.\n\n");

  vector<AffConstPtr> revivedAffordances;
  vector<ConstraintMacroPtr> revivedConstraintMacros;
  DatabaseManager::retrieve("constraints.xml", revivedAffordances, revivedConstraintMacros);

  printf("\nDone retrieving.\n");

  //print the same constraint as created before, but this time using the data from the file 
  printf("\nThis is the same constraint, written to and then reconstructed from a file:\n");

  for (int i = 0; i < revivedConstraintMacros.size(); i++ ){
    if (revivedConstraintMacros[i]->getName() == "Ingress") {
      printConstraintMacro(revivedConstraintMacros[i]);
    }
  }
  return(0);
}
