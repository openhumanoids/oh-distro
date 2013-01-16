#include "Constraint.h"
#include "Action.h"
#include <iostream>
#include <string>
#include <vector>
#include <stdio.h>

void printConstraintList(std::vector<Constraint*> constraintList) {
 for (int i = 0; i < constraintList.size(); i++) {
    printf("\t%s\n", constraintList[i]->getName().c_str());
    printf("\t\t%s ", constraintList[i]->getAffordance1()->getName().c_str());
    switch (constraintList[i]->getConstraintType()){
    case Constraint::TANGENT:
      printf("tangent ");
      break;
    case Constraint::NORMAL:
      printf("normal ");
      break;
    } printf("%s\n", constraintList[i]->getAffordance2()->getName().c_str());
  }
}

void printAffordanceList(std::vector<Affordance*> affordanceList) {
 for (int i = 0; i < affordanceList.size(); i++) {
    printf("\t%s\n", affordanceList[i]->getName().c_str());
  }
}

void printActionList(std::vector<Action*> actionList) {
  for (int i = 0; i < actionList.size(); i++) {
    printf("\t%s\n", actionList[i]->getName().c_str());
    printf("\t==========================\n");
    printConstraintList(actionList[i]->getConstraints());
  }
}

int main() {
  Affordance* rhand = new Affordance("Right Hand");
  Affordance* lhand = new Affordance("Left Hand");
  Affordance* rfoot = new Affordance("Right Foot");
  Affordance* lfoot = new Affordance("Left Foot");
  Affordance* wheel = new Affordance("Steering Wheel");
  Affordance* gas   = new Affordance("Gas Pedal");
  Affordance* brake = new Affordance("Brake Pedal");

  std::vector<Affordance*> affordanceList;
  affordanceList.reserve(10);
  affordanceList.push_back(rhand);
  affordanceList.push_back(lhand);
  affordanceList.push_back(rfoot);
  affordanceList.push_back(lfoot);
  affordanceList.push_back(wheel);
  affordanceList.push_back(gas);
  affordanceList.push_back(brake);

  Constraint* rfoot_gas   = new Constraint("Gas Pedal Constraint", rfoot, gas, Constraint::NORMAL);
  Constraint* lfoot_brake = new Constraint("Brake Pedal Constraint", lfoot, brake, Constraint::TANGENT);
  Constraint* rhand_wheel = new Constraint("Right Hand Wheel Constraint", rhand, wheel, Constraint::TANGENT);
  Constraint* lhand_wheel = new Constraint("Left Hand Wheel Constraint", lhand, wheel, Constraint::TANGENT);
				
  std::vector<Constraint*> constraintList;
  constraintList.reserve(10);
  constraintList.push_back(rfoot_gas);
  constraintList.push_back(lfoot_brake);
  constraintList.push_back(rhand_wheel);
  constraintList.push_back(lhand_wheel);

  Action* ingress_car = new Action("Ingress Car Action");
  ingress_car->addConstraint(rfoot_gas);
  ingress_car->addConstraint(lfoot_brake);
  ingress_car->addConstraint(rhand_wheel);
  ingress_car->addConstraint(lhand_wheel);

  std::vector<Action*> actionList;
  actionList.push_back(ingress_car);

  printf("Test models poplated\n");
  printf("====================\n");
  printf("Created %d affordances:\n", affordanceList.size()); 
  printAffordanceList(affordanceList);
  printf("\n");
  printf("Created %d constraints:\n", constraintList.size());  
  printConstraintList(constraintList);
  printf("\n");
  printf("Created %d actions:\n", actionList.size());
  printActionList(actionList);
  printf("\n\nFinished.\n\n");
  return 0;
}
