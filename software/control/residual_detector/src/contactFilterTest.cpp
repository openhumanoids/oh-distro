//
// Created by manuelli on 11/24/15.
//

#include "ContactFilter.hpp"

int main( int argc, char* argv[]){
  std::cout << "running contact filter test" << std::endl;
  ContactFilter contactFilter;
  contactFilter.addRobotFromURDF();
  contactFilter.runFindLinkTest();
  return 0;
}
