#include <stdio.h>
#include <lcm/lcm-cpp.hpp>
#include <boost/shared_ptr.hpp>
#include "AffordanceServer.h"
#include <iostream>

#include <ConciseArgs>

using namespace boost;
using namespace std;
using namespace affordance;
int main(int argc, char ** argv)
{
  string role = "robot";
  ConciseArgs opt(argc, (char**)argv);
  opt.add(role, "r", "role","Role - robot or base");
  opt.parse();
  std::cout << "role: " << role << "\n";

  shared_ptr<lcm::LCM> theLcm(new lcm::LCM( "" ));
  if (!theLcm->good())
  {
    cerr << "Cannot create lcm object" << endl;
    return -1;
  }

  //create the server
  AffordanceServer s(theLcm);
  s.setRole(role);

  //lcm loop
  cout << "\nstarting lcm loop" << endl;
  while (0 == theLcm->handle());

  return 0;
}
