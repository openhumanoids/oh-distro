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
  

  
  string lcm_url ="";
  if(role.compare("robot") == 0){
     lcm_url = ""; // put robot url if needed
  }else if(role.compare("base") == 0){  
     lcm_url = "udpm://239.255.12.68:1268?ttl=1";
  }else{
    std::cout << "DRC Viewer role not understood, choose: robot or base\n";
    return 1;
  }

  
  shared_ptr<lcm::LCM> theLcm(new lcm::LCM( lcm_url ));
  if (!theLcm->good())
  {
    cerr << "Cannot create lcm object" << endl;
    return -1;
  }

  //create the server
  AffordanceServer s(theLcm);

  //lcm loop
  cout << "\nstarting lcm loop" << endl;
  while (0 == theLcm->handle());

  return 0;
}
