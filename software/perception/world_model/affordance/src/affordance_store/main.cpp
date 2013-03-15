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
  

  string lcm_url="";
  std::string role_upper;
  for(short i = 0; i < role.size(); ++i)
     role_upper+= (std::toupper(role[i]));
  if((role.compare("robot") == 0) || (role.compare("base") == 0) ){
    for(short i = 0; i < role_upper.size(); ++i)
       role_upper[i] = (std::toupper(role_upper[i]));
    string env_variable_name = string("LCM_URL_DRC_" + role_upper); 
    char* env_variable;
    env_variable = getenv (env_variable_name.c_str());
    if (env_variable!=NULL){
      //printf ("The env_variable is: %s\n",env_variable);      
      lcm_url = string(env_variable);
    }else{
      std::cout << env_variable_name << " environment variable has not been set ["<< lcm_url <<"]\n";     
      exit(-1);
    }
  }else{
    std::cout << "Role not understood, choose: robot or base\n";
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
