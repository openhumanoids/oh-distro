#include <stdio.h>
#include <lcm/lcm-cpp.hpp>
#include <boost/shared_ptr.hpp>
#include "AffordanceServer.h"
#include <iostream>

using namespace boost;
using namespace std;
using namespace affordance;
int main(int argc, char ** argv)
{
	shared_ptr<lcm::LCM> theLcm(new lcm::LCM());
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
