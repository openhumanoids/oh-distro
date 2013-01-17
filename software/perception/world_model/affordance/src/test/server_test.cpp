/*
 * server_test.cpp
 *
 *  Created on: Jan 16, 2013
 *      Author: mfleder
 */

#include <stdio.h>
#include <lcm/lcm-cpp.hpp>
#include <boost/shared_ptr.hpp>
#include "AffordanceServer.h"
#include "AffordanceUpWrapper.h"
#include "AffordanceState.h"
#include "OpenGL_Affordance.h"
#include <iostream>

using namespace boost;
using namespace std;
using namespace affordance;

void runTest(const shared_ptr<lcm::LCM> lcm)
{
	boost::posix_time::seconds sleepTime(1); //update every 1 seconds
	AffordanceUpWrapper wrapper(lcm);
	int j = 0;

	while(true)
	{
		cout << "\n\n===" << j << endl;
		cout << wrapper << endl;
		boost::this_thread::sleep(sleepTime); //======sleep

		//draw
		vector<AffPtr> affordances;
		wrapper.getAllAffordances(affordances);
		for (uint i = 0; i < affordances.size(); i++)
		{
			OpenGL_Affordance nextGlAff(*affordances[i]);
			//add to scene, .draw()
		}

		//======add something to the server
		if (j++ == 1 || j == 10)
		{
		  AffordanceState s("First Affordance");
		  s._map_id 	 = 7;
		  s._object_id = 42;
		  s._otdf_id = AffordanceState::CYLINDER;
		  s._states[AffordanceState::RADIUS_NAME] = (j == 10 ? 9999 : 1.41);
		  wrapper.addOrReplace(s);
		}

		if (j == 15)
		{
		  AffordanceState s("Second Affordance");
		  s._map_id 	 = 7;
		  s._object_id = 21;		  
		  s._otdf_id = AffordanceState::BOX;
		  s._states[AffordanceState::LENGTH_NAME] = .11111;
		  wrapper.addOrReplace(s);
		}
	}
}

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
    std::ostream& operator<<( std::ostream& out, AffordanceUpWrapper& other );


    //thread will modify the server state and listen for changes via
    //the wrapper
	boost::thread testThread = boost::thread(runTest, theLcm);

    //lcm loop
    cout << "\nstarting lcm loop" << endl;
    while (0 == theLcm->handle());

    return 0;
}
