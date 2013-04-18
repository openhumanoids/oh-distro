/*
 * server_test.cpp
 *
 *  Created on: Jan 16, 2013
 *      Author: mfleder
 */

#include <stdio.h>
#include <lcm/lcm-cpp.hpp>
#include <boost/shared_ptr.hpp>
#include "affordance/AffordanceServer.h"
#include "affordance/AffordanceUpWrapper.h"
#include "affordance/AffordanceState.h"
//#include "affordance/OpenGL_Affordance.h"
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
        j++;


		//draw
		vector<AffConstPtr> affordances;
		wrapper.getAllAffordances(affordances);
		for (uint i = 0; i < affordances.size(); i++)
		{
		  //OpenGL_Affordance nextGlAff(affordances[i]);
			//add to scene, .draw()
		}

		//======add something to the server
		if (j == 1 || j == 10)
		{
		  AffordanceState s;
		  s._map_id 	 = 7;
		  s._uid = 0;
		  s._params[AffordanceState::RADIUS_NAME] = (j == 10 ? 9999 : 1.41);
          s._params[AffordanceState::LENGTH_NAME] = 9;
          s.setType(AffordanceState::CYLINDER);
          
          if (j == 1)
            wrapper.addNewlyFittedAffordance(s);
          else
            {
              s._uid = 1; //expect to get set to 1
              wrapper.updateTrackedAffordance(s);
            }
		}

		if (j == 15 || j == 20 || j == 25)
		{
		  AffPtr s(new AffordanceState());
		  s->_map_id 	 = 7;
		  s->_uid = 21; //should get set to 1 by the server		  
		  s->_params[AffordanceState::LENGTH_NAME] = .11111;
		  s->_params[AffordanceState::WIDTH_NAME] = 0;
		  s->_params[AffordanceState::HEIGHT_NAME] = 0;
          s->setType(AffordanceState::BOX);

          if (j == 15)
            wrapper.addNewlyFittedAffordance(*s);
          else if (j == 20)
            {
              //need the actual uid now
              s->_uid = 2; //this will be the uid that it gets set to by affordance store.  
              AffordancePlusState plus;
              plus.aff = s;
              plus.points.push_back(Eigen::Vector3f(1.1,2.2,3.3));
              plus.triangles.push_back(Eigen::Vector3i(4,5,6));
              wrapper.updateTrackedAffordancePlus(plus);
            }
          else
            {
              s->_uid = 2; 
              wrapper.deleteAffordance(*s);
            }
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

    //thread will modify the server state and listen for changes via
    //the wrapper
	boost::thread testThread = boost::thread(runTest, theLcm);

    //lcm loop
    cout << "\nstarting lcm loop" << endl;
    while (0 == theLcm->handle());

    return 0;
}
