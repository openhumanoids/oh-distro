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
#include <path_util/path_util.h>
#include <iostream>

using namespace boost;
using namespace std;
using namespace affordance;

/**populates the server w/ a bunch of affordances*/
void runPopulate(const shared_ptr<lcm::LCM> lcm)
{
    AffordanceUpWrapper wrapper(lcm);

    const int mapId = 0;
    int uniqueObjId = 0;

    //sphere
    AffordanceState sphere(uniqueObjId++, mapId,
                           KDL::Frame(KDL::Vector(-0.5, -0.5, 0.0)),
                           Eigen::Vector3f(1.0, 0.0, 1.0));   //color
    sphere._params[AffordanceState::RADIUS_NAME] = 0.125;
    sphere.setType(AffordanceState::SPHERE);
    wrapper.addNewlyFittedAffordance(sphere);

    //box
    float box_height = 0.01;
    AffordanceState box(uniqueObjId++, mapId,
                        KDL::Frame(KDL::Vector(0.0, 0.0, -box_height / 2.0)),
                        Eigen::Vector3f(0.75, 0.75, 0.0)); //color
    box._params[AffordanceState::LENGTH_NAME] = 100;
    box._params[AffordanceState::WIDTH_NAME]  = 100;
    box._params[AffordanceState::HEIGHT_NAME] = box_height;
    box.setType(AffordanceState::BOX);
    wrapper.addNewlyFittedAffordance(box);

    //cylinder
    AffordanceState cylinder(uniqueObjId++, mapId,
                             KDL::Frame(KDL::Vector(0.0, 1.0, 0.0)),
                             Eigen::Vector3f(0.0, 1.0, 1.0)); //color
    cylinder._params[AffordanceState::RADIUS_NAME] = 0.25;
    cylinder._params[AffordanceState::LENGTH_NAME] = 0.25;
    cylinder.setType(AffordanceState::CYLINDER);
    wrapper.addNewlyFittedAffordance(cylinder);

    //==car
    string carUrdfFName = getModelsPath() + string("/mit_gazebo_objects/mit_vehicle/model_drake.urdf");
    cout << "using card urdf = " << carUrdfFName << endl;

    vector<AffPtr> carAffs;
    AffordanceState::getBoxesCylindersSpheres(carUrdfFName, carAffs);
    for(uint i = 0; i < carAffs.size(); i++)
      wrapper.addNewlyFittedAffordance(*carAffs[i]);

    cout << "\n added " << carAffs.size() << " car affordances" << endl;

    //wait a few second so we get an update from the server
    boost::this_thread::sleep(boost::posix_time::seconds(3));

    //===now loop and printout periodically
    boost::posix_time::seconds sleepTime(60); //update (printout) every 60 seconds

    while (true)
    {
        //cout << "\n\n===" << j << endl;
        cout << wrapper << endl;
        boost::this_thread::sleep(sleepTime); //======sleep

        //======modify server
        // user code
        //===finished  modifications 
    }
}

int main(int argc, char **argv)
{
    shared_ptr<lcm::LCM> theLcm(new lcm::LCM());

    if (!theLcm->good())
    {
        cerr << "Cannot create lcm object" << endl;
        return -1;
    }

    //create the server
    AffordanceServer s(theLcm);

    //thread will add to the affordance server
    //and possibly make changes over time
    boost::thread populateThread = boost::thread(runPopulate, theLcm);

    //lcm loop
    cout << "\nstarting lcm loop" << endl;

    while (0 == theLcm->handle())
    {
        ;
    }

    return 0;
}
