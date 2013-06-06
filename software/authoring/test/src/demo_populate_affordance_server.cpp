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
    //AffordanceState sphere(uniqueObjId++, mapId,
                           //KDL::Frame(KDL::Vector(-0.5, -0.5, 0.0)),
                           //Eigen::Vector3f(1.0, 0.0, 1.0));   //color
    //sphere._params[AffordanceState::RADIUS_NAME] = 0.125;
    //sphere.setType(AffordanceState::SPHERE);
    //wrapper.addNewlyFittedAffordance(sphere);

    //box
    float box_height = 0.01;
    AffordanceState box(uniqueObjId++, mapId,
                        KDL::Frame(KDL::Vector(0.0, 0.0, -box_height / 2.0)),
                        Eigen::Vector3f(0.75, 0.75, 0.0)); //color
/*
    box._params[AffordanceState::LENGTH_NAME] = 100;
    box._params[AffordanceState::WIDTH_NAME]  = 100;
    box._params[AffordanceState::HEIGHT_NAME] = box_height;
    box.setType(AffordanceState::BOX);
*/
    box.setToBox( 100.0, 100.0, box_height, 0, 0, KDL::Frame(KDL::Vector(0.0, 0.0, -box_height / 2.0)),
                        Eigen::Vector3f(0.75, 0.75, 0.0));
    wrapper.addNewlyFittedAffordance(box);

    //cylinder
    //AffordanceState cylinder(uniqueObjId++, mapId,
                             //KDL::Frame(KDL::Vector(0.0, 1.0, 0.0)),
                             //Eigen::Vector3f(0.0, 1.0, 1.0)); //color
    //cylinder._params[AffordanceState::RADIUS_NAME] = 0.25;
    //cylinder._params[AffordanceState::LENGTH_NAME] = 0.25;
    //cylinder.setType(AffordanceState::CYLINDER);
    //wrapper.addNewlyFittedAffordance(cylinder);

    //==car
    //drc::affordance_plus_t msg;
    //msg.aff.utime = 0;
    //msg.aff.map_id = 0;
    //msg.aff.uid = 0;
    //msg.aff.otdf_type = "car";
    //msg.aff.modelfile = "car.pcd";
    //msg.aff.aff_store_control = msg.aff.NEW;
    //for(int i = 0; i < 3; i++)
      //{
        //msg.aff.origin_xyz[i] = 0;
        //msg.aff.origin_rpy[i] = 0;
      //}
    
    //double bounding_xyz[]={0,0,0};
    //double bounding_rpy[]={0,0,0};
    //double bounding_lwh[]={0,0,0};
    //bounding_xyz[2] = 1.0;   // center of bounding box is 1m above car origin                                                                                              
    //bounding_lwh[0] = 3.0;
    //bounding_lwh[1] = 1.7;
    //bounding_lwh[2] = 2.2;

    //msg.aff.bounding_xyz[0] = bounding_xyz[0]; msg.aff.bounding_xyz[1] = bounding_xyz[1];msg.aff.bounding_xyz[2] = bounding_xyz[2];
    //msg.aff.bounding_rpy[0] = bounding_rpy[0]; msg.aff.bounding_rpy[1] = bounding_rpy[1];msg.aff.bounding_rpy[2] = bounding_rpy[2];
    //msg.aff.bounding_lwh[0] = bounding_lwh[0]; msg.aff.bounding_lwh[1] = bounding_lwh[1];msg.aff.bounding_lwh[2] = bounding_lwh[2];

    ////=car states

    //msg.aff.nstates = 11;
    //msg.aff.state_names.push_back("steering_joint");
    //msg.aff.state_names.push_back("front_right_steering_joint");
    //msg.aff.state_names.push_back("front_left_wheel_joint");
    //msg.aff.state_names.push_back("rear_left_wheel_joint");
    //msg.aff.state_names.push_back("gas_joint");
    //msg.aff.state_names.push_back("rear_right_wheel_joint");
    //msg.aff.state_names.push_back("FNR_switch_joint");
    //msg.aff.state_names.push_back("hand_brake_joint");
    //msg.aff.state_names.push_back("brake_joint");
    //msg.aff.state_names.push_back("front_right_wheel_joint");
    //msg.aff.state_names.push_back("front_left_steering_joint");

    //for(uint i = 0; i < msg.aff.nstates; i++)
      //msg.aff.states.push_back(0.0);

    ////=car params
    //msg.aff.nparams = 8;
    //msg.aff.param_names.push_back("r_toe_x");
    //msg.aff.param_names.push_back("foot_length");
    //msg.aff.param_names.push_back("r_toe_y");
    //msg.aff.param_names.push_back("toe_target_w");
    //msg.aff.param_names.push_back("l_toe_x");
    //msg.aff.param_names.push_back("toe_target_h");
    //msg.aff.param_names.push_back("l_toe_y");
    //msg.aff.param_names.push_back("foot_width");

    //msg.aff.params.push_back(0.5);
    //msg.aff.params.push_back(0.26);
    //msg.aff.params.push_back(-0.7);
    //msg.aff.params.push_back(0.01);
    //msg.aff.params.push_back(0.32);
    //msg.aff.params.push_back(0.01);
    //msg.aff.params.push_back(-0.7);
    //msg.aff.params.push_back(0.1249);

    //msg.npoints = 0;
    //msg.ntriangles = 0;
    //AffordancePlusState carState(&msg);
    //wrapper.addNewlyFittedAffordance(carState);

    ////--------------wait a few second so we get an update from the server
    boost::this_thread::sleep(boost::posix_time::seconds(3));

    ////===now loop and printout periodically
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
