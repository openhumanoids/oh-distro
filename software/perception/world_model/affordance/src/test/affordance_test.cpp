/*
 * server_test.cpp
 *
 *  Created on: Jan 16, 2013
 *      Author: mfleder
 */

#include <stdio.h>
#include <boost/shared_ptr.hpp>
#include "affordance/AffordanceState.h"
//#include "affordance/OpenGL_Affordance.h"
#include <iostream>

using namespace boost;
using namespace std;
using namespace affordance;


int main(int argc, char ** argv)
{
   //affordance box
   AffordanceState box;
   box.setToBox(1,2,3,//length, width, height
                0,0);//uid/mapid : user shouldn't provide these : affordance server will update
   //could have passed in the kdl frame and color
   //or could later call setFrame and setColor
   cout << box << endl;
   
   //affordance cylinder as a boost pointer
   AffPtr c(new AffordanceState());
   c->setToCylinder(4, 0.18,
                    0,0, //uid/map id.  
                    KDL::Frame(KDL::Vector(0,0,1)),
                    Eigen::Vector3f(0,1,0));
   cout << "\n\n" << *c << endl;


    return 0;
}
