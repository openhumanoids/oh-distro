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
   //create affordance cylinder objects
   AffPtr c(new AffordanceState());
   c->setToCylinder(4, 0.18,
                    0,0, //uid/map id.  
                    KDL::Frame(KDL::Vector(0,0,1)),
                    Eigen::Vector3f(0,1,0));
   cout << "\nOriginal Cylinder:\n" << *c << endl;

   //convert to LCM message
   drc::affordance_t msg;
   c->toMsg(&msg);
   
   //get the raw bytes of the message
   void* buffer = malloc(msg.getEncodedSize());
   msg.encode(buffer, 0, msg.getEncodedSize());

   //convert to lcm::LogEvent
   lcm::LogEvent logEvent;
   logEvent.eventnum = 0;
   logEvent.timestamp = 0;
   logEvent.channel = "test_channel";
   logEvent.datalen = msg.getEncodedSize();
   logEvent.data = buffer;
   
   //write to disk
   //'w' for write
   lcm::LogFile *lFileWriter = new lcm::LogFile("lcm_io_test_save.bin", "w"); 
   lFileWriter->writeEvent(&logEvent);
   delete lFileWriter; 

   //read from disk
   //'r' for read
   lcm::LogFile *lFileReader = new lcm::LogFile("lcm_io_test_save.bin", "r"); 
   const lcm::LogEvent* eventFromFile = lFileReader->readNextEvent();

   //construct message from raw bytes
   drc::affordance_t msgFromFile;
   msgFromFile.decode(eventFromFile->data, 0, eventFromFile->datalen);

   //NOW we can delete the lFileReader.  we couldn't delete it before 
   //b/c we had a pointer to the log event it read and seems to store internally
   delete lFileReader;

   //now convert back to an AffordanceStateObject
   AffPtr cylinderFromFile(new AffordanceState());   
   cylinderFromFile->fromMsg(&msgFromFile);

   cout << "\n\n=====Read from file:\n" << *cylinderFromFile << endl; 
   

   return 0;
}
