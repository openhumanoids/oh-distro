#ifndef __StateEstimateApplication_h
#define __StateEstimateApplication_h

#include <iostream>
#include <stdlib.h>
#include <string>

#include "SharedTypes.h"
#include <basiclegodo/LegOdoWrapper.hpp>

namespace StateEstimate
{

class StateEstimateApplication
{
public:

  StateEstimateApplication(const command_switches &switches);

  virtual ~StateEstimateApplication();

  virtual void handleCommandLineArguments(int argc, char* argv[]);

  // Run state estimate.
  // Returns an error code, 0 means no error.
  virtual int exec();
private:
  const command_switches* _switches;
  CommandLineConfig cl_cfg; // Maurice's leg odometry

  std::string mMotionSimulatorSuffix;
  std::string ERSMsgSuffix;

};

} // end namespace

#endif
