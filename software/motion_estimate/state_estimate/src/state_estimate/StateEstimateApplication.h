#ifndef __StateEstimateApplication_h
#define __StateEstimateApplication_h

#include <iostream>
#include <stdlib.h>
#include <string>

namespace StateEstimate
{

struct command_switches {
  bool MATLAB_MotionSimulator;
};

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

  std::string mMotionSimulatorSuffix;

};

} // end namespace

#endif
