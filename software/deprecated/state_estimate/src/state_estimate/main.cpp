
#include <ConciseArgs>

#include "StateEstimateApplication.h"

#include "SharedTypes.h"

int main(int argc, char* argv[])
{
  // Concise arguments are being handled here and passed down to app object to maintain ConciseArgs command line help menu interface
  StateEstimate::command_switches switches;
  switches.MATLAB_MotionSimulator = false;
  switches.ExperimentalMsgs = false;

  ConciseArgs opt(argc, (char**)argv);
  opt.add(switches.MATLAB_MotionSimulator, "M", "MATLAB_MotionSimulator","Use the MATLAB MotionSimulation LCM interface.");
  opt.add(switches.ExperimentalMsgs, "x", "ExperimentalMessages","Apply _EXP suffix to [ERS] message.");
  opt.parse();
  std::cout << "main -- Using MATLAB MOTION SIMULATOR MODE: " << switches.MATLAB_MotionSimulator << std::endl;
  std::cout << "main -- Using _EXP message suffix: " << switches.ExperimentalMsgs << std::endl;

  StateEstimate::StateEstimateApplication app(switches);
  app.handleCommandLineArguments(argc, argv);
  return app.exec();
}
