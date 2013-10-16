
#include <ConciseArgs>

#include "StateEstimateApplication.h"

int main(int argc, char* argv[])
{
  // Concise arguments are being handled here and passed down to app object to maintain ConciseArgs command line help menu interface
  StateEstimate::command_switches switches;
  switches.MATLAB_MotionSimulator = false;

  ConciseArgs opt(argc, (char**)argv);
  opt.add(switches.MATLAB_MotionSimulator, "M", "MATLAB_MotionSimulator","Use the MATLAB MotionSimulation LCM interface.");
  opt.parse();
  std::cout << "main -- USING MATLAB MOTION SIMULATOR MODE: " << switches.MATLAB_MotionSimulator << std::endl;

  StateEstimate::StateEstimateApplication app(switches);
  app.handleCommandLineArguments(argc, argv);
  return app.exec();
}
