#include "StateEstimateApplication.h"

int main(int argc, char* argv[])
{
  StateEstimate::StateEstimateApplication app;
  app.handleCommandLineArguments(argc, argv);
  return app.exec();
}
