#ifndef __StateEstimateApplication_h
#define __StateEstimateApplication_h


namespace StateEstimate
{

class StateEstimateApplication
{
public:

  StateEstimateApplication();

  virtual ~StateEstimateApplication();

  virtual void handleCommandLineArguments(int argc, char* argv[]);

  // Run state estimate.
  // Returns an error code, 0 means no error.
  virtual int exec();

};

} // end namespace

#endif
