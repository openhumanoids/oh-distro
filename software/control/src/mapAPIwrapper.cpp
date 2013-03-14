/* Calls the perception teams' MAP interface API */

#include <mex.h>

#include <maps/ViewClient.hpp>
#include <maps/BotWrapper.hpp>
#include <maps/DepthImageView.hpp>

#include <lcmtypes/drc/data_request_t.hpp>

struct ViewWrapperData {
  ViewClient* view_client;
  boost::thread lcm_thread;
}


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  // todo : create lcm instance, and fork listener thread

  if init {
mThread = boost::thread(boost::ref(*this));
  ViewClient viewClient;

  BotWrapper::Ptr botWrapper(new BotWrapper(getLcm(), NULL, NULL));
  viewClient.setBotWrapper(botWrapper);

  // start listening for view data
  viewClient.start();
    }
   
  ViewPtr vptr = viewClient.getView(drc::data_request_t::HEIGHTMAP_SCENE);
  // check if null

  vptr->getClosest();
		   // returns false if off the grid		   
}
