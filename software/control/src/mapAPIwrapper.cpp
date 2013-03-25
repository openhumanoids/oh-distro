/* Calls the perception teams' MAP interface API */

#include <mex.h>

#include <Eigen/Dense>
#include <lcm/lcm.h>
#include <boost/thread/thread.hpp>

#include <maps/ViewClient.hpp>
#include <maps/BotWrapper.hpp>
#include <maps/DepthImageView.hpp>

#include <lcmtypes/drc/data_request_t.hpp>

using namespace Eigen;
using namespace maps;

struct ViewWrapperData {
  lcm_t* lcm;
  ViewClient* view_client;
  boost::thread* lcm_thread;
};

void lcmThreadMain(lcm_t* lcm) {
  while(0 == lcm->handle()) {
    if (boost::this_thread::interruption_requested()) return;
  }
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  // usage:
  // ptr = mapAPIwrapper()  - initializes MAP interface
  // mapAPIwrapper(ptr)     - frees MAP interface
  // [closest_terrain_pos,normal] = mapAPIwrapper(ptr,pos)  
  //     - pos is a 3xn matrix
  //     - closest_terrain_pos is a 3xn matrix of the closest points on the terrain
  //     - normal is a 3xn matrix of the surface normals of the terrain at closest_terrain_pos

  struct ViewWrapperData* pdata = NULL;

  if (nrhs<1) { // then it's ptr = mapAPIWrapper()
    pdata = new struct ViewWrapperData;
    pdata->view_client = new ViewClient();

    // create lcm instance, and fork listener thread
    pdata->lcm = lcm_create(NULL); 
    if (!pdata->lcm) mexErrMsgIdAndTxt("DRC:mapAPIwrapper:LCMFailed","Failed to create LCM instance");
    pdata->lcm_thread = new boost::thread(lcmThreadMain,pdata->lcm);

    BotWrapper::Ptr botWrapper(new BotWrapper(pdata->lcm, NULL, NULL));
    pdata->view_client->setBotWrapper(botWrapper);

    // start listening for view data
    pdata->view_client->start();

    // return a pointer to the model
    mxClassID cid;
    if (sizeof(pdata)==4) cid = mxUINT32_CLASS;
    else if (sizeof(pdata)==8) cid = mxUINT64_CLASS;
    else mexErrMsgIdAndTxt("DRC:mapAPIWrapper:PointerSize","Are you on a 32-bit machine or 64-bit machine??");
    plhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
    memcpy(mxGetData(plhs[0]),&pdata,sizeof(pdata));

    return;
  } 

  // load the data ptr
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("DRC:mapAPIWrapper:BadInputs","the first argument should be the ptr");
  memcpy(&pdata,mxGetData(prhs[0]),sizeof(pdata));
  
  if (nrhs<2) { // then free the memory and exit
    delete pdata->view_client;
    pdata->lcm_thread->interrupt();
    delete pdata->lcm_thread;
    lcm_destroy(pdata->lcm);
    delete pdata;
    return;
  } 

  ViewClient::ViewPtr vptr = pdata->view_client->getView(drc::data_request_t::HEIGHT_MAP_SCENE);
  if (!vptr) { // check if null
    mexErrMsgIdAndTxt("DRC:mapAPIwrapper:NullViewPtr","could not get the heightmap view\n");
  }

  int N = mxGetN(prhs[1]);
  if (mxGetM(prhs[1])!=3) mexErrMsgIdAndTxt("DRC:mapAPIwrapper:BadInputs","pos must be 3xn\n"); 

  plhs[0] = mxCreateDoubleMatrix(3,N,mxREAL);
  mxArray* pnormal = mxCreateDoubleMatrix(3,N,mxREAL);

  Map<MatrixXd> pos(mxGetPr(prhs[1]),3,N); 
  Map<MatrixXd> closest_terrain_pos(mxGetPr(plhs[0]),3,N);
  Map<MatrixXd> normal(mxGetPr(pnormal),3,N);

  for (int i=0; i<N; i++) {
    if (!vptr->getClosest(pos.col(i).cast<float>(),closest_terrain_pos.col(i).cast<float>(),normal.col(i)).cast<float>()) {
      // returns false if off the grid		   
      closest_terrain_pos(0,i) = NAN;
      closest_terrain_pos(1,i) = NAN;
      closest_terrain_pos(2,i) = NAN;
    }
  }

  if (nlhs>1) {
    plhs[1] = pnormal;
  } else {
    mxDestroyArray(pnormal);
  }
}

