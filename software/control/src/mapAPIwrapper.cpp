/* Calls the perception teams' MAP interface API */

#include <mex.h>
#include <ctime>
#include <memory>
#include <thread>

#include <Eigen/Dense>
#include <lcm/lcm.h>
//#include <lcm/lcm-cpp.hpp>

#include <maps/ViewClient.hpp>
#include <maps/BotWrapper.hpp>
#include <maps/DepthImageView.hpp>

#include <lcmtypes/drc/data_request_t.hpp>

using namespace Eigen;
using namespace maps;

struct ViewWrapperData {
  //  boost::shared_ptr<lcm::LCM> lcm;
  lcm_t* lcm;
  ViewClient* view_client;
  std::thread* lcm_thread;
  bool b_interrupt_lcm;
};

void lcmThreadMain(struct ViewWrapperData* pdata) {
  //  while(0 == pdata->lcm->handle()) {
  while(!pdata->b_interrupt_lcm) {
    int fn = lcm_get_fileno(pdata->lcm);
    fd_set input_set;
    FD_ZERO(&input_set);
    FD_SET(fn, &input_set);
    struct timeval timeout = { 0, 200*1000 };
    int status = select(fn+1, &input_set, NULL, NULL, &timeout);
    if (status == 0) {
      //
    }
    else if (status < 0) {
      mexPrintf("error in lcm pipe"); mexEvalString("drawnow");
      break;
    }
    else if (FD_ISSET(fn, &input_set)) {
      if (0 != lcm_handle(pdata->lcm)) {
        mexPrintf("error in lcm handle\n"); mexEvalString("drawnow");
        break;
      }
    }
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
  // pts = mapAPIwrapper(ptr,[]);
  //     - returns all of the points as a point cloud
  
  struct ViewWrapperData* pdata = NULL;

  if (nrhs<1) { // then it's ptr = mapAPIWrapper()
    pdata = new struct ViewWrapperData;
    mexPrintf("creating view client\n"); mexEvalString("drawnow");
    pdata->view_client = new ViewClient();

    // create lcm instance, and fork listener thread
    mexPrintf("creating lcm\n"); mexEvalString("drawnow");
    pdata->lcm = lcm_create(NULL);
    if (!pdata->lcm)
    //    pdata->lcm.reset(new lcm::LCM());  //lcm_create(NULL); 
    //    if (!pdata->lcm->good()) 
      mexErrMsgIdAndTxt("DRC:mapAPIwrapper:LCMFailed","Failed to create LCM instance");

    mexPrintf("creating botwrapper\n"); mexEvalString("drawnow");
    BotWrapper::Ptr botWrapper(new BotWrapper(pdata->lcm, NULL, NULL));

    mexPrintf("setting botwrapper in view client\n"); mexEvalString("drawnow");
    pdata->view_client->setBotWrapper(botWrapper);

    // start listening for view data
    mexPrintf("starting view client\n"); mexEvalString("drawnow");
    pdata->view_client->start();

    mexPrintf("spawning LCM thread\n"); mexEvalString("drawnow");
    pdata->b_interrupt_lcm = false;
    pdata->lcm_thread = new std::thread(lcmThreadMain,pdata);

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
    mexPrintf("stopping view client\n"); mexEvalString("drawnow");
    pdata->view_client->stop();
    mexPrintf("deleting view client\n"); mexEvalString("drawnow");
    delete pdata->view_client;
    mexPrintf("interrupting thread\n"); mexEvalString("drawnow");
    //    pdata->lcm_thread->interrupt();
    pdata->b_interrupt_lcm = true;
    mexPrintf("joining thread\n"); mexEvalString("drawnow");
    pdata->lcm_thread->join();
    mexPrintf("destroying thread\n"); mexEvalString("drawnow");
    delete pdata->lcm_thread;
    mexPrintf("done.\n"); mexEvalString("drawnow");
    lcm_destroy(pdata->lcm);  
    // delete lcm handled by shared_ptr and lcm destructor
    delete pdata;
    return;
  } 

  ViewClient::ViewPtr temp_ptr = pdata->view_client->getView(drc::data_request_t::HEIGHT_MAP_SCENE);
  DepthImageView::Ptr vptr = std::dynamic_pointer_cast<DepthImageView>(temp_ptr);

  if (vptr != NULL) {
    // 0 defaults to old behavior;
    // >0 sets radius (in pixels) for normal computation
    vptr->setNormalRadius(2);

    // options are LeastSquares, RobustKernel, and SampleConsensus
    // this is ignored if radius==0
    vptr->setNormalMethod(DepthImageView::NormalMethodLeastSquares);
  }

  // NOTE: tries to handle this more gracefully below now
  //  if (!vptr) { // check if null
  //    mexErrMsgIdAndTxt("DRC:mapAPIwrapper:NullViewPtr","Have not received height map via LCM yet.  Perhaps you still need to request it in the viewer?\n");
  //  }

  int N = mxGetN(prhs[1]);
  if (N<1) {  // then it's mapAPIwrapper(ptr,[]); return the point cloud
    if (!vptr) {
      plhs[0] = mxCreateDoubleMatrix(3,0,mxREAL);
      return;
    }
    maps::PointCloud::Ptr cloud = vptr->getAsPointCloud();
    plhs[0] = mxCreateDoubleMatrix(3,cloud->size(),mxREAL);
    double* ptr = mxGetPr(plhs[0]);
    for (int i = 0; i < cloud->size(); ++i) {
      maps::PointCloud::PointType pt = cloud->points[i];
      ptr[i*3+0] = pt.x;
      ptr[i*3+1] = pt.y;
      ptr[i*3+2] = pt.z;
    }
  } else {
    if (mxGetM(prhs[1])!=3) mexErrMsgIdAndTxt("DRC:mapAPIwrapper:BadInputs","pos must be 3xn\n");
    
    plhs[0] = mxCreateDoubleMatrix(3,N,mxREAL);
    mxArray* pmxNormal = mxCreateDoubleMatrix(3,N,mxREAL);
    
    Vector3f pos, closest_terrain_pos, normal;
    double *ppos = mxGetPr(prhs[1]), *pclosest_terrain_pos = mxGetPr(plhs[0]), *pnormal = mxGetPr(pmxNormal);
    
    int j;
    for (int i=0; i<N; i++) {
      pos << (float) ppos[3*i], (float) ppos[3*i+1], (float) ppos[3*i+2];
      if (vptr && vptr->getClosest(pos,closest_terrain_pos,normal)) {
        for (j=0; j<3; j++) {
          pclosest_terrain_pos[3*i+j] = (double) closest_terrain_pos(j);
          pnormal[3*i+j] = (double) normal(j);
        }
      } else { // returns false if off the grid
        for (j=0; j<3; j++) {
          pclosest_terrain_pos[3*i+j] = NAN;
          pnormal[3*i+j] = NAN;
        }
      }
    }
    
    if (nlhs>1) {
      plhs[1] = pmxNormal;
    } else {
      mxDestroyArray(pmxNormal);
    }
  }
}

