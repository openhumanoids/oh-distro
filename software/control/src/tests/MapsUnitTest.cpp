#include <matrix.h>

#include <ctime>
#include <memory>
#include <unordered_map>
#include <sstream>
#include <set>
#include <vector>

#include <lcm/lcm-cpp.hpp>
#include <drc_utils/Clock.hpp>
#include <drc_utils/LcmWrapper.hpp>

#include <lcmtypes/drc/data_request_t.hpp>

#include <maps/DepthImageView.hpp>
#include <maps/DepthImage.hpp>

#include "mexmaps/MapLib.hpp"
#include "mexmaps/ViewClientWrapper.hpp"

#include <gurobi_c++.h>
#define mex_h
#include "RigidBodyTree.h"
#undef mex_h

namespace mexmaps {

struct WrapperCollection {
  typedef std::unordered_map<int,std::shared_ptr<ViewClientWrapper> > WrapperGroup;
  WrapperGroup mWrappers;
  int mNextId;
  std::shared_ptr<lcm::LCM> mLcm;
  std::shared_ptr<drc::LcmWrapper> mLcmWrapper;

  WrapperCollection() {
    mLcm.reset(new lcm::LCM());
    drc::Clock::instance()->setLcm(mLcm);
    drc::Clock::instance()->setVerbose(false);
    mLcmWrapper.reset(new drc::LcmWrapper(mLcm));
    mNextId = 1;
  }

  ~WrapperCollection() {
    cleanupAll();

    // TODO: don't know why, but segfaults on exit without this line
    //   crashes in clock destructor, internal reference count of
    //   shared pointer to lcm object is invalid memory (from valgrind)
    // valgrind shows no errors when this line is present
    drc::Clock::instance()->setLcm(std::shared_ptr<lcm::LCM>());
  }

  void cleanupAll() {
    mLcmWrapper->stopHandleThread();
    if (mWrappers.size() > 0) {
      fprintf(stderr,"destroying all map instances\n");
      fprintf(stderr,"WARNING: all handles now invalid\n");
    }
    mWrappers.clear();
  }

  std::shared_ptr<ViewClientWrapper> createWrapper() {
    std::shared_ptr<ViewClientWrapper> wrapper(new ViewClientWrapper(mNextId, mLcm));
    ++mNextId;
    mWrappers[wrapper->mId] = wrapper;
    mLcmWrapper->startHandleThread();
    return wrapper;
  }

  std::shared_ptr<ViewClientWrapper> getWrapper(const int iId) {
    WrapperGroup::const_iterator item = mWrappers.find(iId);
    if (item == mWrappers.end()) return std::shared_ptr<ViewClientWrapper>();
    return item->second;
  }

  void destroyWrapper(const int iId) {
    mWrappers.erase(iId);
    if (mWrappers.size() == 0) {
      mLcmWrapper->stopHandleThread();
    }
  }

  static WrapperCollection& instance() {
    static WrapperCollection theCollection;
    return theCollection;
  }

};

struct Utils {
  static std::string getString(const mxArray* iArray) {
    char* chars = mxArrayToString(iArray);
    std::string str = chars;
    mxFree(chars);
    return str;
  }

  static bool getBool(const std::string& iString) {
    std::string str = iString;
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    if ((str == "yes") || (str == "1") || (str == "on") ||
        (str == "true")) {
      return true;
    }
    else if ((str == "no") || (str == "0") || (str == "off") ||
             (str == "false")) {
      return false;
    }
    else {
      fprintf(stderr,"MapWrapper: value must be true or false\n");
      exit(-1);
    }
  }
};

}

using namespace mexmaps;
using namespace std;

void mexFunctionHeightMapWrapper(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

  // first determine which command to apply
  std::string command;
  if (nrhs == 0) command = "create";
  else {
    if (!mxIsChar(prhs[0])) {
      fprintf(stderr,"MapWrapper: first argument must be command string\n");
      exit(-1);
    }
    command = Utils::getString(prhs[0]);
  }
  std::transform(command.begin(), command.end(), command.begin(), ::tolower);

  // create instance
  if (command == "create") {
    auto wrapper = WrapperCollection::instance().createWrapper();
    if (nrhs > 1) {
      if ((nrhs % 2) != 1) {
        fprintf(stderr,"MapWrapper: wrong number of parameter args\n");
        exit(-1);
      }
      for (int i = 1; i < nrhs; i += 2) {
        if (!mxIsChar(prhs[i])) {
          fprintf(stderr,"MapWrapper: key must be string\n");
          exit(-1);
        }
        if (!mxIsChar(prhs[i+1])) {
          fprintf(stderr,"MapWrapper: val must be string\n");
          exit(-1);
        }
        std::string key = Utils::getString(prhs[i]);
        std::string value = Utils::getString(prhs[i+1]);

        std::transform(key.begin(), key.end(), key.begin(), ::tolower);
        if (key == "privatechannel") {
          bool isPrivate = Utils::getBool(value);
          if (isPrivate) {
            wrapper->setHeightMapChannel(MapHandle::kHeightMapChannel,
                                         MapHandle::kHeightMapViewId);
          }
          else {
            wrapper->setHeightMapChannel
              ("MAP_DEPTH", drc::data_request_t::HEIGHT_MAP_SCENE);
          }
        }
        else {
          fprintf(stderr,"MapWrapper: invalid property\n");
          exit(-1);
        }
      }  
    }

    wrapper->start();

    // return index of wrapper object
    plhs[0] = mxCreateDoubleMatrix(1,1,mxREAL);
    double id = wrapper->mId;
    memcpy(mxGetData(plhs[0]),&id,sizeof(double));
    return;
  }

  // get wrapper object from static collection
  if ((nrhs < 2) || (mxGetNumberOfElements(prhs[1]) != 1)) {
    fprintf(stderr,"MapWrapper: second argument must be handle id\n");
    exit(-1);
  }
  double val;
  memcpy(&val, mxGetData(prhs[1]), sizeof(double));
  int wrapperId = (int)(val+0.5);
  auto wrapper = WrapperCollection::instance().getWrapper(wrapperId);
  if (wrapper == NULL) {
    fprintf(stderr,"MapWrapper: handle is invalid; did you clear mex?\n");
    exit(-1);
  }

  // destroy instance
  if (command == "destroy") {
    if (nrhs != 2) {
      fprintf(stderr,"MapWrapper: too many arguments to destroy\n");
      exit(-1);
    }
    WrapperCollection::instance().destroyWrapper(wrapper->mId);
  }

  // get point cloud
  else if (command == "pointcloud") {
    if (nrhs != 2) {
      fprintf(stderr,"MapWrapper: too many arguments to pointcloud\n");
      exit(-1);
    }
    auto view = wrapper->getView();
    if (view == NULL) {
      plhs[0] = mxCreateDoubleMatrix(3,0,mxREAL);
      return;
    }
    auto cloud = view->getAsPointCloud();
    plhs[0] = mxCreateDoubleMatrix(3,cloud->size(),mxREAL);
    double* ptr = mxGetPr(plhs[0]);
    for (int i = 0; i < cloud->size(); ++i) {
      maps::PointCloud::PointType& pt = cloud->points[i];
      ptr[i*3+0] = pt.x;
      ptr[i*3+1] = pt.y;
      ptr[i*3+2] = pt.z;
    }
  }

  // get closest points and normals
  else if (command == "closest") {
    if (nrhs != 3) {
      fprintf(stderr,"MapWrapper: too many arguments to closest\n");
      exit(-1);
    }
    if (mxGetM(prhs[2]) != 3) {
      fprintf(stderr,"MapWrapper: points must be 3xn\n");
      exit(-1);
    }
    if (nlhs > 2) {
      fprintf(stderr,"MapWrapper: too many output arguments\n");
      exit(-1);
    }

    int n = mxGetN(prhs[2]);
    plhs[0] = mxCreateDoubleMatrix(3,n,mxREAL);
    mxArray* pmxNormal = mxCreateDoubleMatrix(3,n,mxREAL);
    
    double* posPtr = mxGetPr(prhs[2]);
    double* closestPtr = mxGetPr(plhs[0]);
    double* normalPtr = mxGetPr(pmxNormal);
    Eigen::Vector3f pos, closest, normal;

    auto view = wrapper->getView();
    for (int i = 0; i < n; ++i) {
      pos << posPtr[3*i], posPtr[3*i+1], posPtr[3*i+2];
      if ((view != NULL) && view->getClosest(pos,closest,normal)) {
        for (int j = 0; j < 3; ++j) {
          closestPtr[3*i+j] = closest[j];
          normalPtr[3*i+j] = normal[j];
        }
      }
      else { // invalid view or off grid
        for (int j = 0; j < 3; ++j) {
          closestPtr[3*i+j] = NAN;
          normalPtr[3*i+j] = NAN;
        }
      }
    }

    if (nlhs == 2) plhs[1] = pmxNormal;
    else mxDestroyArray(pmxNormal);
    sleep(1);
  }

  else if (command == "getrawdepth") {
    if (nrhs != 2) {
      fprintf(stderr,"MapWrapper: too many arguments to getrawdepth\n");
      exit(-1);
    }
    auto view = std::static_pointer_cast<maps::DepthImageView>
      (wrapper->getView());
    if (view == NULL) {
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
    auto img = view->getDepthImage();
    int w(img->getWidth()), h(img->getHeight());
    plhs[0] = mxCreateDoubleMatrix(h,w,mxREAL);
    auto vals = img->getData(maps::DepthImage::TypeDepth);
    float* inPtr = vals.data();
    double* outPtr = mxGetPr(plhs[0]);
    for (int i = 0; i < h; ++i) {
      for (int j = 0; j < w; ++j, ++inPtr) {
        outPtr[j*h+i] = *inPtr;
      }
    }
  }

  else if (command == "setrawdepth") {
    if (nrhs != 3) {
      fprintf(stderr,"MapWrapper: too many arguments to setrawdepth\n");
      exit(-1);
    }
    auto view = std::static_pointer_cast<maps::DepthImageView>
      (wrapper->getView());
    if (view == NULL) {
      fprintf(stderr,"MapWrapper: no view object, so not setting depths\n");
      exit(-1);
    }
    auto img = view->getDepthImage();
    int w(img->getWidth()), h(img->getHeight());
    if ((mxGetM(prhs[2]) != h) || (mxGetN(prhs[2]) != w)) {
      fprintf(stderr,"MapWrapper: depth image sizes do not match\n");
      exit(-1);
    }

    std::vector<float> data(w*h);
    double* inPtr = mxGetPr(prhs[2]);
    for (int j = 0; j < w; ++j) {
      for (int i = 0; i < h; ++i, ++inPtr) {
        data[i*w+j] = (float)(*inPtr);
      }
    }
    img->setData(data, maps::DepthImage::TypeDepth);
  }

  else if (command == "transform") {
    if (nrhs != 2) {
      fprintf(stderr,"MapWrapper: too many arguments to setrawdepth\n");
      exit(-1);
    }
    auto view = wrapper->getView();
    plhs[0] = mxCreateDoubleMatrix(4,4,mxREAL);
    double* matx = mxGetPr(plhs[0]);
    if (view == NULL) {
      for (int i = 0; i < 16; ++i) matx[i] = 0;
      return;
    }
    Eigen::Projective3f transform = view->getTransform();
    for (int i = 0; i < 16; ++i) matx[i] = transform.data()[i];
  }

  else if (command == "wrapper") {
    if (nrhs != 2) {
      fprintf(stderr,"MapWrapper: too many arguments to wrapper\n");
      exit(-1);
    }
    auto ptr = wrapper->mHandle.get();
    mxClassID classId = (sizeof(ptr)==4) ? mxUINT32_CLASS : mxUINT64_CLASS;
    plhs[0] = mxCreateNumericMatrix(1,1,classId,mxREAL);
    memcpy(mxGetPr(plhs[0]), &ptr, sizeof(ptr));
  }

  else if (command == "property") {
    if (nrhs != 4) {
      fprintf(stderr,"MapWrapper: must supply key/value pair\n");
      exit(-1);
    }
    if (!mxIsChar(prhs[2])) {
      fprintf(stderr,"MapWrapper: property key must be string\n");
      exit(-1);
    }
    std::string key = Utils::getString(prhs[2]);
    std::transform(key.begin(), key.end(), key.begin(), ::tolower);

    if (!mxIsChar(prhs[3])) {
      fprintf(stderr,"MapWrapper: property value must be string\n");
      exit(-1);
    }
    std::string value = Utils::getString(prhs[3]);

    if (key == "normalradius") {
      std::istringstream(value) >> wrapper->mNormalRadius;
    }
    else if (key == "fill") {
      wrapper->mShouldFill = Utils::getBool(value);
    }
    else {
      fprintf(stderr,"MapWrapper: invalid property\n");
      exit(-1);
    }
  }

  // none of the above
  else {
    fprintf(stderr,"MapWrapper: invalid command\n");
    exit(-1);
  }
}

















// comment out the #define to disable error checking
#define CHECK_GUROBI_ERRORS

#ifndef CHECK_GUROBI_ERRORS

#define CGE( call, env ) { call ;}

#else

void PGE ( GRBenv *env )
{
  fprintf (stderr,"Gurobi error %s\n", GRBgeterrormsg( env ));
}

#define CGE( call, env ) {int gerror; gerror = call; if (gerror) PGE ( env );}

#endif

const int m_surface_tangents = 2;  // number of faces in the friction cone approx


struct QPControllerData {
  GRBenv *env;
  RigidBodyTree* r;
  double w; // objective function weight
  double slack_limit; // maximum absolute magnitude of acceleration slack variable values
  int Rnnz,*Rind1,*Rind2; double* Rval; // my sparse representation of R_con - the quadratic cost on input
  MatrixXd B, B_con, B_free;
  set<int> free_dof, con_dof, free_inputs, con_inputs; 
  VectorXd umin_con, umax_con;
  ArrayXd umin,umax;
  void* map_ptr;
};

mxArray* myGetProperty(const mxArray* pobj, const char* propname)
{
  mxArray* pm = mxGetProperty(pobj,0,propname);
  if (!pm) {
    fprintf(stderr,"can't find object property %s\n", propname);
    exit(-1);
  }
  return pm;
}


template <typename DerivedA, typename DerivedB>
void getRows(set<int> &rows, MatrixBase<DerivedA> const &M, MatrixBase<DerivedB> &Msub)
{
  if (rows.size()==M.rows()) {
    Msub = M; 
    return;
  }
  
  int i=0;
  for (set<int>::iterator iter=rows.begin(); iter!=rows.end(); iter++)
    Msub.row(i++) = M.row(*iter);
}

template <typename DerivedA, typename DerivedB>
void getCols(set<int> &cols, MatrixBase<DerivedA> const &M, MatrixBase<DerivedB> &Msub)
{
  if (cols.size()==M.cols()) {
    Msub = M;
    return;
  }
  int i=0;
  for (set<int>::iterator iter=cols.begin(); iter!=cols.end(); iter++)
    Msub.col(i++) = M.col(*iter);
}

void collisionDetect(void* map_ptr, Vector3d const & contact_pos, Vector3d &pos, Vector3d &normal)
{
  Vector3f floatPos, floatNormal;
  if (map_ptr) {
    auto state = static_cast<mexmaps::MapHandle*>(map_ptr);
    if (state != NULL) {
      auto view = state->getView();
      if (view != NULL) {
        if (view->getClosest(contact_pos.cast<float>(),floatPos,floatNormal)) {
          pos = floatPos.cast<double>();
          normal = floatNormal.cast<double>();
          return;
        }
      }
    }
  } else {
//    mexPrintf("Warning: using 0,0,1 as normal\n");
    pos << contact_pos.topRows(2), 0;
    normal << 0,0,1;
  }
  // just assume mu = 1 for now
}

void surfaceTangents(const Vector3d & normal, Matrix<double,3,m_surface_tangents> & d)
{
  Vector3d t1,t2;
  double theta;
  
  if (1 - normal(2) < 10e-8) { // handle the unit-normal case (since it's unit length, just check z)
    t1 << 1,0,0;
  } else { // now the general case
    t1 << normal(2), -normal(1), 0; // normal.cross([0;0;1])
    t1 /= sqrt(normal(1)*normal(1) + normal(2)*normal(2));
  }
      
  t2 = t1.cross(normal);
      
  for (int k=0; k<m_surface_tangents; k++) {
    theta = k*M_PI/m_surface_tangents;
    d.col(k)=cos(theta)*t1 + sin(theta)*t2;
  }
}

int contactConstraints(struct QPControllerData* pdata, set<int> body_idx, MatrixXd &n, MatrixXd &D, MatrixXd &Jp, MatrixXd &Jpdot) 
{
  int i, j, k=0, nc = pdata->r->getNumContacts(body_idx), nq = pdata->r->num_dof;

//  phi.resize(nc);
  n.resize(nc,nq);
  D.resize(nq,nc*2*m_surface_tangents);
  Jp.resize(3*nc,nq);
  Jpdot.resize(3*nc,nq);
  
  Vector3d contact_pos,pos,normal; Vector4d tmp;
  MatrixXd J(3,nq);
  Matrix<double,3,m_surface_tangents> d;
  
  for (set<int>::iterator iter = body_idx.begin(); iter!=body_idx.end(); iter++) {
    RigidBody* b = &(pdata->r->bodies[*iter]);
    nc = b->contact_pts.cols();
    if (nc>0) {
      for (i=0; i<nc; i++) {
        tmp = b->contact_pts.col(i);
        pdata->r->forwardKin(*iter,tmp,0,contact_pos);
        pdata->r->forwardJac(*iter,tmp,0,J);

        collisionDetect(pdata->map_ptr,contact_pos,pos,normal);
        
// phi is not even being used right now        
//        pos -= contact_pos;  // now -rel_pos in matlab version
//        phi(k) = pos.norm();
//        if (pos.dot(normal)>0) phi(k)=-phi(k);

        surfaceTangents(normal,d);

        n.row(k) = normal.transpose()*J;
        for (j=0; j<m_surface_tangents; j++) {
          D.col(2*k*m_surface_tangents+j) = J.transpose()*d.col(j);
          D.col((2*k+1)*m_surface_tangents+j) = -D.col(2*k*m_surface_tangents+j);
        }

        // store away kin sols into Jp and Jpdot
        // NOTE: I'm cheating and using a slightly different ordering of J and Jdot here
        Jp.block(3*k,0,3,nq) = J;
        pdata->r->forwardJacDot(*iter,tmp,J);
        Jpdot.block(3*k,0,3,nq) = J;
        
        k++;
      }
    }
  }
  
  return k;
}


void mexFunctionQPController(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  int error;
  if (nrhs<1) {
    fprintf(stderr,"usage: ptr = QPControllermex(0,control_obj,robot_obj); alpha=QPControllermex(ptr,...,...)\n");
    exit(-1);
  }
  if (nlhs<1) {
    fprintf(stderr,"take at least one output... please.\n");
    exit(-1);
  }
  
  struct QPControllerData* pdata;
  mxArray* pm;
  double* pr;
  int i,j;

  if (mxGetScalar(prhs[0])==0) { // then construct the data object and return
    pdata = new struct QPControllerData;
    
    // get control object properties
    const mxArray* pobj = prhs[1];
    
    pm= myGetProperty(pobj,"w");
    pdata->w = mxGetScalar(pm);
    
    pm = myGetProperty(pobj,"slack_limit");
    pdata->slack_limit = mxGetScalar(pm);

    pm = myGetProperty(pobj,"free_dof");
    pr = mxGetPr(pm);
    for (i=0; i<mxGetNumberOfElements(pm); i++)
      pdata->free_dof.insert((int)pr[i] - 1);
    
    pm = myGetProperty(pobj,"con_dof");
    pr = mxGetPr(pm);
    for (i=0; i<mxGetNumberOfElements(pm); i++)
      pdata->con_dof.insert((int)pr[i] - 1);
    int nq_con = pdata->con_dof.size();

    pm = myGetProperty(pobj,"free_inputs");
    pr = mxGetPr(pm);
    for (i=0; i<mxGetNumberOfElements(pm); i++)
      pdata->free_inputs.insert((int)pr[i] - 1);

    pm = myGetProperty(pobj,"con_inputs");
    pr = mxGetPr(pm);
    for (i=0; i<mxGetNumberOfElements(pm); i++)
      pdata->con_inputs.insert((int)pr[i] - 1);
    int nu_con = pdata->con_inputs.size();
    
    // get robot mex model ptr
    if (!mxIsNumeric(prhs[2]) || mxGetNumberOfElements(prhs[2])!=1) {
      fprintf(stderr,"the third argument should be the robot mex ptr\n");
      exit(-1);
    }
    memcpy(&(pdata->r),mxGetData(prhs[2]),sizeof(pdata->r));
    
    pdata->B.resize(mxGetM(prhs[3]),mxGetN(prhs[3]));
    memcpy(pdata->B.data(),mxGetPr(prhs[3]),sizeof(double)*mxGetM(prhs[3])*mxGetN(prhs[3]));

    int nq = pdata->r->num_dof, nu = pdata->B.cols();
    
    pdata->umin.resize(nu);
    pdata->umax.resize(nu);
    memcpy(pdata->umin.data(),mxGetPr(prhs[4]),sizeof(double)*nu);
    memcpy(pdata->umax.data(),mxGetPr(prhs[5]),sizeof(double)*nu);
    pdata->umin_con.resize(nu_con);
    pdata->umax_con.resize(nu_con);
    getRows(pdata->con_inputs,pdata->umin.matrix(),pdata->umin_con);
    getRows(pdata->con_inputs,pdata->umax.matrix(),pdata->umax_con);

    {
      pdata->B_con.resize(pdata->con_dof.size(),pdata->con_inputs.size());
      MatrixXd tmp(nq,pdata->con_inputs.size());
      getCols(pdata->con_inputs,pdata->B,tmp);
      getRows(pdata->con_dof,tmp,pdata->B_con);
    
      if (pdata->free_dof.size()>0 && pdata->free_inputs.size()>0) {
        pdata->B_free.resize(pdata->free_dof.size(),pdata->free_inputs.size());
        tmp.resize(nq,pdata->free_inputs.size());
        getCols(pdata->free_inputs,pdata->B,tmp);
        getRows(pdata->free_dof,tmp,pdata->B_free);
      }
    }

    {
      pm = myGetProperty(pobj,"R");
      Map<MatrixXd> R(mxGetPr(pm),mxGetM(pm),mxGetN(pm));
      MatrixXd R_con(nu_con,nu_con), tmp(nu,nu_con);
      getCols(pdata->con_inputs,R,tmp);
      getRows(pdata->con_inputs,tmp,R_con);
    
      int nnz=0;
      for (i=0; i<nu_con; i++)
        for (j=0; j<nu_con; j++)
          if (abs(R_con(i,j))>1e-10) nnz++;
      pdata->Rind1 = new int[nnz];
      pdata->Rind2 = new int[nnz];
      pdata->Rval = new double[nnz];
      pdata->Rnnz = nnz;
      nnz=0;
      for (i=0; i<nu_con; i++)
        for (j=0; j<nu_con; j++)
          if (abs(R_con(i,j))>1e-10) {
            pdata->Rind1[nnz] = i+nq_con;
            pdata->Rind2[nnz] = j+nq_con;
            pdata->Rval[nnz++] = R_con(i,j);
          }
    }    

     // get the map ptr back from matlab
    if (!mxIsNumeric(prhs[6]) || mxGetNumberOfElements(prhs[6])!=1) {
      fprintf(stderr,"the seventh argument should be the map ptr\n");
      exit(-1);
    }
     memcpy(&pdata->map_ptr,mxGetPr(prhs[6]),sizeof(pdata->map_ptr));
    
//    pdata->map_ptr = NULL;
     if (!pdata->map_ptr) {
       fprintf(stderr,"Map ptr is NULL.  Assuming flat terrain at z=0\n");
     }
    
    // create gurobi environment
    error = GRBloadenv(&(pdata->env),NULL);

    // set solver params (http://www.gurobi.com/documentation/5.5/reference-manual/node798#sec:Parameters)
    error = GRBsetintparam(pdata->env,"outputflag",0);
    error = GRBsetintparam(pdata->env,"method",2);
    error = GRBsetintparam(pdata->env,"presolve",0);
    error = GRBsetintparam(pdata->env,"bariterlimit",20);
    error = GRBsetintparam(pdata->env,"barhomogenous",0);
    error = GRBsetdblparam(pdata->env,"barconvtol",0.0005);
  
    mxClassID cid;
    if (sizeof(pdata)==4) cid = mxUINT32_CLASS;
    else if (sizeof(pdata)==8) cid = mxUINT64_CLASS;
    else {
      fprintf(stderr,"Are you on a 32-bit machine or 64-bit machine??\n");
      exit(-1);
    }
    
    plhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
    memcpy(mxGetData(plhs[0]),&pdata,sizeof(pdata));
    return;
  }
  
  // first get the ptr back from matlab
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1) {
    fprintf(stderr,"the first argument should be the ptr\n");
    exit(-1);
  }
  
  memcpy(&pdata,mxGetData(prhs[0]),sizeof(pdata));

  int nu = pdata->B.cols(), 
      nq = pdata->r->num_dof,
      nq_free = pdata->free_dof.size(),
      nq_con = pdata->con_dof.size(),
      nu_con = pdata->con_inputs.size();
  const int dim = 3, // 3D
      nd = 2*m_surface_tangents; // for friction cone approx, hard coded for now
  
  assert(nq_con+nq_free == nq);
  assert(nu_con+pdata->free_inputs.size() == nu);

  int narg=1;

  Map< VectorXd > q_ddot_des(mxGetPr(prhs[narg++]),nq);
  
  double *q = mxGetPr(prhs[narg++]);
  double *qd = &q[nq];
  
  set<int> active_supports;
  pr = mxGetPr(prhs[narg]);
  for (i=0; i<mxGetNumberOfElements(prhs[narg]); i++)
    active_supports.insert((int)pr[i] - 1);
  narg++;
  
  assert(mxGetM(prhs[narg])==4); assert(mxGetN(prhs[narg])==4);
  Map< Matrix4d > A_ls(mxGetPr(prhs[narg++]));

  assert(mxGetM(prhs[narg])==4); assert(mxGetN(prhs[narg])==2);
  Map< Matrix<double,4,2> > B_ls(mxGetPr(prhs[narg++]));

  assert(mxGetM(prhs[narg])==2); assert(mxGetN(prhs[narg])==2);
  Map< Matrix2d > Qy(mxGetPr(prhs[narg++]));

  assert(mxGetM(prhs[narg])==2); assert(mxGetN(prhs[narg])==2);
  Map< Matrix2d > R_ls(mxGetPr(prhs[narg++]));

  assert(mxGetM(prhs[narg])==2); assert(mxGetN(prhs[narg])==4);
  Map< Matrix<double,2,4> > C_ls(mxGetPr(prhs[narg++]));

  assert(mxGetM(prhs[narg])==2); assert(mxGetN(prhs[narg])==2);
  Map< Matrix2d > D_ls(mxGetPr(prhs[narg++]));

  assert(mxGetM(prhs[narg])==4); assert(mxGetN(prhs[narg])==4);
  Map< Matrix4d > S(mxGetPr(prhs[narg++]));

  assert(mxGetM(prhs[narg])==4); assert(mxGetN(prhs[narg])==1);
  Map< Vector4d > s1(mxGetPr(prhs[narg++]));
  
  assert(mxGetM(prhs[narg])==4); assert(mxGetN(prhs[narg])==1);
  Map< Vector4d > x0(mxGetPr(prhs[narg++]));

  assert(mxGetM(prhs[narg])==2); assert(mxGetN(prhs[narg])==1);
  Map< Vector2d > u0(mxGetPr(prhs[narg++]));

  assert(mxGetM(prhs[narg])==2); assert(mxGetN(prhs[narg])==1);
  Map< Vector2d > y0(mxGetPr(prhs[narg++]));

  double mu = mxGetScalar(prhs[narg++]);

  Matrix2d R_DQyD_ls = R_ls + D_ls.transpose()*Qy*D_ls;
  
  pdata->r->doKinematics(q,qd);
  
  MatrixXd H(nq,nq);
  VectorXd C(nq);
  
  pdata->r->HandC(q,qd,(MatrixXd*)NULL,H,C,(MatrixXd*)NULL,(MatrixXd*)NULL);

  MatrixXd H_con(nq_con,nq), H_free(nq_free,nq);
  VectorXd C_con(nq_con), C_free(nq_free);
  
  getRows(pdata->con_dof,H,H_con);
  getRows(pdata->con_dof,C,C_con);
  
  if (nq_free>0) {
    getRows(pdata->free_dof,H,H_free);
    getRows(pdata->free_dof,C,C_free);
  }
  
  Vector3d xcom;
  MatrixXd J(3,nq), Jdot(3,nq);
  // consider making all J's into row-major
  
  pdata->r->getCOM(xcom);
  pdata->r->getCOMJac(J);
  pdata->r->getCOMJacDot(Jdot);

  Map<VectorXd> qdvec(qd,nq);
  VectorXd qd_con(nq_con);
  getRows(pdata->con_dof,qdvec,qd_con);
  
//  VectorXd phi;
  MatrixXd Jz,Jp,Jpdot,D;
  int nc = contactConstraints(pdata,active_supports,Jz,D,Jp,Jpdot);
  int neps = nc*dim;

  Vector4d x_bar,xlimp;
  MatrixXd Jz_con(Jz.rows(),nq_con),Jp_con(Jp.rows(),nq_con),Jpdot_con(Jpdot.rows(),nq_con),D_con(nq_con,D.cols());
  if (nc>0) {
    xlimp << xcom.topRows(2),J.topRows(2)*qdvec;
    x_bar << xlimp.topRows(2)-x0.topRows(2),xlimp.bottomRows(2)-x0.bottomRows(2);
    getCols(pdata->con_dof,Jz,Jz_con);
    getRows(pdata->con_dof,D,D_con);
    getCols(pdata->con_dof,Jp,Jp_con);
    getCols(pdata->con_dof,Jpdot,Jpdot_con);
  }
  

  //---------------------------------------------------------------------
  // Free DOF cost function ----------------------------------------------
  
  VectorXd qdd_free(nq_free);
  if (nq_free > 0) {
    MatrixXd Hqp(nq_free,nq_free);
    RowVectorXd fqp(nq_free);

    if (nc > 0) {
      VectorXd q_ddot_des_free(nq_free);
      getRows(pdata->free_dof,q_ddot_des,q_ddot_des_free);
      
      MatrixXd J_free(2,nq_free),Jdot_free(2,nq_free);
      VectorXd qd_free(nq_free);
      getCols(pdata->free_dof,J.topRows(2),J_free);
      getCols(pdata->free_dof,Jdot.topRows(2),Jdot_free);
      getRows(pdata->free_dof,qdvec,qd_free);
      
      // approximate quadratic cost for free dofs with the appropriate matrix block
      Hqp = J_free.transpose()*R_DQyD_ls*J_free;
      Hqp += pdata->w*MatrixXd::Identity(nq_free,nq_free);
              
      fqp = (C_ls*xlimp).transpose()*Qy*D_ls*J_free;
      fqp += (Jdot_free*qd_free).transpose()*R_DQyD_ls*J_free;
      fqp += (S*x_bar + 0.5*s1).transpose()*B_ls*J_free;
      fqp -= u0.transpose()*R_DQyD_ls*J_free;
      fqp -= y0.transpose()*Qy*D_ls*J_free;
      fqp -= pdata->w*q_ddot_des_free.transpose();
      
      // solve for qdd_free unconstrained
      qdd_free = -Hqp.ldlt().solve(fqp.transpose());
    } else {
      // qdd_free = q_ddot_des_free;
      getRows(pdata->free_dof,q_ddot_des,qdd_free);
    }        
    
  }

  int nf = nc+nc*nd; // number of contact force variables
  int nparams = nq_con+nu_con+nf+neps;
  
  GRBmodel *model = NULL;

  // set obj,lb,up
  VectorXd lb(nparams), ub(nparams);
  lb.topRows(nq_con) = -1e3*VectorXd::Ones(nq_con);
  ub.topRows(nq_con) = 1e3*VectorXd::Ones(nq_con);
  lb.block(nq_con,0,nu_con,1) = pdata->umin_con;
  ub.block(nq_con,0,nu_con,1) = pdata->umax_con;
  lb.block(nq_con+nu_con,0,nf,1) = VectorXd::Zero(nf);
  ub.block(nq_con+nu_con,0,nf,1) = 500*VectorXd::Ones(nf);
  lb.bottomRows(neps) = -pdata->slack_limit*VectorXd::Ones(neps);
  ub.bottomRows(neps) = pdata->slack_limit*VectorXd::Ones(neps);
  
  CGE (GRBnewmodel(pdata->env,&model,"QPController",nparams,NULL,lb.data(),ub.data(),NULL,NULL), pdata->env);
  
  //----------------------------------------------------------------------
  // QP cost function ----------------------------------------------------
  //
  //  min: quad(Jdot*qd + J*qdd,R_ls)+quad(C*x+D*(Jdot*qd + J*qdd),Qy) + (2*x'*S + s1')*(A*x + B*(Jdot*qd + J*qdd)) + w*quad(qddot_ref - qdd) + quad(u,R) + quad(epsilon)

  {      
    VectorXd q_ddot_des_con(nq_con);
    getRows(pdata->con_dof,q_ddot_des,q_ddot_des_con);
    if (nc > 0) {
      MatrixXd Hqp_con(nq_con,nq_con);
      RowVectorXd fqp_con(nq_con);
      
      MatrixXd J_con(2,nq_con),Jdot_con(2,nq_con);
      getCols(pdata->con_dof,J.topRows(2),J_con);
      getCols(pdata->con_dof,Jdot.topRows(2),Jdot_con);
      
      Hqp_con = J_con.transpose()*R_DQyD_ls*J_con;
      Hqp_con += pdata->w*MatrixXd::Identity(nq_con,nq_con);
              
      fqp_con = (C_ls*xlimp).transpose()*Qy*D_ls*J_con;
      fqp_con += (Jdot_con*qd_con).transpose()*R_DQyD_ls*J_con;
      fqp_con += (S*x_bar + 0.5*s1).transpose()*B_ls*J_con;
      fqp_con -= u0.transpose()*R_DQyD_ls*J_con;
      fqp_con -= y0.transpose()*Qy*D_ls*J_con;
      fqp_con -= pdata->w*q_ddot_des_con.transpose();

      // Q(1:nq_con,1:nq_con) = Hqp_con
      for (i=0; i<nq_con; i++)
        for (j=0; j<nq_con; j++)
          if (abs(Hqp_con(i,j))>1e-10) 
            CGE (GRBaddqpterms(model,1,&i,&j,&(Hqp_con(i,j))), pdata->env);

      // obj(1:nq_con) = 2*fqp_con
      fqp_con *= 2;
      CGE (GRBsetdblattrarray(model,"Obj",0,nq_con,fqp_con.data()), pdata->env);
      
      // quadratic slack var cost, Q(nparams-neps:end,nparams-neps:end)=eye(neps)
      double cost = .001;
      for (i=nparams-neps; i<nparams; i++) {
        CGE (GRBaddqpterms(model,1,&i,&i,&cost), pdata->env);
      }
    } else {
      // Q(1:nq_con,1:nq_con) = eye(nq_con)
      double cost = 1;
      for (i=0; i<nq_con; i++) {  
        CGE (GRBaddqpterms(model,1,&i,&i,&cost), pdata->env);
      }
      // obj(1:nq_con) = -qddot_des_con
      q_ddot_des_con *= -1;
      CGE (GRBsetdblattrarray(model,"Obj",0,nq_con,q_ddot_des_con.data()), pdata->env);
    } 
    
    // quadratic input cost Q(nq_con+(1:nu_con),nq_con+(1:nu_con))=R
    CGE (GRBaddqpterms(model,pdata->Rnnz,pdata->Rind1,pdata->Rind2,pdata->Rval), pdata->env);
  }      
  
  { // constrained dynamics
    //  H*qdd - B*u - J*lambda - Dbar*beta = -C
    MatrixXd H_con_con(nq_con,nq_con), H_con_free(nq_con,nq_free);
    getCols(pdata->con_dof,H_con,H_con_con);
    
    MatrixXd Aeq;
    VectorXd beq(nq_con);
    if (nc>0) {
      Aeq.resize(nq_con,nparams-neps);
      Aeq.block(0,nq_con+nu_con,nq_con,nc) = -Jz_con.transpose();
      Aeq.block(0,nq_con+nu_con+nc,nq_con,nc*nd) = -D_con;
    } else {
      Aeq.resize(nq_con,nq_con+nu_con);
    }
    Aeq.topLeftCorner(nq_con,nq_con) = H_con_con;
    Aeq.block(0,nq_con,nq_con,nu_con) = -pdata->B_con;
    beq = -C_con;
    if (nq_free>0) {
      getCols(pdata->free_dof,H_con,H_con_free);
      beq -= H_con_free*qdd_free;
    }      

    CGE (myGRBaddconstrs(model,Aeq,beq,GRB_EQUAL), pdata->env);
  }

  if (nc > 0) {
    // relative acceleration constraint
    MatrixXd Aeq(neps,nparams);
    VectorXd beq(neps);
    
    Aeq.topLeftCorner(neps,nq_con) = Jp_con;
    Aeq.block(0,nq_con,neps,nu_con+nf) = MatrixXd::Zero(neps,nu_con+nf);  // todo:  this is very inefficient
    Aeq.topRightCorner(neps,neps) = MatrixXd::Identity(neps,neps);
    beq = (-Jpdot_con - 1.0*Jp_con)*qd_con;
    
    CGE (myGRBaddconstrs(model,Aeq,beq,GRB_EQUAL), pdata->env);
  }    
  
  if (nc>0) {
    // linear friction constraints
    int cind[1+nd];
    double cval[1+nd] = {-mu,1,1,1,1}; // this will have to change if m_surface_tangents changes!
    
    for (i=0; i<nc; i++) {
      // -mu*lambda[i] + sum(beta[i]s) <= 0
      cind[0] = nq_con+nu_con+i;
      for (j=0; j<nd; j++) cind[j+1]=nq_con+nu_con+nc+i*nd+j;
      CGE (GRBaddconstr(model,1+nd,cind,cval,GRB_LESS_EQUAL,0,NULL), pdata->env);
    }
  }    
  
  CGE (GRBupdatemodel(model), pdata->env);
  CGE (GRBoptimize(model), pdata->env);
  
  VectorXd alpha(nparams);
  CGE (GRBgetdblattrarray(model, GRB_DBL_ATTR_X, 0, nparams, alpha.data()), pdata->env);
  
  //----------------------------------------------------------------------
  // Solve for free inputs -----------------------------------------------
  VectorXd y(nu);
  VectorXd qdd(nq);
  if (nq_free > 0) {
    set<int>::iterator iter;
    i=0;
    for (iter=pdata->free_dof.begin(); iter!=pdata->free_dof.end(); iter++)
      qdd(*iter) = qdd_free(i++);
    i=0;
    for (iter=pdata->con_dof.begin(); iter!=pdata->con_dof.end(); iter++)
      qdd(*iter) = alpha(i++);
  
    VectorXd u_free = pdata->B_free.jacobiSvd(ComputeThinU|ComputeThinV).solve(H_free*qdd + C_free);

    i=0;
    for (iter=pdata->free_inputs.begin(); iter!=pdata->free_inputs.end(); iter++)
      y(*iter) = u_free(i++);
    i=0;
    for (iter=pdata->con_inputs.begin(); iter!=pdata->con_inputs.end(); iter++) {
      y(*iter) = alpha(nq_con + i);
      i++;
    }    
    
    // saturate inputs
    ArrayXd tmp = pdata->umin.max(y.array());
    y = tmp.min(pdata->umax).matrix();
  } else {
    y = alpha.block(nq,0,nu,1); 
    qdd = alpha.block(0,0,nq,1);
  }
  
  if (nlhs>0) plhs[0] = eigenToMatlab(y);

  
  if (nlhs>1) {
    double Vdot;
    if (nc>0) 
      Vdot = (2*x_bar.transpose()*S + s1.transpose())*(A_ls*x_bar + B_ls*(Jdot.topRows(2)*qdvec + J.topRows(2)*qdd));
    else
      Vdot = 0;
    plhs[1] = mxCreateDoubleScalar(Vdot);
  }
    
  
  if (nlhs>2) {  // return model.Q (for unit testing)
    int qnz;
    CGE (GRBgetintattr(model,"NumQNZs",&qnz), pdata->env);
    int *qrow = new int[qnz], *qcol = new int[qnz];
    double* qval = new double[qnz];
    CGE (GRBgetq(model,&qnz,qrow,qcol,qval), pdata->env);
    plhs[2] = mxCreateDoubleMatrix(nparams,nparams,mxREAL);
    double* pm = mxGetPr(plhs[2]);
    memset(pm,0,sizeof(double)*nparams*nparams);
    for (i=0; i<qnz; i++)
      pm[qrow[i]+nparams*qcol[i]] = qval[i];
    delete[] qrow;
    delete[] qcol;
    delete[] qval;
    
    if (nlhs>3) {  // return model.obj (for unit testing)
      plhs[3] = mxCreateDoubleMatrix(1,nparams,mxREAL);
      CGE (GRBgetdblattrarray(model, "Obj", 0, nparams, mxGetPr(plhs[3])), pdata->env);

      if (nlhs>4) {  // return model.A (for unit testing)
        int numcon;
        CGE (GRBgetintattr(model,"NumConstrs",&numcon), pdata->env);
        plhs[4] = mxCreateDoubleMatrix(numcon,nparams,mxREAL);
        double *pm = mxGetPr(plhs[4]);
        for (i=0; i<numcon; i++)
          for (j=0; j<nparams; j++)
            CGE (GRBgetcoeff(model,i,j,&pm[i+j*numcon]), pdata->env);
        
        if (nlhs>5) {  // return model.rhs (for unit testing)
          plhs[5] = mxCreateDoubleMatrix(numcon,1,mxREAL);
          CGE (GRBgetdblattrarray(model,"RHS",0,numcon,mxGetPr(plhs[5])), pdata->env);
        } 
        
        if (nlhs>6) { // return model.sense
          char* sense = new char[numcon+1];
          CGE (GRBgetcharattrarray(model,"Sense",0,numcon,sense), pdata->env);
          sense[numcon]='\0';
          plhs[6] = mxCreateString(sense);
          // delete[] sense;  // it seems that I'm not supposed to free this
        }
        
        if (nlhs>7) plhs[7] = eigenToMatlab(lb);
        if (nlhs>8) plhs[8] = eigenToMatlab(ub);
      }
    }
  }
  
  GRBfreemodel(model);
//  GRBfreeenv(env);
} 


















int main() {

  int nlhs;
  mxArray** plhs;
  int nrhs;
  const mxArray** prhs;

  mexFunctionHeightMapWrapper(nlhs, plhs, nrhs, prhs);

  // first determine which command to apply
  std::string command;
  if (nrhs == 0) command = "create";
  else {
    if (!mxIsChar(prhs[0])) {
      fprintf(stderr,"MapWrapper: first argument must be command string\n");
      exit(-1);
    }
    command = Utils::getString(prhs[0]);
  }
  std::transform(command.begin(), command.end(), command.begin(), ::tolower);

  // create instance
  if (command == "create") {
    auto wrapper = WrapperCollection::instance().createWrapper();
    if (nrhs > 1) {
      if ((nrhs % 2) != 1) {
        fprintf(stderr,"MapWrapper: wrong number of parameter args\n");
        exit(-1);
      }
      for (int i = 1; i < nrhs; i += 2) {
        if (!mxIsChar(prhs[i])) {
          fprintf(stderr,"MapWrapper: key must be string\n");
          exit(-1);
        }
        if (!mxIsChar(prhs[i+1])) {
          fprintf(stderr,"MapWrapper: val must be string\n");
          exit(-1);
        }
        std::string key = Utils::getString(prhs[i]);
        std::string value = Utils::getString(prhs[i+1]);

        std::transform(key.begin(), key.end(), key.begin(), ::tolower);
        if (key == "privatechannel") {
          bool isPrivate = Utils::getBool(value);
          if (isPrivate) {
            wrapper->setHeightMapChannel(MapHandle::kHeightMapChannel,
                                         MapHandle::kHeightMapViewId);
          }
          else {
            wrapper->setHeightMapChannel
              ("MAP_DEPTH", drc::data_request_t::HEIGHT_MAP_SCENE);
          }
        }
        else {
          fprintf(stderr,"MapWrapper: invalid property\n");
          exit(-1);
        }
      }  
    }

    wrapper->start();

    // return index of wrapper object
    plhs[0] = mxCreateDoubleMatrix(1,1,mxREAL);
    double id = wrapper->mId;
    memcpy(mxGetData(plhs[0]),&id,sizeof(double));
    return 1;
  }

  // get wrapper object from static collection
  if ((nrhs < 2) || (mxGetNumberOfElements(prhs[1]) != 1)) {
    fprintf(stderr,"MapWrapper: second argument must be handle id\n");
    exit(-1);
  }
  double val;
  memcpy(&val, mxGetData(prhs[1]), sizeof(double));
  int wrapperId = (int)(val+0.5);
  auto wrapper = WrapperCollection::instance().getWrapper(wrapperId);
  if (wrapper == NULL) {
    fprintf(stderr,"MapWrapper: handle is invalid; did you clear mex?\n");
    exit(-1);
  }

  // destroy instance
  if (command == "destroy") {
    if (nrhs != 2) {
      fprintf(stderr,"MapWrapper: too many arguments to destroy\n");
      exit(-1);
    }
    WrapperCollection::instance().destroyWrapper(wrapper->mId);
  }

  // get point cloud
  else if (command == "pointcloud") {
    if (nrhs != 2) {
      fprintf(stderr,"MapWrapper: too many arguments to pointcloud\n");
      exit(-1);
    }
    auto view = wrapper->getView();
    if (view == NULL) {
      plhs[0] = mxCreateDoubleMatrix(3,0,mxREAL);
      return 1;
    }
    auto cloud = view->getAsPointCloud();
    plhs[0] = mxCreateDoubleMatrix(3,cloud->size(),mxREAL);
    double* ptr = mxGetPr(plhs[0]);
    for (int i = 0; i < cloud->size(); ++i) {
      maps::PointCloud::PointType& pt = cloud->points[i];
      ptr[i*3+0] = pt.x;
      ptr[i*3+1] = pt.y;
      ptr[i*3+2] = pt.z;
    }
  }

  // get closest points and normals
  else if (command == "closest") {
    if (nrhs != 3) {
      fprintf(stderr,"MapWrapper: too many arguments to closest\n");
      exit(-1);
    }
    if (mxGetM(prhs[2]) != 3) {
      fprintf(stderr,"MapWrapper: points must be 3xn\n");
      exit(-1);
    }
    if (nlhs > 2) {
      fprintf(stderr,"MapWrapper: too many output arguments\n");
      exit(-1);
    }

    int n = mxGetN(prhs[2]);
    plhs[0] = mxCreateDoubleMatrix(3,n,mxREAL);
    mxArray* pmxNormal = mxCreateDoubleMatrix(3,n,mxREAL);
    
    double* posPtr = mxGetPr(prhs[2]);
    double* closestPtr = mxGetPr(plhs[0]);
    double* normalPtr = mxGetPr(pmxNormal);
    Eigen::Vector3f pos, closest, normal;

    auto view = wrapper->getView();
    for (int i = 0; i < n; ++i) {
      pos << posPtr[3*i], posPtr[3*i+1], posPtr[3*i+2];
      if ((view != NULL) && view->getClosest(pos,closest,normal)) {
        for (int j = 0; j < 3; ++j) {
          closestPtr[3*i+j] = closest[j];
          normalPtr[3*i+j] = normal[j];
        }
      }
      else { // invalid view or off grid
        for (int j = 0; j < 3; ++j) {
          closestPtr[3*i+j] = NAN;
          normalPtr[3*i+j] = NAN;
        }
      }
    }

    if (nlhs == 2) plhs[1] = pmxNormal;
    else mxDestroyArray(pmxNormal);
    sleep(1);
  }

  else if (command == "getrawdepth") {
    if (nrhs != 2) {
      fprintf(stderr,"MapWrapper: too many arguments to getrawdepth\n");
      exit(-1);
    }
    auto view = std::static_pointer_cast<maps::DepthImageView>
      (wrapper->getView());
    if (view == NULL) {
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
    auto img = view->getDepthImage();
    int w(img->getWidth()), h(img->getHeight());
    plhs[0] = mxCreateDoubleMatrix(h,w,mxREAL);
    auto vals = img->getData(maps::DepthImage::TypeDepth);
    float* inPtr = vals.data();
    double* outPtr = mxGetPr(plhs[0]);
    for (int i = 0; i < h; ++i) {
      for (int j = 0; j < w; ++j, ++inPtr) {
        outPtr[j*h+i] = *inPtr;
      }
    }
  }

  else if (command == "setrawdepth") {
    if (nrhs != 3) {
      fprintf(stderr,"MapWrapper: too many arguments to setrawdepth\n");
      exit(-1);
    }
    auto view = std::static_pointer_cast<maps::DepthImageView>
      (wrapper->getView());
    if (view == NULL) {
      fprintf(stderr,"MapWrapper: no view object, so not setting depths\n");
      exit(-1);
    }
    auto img = view->getDepthImage();
    int w(img->getWidth()), h(img->getHeight());
    if ((mxGetM(prhs[2]) != h) || (mxGetN(prhs[2]) != w)) {
      fprintf(stderr,"MapWrapper: depth image sizes do not match\n");
      exit(-1);
    }

    std::vector<float> data(w*h);
    double* inPtr = mxGetPr(prhs[2]);
    for (int j = 0; j < w; ++j) {
      for (int i = 0; i < h; ++i, ++inPtr) {
        data[i*w+j] = (float)(*inPtr);
      }
    }
    img->setData(data, maps::DepthImage::TypeDepth);
  }

  else if (command == "transform") {
    if (nrhs != 2) {
      fprintf(stderr,"MapWrapper: too many arguments to setrawdepth\n");
      exit(-1);
    }
    auto view = wrapper->getView();
    plhs[0] = mxCreateDoubleMatrix(4,4,mxREAL);
    double* matx = mxGetPr(plhs[0]);
    if (view == NULL) {
      for (int i = 0; i < 16; ++i) matx[i] = 0;
      return 1;
    }
    Eigen::Projective3f transform = view->getTransform();
    for (int i = 0; i < 16; ++i) matx[i] = transform.data()[i];
  }

  else if (command == "wrapper") {
    if (nrhs != 2) {
      fprintf(stderr,"MapWrapper: too many arguments to wrapper\n");
      exit(-1);
    }
    auto ptr = wrapper->mHandle.get();
    mxClassID classId = (sizeof(ptr)==4) ? mxUINT32_CLASS : mxUINT64_CLASS;
    plhs[0] = mxCreateNumericMatrix(1,1,classId,mxREAL);
    memcpy(mxGetPr(plhs[0]), &ptr, sizeof(ptr));
  }

  else if (command == "property") {
    if (nrhs != 4) {
      fprintf(stderr,"MapWrapper: must supply key/value pair\n");
      exit(-1);
    }
    if (!mxIsChar(prhs[2])) {
      fprintf(stderr,"MapWrapper: property key must be string\n");
      exit(-1);
    }
    std::string key = Utils::getString(prhs[2]);
    std::transform(key.begin(), key.end(), key.begin(), ::tolower);

    if (!mxIsChar(prhs[3])) {
      fprintf(stderr,"MapWrapper: property value must be string\n");
      exit(-1);
    }
    std::string value = Utils::getString(prhs[3]);

    if (key == "normalradius") {
      std::istringstream(value) >> wrapper->mNormalRadius;
    }
    else if (key == "fill") {
      wrapper->mShouldFill = Utils::getBool(value);
    }
    else {
      fprintf(stderr,"MapWrapper: invalid property\n");
      exit(-1);
    }
  }

  // none of the above
  else {
    fprintf(stderr,"MapWrapper: invalid command\n");
    exit(-1);
  }
  

  return 1;
}
