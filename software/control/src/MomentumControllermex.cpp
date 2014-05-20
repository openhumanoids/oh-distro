/* 
 * A c++ version of (significant pieces of) the MomentumControlBlock.m mimoOutput method. 
 *
 * Todo:
 *   switch to spatial accelerations in motion constraints
 *   handle the no supports case (arguments into mex starting with B_ls will currently fail)
 *   use fixed-size matrices (or at least pre-allocated)
 *       for instance: #define nq 
 *       set MaxRowsAtCompileTime (http://eigen.tuxfamily.org/dox/TutorialMatrixClass.html)
 *   some matrices might be better off using RowMajor
 */

#include "QPCommon.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  int error;
  if (nrhs<1) mexErrMsgTxt("usage: ptr = MomentumControllermex(0,control_obj,robot_obj,...); alpha=MomentumControllermex(ptr,...,...)");
  if (nlhs<1) mexErrMsgTxt("take at least one output... please.");
  
  struct QPControllerData* pdata;
  mxArray* pm;
  double* pr;
  int i,j;


  if (mxGetScalar(prhs[0])==0) { // then construct the data object and return
    pdata = new struct QPControllerData;
    
    // get control object properties
    const mxArray* pobj = prhs[1];
    
    pm = myGetProperty(pobj,"slack_limit");
    pdata->slack_limit = mxGetScalar(pm);


    pm = myGetProperty(pobj,"W_hdot");
    assert(mxGetM(pm)==6); assert(mxGetN(pm)==6);
    pdata->W_hdot.resize(mxGetM(pm),mxGetN(pm));
    memcpy(pdata->W_hdot.data(),mxGetPr(pm),sizeof(double)*mxGetM(pm)*mxGetN(pm));

    pm= myGetProperty(pobj,"w_grf");
    pdata->w_grf = mxGetScalar(pm);    

    pm= myGetProperty(pobj,"w_slack");
    pdata->w_slack = mxGetScalar(pm);    

    pm= myGetProperty(pobj,"Kp");
    pdata->Kp = mxGetScalar(pm);    

    pm= myGetProperty(pobj,"Kd");
    pdata->Kd = mxGetScalar(pm);    

    pm= myGetProperty(pobj,"mass");
    pdata->mass = mxGetScalar(pm);    

    // get robot mex model ptr
    if (!mxIsNumeric(prhs[2]) || mxGetNumberOfElements(prhs[2])!=1)
      mexErrMsgIdAndTxt("DRC:QPControllermex:BadInputs","the third argument should be the robot mex ptr");
    memcpy(&(pdata->r),mxGetData(prhs[2]),sizeof(pdata->r));
    
    pdata->B.resize(mxGetM(prhs[3]),mxGetN(prhs[3]));
    memcpy(pdata->B.data(),mxGetPr(prhs[3]),sizeof(double)*mxGetM(prhs[3])*mxGetN(prhs[3]));

    int nq = pdata->r->num_dof, nu = pdata->B.cols();
    
    pm = myGetProperty(pobj,"w_qdd");
    pdata->w_qdd.resize(nq);
    memcpy(pdata->w_qdd.data(),mxGetPr(pm),sizeof(double)*nq);

    pdata->num_spatial_accel_constraints = mxGetScalar(prhs[4]);

    pdata->umin.resize(nu);
    pdata->umax.resize(nu);
    memcpy(pdata->umin.data(),mxGetPr(prhs[5]),sizeof(double)*nu);
    memcpy(pdata->umax.data(),mxGetPr(prhs[6]),sizeof(double)*nu);

    pdata->B_act.resize(nu,nu);
    pdata->B_act = pdata->B.bottomRows(nu);

     // get the map ptr back from matlab
     if (!mxIsNumeric(prhs[7]) || mxGetNumberOfElements(prhs[7])!=1)
     mexErrMsgIdAndTxt("DRC:QPControllermex:BadInputs","the seventh argument should be the map ptr");
     memcpy(&pdata->map_ptr,mxGetPr(prhs[7]),sizeof(pdata->map_ptr));
    
//    pdata->map_ptr = NULL;
    if (!pdata->map_ptr)
      mexWarnMsgTxt("Map ptr is NULL.  Assuming flat terrain at z=0");
    
    // get the multi-robot ptr back from matlab
    if (!mxIsNumeric(prhs[8]) || mxGetNumberOfElements(prhs[8])!=1)
    mexErrMsgIdAndTxt("DRC:QPControllermex:BadInputs","the eigth argument should be the multi_robot ptr");
    memcpy(&pdata->multi_robot,mxGetPr(prhs[8]),sizeof(pdata->multi_robot));

    // create gurobi environment
    error = GRBloadenv(&(pdata->env),NULL);

    // set solver params (http://www.gurobi.com/documentation/5.5/reference-manual/node798#sec:Parameters)
    mxArray* psolveropts = myGetProperty(pobj,"gurobi_options");
    int method = (int) mxGetScalar(myGetField(psolveropts,"method"));
    CGE ( GRBsetintparam(pdata->env,"outputflag",0), pdata->env );
    CGE ( GRBsetintparam(pdata->env,"method",method), pdata->env );
    // CGE ( GRBsetintparam(pdata->env,"method",method), pdata->env );
    CGE ( GRBsetintparam(pdata->env,"presolve",0), pdata->env );
    if (method==2) {
    	CGE ( GRBsetintparam(pdata->env,"bariterlimit",20), pdata->env );
    	CGE ( GRBsetintparam(pdata->env,"barhomogeneous",0), pdata->env );
    	CGE ( GRBsetdblparam(pdata->env,"barconvtol",0.0005), pdata->env );
    }

    mxClassID cid;
    if (sizeof(pdata)==4) cid = mxUINT32_CLASS;
    else if (sizeof(pdata)==8) cid = mxUINT64_CLASS;
    else mexErrMsgIdAndTxt("Drake:constructModelmex:PointerSize","Are you on a 32-bit machine or 64-bit machine??");
    
    plhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
    memcpy(mxGetData(plhs[0]),&pdata,sizeof(pdata));
    
    // preallocate some memory
    pdata->H.resize(nq,nq);
    pdata->H_float.resize(6,nq);
    pdata->H_act.resize(nu,nq);

    pdata->C.resize(nq);
    pdata->C_float.resize(6);
    pdata->C_act.resize(nu);

    pdata->J.resize(3,nq);
    pdata->Jdot.resize(3,nq);
    pdata->J_xy.resize(2,nq);
    pdata->Jdot_xy.resize(2,nq);
    pdata->Hqp.resize(nq,nq);
    pdata->fqp.resize(nq);
    pdata->Ag.resize(6,nq);
    pdata->Agdot.resize(6,nq);
    

    pdata->vbasis_len = 0;
    pdata->cbasis_len = 0;
    pdata->vbasis = NULL;
    pdata->cbasis = NULL;
    return;
  }
  
  // first get the ptr back from matlab
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("DRC:QPControllermex:BadInputs","the first argument should be the ptr");
  memcpy(&pdata,mxGetData(prhs[0]),sizeof(pdata));

//  for (i=0; i<pdata->r->num_bodies; i++)
//  	mexPrintf("body %d (%s) has %d contact points\n", i, pdata->r->bodies[i].linkname.c_str(), pdata->r->bodies[i].contact_pts.cols());

  int nu = pdata->B.cols(), nq = pdata->r->num_dof;
  const int dim = 3, // 3D
      nd = 2*m_surface_tangents; // for friction cone approx, hard coded for now
  
  assert(nu+6 == nq);

  int narg=1;

  int use_fast_qp = (int) mxGetScalar(prhs[narg++]);
  
  Map< VectorXd > qddot_des(mxGetPr(prhs[narg++]),nq);
  
  double *q = mxGetPr(prhs[narg++]);
  double *qd = &q[nq];
//  double *q_multi = mxGetPr(prhs[narg++]);

  vector<VectorXd> spatial_accel_constraints;
  for (int i=0; i<pdata->num_spatial_accel_constraints; i++) {
    assert(mxGetM(prhs[narg])==7); assert(mxGetN(prhs[narg])==1);
    VectorXd v = VectorXd::Zero(7,1);
    memcpy(v.data(),mxGetPr(prhs[narg++]),sizeof(double)*7);
    spatial_accel_constraints.push_back(v);
  }
  
  int num_condof;
  VectorXd condof;
  if (!mxIsEmpty(prhs[narg])) {
    assert(mxGetN(prhs[narg])==1);
    num_condof=mxGetM(prhs[narg]);
    condof = VectorXd::Zero(num_condof);
    memcpy(condof.data(),mxGetPr(prhs[narg++]),sizeof(double)*num_condof);
  }
  else {
    num_condof=0;
    narg++; // skip over empty vector
  }

  int desired_support_argid = narg++;

  assert(mxGetM(prhs[narg])==2); assert(mxGetN(prhs[narg])==4);
  MatrixXd K = MatrixXd::Zero(2,4);
  memcpy(K.data(),mxGetPr(prhs[narg++]),sizeof(double)*2*4);

  assert(mxGetM(prhs[narg])==4); assert(mxGetN(prhs[narg])==1);
  Map< Vector4d > x0(mxGetPr(prhs[narg++]));

  assert(mxGetM(prhs[narg])==2); assert(mxGetN(prhs[narg])==1);
  Map< Vector2d > y0(mxGetPr(prhs[narg++]));

  // desired com-z refs
  double comz_des = mxGetScalar(prhs[narg++]);
  double dcomz_des = mxGetScalar(prhs[narg++]);
  double ddcomz_des = mxGetScalar(prhs[narg++]);

  double mu = mxGetScalar(prhs[narg++]);

  double* double_contact_sensor = mxGetPr(prhs[narg]); int len = mxGetNumberOfElements(prhs[narg++]);
  VectorXi contact_sensor(len);  
  for (i=0; i<len; i++)
    contact_sensor(i)=(int)double_contact_sensor[i];
  double contact_threshold = mxGetScalar(prhs[narg++]);
  double terrain_height = mxGetScalar(prhs[narg++]); // nonzero if we're using DRCFlatTerrainMap
  
  pdata->r->doKinematics(q,false,qd);

  #ifdef BULLET_COLLISION
  // if (pdata->multi_robot) {
  //   auto multi_robot = static_cast<RigidBodyManipulator*>(pdata->multi_robot);
  //   multi_robot->doKinematics(q_multi,false);
  // }
  #endif
  
  //---------------------------------------------------------------------
  // Compute active support from desired supports -----------------------

  vector<SupportStateElement> active_supports;
  int num_active_contact_pts=0;
  if (!mxIsEmpty(prhs[desired_support_argid])) {
    VectorXd phi;
    mxArray* mxBodies = mxGetProperty(prhs[desired_support_argid],0,"bodies");
    if (!mxBodies) mexErrMsgTxt("couldn't get bodies");
    double* pBodies = mxGetPr(mxBodies);
    mxArray* mxContactPts = mxGetProperty(prhs[desired_support_argid],0,"contact_pts");
    if (!mxContactPts) mexErrMsgTxt("couldn't get contact points");
    mxArray* mxContactSurfaces = mxGetProperty(prhs[desired_support_argid],0,"contact_surfaces");
    if (!mxContactSurfaces) mexErrMsgTxt("couldn't get contact surfaces");
    double* pContactSurfaces = mxGetPr(mxContactSurfaces);
    
    for (i=0; i<mxGetNumberOfElements(mxBodies);i++) {
      mxArray* mxBodyContactPts = mxGetCell(mxContactPts,i);
      int nc = mxGetNumberOfElements(mxBodyContactPts);
      if (nc<1) continue;
      
      SupportStateElement se;
      se.body_idx = (int) pBodies[i]-1;
      pr = mxGetPr(mxBodyContactPts); 
      for (j=0; j<nc; j++) {
//      	mexPrintf("adding pt %d to body %d\n", (int)pr[j]-1, se.body_idx);
        se.contact_pt_inds.insert((int)pr[j]-1);
      }
      se.contact_surface = (int) pContactSurfaces[i]-1;
      
      if (contact_threshold == -1) { // ignore terrain
        if (contact_sensor(i)!=0) { // no sensor info, or sensor says yes contact
          active_supports.push_back(se);
          num_active_contact_pts += nc;
        }
      } else {
        contactPhi(pdata,se,phi,terrain_height);
        if (phi.minCoeff()<=contact_threshold || contact_sensor(i)==1) { // any contact below threshold (kinematically) OR contact sensor says yes contact
          active_supports.push_back(se);
          num_active_contact_pts += nc;
        }
      }
    }
  }

  pdata->r->HandC(q,qd,(MatrixXd*)NULL,pdata->H,pdata->C,(MatrixXd*)NULL,(MatrixXd*)NULL,(MatrixXd*)NULL);
  pdata->H_float = pdata->H.topRows(6);
  pdata->H_act = pdata->H.bottomRows(nu);
  pdata->C_float = pdata->C.head(6);
  pdata->C_act = pdata->C.tail(nu);
 

  pdata->r->getCMM(q,qd,pdata->Ag,pdata->Agdot);
  
  Vector3d xcom;
  // consider making all J's into row-major
  
  pdata->r->getCOM(xcom);
  pdata->r->getCOMJac(pdata->J);
  pdata->r->getCOMJacDot(pdata->Jdot);
  // copy to xy versions for computations below (avoid doing topRows a bunch of times)
  pdata->J_xy = pdata->J.topRows(2);
  pdata->Jdot_xy = pdata->Jdot.topRows(2);

  Map<VectorXd> qdvec(qd,nq);
  
  MatrixXd Jz,Jp,Jpdot,D;
  int nc = contactConstraintsBV(pdata,num_active_contact_pts,mu,active_supports,Jz,D,Jp,Jpdot,terrain_height);
  int neps = nc*dim;

  Vector2d ustar = Vector2d::Zero();
  Vector4d x_bar,xlimp;
  MatrixXd D_float(6,D.cols()), D_act(nu,D.cols());
  if (nc>0) {
    xlimp << xcom.topRows(2),pdata->J_xy*qdvec;
    x_bar = xlimp-x0;

    D_float = D.topRows(6);
    D_act = D.bottomRows(nu);

    ustar = K * x_bar + y0;
  }

  int nf = nc*nd; // number of contact force variables
  int nparams = nq+nf+neps;

  Vector3d ldot_des;
  ldot_des << ustar[0]*pdata->mass, 
              ustar[1]*pdata->mass, 
              (pdata->Kp*(comz_des-xcom[2]) + pdata->Kd*(dcomz_des-pdata->J.row(2)*qdvec) + ddcomz_des)*pdata->mass;

  VectorXd h = pdata->Ag*qdvec;
  Vector3d k = h.head(3);
  Vector3d kdot_des = -5.0 *k; 

  VectorXd hdot_des(6,1);
  hdot_des << kdot_des[0], kdot_des[1], kdot_des[2], ldot_des[0], ldot_des[1], ldot_des[2];

  //----------------------------------------------------------------------
  // QP cost function ----------------------------------------------------
  //
  //  min: quad(Jdot*qd + J*qdd,R_ls)+quad(C*x+D*(Jdot*qd + J*qdd),Qy) + (2*x'*S + s1')*(A*x + B*(Jdot*qd + J*qdd)) + w*quad(qddot_ref - qdd) + quad(u,R) + quad(epsilon)
  VectorXd f(nparams);
  {      
    if (nc > 0) {
      // NOTE: moved Hqp calcs below, because I compute the inverse directly for FastQP (and sparse Hqp for gurobi)
      pdata->fqp = qdvec.transpose()*pdata->Agdot.transpose()*pdata->W_hdot*pdata->Ag;
      pdata->fqp -= hdot_des.transpose()*pdata->W_hdot*pdata->Ag;
      pdata->fqp -= (pdata->w_qdd.array()*qddot_des.array()).matrix().transpose();
  
      // obj(1:nq) = fqp
      f.head(nq) = pdata->fqp.transpose();
     } else {
      // obj(1:nq) = -qddot_des
      f.head(nq) = -qddot_des;
    } 
  }
  f.tail(nf+neps) = VectorXd::Zero(nf+neps);
  

  int neq = 6+neps+6*pdata->num_spatial_accel_constraints+num_condof;
  MatrixXd Aeq = MatrixXd::Zero(neq,nparams);
  VectorXd beq = VectorXd::Zero(neq);
  
  // constrained floating base dynamics
  //  H_float*qdd - J_float'*lambda - Dbar_float*beta = -C_float
  Aeq.topLeftCorner(6,nq) = pdata->H_float;
  beq.topRows(6) = -pdata->C_float;
    
  if (nc>0) {
    Aeq.block(0,nq,6,nc*nd) = -D_float;
  }
  
  if (nc > 0) {
    // relative acceleration constraint
    Aeq.block(6,0,neps,nq) = Jp;
    Aeq.block(6,nq,neps,nf) = MatrixXd::Zero(neps,nf);  // note: obvious sparsity here
    Aeq.block(6,nq+nf,neps,neps) = MatrixXd::Identity(neps,neps);             // note: obvious sparsity here
    beq.segment(6,neps) = (-Jpdot - 0.0*Jp)*qdvec;
  }    
  
  // add in body spatial equality constraints
  VectorXd body_vdot;
  MatrixXd orig = MatrixXd::Zero(4,1);
  orig(3,0) = 1;
  int body_idx;
  int equality_ind = 6+neps;
  MatrixXd Jb(6,nq);
  MatrixXd Jbdot(6,nq);
  for (int i=0; i<pdata->num_spatial_accel_constraints; i++) {
    
    body_vdot = spatial_accel_constraints[i].bottomRows(6);
    body_idx = (int)(spatial_accel_constraints[i][0])-1;

    if (!inSupport(active_supports,body_idx)) {
      pdata->r->forwardJac(body_idx,orig,1,Jb);
      pdata->r->forwardJacDot(body_idx,orig,1,Jbdot);

      for (int j=0; j<6; j++) {
        if (!std::isnan(body_vdot[j])) {
          Aeq.block(equality_ind,0,1,nq) = Jb.row(j);
          beq[equality_ind++] = -Jbdot.row(j)*qdvec + body_vdot[j];
        }
      }
    }
  }

  if (num_condof>0) {
    // add joint acceleration constraints
    for (int i=0; i<num_condof; i++) {
      Aeq(equality_ind,(int)condof[i]-1) = 1;
      beq[equality_ind++] = qddot_des[(int)condof[i]-1];
    }
  }  
  
  MatrixXd Ain = MatrixXd::Zero(2*nu,nparams);  // note: obvious sparsity here
  VectorXd bin = VectorXd::Zero(2*nu);

  // linear input saturation constraints
  // u=B_act'*(H_act*qdd + C_act - Jz_act'*z - Dbar_act*beta)
  // using transpose instead of inverse because B is orthogonal
  Ain.topLeftCorner(nu,nq) = pdata->B_act.transpose()*pdata->H_act;
  Ain.block(0,nq,nu,nc*nd) = -pdata->B_act.transpose()*D_act;
  bin.head(nu) = -pdata->B_act.transpose()*pdata->C_act + pdata->umax;

  Ain.block(nu,0,nu,nparams) = -1*Ain.block(0,0,nu,nparams);
  bin.segment(nu,nu) = pdata->B_act.transpose()*pdata->C_act - pdata->umin;


  GRBmodel * model = NULL;
  int info=-1;
  
  // set obj,lb,up
  VectorXd lb(nparams), ub(nparams);
  lb.head(nq) = -1e3*VectorXd::Ones(nq);
  ub.head(nq) = 1e3*VectorXd::Ones(nq);
  lb.segment(nq,nf) = VectorXd::Zero(nf);
  ub.segment(nq,nf) = 1e3*VectorXd::Ones(nf);
  lb.tail(neps) = -pdata->slack_limit*VectorXd::Ones(neps);
  ub.tail(neps) = pdata->slack_limit*VectorXd::Ones(neps);

  VectorXd alpha(nparams);

  MatrixXd Qnfdiag(nf,1), Qneps(neps,1);
  vector< MatrixXd* > QBlkDiag( nc>0 ? 3 : 1 );  // nq, nf, neps   // this one is for gurobi


	if (nc>0) {
		VectorXd w = (pdata->w_qdd.array() + REG).matrix();
    pdata->Hqp = pdata->Ag.transpose()*pdata->W_hdot*pdata->Ag;
		pdata->Hqp += w.asDiagonal();
	} else {
  	pdata->Hqp = MatrixXd::Constant(nq,1,1+REG);
	}

  Qnfdiag = MatrixXd::Constant(nf,1,pdata->w_grf+REG);
  Qneps = MatrixXd::Constant(neps,1,pdata->w_slack+REG);

  QBlkDiag[0] = &pdata->Hqp;
  if (nc>0) {
  	QBlkDiag[1] = &Qnfdiag;
  	QBlkDiag[2] = &Qneps;     // quadratic slack var cost, Q(nparams-neps:end,nparams-neps:end)=eye(neps)
  }

  MatrixXd Ain_lb_ub(2*nu+2*nparams,nparams);
  VectorXd bin_lb_ub(2*nu+2*nparams);
  Ain_lb_ub << Ain, 			     // note: obvious sparsity here
  		-MatrixXd::Identity(nparams,nparams),
  		MatrixXd::Identity(nparams,nparams);
  bin_lb_ub << bin, -lb, ub;


  if (use_fast_qp > 0)
  { // set up and call fastqp
    info = fastQP(QBlkDiag, f, Aeq, beq, Ain_lb_ub, bin_lb_ub, pdata->active, alpha);
    if (info<0)  	mexPrintf("fastQP info=%d... calling Gurobi.\n", info);
  }
  else {
    // temporarily disabled because drake is not up to date yet in DRC repo
    // // use gurobi active set 
    // model = gurobiActiveSetQP(pdata->env,QBlkDiag,f,Aeq,beq,Ain,bin,lb,ub,pdata->vbasis,pdata->vbasis_len,pdata->cbasis,pdata->cbasis_len,alpha);
    // CGE(GRBgetintattr(model,"NumVars",&pdata->vbasis_len), pdata->env);
    // CGE(GRBgetintattr(model,"NumConstrs",&pdata->cbasis_len), pdata->env);
    // info=66;
    info = -1;
  }

  if (info<0) {
    model = gurobiQP(pdata->env,QBlkDiag,f,Aeq,beq,Ain,bin,lb,ub,pdata->active,alpha);
    int status; CGE(GRBgetintattr(model, "Status", &status), pdata->env);
	  if (status!=2) mexPrintf("Gurobi reports non-optimal status = %d.\n", status);
  }
  
  // temp, for testing: 
  //pdata->active.clear();


  //----------------------------------------------------------------------
  // Solve for inputs ----------------------------------------------------
  VectorXd y(nu);
  VectorXd qdd = alpha.head(nq);
  VectorXd beta = alpha.segment(nq,nc*nd);

  // use transpose because B_act is orthogonal
  y = pdata->B_act.transpose()*(pdata->H_act*qdd + pdata->C_act - D_act*beta);
  //y = pdata->B_act.jacobiSvd(ComputeThinU|ComputeThinV).solve(pdata->H_act*qdd + pdata->C_act - Jz_act.transpose()*lambda - D_act*beta);
  
  if (nlhs>0) {
    plhs[0] = eigenToMatlab(y);
  }
  
  if (nlhs>1) {
    plhs[1] = eigenToMatlab(qdd);
  }

  if (nlhs>2) {
    plhs[2] = mxCreateNumericMatrix(1,1,mxINT32_CLASS,mxREAL);
    memcpy(mxGetData(plhs[2]),&info,sizeof(int));
  }

  if (nlhs>3) {
      plhs[3] = mxCreateDoubleMatrix(1,active_supports.size(),mxREAL);
      pr = mxGetPr(plhs[3]);
      int i=0;
      for (vector<SupportStateElement>::iterator iter = active_supports.begin(); iter!=active_supports.end(); iter++) {
          pr[i++] = (double) (iter->body_idx + 1);
      }
  }

  if (nlhs>4) {
    plhs[4] = eigenToMatlab(pdata->Hqp);
  }

  if (nlhs>5) {
    plhs[5] = eigenToMatlab(f);
  }

  if (nlhs>6) {
    plhs[6] = eigenToMatlab(Aeq);
  }

  if (nlhs>7) {
    plhs[7] = eigenToMatlab(beq);
  }

  if (nlhs>8) {
    plhs[8] = eigenToMatlab(Ain_lb_ub);
  }

  if (nlhs>9) {
    plhs[9] = eigenToMatlab(bin_lb_ub);
  }

  if (nlhs>10) {
    plhs[10] = eigenToMatlab(Qnfdiag);
  }

  if (nlhs>11) {
    plhs[11] = eigenToMatlab(Qneps);
  }

  if (nlhs>12) {
    plhs[12] = eigenToMatlab(alpha);
  }

  if (nlhs>13) {
    plhs[13] = mxCreateDoubleMatrix(1,pdata->active.size(),mxREAL);
    pr = mxGetPr(plhs[13]);
    int i=0;
    for (set<int>::iterator iter = pdata->active.begin(); iter!=pdata->active.end(); iter++) {
      pr[i++] = (double) (*iter);
    }
  }

  if (nlhs>14) {
    plhs[14] = eigenToMatlab(h);
  }

  if (nlhs>15) {
    plhs[15] = eigenToMatlab(hdot_des);
  }




  if (model) { 
    GRBfreemodel(model); 
  } 
  //  GRBfreeenv(env);
} 
