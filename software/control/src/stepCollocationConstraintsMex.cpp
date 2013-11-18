#include "mex.h"
#include <math.h>
#include <Eigen/Dense>

using namespace std;

void constraints(mxArray* c_out, mxArray* ceq_out, mxArray* dc_out, mxArray* dceq_out, const Eigen::MatrixXd& x, const Eigen::Vector3d& c0, const Eigen::Vector3d& cf, float max_line_deviation, size_t nceq, size_t nv, size_t nsteps)
{
  Eigen::MatrixXd steps = x.block(0, 0, 6, nsteps);
  Eigen::MatrixXd rel_steps = x.block(6, 0, 6, nsteps); 

  Eigen::MatrixXd c(nsteps, 1);
  c = Eigen::MatrixXd::Zero(nsteps, 1);
  Eigen::MatrixXd ceq(2, nsteps);
  ceq = Eigen::MatrixXd::Zero(2, nsteps);
  Eigen::MatrixXd dc(nv, nsteps);
  dc = Eigen::MatrixXd::Zero(nv, nsteps);
  Eigen::MatrixXd dceq(nv, nceq);
  dceq = Eigen::MatrixXd::Zero(nv, nceq);
  Eigen::Matrix2d R;
  Eigen::Vector2d dxy;
  Eigen::Vector2d proj;
  Eigen::Vector2d u;
  Eigen::Vector2d al;

  ceq.block(0,0,2,1) = steps.block(0,0,2,1) - rel_steps.block(0,0,2,1);
  dceq.block(0,0,2,2) << 1, 0, 0, 1;
  dceq.block(6,0,2,2) << -1, 0, 0, -1;

  int j;
  for (j = 2; j <= nsteps; j++) {
    int con_ndx = (j-1)*2;
    int con_dndx = 2;
    double si = sin(steps(5,j-2));
    double co = cos(steps(5,j-2));
    R << co, -si, si, co;
    dxy = R * rel_steps.block(0,j-1,2,1);
    proj = steps.block(0,j-2,2,1) + dxy;
    ceq.block(0,j-1,2,1) = steps.block(0,j-1,2,1) - proj;
    int x1_ndx = (j-2)*12;
    int dx_ndx = (j-1)*12+6;
    int x2_ndx = (j-1)*12;
    dceq.block(x2_ndx,con_ndx,2,con_dndx) << 1, 0, 0, 1;
    dceq.block(x1_ndx,con_ndx,2,con_dndx) << -1, 0, 0, -1;
    double dx = rel_steps(0,j-1);
    double dy = rel_steps(1,j-1);
    dceq.block(x1_ndx+5,con_ndx,1,con_dndx) << dx*si + dy*co, -dx*co + dy*si;
    dceq.block(dx_ndx,con_ndx,1,con_dndx) << -co, -si;
    dceq.block(dx_ndx+1,con_ndx,1,con_dndx) << si, -co;
  }

  u = cf.segment(0,2) - c0.segment(0,2);
  double norm = u.norm();
  u(0) /= norm;
  u(1) /= norm;
  al << -u(1), u(0);
  double bl = al.dot(c0.segment(0,2));

  for (j = 1; j <= nsteps; j++) {
    double g = al(0) * steps(0,j-1) + al(1) * steps(1,j-1) - bl;
    c(j-1) = g*g - max_line_deviation*max_line_deviation;
    int x1_ndx = (j-1)*12;
    dc.block(x1_ndx, j-1, 2, 1) = 2*g*al;
  }
  memcpy(mxGetPr(c_out), c.data(), sizeof(double)*c.rows()*c.cols());
  memcpy(mxGetPr(dc_out), dc.data(), sizeof(double)*dc.rows()*dc.cols());
  memcpy(mxGetPr(ceq_out), ceq.data(), sizeof(double)*ceq.rows()*ceq.cols());
  memcpy(mxGetPr(dceq_out), dceq.data(), sizeof(double)*dceq.rows()*dceq.cols());

  return;
}

void printit(double x[]){
  mexPrintf("%f\n", x[0]);
}

void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
  size_t nv = mxGetM(prhs[0]);
  size_t nsteps = nv / 12;
  size_t nceq = 2 * nsteps;
  double max_line_deviation = mxGetScalar(prhs[3]);

  Eigen::MatrixXd x = Eigen::Map<Eigen::MatrixXd>(mxGetPr(prhs[0]), 12, nsteps);
  Eigen::Vector3d c0 = Eigen::Map<Eigen::Vector3d>(mxGetPr(prhs[1]));
  Eigen::Vector3d cf = Eigen::Map<Eigen::Vector3d>(mxGetPr(prhs[2]));

  /* Create matrix for the return argument. */
  plhs[0] = mxCreateDoubleMatrix((mwSize) nsteps, (mwSize)1, mxREAL); // c
  plhs[1] = mxCreateDoubleMatrix((mwSize)2*nsteps, (mwSize)1, mxREAL); // ceq
  plhs[2] = mxCreateDoubleMatrix((mwSize)nv, (mwSize)nsteps, mxREAL); // dc
  plhs[3] = mxCreateDoubleMatrix((mwSize)nv, (mwSize)nceq, mxREAL); // dceq

  /* Assign pointers to each input and output. */
  mxArray* c = plhs[0];
  mxArray* ceq = plhs[1];
  mxArray* dc = plhs[2];
  mxArray* dceq = plhs[3];

  constraints(c, ceq, dc, dceq, x, c0, cf, max_line_deviation, nceq, nv, nsteps);
}