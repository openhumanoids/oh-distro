#include <mav_state_est/noise_id.hpp>
#include <mex.h>

template<typename EigenType>
typename Eigen::Map<const EigenType> mxArrayToEigen(const mxArray * mx_array, const std::string & var_name = "")
{
  int M_mx = mxGetM(mx_array);
  int M_eig = EigenType::RowsAtCompileTime;
  if (M_eig != Eigen::Dynamic && M_eig != M_mx) {
    if (!var_name.empty())
      mexPrintf("error loading variable %s\n", var_name.c_str());
    char error_string[1024];
    sprintf(error_string, "expected matrix with %d rows, got matrix with %d", M_eig, M_mx);
    mexErrMsgTxt(error_string);
  }

  int N_mx = mxGetN(mx_array);
  int N_eig = EigenType::ColsAtCompileTime;
  if (N_eig != Eigen::Dynamic && N_eig != N_mx) {
    if (!var_name.empty())
      mexPrintf("error loading variable %s\n", var_name.c_str());
    char error_string[1024];
    sprintf(error_string, "expected matrix with %d cols, got matrix with %d", N_eig, N_eig);
    mexErrMsgTxt(error_string);
  }

  const double * mx_ptr = mxGetPr(mx_array);
  if (mx_ptr == NULL) {
    mexErrMsgTxt("unexpected data type error");
  }

  Eigen::Map<const EigenType> ret(mx_ptr, M_mx, N_mx);

  return ret;
}

Eigen::Map<Eigen::MatrixXd> getmxArrayEigenMapXd(int rows, int cols, mxArray ** mx_array)
{
  *mx_array = mxCreateDoubleMatrix(rows, cols, mxREAL);
  Eigen::Map<Eigen::MatrixXd> ret(mxGetPr(*mx_array), rows, cols);
  return ret;
}

std::string mxArrayToCPPString(const mxArray * mx_array, const std::string & var_name = "-")
{
  if (mxIsChar(mx_array) != 1) {
    mexPrintf("expected string for variable %s\n", var_name.c_str());
    mexErrMsgTxt("didn't get string");
  }

  if (mxGetM(mx_array) != 1) {
    mexPrintf("expected row vector string for variable %s\n", var_name.c_str());
    mexErrMsgTxt("not a row vector");
  }

  mwSize buflen = (mxGetM(mx_array) * mxGetN(mx_array)) + 1;
  char * input_buf = mxArrayToString(mx_array);
  if (input_buf == NULL) {
    mexPrintf("error loading %s\n", var_name.c_str());
    mexErrMsgTxt("Could not convert input to string.");
  }
  std::string str = input_buf;
  mxFree(input_buf);

  return str;
}

#define DT .01

using namespace std;
using namespace Eigen;

static list<RBIS> state_list = list<RBIS>();
static RBIMList cov_list = RBIMList();

void exitFcn()
{
  mexPrintf("emptying filter history!\n");
}

void usage()
{
  mexErrMsgTxt("expects 4 or 5 inputs: q_vec,N_window,inds_mode,start_end_utimes,[logFilename]");
}

//noiseParamLikelihood([q_gyro,q_accel],N_window,inds_mode,[start_T,end_T],logFilename)
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nrhs != 5 && nrhs != 4)
    usage();

  Map<const Vector2d> q_vec = mxArrayToEigen<Vector2d>(prhs[0], "q_vec");
  double cov_gyro = bot_sq(bot_to_radians(q_vec(0)));
  double cov_accel = bot_sq(q_vec(1));
  int N_window = round(mxArrayToEigen<Matrix<double, 1, 1> >(prhs[1], "N_window")(0));
  int inds_mode = round(mxArrayToEigen<Matrix<double, 1, 1> >(prhs[2], "inds_mode")(0));
  Map<const Vector2d> utimes = mxArrayToEigen<Vector2d>(prhs[3], "utimes");
  if (nrhs == 5) {
    string logFileName = mxArrayToCPPString(prhs[4], "logFileName");
    loadFilterHistory(logFileName, "STATE_ESTIMATOR_STATE", utimes(0), utimes(1), state_list, cov_list);
    mexPrintf("loaded %d pose messages between %jd and %jd\n", state_list.size(), (int64_t) utimes(0),
        (int64_t) utimes(1));

    if (state_list.empty()) {
      mexErrMsgTxt("nothing loaded from log, exiting!");
    }
  }
  else if (state_list.empty()) {
    mexErrMsgTxt("must supply a log filename, no poses loaded");
  }

  VectorXi active_inds;
  switch (inds_mode) {
  case 0:
    active_inds.resize(9);
    active_inds.head(3) = RBIS::velocityInds();
    active_inds.segment(3, 3) = RBIS::chiInds();
    active_inds.tail(3) = RBIS::positionInds();
    break;
  case 1:
    active_inds.resize(6);
    active_inds.head(3) = RBIS::velocityInds();
    active_inds.segment(3, 3) = RBIS::chiInds();
    break;
  default:
    mexPrintf("unrecognized index specification %d\n", inds_mode);
    mexErrMsgTxt("can't continue");
  }

  list<RBIS> state_errors = list<RBIS>();
  RBIMList error_covs = RBIMList();

  list<RBIS> rolled_states = list<RBIS>();
  RBIMList rolled_covs = RBIMList();
  sampleProcessForward(state_list, cov_list, DT, cov_gyro, cov_accel, N_window, state_errors, error_covs,
      rolled_states, rolled_covs);
  double neglike = negLogLikelihood(state_errors, error_covs, active_inds);

  plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
  *mxGetPr(plhs[0]) = neglike;

  if (nlhs > 1) {
    Map<MatrixXd> state_errors_matrix = getmxArrayEigenMapXd(state_errors.begin()->vec.rows(), state_errors.size(),
        &plhs[1]);

    int ii = 0;
    for (list<RBIS>::iterator it = state_errors.begin(); it != state_errors.end(); it++) {
      state_errors_matrix.col(ii) = it->vec;
      ii++;
    }
  }

  if (nlhs > 2) {
    int N = error_covs.begin()->rows();
    Map<MatrixXd> error_covs_mat = getmxArrayEigenMapXd(N, N * error_covs.size(), &plhs[2]);

    int ii = 0;
    for (RBIMList::iterator it = error_covs.begin(); it != error_covs.end(); it++) {
      error_covs_mat.block(0, ii * N, N, N) = *it;
      ii++;
    }
  }

//  eigen_dump(q_vec);
//  eigen_dump(N_window);
//  eigen_dump(inds_mode);
//  eigen_dump(logFileName);

}
