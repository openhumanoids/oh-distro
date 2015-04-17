#include <mex.h>
#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <cstring>

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs != 0) {
    mexErrMsgIdAndTxt("drc:bot_param_get_server_id:arguments","Too many input arguments");
  }
  if (nlhs > 1) {
    mexErrMsgIdAndTxt("drc:bot_param_get_server_id:arguments","Too many output arguments");
  }

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  BotParam* botparam_;
  botparam_ = bot_param_new_from_server(lcm->getUnderlyingLCM(), 0);
  int64_t server_id;
  if (botparam_==NULL) {
    mexPrintf("bot-param server doesn't seem to be running... returning 0.");
    server_id=0;
  }
  else {
    server_id = bot_param_get_server_id(botparam_);
  }
  plhs[0] = mxCreateNumericMatrix(1,1,mxINT64_CLASS,mxREAL);
  memcpy(mxGetData(plhs[0]),&server_id,sizeof(int));
  return;
}
