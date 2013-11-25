#include <mex.h>

#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include <bot_param/param_client.h>
#include <bot_core/timestamp.h>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_param/set_t.hpp>

namespace {
  std::string getString(const mxArray* iArray) {
    char* chars = mxArrayToString(iArray);
    std::string str = chars;
    mxFree(chars);
    return str;
  }
}

class BotParamClient {
public:
  BotParamClient() {
    mBotParam = NULL;
    mLcm.reset(new lcm::LCM());
    getParamFromServer();
  }

  ~BotParamClient() {
  }

  static BotParamClient& instance() {
    static BotParamClient theClient;
    return theClient;
  }

  std::shared_ptr<lcm::LCM> getLcm() {
    return mLcm;
  }

  BotParam* getUnderlyingBotParam() {
    if (mBotParam == NULL) {
      getParamFromServer();
    }
    return mBotParam;
  }

  static void onParamUpdate(BotParam* iOldParam, BotParam* iNewParam,
                            int64_t iUtime, void* iUser) {
    BotParamClient* self = (BotParamClient*)iUser;
    // TODO: update stuff if necessary
  }

protected:
  void getParamFromServer() {
    if (mBotParam != NULL) return;
    mBotParam = bot_param_get_global(mLcm->getUnderlyingLCM(), 1);
    // mBotParam = bot_param_new_from_server(mLcm->getUnderlyingLCM(), 1);
    if (mBotParam == NULL) {
      mexPrintf("BotParam not received; is server running?\n");
      return;
    }
    bot_param_add_update_subscriber(mBotParam, onParamUpdate, this);
  }

protected:
  std::shared_ptr<lcm::LCM> mLcm;
  BotParam* mBotParam;
};

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if ((nrhs == 0) || !mxIsChar(prhs[0])) {
    mexErrMsgTxt("BotParamClient: first argument must be command string");
  }
  if ((nrhs < 2) || !mxIsChar(prhs[1])) {
    mexErrMsgTxt("BotParamClient: second argument must be key");
  }

  std::string command = ::getString(prhs[0]);
  std::transform(command.begin(), command.end(), command.begin(), ::tolower);
  std::string key = ::getString(prhs[1]);

  if ((nrhs > 3) || ((nrhs == 3) && (command != "setkey"))) {
    mexErrMsgTxt("BotParamClient: too many input arguments");
  }

  BotParam* param = BotParamClient::instance().getUnderlyingBotParam();
  if (param == NULL) {
    mexErrMsgTxt("BotParamClient: no param client; is server running?");
  }

  bool hasKey = (0 != bot_param_has_key(param, key.c_str()));
  if (!hasKey && (command != "setkey") && (command != "haskey")) {
    mexErrMsgTxt("BotParamClient: invalid key");
  }

  if (command == "haskey") {
    plhs[0] = mxCreateLogicalMatrix(1,1);
    mxLogical* out = mxGetLogicals(plhs[0]);
    out[0] = hasKey;
  }

  else if (command == "subkeys") {
    char** subkeysRaw = bot_param_get_subkeys(param, key.c_str());
    std::vector<std::string> subkeys;
    for (char** subkeyPtr = subkeysRaw; *subkeyPtr != NULL; ++subkeyPtr) {
      subkeys.push_back(std::string(*subkeyPtr));
    }
    bot_param_str_array_free(subkeysRaw);
    plhs[0] = mxCreateCellMatrix(1,subkeys.size());
    for (size_t i = 0; i < subkeys.size(); ++i) {
      mxSetCell(plhs[0], i, mxCreateString(subkeys[i].c_str()));
    }
  }

  else if (command == "getnum") {
    int len = bot_param_get_array_len(param, key.c_str());
    std::vector<double> vals;
    if (len <= 0) {
      double val;
      if (bot_param_get_double(param, key.c_str(), &val) != 0) {
        mexErrMsgTxt("BotParamClient: cannot find numeric");
      }
      vals.push_back(val);
    }
    else {
      vals.resize(len);
      if (bot_param_get_double_array(param, key.c_str(), vals.data(), len) != len) {
        mexErrMsgTxt("BotParamClient: non-numeric value(s)");
      }
    }
    plhs[0] = mxCreateDoubleMatrix(1,vals.size(),mxREAL);
    double* ptr = mxGetPr(plhs[0]);
    for (size_t i = 0; i < vals.size(); ++i) {
      ptr[i] = vals[i];
    }
  }

  else if (command == "getbool") {
    int len = bot_param_get_array_len(param, key.c_str());
    std::vector<bool> vals;
    if (len <= 0) {
      int val;
      if (bot_param_get_boolean(param, key.c_str(), &val) != 0) {
        mexErrMsgTxt("BotParamClient: cannot find bool");
      }
      vals.push_back(val!=0);
    }
    else {
      int valsRaw[len];
      if (bot_param_get_boolean_array(param, key.c_str(), valsRaw, len) != len) {
        mexErrMsgTxt("BotParamClient: non-boolean value(s)");
      }
      for (int i = 0; i < len; ++i) {
        vals.push_back(valsRaw[i]!=0);
      }
    }
    plhs[0] = mxCreateLogicalMatrix(1,vals.size());
    mxLogical* out = mxGetLogicals(plhs[0]);
    for (size_t i = 0; i < vals.size(); ++i) {
      out[i] = vals[i];
    }
  }

  else if (command == "getstr") {
    int len = bot_param_get_array_len(param, key.c_str());
    std::vector<std::string> vals;
    if (len <= 0) {
      char* val = NULL;
      if (bot_param_get_str(param, key.c_str(), &val) != 0) {
        mexErrMsgTxt("BotParamClient: cannot find string");
      }
      vals.push_back(std::string(val));
      free(val);
    }
    else {
      char** valsRaw = bot_param_get_str_array_alloc(param, key.c_str());
      for (char** valsPtr = valsRaw; *valsPtr != NULL; ++valsPtr) {
        vals.push_back(std::string(*valsPtr));
      }
      bot_param_str_array_free(valsRaw);
    }
    plhs[0] = mxCreateCellMatrix(1,vals.size());
    for (size_t i = 0; i < vals.size(); ++i) {
      mxSetCell(plhs[0], i, mxCreateString(vals[i].c_str()));
    }
  }

  else if (command == "setkey") {
    if (!mxIsChar(prhs[1])) {
      mexErrMsgTxt("BotParamClient: third argument must be value string");
    }
    std::string value = ::getString(prhs[2]);

    bot_param::set_t msg;
    msg.utime = bot_timestamp_now();
    msg.sequence_number = bot_param_get_seqno(param);
    msg.server_id = bot_param_get_server_id(param);
    bot_param::entry_t entry;
    entry.key = key;
    entry.value = value;
    msg.numEntries = 1;
    msg.entries.resize(1);
    msg.entries[0] = entry;

    std::string channel("PARAM_SET");
    BotParamClient::instance().getLcm()->publish(channel, &msg);
  }

  else if (command == "print") {
    bot_param_write(param, stderr);
  }

  else {
    mexErrMsgTxt("BotParamClient: command must be haskey, subkeys, getnum, getbool, getstr, or setkey");
  }
}
