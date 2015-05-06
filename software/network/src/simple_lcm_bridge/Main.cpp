#include <iostream>
#include <chrono>
#include <thread>

#include <lcm/lcm-cpp.hpp>
#include <ConciseArgs>
#include <drc_utils/BotWrapper.hpp>
#include <bot_param/param_client.h>

#include "Bridge.hpp"

bool configure(const std::string& iConfigFile, const std::string& iName,
               Bridge& oBridge) {
  std::shared_ptr<lcm::LCM> lcm(new lcm::LCM());
  BotParam* param;
  if (iConfigFile.length() == 0) {
    param = bot_param_new_from_server(lcm->getUnderlyingLCM(), false);
  }
  else {
    param = bot_param_new_from_file(iConfigFile.c_str());
    if (param == NULL) {
      std::cout << "error: could not read config file " <<
        iConfigFile << std::endl;
    }
  }
  drc::BotWrapper botWrapper(lcm, param);
  std::string keyBase = "network.lcm_bridge";

  // communities
  bool found = false;
  auto names = botWrapper.getKeys(keyBase + ".communities");
  for (auto name : names) {
    std::string key = keyBase + ".communities." + name;
    std::string url = botWrapper.get(key);
    oBridge.addCommunity(name, url);
    std::cout << "added community " << name << " at " << url << std::endl;
    found = true;
  }
  if (!found) {
    std::cout << "error: no communities found" << std::endl;
    return false;
  }

  // bindings
  std::string bindingsKey = keyBase + ".bindings." + iName;
  std::vector<std::string> bindingStrings;
  if (!botWrapper.get(bindingsKey, bindingStrings)) {
    std::cout << "error: cannot find bindings in config" << std::endl;
    return false;
  }
  if (bindingStrings.size() == 0) {
    std::cout << "error: no bindings in config" << std::endl;
    return false;
  }
  if ((bindingStrings.size() % 5) != 0) {
    std::cout << "error: bindings must all have five entries" << std::endl;
    return false;
  }

  int numBindings = 0;
  for (int i = 0; i < (int)bindingStrings.size(); i+=5) {
    Bridge::BindingSpec spec;
    spec.mInputCommunity  = bindingStrings[i+0];
    spec.mOutputCommunity = bindingStrings[i+1];
    spec.mInputChannel    = bindingStrings[i+2];
    spec.mOutputChannel   = bindingStrings[i+3];
    spec.mOutputFrequency = std::stof(bindingStrings[i+4]);
    oBridge.addBinding(spec);
    ++numBindings;
  }

  // rate stats (optional)
  std::string rateKey = keyBase + ".rate_info." + iName;
  std::vector<std::string> rateStrings;
  if (botWrapper.get(rateKey, rateStrings) && (rateStrings.size() > 0)) {
    for (int i = 0; i < (int)rateStrings.size(); i+=6) {
      Bridge::RateInfoSpec spec;
      spec.mInputCommunity  = rateStrings[i+0];
      spec.mInputChannel    = rateStrings[i+1];
      spec.mEnumValue       = std::stoi(rateStrings[i+2]);
      spec.mOutputCommunity = rateStrings[i+3];
      spec.mOutputChannel   = rateStrings[i+4];
      spec.mOutputFrequency = std::stof(rateStrings[i+5]);
      oBridge.addRateInfo(spec);
    }
  }

  std::cout << "added " << numBindings << " channel bindings" << std::endl;

  return true;
}

int main(const int iArgc, const char** iArgv) {
  std::string configFile;
  std::string name;
  bool verbose = false;

  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(name, "n", "name",
          "name identifier of this bridge", true);
  opt.add(configFile, "c", "config-file",
          "config file (use param server if empty)");
  opt.add(verbose, "v", "verbose",
          "turn on verbosity");
  opt.parse();

  std::cout << "setting up bridge \"" << name << "\"" << std::endl;
  if (configFile.length() == 0) {
    std::cout << "using config from server" << std::endl;
  }
  else {
    std::cout << "using config file " << configFile << std::endl;
  }

  Bridge bridge;
  if (!configure(configFile, name, bridge)) {
    std::cout << "error: cannot configure bridge" << std::endl;
    return -1;
  }
  bridge.setVerbose(verbose);

  if (!bridge.start()) {
    std::cout << "error: cannot start bridge" << std::endl;
    return -1;
  }
  std::cout << "bridge started" << std::endl;

  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  bridge.stop();

  return 1;
}
