#include <iostream>
#include <chrono>
#include <thread>

#include <lcm/lcm-cpp.hpp>
#include <ConciseArgs>
#include <drc_utils/BotWrapper.hpp>
#include <bot_param/param_client.h>

#include "Bridge.hpp"

bool configure(const std::string& iConfigFile, Bridge& oBridge) {
  std::shared_ptr<lcm::LCM> lcm(new lcm::LCM());
  BotParam* param;
  if (iConfigFile.length() == 0) {
    param = bot_param_new_from_server(lcm->getUnderlyingLCM(), false);
  }
  else {
    param = bot_param_new_from_file(iConfigFile.c_str());
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
  std::vector<std::string> bindingStrings;
  if (!botWrapper.get(keyBase + ".bindings", bindingStrings)) {
    std::cout << "error: cannot find bindings in config" << std::endl;
    return false;
  }
  if (bindingStrings.size() == 0) {
    std::cout << "error: no bindings in config" << std::endl;
    return false;
  }
  if ((bindingStrings.size() % 4) != 0) {
    std::cout << "error: bindings must all have four entries" << std::endl;
    return false;
  }

  int numBindings = 0;
  for (int i = 0; i < (int)bindingStrings.size(); i+=4) {
    Bridge::BindingSpec spec;
    spec.mInputCommunity  = bindingStrings[i+0];
    spec.mOutputCommunity = bindingStrings[i+1];
    spec.mInputChannel    = bindingStrings[i+2];
    spec.mOutputChannel   = bindingStrings[i+3];
    oBridge.addBinding(spec);
    ++numBindings;
  }
  std::cout << "added " << numBindings << " channel bindings" << std::endl;

  return true;
}

int main(const int iArgc, const char** iArgv) {
  std::string configFile;

  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(configFile, "c", "config-file",
          "config file (use param server if empty)");
  opt.parse();

  Bridge bridge;
  if (!configure(configFile, bridge)) {
    std::cout << "cannot configure bridge" << std::endl;
    return -1;
  }

  if (!bridge.start()) {
    std::cout << "cannot start bridge" << std::endl;
    return -1;
  }
  std::cout << "bridge started" << std::endl;

  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  bridge.stop();

  return 1;
}
