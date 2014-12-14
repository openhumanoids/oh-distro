#include "ParamManager.hpp"

#include <bot_param/param_client.h>
#include <lcmtypes/bot_param/set_t.hpp>
#include <lcmtypes/bot_param/entry_t.hpp>
#include <drc_utils/BotWrapper.hpp>
#include <drc_utils/Clock.hpp>

#include <lcm/lcm-cpp.hpp>

#include <gtkmm.h>

namespace {
  void onParamChange(BotParam* iOldParam, BotParam* iNewParam,
                     int64_t iTimestamp, void* iUser) {
    ParamManager& manager = *((ParamManager*)iUser);
    BotParam* botParam = manager.getBotWrapper()->getBotParam();
    if (botParam == NULL) return;
    if (iOldParam == botParam) {
      std::shared_ptr<drc::BotWrapper> tempWrapper
        (new drc::BotWrapper(manager.getBotWrapper()->getLcm(), iNewParam));
      manager.onParamChange(tempWrapper);
    }
  }
}


ParamManager::
ParamManager(const std::shared_ptr<drc::BotWrapper>& iBotWrapper) {
  mBotWrapper = iBotWrapper;
  initialize();
}

ParamManager::
~ParamManager() {
}

void ParamManager::
initialize() {
  BotParam* botParam = mBotWrapper->getBotParam();
  if (botParam == NULL) return;
  bot_param_add_update_subscriber(mBotWrapper->getBotParam(),
                                  ::onParamChange, this);
  onParamChange();
}

std::shared_ptr<drc::BotWrapper> ParamManager::
getBotWrapper() const {
  return mBotWrapper;
}

void ParamManager::
setKeyBase(const std::string& iBase) {
  mKeyBase = iBase;
}

bool ParamManager::
bind(const std::string& iSubKey, Gtk::ComboBox& iWidget) {
  Binding::Ptr binding(new Binding());
  binding->mSubKey = iSubKey;
  binding->mWidget = &iWidget;
  binding->mType = WidgetTypeCombo;
  mBindings[iSubKey] = binding;
  return true;
}

bool ParamManager::
bind(const std::string& iSubKey, Gtk::SpinButton& iWidget) {
  Binding::Ptr binding(new Binding());
  binding->mSubKey = iSubKey;
  binding->mWidget = &iWidget;
  binding->mType = WidgetTypeSpin;
  mBindings[iSubKey] = binding;
  return true;
}

bool ParamManager::
bind(const std::string& iSubKey, Gtk::CheckButton& iWidget) {
  Binding::Ptr binding(new Binding());
  binding->mSubKey = iSubKey;
  binding->mWidget = &iWidget;
  binding->mType = WidgetTypeCheck;
  mBindings[iSubKey] = binding;
  return true;
}

bool ParamManager::
bind(const std::string& iSubKey, Gtk::Scale& iWidget) {
  Binding::Ptr binding(new Binding());
  binding->mSubKey = iSubKey;
  binding->mWidget = &iWidget;
  binding->mType = WidgetTypeSlider;
  mBindings[iSubKey] = binding;
  return true;
}


void ParamManager::
pushValues() {
  BotParam* botParam = mBotWrapper->getBotParam();
  if (botParam == NULL) return;
  bot_param::set_t msg;
  msg.utime = drc::Clock::instance()->getCurrentTime();
  msg.sequence_number = bot_param_get_seqno(botParam);
  msg.server_id = bot_param_get_server_id(botParam);
  bot_param::entry_t entry;
  entry.is_array = false;
  for (auto item : mBindings) {
    std::string key = mKeyBase + "." + item.second->mSubKey;
    entry.key = key;
    std::ostringstream oss;
    Gtk::Widget* widget = item.second->mWidget;
    switch (item.second->mType) {
    case WidgetTypeCombo:
      oss << ((Gtk::ComboBox*)widget)->get_active_row_number();
      break;
    case WidgetTypeSpin:
      oss << ((Gtk::SpinButton*)widget)->get_value();
      break;
    case WidgetTypeCheck:
      oss << (((Gtk::CheckButton*)widget)->get_active() ? "true" : "false");
      break;
    case WidgetTypeSlider:
      oss << ((Gtk::Scale*)widget)->get_value();
      break;
    default:
      break;
    }
    entry.value = oss.str();
    if (mBotWrapper->get(entry.key)!=entry.value) {
      msg.entries.push_back(entry);
    }
  }

  msg.numEntries = msg.entries.size();
  if (msg.numEntries > 0) {
    mBotWrapper->getLcm()->publish("PARAM_SET", &msg);
  }
}


void ParamManager::
onParamChange(const std::shared_ptr<drc::BotWrapper>& iBotWrapper) {
  auto botWrapper = (iBotWrapper == NULL) ? mBotWrapper : iBotWrapper;
  for (auto item : mBindings) {
    std::string key = mKeyBase + "." + item.second->mSubKey;
    if (!botWrapper->hasKey(key)) continue;

    Gtk::Widget* widget = item.second->mWidget;
    switch (item.second->mType) {
    case WidgetTypeCombo: {
      int val = botWrapper->getInt(key);
      if (((Gtk::ComboBox*)widget)->get_active_row_number() != val) {
        ((Gtk::ComboBox*)widget)->set_active(val);
      }
      break;
    }
    case WidgetTypeSpin: {
      double val = botWrapper->getDouble(key);
      if (((Gtk::SpinButton*)widget)->get_value() != val) {
        ((Gtk::SpinButton*)widget)->set_value(val);
      }
      break;
    }
    case WidgetTypeCheck: {
      bool val = botWrapper->getBool(key);
      if (((Gtk::CheckButton*)widget)->get_active() != val) {
        ((Gtk::CheckButton*)widget)->set_active(val);
      }
      break;
    }
    case WidgetTypeSlider: {
      double val = botWrapper->getDouble(key);
      if (((Gtk::Scale*)widget)->get_value() != val) {
        ((Gtk::Scale*)widget)->set_value(val);      
      }
    }
    default:
      break;
    }
  }
}
