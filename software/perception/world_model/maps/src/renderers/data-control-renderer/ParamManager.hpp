#ifndef _maps_ParamManager_hpp_
#define _maps_ParamManager_hpp_

#include <memory>
#include <string>
#include <map>

namespace maps {
  class BotWrapper;
}

namespace Gtk {
  class Widget;
  class ComboBox;
  class SpinButton;
  class CheckButton;
}

class ParamManager {
protected:
  enum WidgetType {
    WidgetTypeSpin,
    WidgetTypeCheck,
    WidgetTypeCombo,
  };

  struct Binding {
    typedef std::shared_ptr<Binding> Ptr;
    std::string mSubKey;
    Gtk::Widget* mWidget;
    WidgetType mType;
  };

protected:
  std::shared_ptr<maps::BotWrapper> mBotWrapper;
  std::string mKeyBase;
  std::map<std::string,Binding::Ptr> mBindings;

public:
  ParamManager(const std::shared_ptr<maps::BotWrapper>& iBotWrapper);
  ~ParamManager();

  std::shared_ptr<maps::BotWrapper> getBotWrapper() const;

  void setKeyBase(const std::string& iBase);
  bool bind(const std::string& iSubKey, Gtk::ComboBox& iWidget);
  bool bind(const std::string& iSubKey, Gtk::SpinButton& iWidget);
  bool bind(const std::string& iSubKey, Gtk::CheckButton& iWidget);

  void pushValues();
  void onParamChange();

protected:

};

#endif
