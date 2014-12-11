#ifndef _ParamManager_hpp_
#define _ParamManager_hpp_

#include <memory>
#include <string>
#include <map>

namespace drc {
  class BotWrapper;
}

namespace Gtk {
  class Widget;
  class ComboBox;
  class SpinButton;
  class CheckButton;
  class Scale;
}

class ParamManager {
protected:
  enum WidgetType {
    WidgetTypeSpin,
    WidgetTypeCheck,
    WidgetTypeCombo,
    WidgetTypeSlider,
  };

  struct Binding {
    typedef std::shared_ptr<Binding> Ptr;
    std::string mSubKey;
    Gtk::Widget* mWidget;
    WidgetType mType;
  };

protected:
  std::shared_ptr<drc::BotWrapper> mBotWrapper;
  std::string mKeyBase;
  std::map<std::string,Binding::Ptr> mBindings;

public:
  ParamManager(const std::shared_ptr<drc::BotWrapper>& iBotWrapper);
  ~ParamManager();

  std::shared_ptr<drc::BotWrapper> getBotWrapper() const;

  void setKeyBase(const std::string& iBase);
  bool bind(const std::string& iSubKey, Gtk::ComboBox& iWidget);
  bool bind(const std::string& iSubKey, Gtk::SpinButton& iWidget);
  bool bind(const std::string& iSubKey, Gtk::CheckButton& iWidget);
  bool bind(const std::string& iSubKey, Gtk::Scale& iWidget);

  void pushValues();
  void onParamChange(const std::shared_ptr<drc::BotWrapper>& iBotWrapper=NULL);

protected:
  void initialize();
};

#endif
