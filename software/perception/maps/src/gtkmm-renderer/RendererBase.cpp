#include "RendererBase.hpp"

#include <bot_vis/bot_vis.h>
#include <bot_param/param_util.h>
#include <bot_frames/bot_frames.h>
#include <lcm/lcm-cpp.hpp>
#include <drc_utils/Clock.hpp>

#include <gtkmm.h>
#include <list>
#include <unordered_map>
#include <iostream>

using namespace gtkmm;

// convenience class for combo boxes
struct ComboColumns : public Gtk::TreeModel::ColumnRecord {
  Gtk::TreeModelColumn<int> mId;
  Gtk::TreeModelColumn<Glib::ustring> mLabel;
  ComboColumns() { add(mId); add(mLabel); }
};

// proxies for widget-data bindings
struct WidgetTypeToggle {};
struct WidgetTypeValue {};
struct WidgetTypeList {};

// traits to map actual widget class type to proxy type
template<typename W> struct WidgetTraits {};
#define _MakeWidgetTraits(x,y)\
  template<> struct WidgetTraits<x> { typedef y WidgetType; };
_MakeWidgetTraits(Gtk::CheckButton, WidgetTypeToggle);
_MakeWidgetTraits(Gtk::ToggleButton, WidgetTypeToggle);
_MakeWidgetTraits(Gtk::SpinButton, WidgetTypeValue);
_MakeWidgetTraits(Gtk::HScale, WidgetTypeValue);
_MakeWidgetTraits(Gtk::ComboBox, WidgetTypeList);
_MakeWidgetTraits(Gtk::ComboBoxText, WidgetTypeList);
#undef _MakeWidgetTraits

// base class for bindings
struct WidgetBinding {
  typedef std::shared_ptr<WidgetBinding> Ptr;
  typedef Glib::SignalProxy0<void> Signal;
  Gtk::Widget* mWidget;
  virtual void load(const Glib::KeyFile& iKeyFile, const std::string& iGroup,
                    const std::string& iKey) = 0;
  virtual void save(Glib::KeyFile& iKeyFile, const std::string& iGroup,
                    const std::string& iKey) = 0;
  virtual Signal signal() = 0;
};

// most generic subclass for bindings (needs to be specialized below)
template<typename WidgetClassT, typename WidgetT, typename DataT>
struct WidgetBindingImpl : public WidgetBinding {};

// specialized binding for 'value' widgets
template<typename W, typename T>
struct WidgetBindingImpl<WidgetTypeValue,W,T> : public WidgetBinding {
  T* mData;
  typedef WidgetBindingImpl<WidgetTypeValue,W,T> MyType;
  void set(Gtk::Widget* iWidget, T& iData) {
    mWidget = iWidget;
    mData = &iData;
    signal().connect(sigc::mem_fun(*this, &MyType::callback));
  }
  void load(const Glib::KeyFile& iKeyFile, const std::string& iGroup,
            const std::string& iKey) {
    ((W*)mWidget)->set_value(iKeyFile.get_double(iGroup, iKey));
  }
  void save(Glib::KeyFile& iKeyFile, const std::string& iGroup,
            const std::string& iKey) {
    iKeyFile.set_double(iGroup, iKey, ((W*)mWidget)->get_value());
  }
  Signal signal() { return ((W*)mWidget)->signal_value_changed(); }
  void callback() { *mData = static_cast<T>(((W*)mWidget)->get_value()); }
};

// binding for 'toggle' widgets
template<typename W, typename T>
struct WidgetBindingImpl<WidgetTypeToggle,W,T> : public WidgetBinding {
  T* mData;
  typedef WidgetBindingImpl<WidgetTypeToggle,W,T> MyType;
  void set(Gtk::Widget* iWidget, T& iData) {
    mWidget = iWidget;
    mData = &iData;
    signal().connect(sigc::mem_fun(*this, &MyType::callback));
  }
  void load(const Glib::KeyFile& iKeyFile, const std::string& iGroup,
            const std::string& iKey) {
    ((W*)mWidget)->set_active(iKeyFile.get_boolean(iGroup, iKey));
  }
  void save(Glib::KeyFile& iKeyFile, const std::string& iGroup,
            const std::string& iKey) {
    iKeyFile.set_boolean(iGroup, iKey, ((W*)mWidget)->get_active());
  }
  Signal signal() { return ((W*)mWidget)->signal_toggled(); }
  void callback() { *mData = static_cast<T>(((W*)mWidget)->get_active()); }
};

// binding for 'list' widgets
template<typename W, typename T>
struct WidgetBindingImpl<WidgetTypeList,W,T> : public WidgetBinding {
  T* mData;
  typedef WidgetBindingImpl<WidgetTypeList,W,T> MyType;
  void set(Gtk::Widget* iWidget, T& iData) {
    mWidget = iWidget;
    mData = &iData;
    signal().connect(sigc::mem_fun(*this, &MyType::callback));
  }
  void load(const Glib::KeyFile& iKeyFile, const std::string& iGroup,
            const std::string& iKey) {
    ((W*)mWidget)->set_active(iKeyFile.get_integer(iGroup, iKey));
  }
  void save(Glib::KeyFile& iKeyFile, const std::string& iGroup,
            const std::string& iKey) {
    iKeyFile.set_integer(iGroup, iKey, ((W*)mWidget)->get_active_row_number());
  }
  Signal signal() { return ((W*)mWidget)->signal_changed(); }
  void callback() {
    Gtk::TreeModel::iterator iter = ((W*)mWidget)->get_active();
    if (iter) *mData = static_cast<T>((*iter)[ComboColumns().mId]);
  }
};


struct RendererBase::InternalHelper {

  // these are passed into renderer constructor
  std::string mName;
  BotViewer* mBotViewer;
  BotParam* mBotParam;
  BotFrames* mBotFrames;
  std::shared_ptr<lcm::LCM> mLcm;

  // viewer-related internal variables
  BotRenderer mBotRenderer;
  BotGtkParamWidget* mBotGtkParamWidget;
  BotEventHandler mBotEventHandler;
  Gtk::Container* mGtkContainer;
  Gtk::Toolbar* mGtkToolbar;
  Gtk::DrawingArea* mGlDrawingArea;

  // other data
  std::list<lcm::Subscription*> mLcmSubscriptions;
  typedef std::unordered_map<std::string, WidgetBinding::Ptr> WidgetBindingMap;
  WidgetBindingMap mWidgetBindings;

  InternalHelper() {
    mName = "Anonymous";
    mBotViewer = NULL;
    mBotParam = NULL;
    mBotFrames = NULL;
  }

  ~InternalHelper() {
    std::list<lcm::Subscription*>::iterator iter = mLcmSubscriptions.begin();
    for (; iter != mLcmSubscriptions.end(); ++iter) {
      mLcm->unsubscribe(*iter);
    }
    free(mBotRenderer.name);
  }

  static void destroy(BotRenderer* iRenderer) {
    RendererBase* renderer = reinterpret_cast<RendererBase*>(iRenderer->user);
    delete renderer;
  }

  static void draw(BotViewer* iViewer, BotRenderer* iRenderer) {
    RendererBase* renderer = reinterpret_cast<RendererBase*>(iRenderer->user);
    return renderer->draw();
  }

  static double pickQuery(BotViewer* iViewer, BotEventHandler* iHandler, 
                          const double iRayStart[3], const double iRayDir[3]) {
    RendererBase* renderer = reinterpret_cast<RendererBase*>(iHandler->user);
    return renderer->pickQuery(iRayStart, iRayDir);
  }

  static double hoverQuery(BotViewer* iViewer, BotEventHandler* iHandler, 
                           const double iRayStart[3], const double iRayDir[3]) {
    RendererBase* renderer = reinterpret_cast<RendererBase*>(iHandler->user);
    return renderer->hoverQuery(iRayStart, iRayDir);
  }

  static int mousePress(BotViewer* iViewer, BotEventHandler* iHandler,
                        const double iRayStart[3], const double iRayDir[3], 
                        const GdkEventButton* iEvent) {
    RendererBase* renderer = reinterpret_cast<RendererBase*>(iHandler->user);
    return (renderer->mousePress(iEvent, iRayStart, iRayDir) ? 1 : 0);
  }

  static int mouseRelease(BotViewer* iViewer, BotEventHandler* iHandler,
                          const double iRayStart[3], const double iRayDir[3], 
                          const GdkEventButton *iEvent) {
    RendererBase* renderer = reinterpret_cast<RendererBase*>(iHandler->user);
    return (renderer->mouseRelease(iEvent, iRayStart, iRayDir) ? 1 : 0);
  }

  static int mouseMotion(BotViewer* iViewer, BotEventHandler* iHandler,
                         const double iRayStart[3], const double iRayDir[3], 
                         const GdkEventMotion *iEvent) {
    RendererBase* renderer = reinterpret_cast<RendererBase*>(iHandler->user);
    return (renderer->mouseMotion(iEvent, iRayStart, iRayDir) ? 1 : 0);
  }

  static int mouseScroll(BotViewer* iViewer, BotEventHandler* iHandler,
                         const double iRayStart[3], const double iRayDir[3], 
                         const GdkEventScroll* iEvent) {
    RendererBase* renderer = reinterpret_cast<RendererBase*>(iHandler->user);
    return (renderer->mouseScroll(iEvent, iRayStart, iRayDir) ? 1 : 0);
  }

  static int keyPress(BotViewer* iViewer, BotEventHandler* iHandler, 
                      const GdkEventKey* iEvent) {
    RendererBase* renderer = reinterpret_cast<RendererBase*>(iHandler->user);
    return (renderer->keyPress(iEvent) ? 1 : 0);
  }

  static int keyRelease(BotViewer* iViewer, BotEventHandler* iHandler, 
                        const GdkEventKey* iEvent) {
    RendererBase* renderer = reinterpret_cast<RendererBase*>(iHandler->user);
    return (renderer->keyPress(iEvent) ? 1 : 0);
  }

  static void loadPreferences(BotViewer* iViewer, GKeyFile* iKeyfile,
                              RendererBase* iRenderer) {
    bot_gtk_param_widget_load_from_key_file
      (iRenderer->mHelper->mBotGtkParamWidget, iKeyfile,
       iRenderer->mHelper->mName.c_str());

    Glib::KeyFile keyFile(iKeyfile);
    if (!keyFile.has_group(iRenderer->mHelper->mName)) return;
    WidgetBindingMap& bindings = iRenderer->mHelper->mWidgetBindings;
    std::string rendererName = iRenderer->mHelper->mName;
    for (auto it = bindings.begin(); it != bindings.end(); ++it) {
      if (!keyFile.has_key(rendererName, it->first)) continue;
      it->second->load(keyFile, rendererName, it->first);
    }
  }

  static void savePreferences(BotViewer* iViewer, GKeyFile* iKeyfile,
                              RendererBase* iRenderer) {
    bot_gtk_param_widget_save_to_key_file
      (iRenderer->mHelper->mBotGtkParamWidget, iKeyfile,
       iRenderer->mHelper->mName.c_str());

    Glib::KeyFile keyFile(iKeyfile);
    WidgetBindingMap& bindings = iRenderer->mHelper->mWidgetBindings;
    std::string rendererName = iRenderer->mHelper->mName;
    for (auto it = bindings.begin(); it != bindings.end(); ++it) {
      it->second->save(keyFile, rendererName, it->first);
    }
  }
};


RendererBase::
RendererBase(const std::string& iName,
             BotViewer* iViewer, const int iPriority,
             const lcm_t* iLcm,
             const BotParam* iParam, const BotFrames* iFrames,
             const int iWhichSide) {

  // set initial state values
  mHelper.reset(new InternalHelper());
  mHelper->mName = iName;
  mHelper->mBotViewer = const_cast<BotViewer*>(iViewer);
  mHelper->mLcm.reset(new lcm::LCM(const_cast<lcm_t*>(iLcm)));
  mHelper->mBotFrames = const_cast<BotFrames*>(iFrames);
  mHelper->mBotParam = const_cast<BotParam*>(iParam);

  // set up internal renderer
  mHelper->mBotRenderer.draw = InternalHelper::draw;
  mHelper->mBotRenderer.destroy = InternalHelper::destroy;
  mHelper->mBotRenderer.name = strdup(iName.c_str());
  mHelper->mBotRenderer.user = this;
  mHelper->mBotRenderer.enabled = 1;
  mHelper->mBotRenderer.widget = gtk_alignment_new(0, 0.5, 1.0, 0);

  // other initializations
  drc::Clock::instance()->setLcm(mHelper->mLcm);
  Gtk::Main::init_gtkmm_internals();
  
  // assign event handlers
  memset(&mHelper->mBotEventHandler, 0, sizeof(BotEventHandler));
  mHelper->mBotEventHandler.name = mHelper->mBotRenderer.name;
  mHelper->mBotEventHandler.enabled = 1;
  mHelper->mBotEventHandler.key_press = InternalHelper::keyPress;
  mHelper->mBotEventHandler.key_release = InternalHelper::keyRelease;
  mHelper->mBotEventHandler.mouse_press = InternalHelper::mousePress;
  mHelper->mBotEventHandler.mouse_release = InternalHelper::mouseRelease;
  mHelper->mBotEventHandler.mouse_motion = InternalHelper::mouseMotion;
  mHelper->mBotEventHandler.mouse_scroll = InternalHelper::mouseScroll;
  mHelper->mBotEventHandler.pick_query = InternalHelper::pickQuery;
  mHelper->mBotEventHandler.hover_query = InternalHelper::hoverQuery;
  mHelper->mBotEventHandler.user = this;
  bot_viewer_add_event_handler(iViewer, &mHelper->mBotEventHandler, iPriority);

  // set up base widget container
  mHelper->mBotGtkParamWidget =
    BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
  gtk_container_add(GTK_CONTAINER (mHelper->mBotRenderer.widget),
                    GTK_WIDGET(mHelper->mBotGtkParamWidget));
  gtk_widget_show(GTK_WIDGET(mHelper->mBotGtkParamWidget));
  mHelper->mGtkContainer =
    Glib::wrap(GTK_CONTAINER(mHelper->mBotGtkParamWidget));
  mHelper->mGtkToolbar =
    Glib::wrap(GTK_TOOLBAR(iViewer->toolbar));
  mHelper->mGlDrawingArea =
    Glib::wrap(GTK_DRAWING_AREA(mHelper->mBotViewer->gl_area));

  // register callbacks for saving/loading preferences
  g_signal_connect (G_OBJECT(iViewer), "load-preferences",
                    G_CALLBACK(InternalHelper::loadPreferences), this);
  g_signal_connect (G_OBJECT(iViewer), "save-preferences",
                    G_CALLBACK(InternalHelper::savePreferences), this);

  // finally, register this renderer with the bot viewer
  bot_viewer_add_renderer_on_side(iViewer, &mHelper->mBotRenderer,
                                  iPriority, iWhichSide);
}

RendererBase::
~RendererBase() {
}

std::string RendererBase::
getName() const {
  return mHelper->mName;
}

std::shared_ptr<lcm::LCM> RendererBase::
getLcm() const {
  return mHelper->mLcm;
}

BotParam* RendererBase::
getBotParam() const {
  return mHelper->mBotParam;
}

BotFrames* RendererBase::
getBotFrames() const {
  return mHelper->mBotFrames;
}

BotViewer* RendererBase::
getBotViewer() const {
  return mHelper->mBotViewer;
}

BotRenderer* RendererBase::
getBotRenderer() const {
  return &mHelper->mBotRenderer;
}


BotEventHandler* RendererBase::
getBotEventHandler() const {
  return &mHelper->mBotEventHandler;
}

Gtk::DrawingArea* RendererBase::
getGlDrawingArea() const {
  return mHelper->mGlDrawingArea;
}

Gtk::Container* RendererBase::
getGtkContainer() const {
  return mHelper->mGtkContainer;
}

Gtk::Toolbar* RendererBase::
getGtkToolbar() const {
  return mHelper->mGtkToolbar;
}

Gtk::Widget* RendererBase::
getGtkWidget(const std::string& iName) const {
  InternalHelper::WidgetBindingMap::const_iterator item =
    mHelper->mWidgetBindings.find(iName);
  if (item == mHelper->mWidgetBindings.end()) return NULL;
  return item->second->mWidget;
}

int64_t RendererBase::
now() const {
  return drc::Clock::instance()->getCurrentTime();
}

void RendererBase::
requestDraw() {
  return bot_viewer_request_redraw(mHelper->mBotViewer);
}

void RendererBase::
add(const lcm::Subscription* iSubscription) {
  mHelper->mLcmSubscriptions.
    push_back(const_cast<lcm::Subscription*>(iSubscription));
}

template<typename W, typename T>
bool RendererBase::
bind(W* iWidget, const std::string& iName, T& iData) {
  typedef typename WidgetTraits<W>::WidgetType WidgetType;
  typedef WidgetBindingImpl<WidgetType,W,T> BindingType;
  std::shared_ptr<BindingType> binding(new BindingType());
  binding->set(iWidget, iData);
  mHelper->mWidgetBindings[iName] =
    std::dynamic_pointer_cast<BindingType>(binding);
  binding->signal().connect(sigc::mem_fun(*this, &RendererBase::requestDraw));
  return true;
}


Gtk::ComboBox* RendererBase::
createCombo(int& iData, const std::vector<std::string>& iLabels,
            const std::vector<int>& iIndices) {
  if (iIndices.size() != iLabels.size()) return NULL;
  ComboColumns columns;
  Glib::RefPtr<Gtk::ListStore> treeModel = Gtk::ListStore::create(columns);
  for (size_t i = 0; i < iIndices.size(); ++i) {
    const Gtk::TreeModel::Row& row = *(treeModel->append());
    row[columns.mId] = iIndices[i];
    row[columns.mLabel] = iLabels[i];
  }
  Gtk::ComboBox* combo = Gtk::manage(new Gtk::ComboBox());
  combo->set_model(treeModel);
  combo->pack_start(columns.mLabel);
  combo->signal_changed().connect([combo,&iData]{
    Gtk::TreeModel::iterator iter = combo->get_active();
    if (iter) iData = (int)((*iter)[ComboColumns().mId]);
    });
  combo->set_active(iData);
  return combo;
}

Gtk::SpinButton* RendererBase::
createSpin(double& iData, const double iMin, const double iMax,
           const double iStep) {
  Gtk::SpinButton* spin = Gtk::manage(new Gtk::SpinButton());
  spin->set_range(iMin, iMax);
  spin->set_increments(iStep, 10*iStep);
  spin->set_digits(2);
  spin->signal_value_changed().connect([spin,&iData]{
      iData = (double)spin->get_value();});
  spin->set_value(iData);
  return spin;
}

Gtk::SpinButton* RendererBase::
createSpin(int& iData, const int iMin, const int iMax, const int iStep) {
  Gtk::SpinButton* spin = Gtk::manage(new Gtk::SpinButton());
  spin->set_range(iMin, iMax);
  spin->set_increments(iStep, 10*iStep);
  spin->signal_value_changed().connect([spin,&iData]{
      iData = (int)spin->get_value();});
  spin->set_value(iData);
  return spin;
}

Gtk::HScale* RendererBase::
createSlider(double& iData, const double iMin, const double iMax,
             const double iStep) {
  Gtk::HScale* slider = Gtk::manage(new Gtk::HScale(iMin, iMax, iStep));
  slider->set_digits(2);
  slider->set_value_pos(Gtk::POS_LEFT);
  slider->signal_value_changed().connect([slider,&iData]{
      iData = (double)slider->get_value();});
  slider->set_value(iData);
  return slider;
}

Gtk::HScale* RendererBase::
createSlider(int& iData, const int iMin, const int iMax, const int iStep) {
  Gtk::HScale* slider = Gtk::manage(new Gtk::HScale(iMin, iMax, iStep));
  slider->set_value_pos(Gtk::POS_LEFT);
  slider->set_value(iData);
  slider->signal_value_changed().connect([slider,&iData]{
      iData = (int)slider->get_value();});
  return slider;  
}



Gtk::ToggleButton* RendererBase::
addToggle(const std::string& iName, bool& iData, Gtk::Box* iContainer) {
  Gtk::ToggleButton* button = Gtk::manage(new Gtk::ToggleButton(iName));
  bind(button, iName, iData);
  button->set_active(iData);
  Gtk::Container* container = iContainer;
  if (container == NULL) container = mHelper->mGtkContainer;
  container->add(*button);
  button->show();
  return button;
}

Gtk::CheckButton* RendererBase::
addCheck(const std::string& iName, bool& iData, Gtk::Box* iContainer) {
  Gtk::CheckButton* button = Gtk::manage(new Gtk::CheckButton(iName));
  bind(button, iName, iData);
  button->set_active(iData);
  Gtk::Container* container = iContainer;
  if (container == NULL) container = mHelper->mGtkContainer;
  container->add(*button);
  button->show();
  return button;
}

Gtk::SpinButton* RendererBase::
addSpin(const std::string& iName, double& iData,
        const double iMin, const double iMax, const double iStep,
        Gtk::Box* iContainer) {
  Gtk::HBox* box = Gtk::manage(new Gtk::HBox());
  Gtk::Label* label = Gtk::manage(new Gtk::Label(iName, Gtk::ALIGN_LEFT));
  box->add(*label);
  Gtk::SpinButton* spin = Gtk::manage(new Gtk::SpinButton());
  spin->set_range(iMin, iMax);
  spin->set_increments(iStep, 10*iStep);
  spin->set_digits(2);
  bind(spin, iName, iData);
  spin->set_value(iData);
  box->add(*spin);
  Gtk::Container* container = iContainer;
  if (container == NULL) container = mHelper->mGtkContainer;
  container->add(*box);
  box->show_all();
  return spin;
}

Gtk::SpinButton* RendererBase::
addSpin(const std::string& iName, int& iData,
        const int iMin, const int iMax, const int iStep,
        Gtk::Box* iContainer) {
  Gtk::HBox* box = Gtk::manage(new Gtk::HBox());
  Gtk::Label* label = Gtk::manage(new Gtk::Label(iName, Gtk::ALIGN_LEFT));
  box->add(*label);
  Gtk::SpinButton* spin = Gtk::manage(new Gtk::SpinButton());
  spin->set_range(iMin, iMax);
  spin->set_increments(iStep, 10*iStep);
  bind(spin, iName, iData);
  spin->set_value(iData);
  box->add(*spin);
  Gtk::Container* container = iContainer;
  if (container == NULL) container = mHelper->mGtkContainer;
  container->add(*box);
  box->show_all();
  return spin;
}

Gtk::HScale* RendererBase::
addSlider(const std::string& iName, double& iData,
          const double iMin, const double iMax, const double iStep,
          Gtk::Box* iContainer) {
  Gtk::HBox* box = Gtk::manage(new Gtk::HBox());
  Gtk::Label* label = Gtk::manage(new Gtk::Label(iName, Gtk::ALIGN_LEFT));
  box->add(*label);
  Gtk::HScale* slider = Gtk::manage(new Gtk::HScale(iMin, iMax, iStep));
  slider->set_digits(2);
  slider->set_value_pos(Gtk::POS_LEFT);
  bind(slider, iName, iData);
  slider->set_value(iData);
  box->add(*slider);
  Gtk::Container* container = iContainer;
  if (container == NULL) container = mHelper->mGtkContainer;
  container->add(*box);
  box->show_all();
  return slider;
}

Gtk::HScale* RendererBase::
addSlider(const std::string& iName, int& iData,
          const int iMin, const int iMax, const int iStep,
          Gtk::Box* iContainer) {
  Gtk::HBox* box = Gtk::manage(new Gtk::HBox());
  Gtk::Label* label = Gtk::manage(new Gtk::Label(iName, Gtk::ALIGN_LEFT));
  box->add(*label);
  Gtk::HScale* slider = Gtk::manage(new Gtk::HScale(iMin, iMax, iStep));
  slider->set_value_pos(Gtk::POS_LEFT);
  bind(slider, iName, iData);
  slider->set_value(iData);
  box->add(*slider);
  Gtk::Container* container = iContainer;
  if (container == NULL) container = mHelper->mGtkContainer;
  container->add(*box);
  box->show_all();
  return slider;  
}

Gtk::ComboBox* RendererBase::
addCombo(const std::string& iName, int& iData,
         const std::vector<std::string>& iLabels, Gtk::Box* iContainer) {
  std::vector<int> ids(iLabels.size());
  for (size_t i = 0; i < iLabels.size(); ++i) { ids[i] = i; }
  return addCombo(iName, iData, iLabels, ids);
}

Gtk::ComboBox* RendererBase::
addCombo(const std::string& iName, int& iData,
         const std::vector<std::string>& iLabels,
         const std::vector<int>& iIndices, Gtk::Box* iContainer) {
  if (iIndices.size() != iLabels.size()) {
    return NULL;
  }

  Gtk::HBox* box = Gtk::manage(new Gtk::HBox());
  Gtk::Label* label = Gtk::manage(new Gtk::Label(iName, Gtk::ALIGN_LEFT));
  box->add(*label);
  ComboColumns columns;
  Glib::RefPtr<Gtk::ListStore> treeModel = Gtk::ListStore::create(columns);
  for (size_t i = 0; i < iIndices.size(); ++i) {
    const Gtk::TreeModel::Row& row = *(treeModel->append());
    row[columns.mId] = iIndices[i];
    row[columns.mLabel] = iLabels[i];
  }
  Gtk::ComboBox* combo = Gtk::manage(new Gtk::ComboBox());
  combo->set_model(treeModel);
  combo->pack_start(columns.mLabel);
  bind(combo, iName, iData);
  combo->set_active(iData);
  box->add(*combo);
  Gtk::Box* container = iContainer;
  if (container == NULL) container = (Gtk::Box*)mHelper->mGtkContainer;
  container->pack_start(*box,false,false);
  box->show_all();
  return combo;
}


// explicit instantiations for widget binding
#define _DefineBinding(x,y)\
  template bool RendererBase::\
  bind(x* iWidget, const std::string& iName, y& iData);
#define _DefineAllTypes(x)\
  _DefineBinding(x,bool)\
  _DefineBinding(x,uint8_t)\
  _DefineBinding(x,uint16_t)\
  _DefineBinding(x,uint32_t)\
  _DefineBinding(x,uint64_t)\
  _DefineBinding(x,int8_t)\
  _DefineBinding(x,int16_t)\
  _DefineBinding(x,int32_t)\
  _DefineBinding(x,int64_t)\
  _DefineBinding(x,float)\
  _DefineBinding(x,double)
_DefineAllTypes(Gtk::CheckButton)
_DefineAllTypes(Gtk::ToggleButton)
_DefineAllTypes(Gtk::SpinButton)
_DefineAllTypes(Gtk::HScale)
_DefineAllTypes(Gtk::ComboBox)
_DefineAllTypes(Gtk::ComboBoxText)
#undef _DefineAllTypes
#undef _DefineBinding
