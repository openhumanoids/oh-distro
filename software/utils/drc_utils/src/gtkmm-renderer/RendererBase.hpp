#ifndef _gtkmm_RendererBase_hpp_
#define _gtkmm_RendererBase_hpp_

#include <string>
#include <memory>
#include <vector>

// forward declarations
typedef struct _BotRenderer BotRenderer;
typedef struct _BotViewer BotViewer;
typedef struct _BotParam BotParam;
typedef struct _BotFrames BotFrames;
typedef struct _BotEventHandler BotEventHandler;
typedef struct _lcm_t lcm_t;
typedef struct _GdkEventKey GdkEventKey;
typedef struct _GdkEventButton GdkEventButton;
typedef struct _GdkEventMotion GdkEventMotion;
typedef struct _GdkEventScroll GdkEventScroll;
namespace Gtk {
  class Widget;
  class Container;
  class Box;
  class Toolbar;
  class DrawingArea;
  class ToggleButton;
  class CheckButton;
  class SpinButton;
  class HScale;
  class ComboBox;
}
namespace lcm {
  class LCM;
  class Subscription;
}

namespace gtkmm {

class RendererBase {
public:
  RendererBase(const std::string& iName,
               BotViewer* iViewer, const int iPriority,
               const lcm_t* iLcm,
               const BotParam* iParam, const BotFrames* iFrames,
               const int iWhichSide=1);
  virtual ~RendererBase();

  // convenience methods to create and bind common types anonymously
  static Gtk::ComboBox* createCombo(int& iData,
                                    const std::vector<std::string>& iLabels,
                                    const std::vector<int>& iIndices);
  static Gtk::SpinButton* createSpin(double& iData,
                                     const double iMin, const double iMax,
                                     const double iStep);
  static Gtk::SpinButton* createSpin(int& iData, const int iMin, const int iMax,
                                     const int iStep);
  static Gtk::HScale* createSlider(double& iData,
                                   const double iMin, const double iMax,
                                   const double iStep);
  static Gtk::HScale* createSlider(int& iData, const int iMin, const int iMax,
                                   const int iStep);


protected:
  // convenience methods for data access to subclasses
  std::string getName() const;
  std::shared_ptr<lcm::LCM> getLcm() const;
  BotParam* getBotParam() const;
  BotFrames* getBotFrames() const;
  BotViewer* getBotViewer() const;
  BotRenderer* getBotRenderer() const;
  BotEventHandler* getBotEventHandler() const;
  Gtk::DrawingArea* getGlDrawingArea() const;
  Gtk::Container* getGtkContainer() const;
  Gtk::Toolbar* getGtkToolbar() const;
  Gtk::Widget* getGtkWidget(const std::string& iName) const;
  int64_t now() const;
  void requestDraw();

  // methods for registering commonly used objects
  void add(const lcm::Subscription* iSubscription);
  template<typename W, typename T>
  bool bind(W* iWidget, const std::string& iName, T& iData);

  // convenience methods to create, add, bind, and register common widgets
  Gtk::ToggleButton* addToggle(const std::string& iName, bool& iData,
                               Gtk::Box* iContainer=NULL);
  Gtk::CheckButton* addCheck(const std::string& iName, bool& iData,
                             Gtk::Box* iContainer=NULL);
  Gtk::SpinButton* addSpin(const std::string& iName, double& iData,
                           const double iMin, const double iMax,
                           const double iStep,
                           Gtk::Box* iContainer=NULL);
  Gtk::SpinButton* addSpin(const std::string& iName, int& iData,
                           const int iMin, const int iMax,
                           const int iStep,
                           Gtk::Box* iContainer=NULL);
  Gtk::HScale* addSlider(const std::string& iName, double& iData,
                         const double iMin, const double iMax,
                         const double iStep,
                         Gtk::Box* iContainer=NULL);
  Gtk::HScale* addSlider(const std::string& iName, int& iData,
                         const int iMin, const int iMax, const int iStep,
                         Gtk::Box* iContainer=NULL);
  Gtk::ComboBox* addCombo(const std::string& iName, int& iData,
                          const std::vector<std::string>& iLabels,
                          Gtk::Box* iContainer=NULL);
  Gtk::ComboBox* addCombo(const std::string& iName, int& iData,
                          const std::vector<std::string>& iLabels,
                          const std::vector<int>& iIndices,
                          Gtk::Box* iContainer=NULL);

  // event handling methods (can be overridden by subclasses)
  virtual double pickQuery(const double iRayStart[3],
                           const double iRayDir[3])        { return -1; }
  virtual double hoverQuery(const double iRayStart[3],
                            const double iRayDir[3])       { return -1; }
  virtual bool keyPress(const GdkEventKey* iEvent)         { return false; }
  virtual bool keyRelease(const GdkEventKey* iEvent)       { return false; }
  virtual bool mousePress(const GdkEventButton* iEvent,
                          const double iRayStart[3],
                          const double iRayDir[3])         { return false; }
  virtual bool mouseRelease(const GdkEventButton* iEvent,
                            const double iRayStart[3],
                            const double iRayDir[3])       { return false; }
  virtual bool mouseMotion(const GdkEventMotion* iEvent,
                           const double iRayStart[3],
                           const double iRayDir[3])        { return false; }
  virtual bool mouseScroll(const GdkEventScroll* iEvent,
                           const double iRayStart[3],
                           const double iRayDir[3])        { return false; }

  // actual rendering method (must be defined by subclasses)
  virtual void draw() = 0;


private:
  struct InternalHelper;
  std::shared_ptr<InternalHelper> mHelper;
};

}

#endif
