#ifndef _drc_SpaceMouse_hpp_
#define _drc_SpaceMouse_hpp_

#include <iostream>
#include <list>
#include <memory>
#include <thread>
#include <spnav.h>

union spnav_event;

namespace drc {

class SpaceMouse {
public:

  static const int BUTTON1_MASK = 0x01;
  static const int BUTTON2_MASK = 0x02;

  struct ButtonEvent {
    bool mPressed;   // true if pressed, false if released
    int mButtonId;   // button number that was pressed or released
  };

  struct MotionEvent {
    int mButtonMask;  // which button(s) are pressed if any
    float mVelX;
    float mVelY;
    float mVelZ;
    float mRateRoll;
    float mRatePitch;
    float mRateYaw;
  };

  // derive from this class to listen for space mouse events
  class Listener {
  public:
    virtual void notify(const ButtonEvent& iEvent) {}
    virtual void notify(const MotionEvent& iEvent) {}
    virtual void notify(const spnav_event& iEvent) {}
  };

public:
  SpaceMouse();
  ~SpaceMouse();

  // whether we are connected to the driver
  bool isGood() const;

  // start and stop handling events
  bool start();
  bool stop();
  bool isRunning();

  // add/remove listeners that are notified when events arrive
  void addListener(const Listener& iListener);
  void removeListener(const Listener& iListener);
  void removeAllListeners();

  // for thread
  void operator()();

protected:
  bool mIsGood;
  bool mIsRunning;
  std::thread mEventThread;
  std::list<Listener*> mListeners;
  int mButtonMask;
};

}

#endif
