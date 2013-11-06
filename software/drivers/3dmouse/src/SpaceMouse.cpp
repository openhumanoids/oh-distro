#include "SpaceMouse.hpp"

using namespace drc;

namespace {
  const float MAX_MOTION_VALUE = 350.0f;
}

SpaceMouse::
SpaceMouse() {
  mIsRunning = false;
  mIsGood = (spnav_open() != -1);
  mButtonMask = 0;
}

SpaceMouse::
~SpaceMouse() {
  stop();
  spnav_close();
}

bool SpaceMouse::
isGood() const {
  return mIsGood;
}

bool SpaceMouse::
start() {
  if (mIsRunning) return false;
  mIsRunning = true;
  mEventThread = std::thread(std::ref(*this));
  return true;
}

bool SpaceMouse::
stop() {
  if (!mIsRunning) return false;
  mIsRunning = false;
  if (mEventThread.joinable()) mEventThread.join();
  return true;
}

bool SpaceMouse::
isRunning() {
  return mIsRunning;
}

void SpaceMouse::
addListener(const Listener& iListener) {
  mListeners.push_back(const_cast<Listener*>(&iListener));
}

void SpaceMouse::
removeListener(const Listener& iListener) {
  mListeners.remove(const_cast<Listener*>(&iListener));
}

void SpaceMouse::
removeAllListeners() {
  mListeners.clear();
}

void SpaceMouse::
operator()() {
  while (mIsRunning) {
    spnav_event event;
    if (0 == spnav_poll_event(&event)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }
    if (event.type == SPNAV_EVENT_BUTTON) {
      if (event.button.press == 0) mButtonMask &= ~(1 << event.button.bnum);
      else mButtonMask |= (1 << event.button.bnum);
    }
    for (auto listener : mListeners) {
      listener->notify(event);
      if (event.type == SPNAV_EVENT_BUTTON) {
        ButtonEvent buttonEvent;
        buttonEvent.mPressed = (event.button.press != 0);
        buttonEvent.mButtonId = event.button.bnum+1;
        listener->notify(buttonEvent);
      }
      else if (event.type == SPNAV_EVENT_MOTION) {
        MotionEvent motionEvent;
        motionEvent.mButtonMask = mButtonMask;
        motionEvent.mVelX = event.motion.z/MAX_MOTION_VALUE;
        motionEvent.mVelY = -event.motion.x/MAX_MOTION_VALUE;
        motionEvent.mVelZ = event.motion.y/MAX_MOTION_VALUE;
        motionEvent.mRateRoll = event.motion.rz/MAX_MOTION_VALUE;
        motionEvent.mRatePitch = -event.motion.rx/MAX_MOTION_VALUE;
        motionEvent.mRateYaw = event.motion.ry/MAX_MOTION_VALUE;
        listener->notify(motionEvent);
      }
    }
    spnav_remove_events(SPNAV_EVENT_MOTION);
  }
}
