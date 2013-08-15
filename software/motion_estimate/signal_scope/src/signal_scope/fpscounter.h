#ifndef _FPSCOUNTER_H_
#define _FPSCOUNTER_H_

#include <QTime>

// A class for keeping a exponential moving average of frames per second.

#include <cstdio>

class FPSCounter
{
public:


  FPSCounter()
  {
    mAlpha = 0.9;
    mTimeWindow = 1.0;
    mAverageFPS = 1.0;
    mFramesThisWindow = 0;
    mTime.restart();
  }

  ~FPSCounter()
  {
  }

  double alpha() const
  {
    return mAlpha;
  }

  void setAlpha(double alpha)
  {
    mAlpha = alpha;
  }

  double timeWindow() const
  {
    return mTimeWindow;
  }

  void setTimeWindow(double seconds)
  {
    mTimeWindow = seconds;
  }

  void update()
  {
    ++mFramesThisWindow;
    updateAverage();
  }

  double averageFPS()
  {
    updateAverage();
    return mAverageFPS;
  }

private:

  void updateAverage()
  {
    // check if a time window has elapsed
    double elapsedTime = mTime.elapsed() / 1000.0;

    if (elapsedTime > mTimeWindow)
    {
      // compute FPS for this time window
      double averageFPSThisWindow = mFramesThisWindow / elapsedTime;

      // update moving average
      mAverageFPS = mAlpha * averageFPSThisWindow + (1.0 - mAlpha) * mAverageFPS;

      // reset counters
      mTime.restart();
      mFramesThisWindow = 0;
    }
  }

  FPSCounter(const FPSCounter&); // Not implemented
  void operator=(const FPSCounter&); // Not implemented

  double mAlpha;
  double mAverageFPS;
  double mTimeWindow;

  size_t mFramesThisWindow;

  QTime mTime;
};

#endif
