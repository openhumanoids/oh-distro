#include "Collector.hpp"

#include "SensorDataReceiver.hpp"
#include "MapManager.hpp"
#include "PointDataBuffer.hpp"

#include <set>
#include <thread>

using namespace maps;

struct Collector::Helper {
  std::shared_ptr<BotWrapper> mBotWrapper;
  std::shared_ptr<SensorDataReceiver> mDataReceiver;
  std::shared_ptr<MapManager> mMapManager;
  std::thread mConsumerThread;
  bool mRunning;
  std::set<DataListener*> mDataListeners;

  static const float kPi;

  struct DataConsumer {
    Helper* mHelper;
    DataConsumer(Helper* iHelper) : mHelper(iHelper) {}
    void operator()() {
      while (mHelper->mRunning) {
        SensorDataReceiver::SensorData data;
        if (mHelper->mDataReceiver->waitForData(data)) {
          mHelper->mMapManager->addData(*data.mPointSet);
          std::set<DataListener*>::const_iterator iter =
            mHelper->mDataListeners.begin();
          for (; iter != mHelper->mDataListeners.end(); ++iter) {
            (*iter)->notify(data);
          }
        }
      }
    }
  };

  float computeAngleFromHorizontal(const Eigen::Quaternionf& iQuat) {
    Eigen::Matrix3f rot = iQuat.matrix();
    float angle = kPi/2-atan2(rot(2,2), rot(2,1));
    if (angle < 0) angle += 2*kPi;
    return angle;
  }
};

const float Collector::Helper::kPi = 4*atan(1);

Collector::
Collector() {
  mHelper.reset(new Helper());
  mHelper->mDataReceiver.reset(new SensorDataReceiver());
  mHelper->mMapManager.reset(new MapManager());
}

Collector::
~Collector() {
  mHelper->mDataListeners.clear();
  stop();
}

void Collector::
setBotWrapper(const std::shared_ptr<BotWrapper>& iWrapper) {
  mHelper->mBotWrapper = iWrapper;
  mHelper->mDataReceiver->setBotWrapper(iWrapper);
}  

bool Collector::
start() {
  if (mHelper->mRunning) return false;
  mHelper->mRunning = true;
  mHelper->mDataReceiver->start();
  mHelper->mConsumerThread = std::thread(Helper::DataConsumer(mHelper.get()));
  return true;
}

bool Collector::
stop() {
  if (!mHelper->mRunning) return false;
  mHelper->mRunning = false;
  mHelper->mDataReceiver->stop();
  mHelper->mDataReceiver->unblock();
  if (mHelper->mConsumerThread.joinable()) mHelper->mConsumerThread.join();
  return true;
}

void Collector::
addListener(const DataListener& iListener) {
  mHelper->mDataListeners.insert(const_cast<DataListener*>(&iListener));
}

void Collector::
removeListener(const DataListener& iListener) {
  mHelper->mDataListeners.erase(const_cast<DataListener*>(&iListener));
}

std::shared_ptr<SensorDataReceiver> Collector::
getDataReceiver() const {
  return mHelper->mDataReceiver;
}

std::shared_ptr<MapManager> Collector::
getMapManager() const {
  return mHelper->mMapManager;
}

bool Collector::
getLatestFullSweep(int64_t& oStartTime, int64_t& oEndTime) const {
  PointDataBuffer::Ptr buf = mHelper->mMapManager->getPointData();
  std::vector<PointSet> pointSets = buf->getAll();
  if (pointSets.size() < 5) return false;

  const PointSet& currPointSet = pointSets.back();
  float currAngle = mHelper->
    computeAngleFromHorizontal(currPointSet.mCloud->sensor_orientation_);
  float prevAngle = mHelper->
    computeAngleFromHorizontal
    (pointSets[pointSets.size()-5].mCloud->sensor_orientation_);
  float delta = currAngle-prevAngle;
  float spinSign = (((delta < 0) && (-delta < Helper::kPi)) ||
                    ((delta > 0) && (delta > Helper::kPi))) ? -1 : 1;
  
  oStartTime = oEndTime = currPointSet.mTimestamp;

  for (std::vector<PointSet>::const_reverse_iterator iter = pointSets.rbegin();
       iter != pointSets.rend(); ++iter) {
    float angle = mHelper->
      computeAngleFromHorizontal(iter->mCloud->sensor_orientation_);
    float angleDiff = spinSign*(currAngle-angle);

    oStartTime = iter->mTimestamp;
    if (angleDiff > Helper::kPi) break;
  }
  return true;
}

bool Collector::
getLatestSwath(const float iMinAngle, const float iMaxAngle,
               int64_t& oStartTime, int64_t& oEndTime) const {
  PointDataBuffer::Ptr buf = mHelper->mMapManager->getPointData();
  std::vector<PointSet> pointSets = buf->getAll();
  if (pointSets.size() < 5) return false;

  bool first = true;
  bool insidePrev = true;
  bool done = false;
  bool started = false;
  float anglePrev = 0;
  int64_t timePrev = 0;
  for (std::vector<PointSet>::const_reverse_iterator iter = pointSets.rbegin();
       iter != pointSets.rend(); ++iter) {
    float angle = mHelper->
      computeAngleFromHorizontal(iter->mCloud->sensor_orientation_);
    if (angle > Helper::kPi) angle -= Helper::kPi;

    if (first) {
      anglePrev = angle;
      timePrev = iter->mTimestamp;
      first = false;
    }

    bool inside = (angle >= iMinAngle) && (angle <= iMaxAngle);
    bool jumped = fabs(anglePrev-angle) > (iMaxAngle-iMinAngle)/2;

    if (!started && inside && (jumped || !insidePrev)) {
      oEndTime = iter->mTimestamp;
      started = true;
    } else if (started && ((inside && jumped) || (!inside && insidePrev))) {
      oStartTime = timePrev;
      done = true;
      break;
    }

    anglePrev = angle;
    timePrev = iter->mTimestamp;
    insidePrev = inside;
  }

  return done;
}
