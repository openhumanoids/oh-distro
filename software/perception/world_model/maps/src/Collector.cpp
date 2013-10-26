#include "Collector.hpp"

#include "SensorDataReceiver.hpp"
#include "MapManager.hpp"
#include "PointDataBuffer.hpp"

#include <set>
#include <unordered_map>
#include <thread>

using namespace maps;

struct Collector::Helper {
  std::shared_ptr<BotWrapper> mBotWrapper;
  std::shared_ptr<SensorDataReceiver> mDataReceiver;
  std::shared_ptr<MapManager> mMapManager;
  std::thread mConsumerThread;
  bool mRunning;
  std::set<DataListener*> mDataListeners;
  std::unordered_map<std::string,int64_t> mChannelToMapBindings;

  static const double kPi;

  struct DataConsumer {
    Helper* mHelper;
    DataConsumer(Helper* iHelper) : mHelper(iHelper) {}
    void operator()() {
      while (mHelper->mRunning) {
        SensorDataReceiver::SensorData data;
        if (mHelper->mDataReceiver->waitForData(data)) {
          int64_t mapId = -1;
          if (mHelper->mChannelToMapBindings.size() > 0) {
            auto item = mHelper->mChannelToMapBindings.find(data.mChannel);
            if (item != mHelper->mChannelToMapBindings.end()) {
              mapId = item->second;
            }
          }
          mHelper->mMapManager->addData(*data.mPointSet, mapId);
          std::set<DataListener*>::const_iterator iter =
            mHelper->mDataListeners.begin();
          for (; iter != mHelper->mDataListeners.end(); ++iter) {
            (*iter)->notify(data);
          }
        }
      }
    }
  };

  double computeAngleFromHorizontal(const Eigen::Quaternionf& iQuat) {
    Eigen::Matrix3f rot = iQuat.matrix();
    double angle = atan2(rot(2,1), rot(2,2));
    if (angle < 0) angle += 2*kPi;
    return angle;
  }
};

const double Collector::Helper::kPi = 4*atan(1);

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

void Collector::
bind(const std::string& iChannel, const int64_t iMapId) {
  mHelper->mChannelToMapBindings[iChannel] = iMapId;
}

void Collector::
unbind(const std::string& iChannel) {
  auto item = mHelper->mChannelToMapBindings.find(iChannel);
  if (item != mHelper->mChannelToMapBindings.end()) {
    mHelper->mChannelToMapBindings.erase(item);
  }
}

void Collector::
unbindAll() {
  mHelper->mChannelToMapBindings.clear();
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
  if (pointSets.size() < 10) return false;

  // determine whether angle is increasing or decreasing
  double initAngle = mHelper->computeAngleFromHorizontal
    (pointSets.back().mCloud->sensor_orientation_);
  int counter = 0;
  bool increasing = false;
  for (auto iter = pointSets.rbegin(); iter != pointSets.rend(); ++iter) {
    double angle = mHelper->
      computeAngleFromHorizontal(iter->mCloud->sensor_orientation_);
    increasing = angle-initAngle < 0;
    ++counter;
    if (counter > 10) break;
  }
  double signVal = (increasing ? 1 : -1);

  double angleRange = (signVal)*(iMaxAngle - iMinAngle);
  double maxAngle = increasing ? iMaxAngle : iMinAngle;
  if (maxAngle >= Helper::kPi) {
    maxAngle -= int(maxAngle/Helper::kPi)*Helper::kPi;
  }

  // stage 0: not started; stage 1: pre-range; stage 2: in range
  int stage = 0;
  double prevAngle = 0;
  double accumAngle = 0;
  for (auto iter = pointSets.rbegin(); iter != pointSets.rend(); ++iter) {
    double angle = mHelper->
      computeAngleFromHorizontal(iter->mCloud->sensor_orientation_);
    if (stage == 0) {
      if (angle >= Helper::kPi) angle -= Helper::kPi;
      if (angle*signVal > maxAngle*signVal) stage = 1;
    }
    else if (stage == 1) {
      prevAngle = angle;
      if (angle >= Helper::kPi) angle -= Helper::kPi;
      if (angle*signVal <= maxAngle*signVal) {
        oStartTime = oEndTime = iter->mTimestamp;
        stage = 2;
      }
    }
    else {
      double deltaAngle = prevAngle-angle;
      if (deltaAngle > Helper::kPi) deltaAngle -= 2*Helper::kPi;
      else if (deltaAngle < -Helper::kPi) deltaAngle += 2*Helper::kPi;
      accumAngle += deltaAngle;
      if (accumAngle*signVal > angleRange*signVal) {
        stage = 3;
        break;
      }
      oStartTime = iter->mTimestamp;
      prevAngle = angle;
    }
  }

  return (stage == 3);
}
