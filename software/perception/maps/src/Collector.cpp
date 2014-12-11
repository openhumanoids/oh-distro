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
  std::unordered_map<std::string,std::vector<int64_t> > mChannelToMapBindings;

  static const double kPi;

  struct DataConsumer {
    Helper* mHelper;
    DataConsumer(Helper* iHelper) : mHelper(iHelper) {}
    void operator()() {
      while (mHelper->mRunning) {
        SensorDataReceiver::SensorData data;
        if (mHelper->mDataReceiver->waitForData(data)) {
          if (mHelper->mChannelToMapBindings.size() > 0) {
            auto item = mHelper->mChannelToMapBindings.find(data.mChannel);
            if (item != mHelper->mChannelToMapBindings.end()) {
              for (auto mapId : item->second) {
                if (data.mScan == NULL) {
                  mHelper->mMapManager->addData(*data.mPointSet, mapId);
                }
                else {
                  mHelper->mMapManager->addData(*data.mScan, mapId);
                }
              }
            }
          }
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
  mHelper->mChannelToMapBindings[iChannel].push_back(iMapId);
}

void Collector::
unbind(const std::string& iChannel, const int64_t iMapId) {
  auto item = mHelper->mChannelToMapBindings.find(iChannel);
  if (item != mHelper->mChannelToMapBindings.end()) {
    if (iMapId < 0) {
      mHelper->mChannelToMapBindings.erase(item);
    }
    else {
      auto& ids = item->second;
      ids.erase(std::remove(ids.begin(), ids.end(), iMapId), ids.end());
    }
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
               int64_t& oStartTime, int64_t& oEndTime,
               const bool iRelative) const {
  PointDataBuffer::Ptr buf = mHelper->mMapManager->getPointData();
  std::vector<PointSet> pointSets = buf->getAll();
  if (pointSets.size() < 10) return false;

  // determine whether angle is increasing or decreasing
  double initAngle = mHelper->computeAngleFromHorizontal
    (pointSets.back().mCloud->sensor_orientation_);
  bool increasing = false;
  {
    int counter = 0;
    double prevAngle = initAngle;
    double totalDelta = 0;
    for (auto iter = pointSets.rbegin(); iter != pointSets.rend(); ++iter) {
      double angle = mHelper->
        computeAngleFromHorizontal(iter->mCloud->sensor_orientation_);
      double delta = angle-prevAngle;
      if (delta > Helper::kPi) delta -= 2*Helper::kPi;
      else if (delta < -Helper::kPi) delta += 2*Helper::kPi;
      totalDelta += delta;
      prevAngle = angle;
      ++counter;
      if (counter > 10) break;
    }
    increasing = totalDelta<0;
  }

  double angleRange = iMaxAngle - iMinAngle;
  double maxAngle = iMaxAngle;
  if (iRelative) {
    maxAngle += initAngle;
  }
  Eigen::Vector2d maxLine(sin(maxAngle), -cos(maxAngle));

  // look for zero crossings of angle wrt max angle
  // stage 0: looking for zero crossing
  // stage 1: accumulating angle traveled
  // stage 2: done (travel >= delta angle)
  int stage = 0;
  double accumAngle = 0;
  bool first = true;
  Eigen::Vector2d prevVect;
  for (auto iter = pointSets.rbegin(); iter != pointSets.rend(); ++iter) {
    double angle = mHelper->
      computeAngleFromHorizontal(iter->mCloud->sensor_orientation_);
    Eigen::Vector2d vect(cos(angle),sin(angle));

    // check for zero crossing
    if (stage == 0) {
      double dot = maxLine.dot(vect);
      if (!first) {
        if (dot * maxLine.dot(prevVect) <= 0) {
          oStartTime = oEndTime = iter->mTimestamp;
          stage = 1;
        }
      }
      else first = false;
    }

    // count angle traveled
    else if (stage == 1) {
      double diffAngle = atan2(vect[1],vect[0]) -
        atan2(prevVect[1],prevVect[0]);
      if (diffAngle > Helper::kPi) diffAngle -= 2*Helper::kPi;
      else if (diffAngle < -Helper::kPi) diffAngle += 2*Helper::kPi;
      accumAngle += diffAngle;
      if (accumAngle*(increasing ? -1 : 1) >= angleRange) {
        stage = 2;
      }
      else oStartTime = iter->mTimestamp;
    }

    // done, exit loop
    else if (stage == 2) break;

    // update previous vector
    prevVect = vect;
  }

  // return true only if we got to stage 2
  return (stage == 2);
}
