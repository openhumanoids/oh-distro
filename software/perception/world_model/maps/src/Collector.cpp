#include "Collector.hpp"

#include "SensorDataReceiver.hpp"
#include "MapManager.hpp"
#include "PointDataBuffer.hpp"

#include <boost/thread.hpp>

using namespace maps;

struct Collector::Helper {
  boost::shared_ptr<lcm::LCM> mLcm;
  boost::shared_ptr<SensorDataReceiver> mDataReceiver;
  boost::shared_ptr<MapManager> mMapManager;
  boost::thread mConsumerThread;
  bool mRunning;

  static const float kPi;

  struct DataConsumer {
    Helper* mHelper;
    DataConsumer(Helper* iHelper) : mHelper(iHelper) {}
    void operator()() {
      while (mHelper->mRunning) {
        SensorDataReceiver::SensorData data;
        if (mHelper->mDataReceiver->waitForData(data)) {
          mHelper->mMapManager->addData(*data.mPointSet);
        }
      }
    }
  };

  float computeAngleFromHorizontal(const Eigen::Quaternionf& iQuat) {
    Eigen::Matrix3f rot = iQuat.matrix();
    Eigen::Matrix3f rotNew;
    rotNew.col(2) = rot.col(0);
    rotNew.col(0) = Eigen::Vector3f::UnitZ().cross(rot.col(0));
    rotNew.col(0).normalize();
    rotNew.col(1) = rotNew.col(2).cross(rotNew.col(0));
    rotNew.col(1).normalize();
    Eigen::Vector3f v = rotNew.transpose()*rot.col(1);
    float angle = atan2(v(1),v(0));
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
  stop();
}

void Collector::
setLcm(const boost::shared_ptr<lcm::LCM>& iLcm) {
  mHelper->mLcm = iLcm;
  mHelper->mDataReceiver->setLcm(iLcm);
}  

bool Collector::
start() {
  if (mHelper->mRunning) return false;
  mHelper->mRunning = true;
  mHelper->mDataReceiver->start();
  mHelper->mConsumerThread =
    boost::thread(Helper::DataConsumer(mHelper.get()));
  return true;
}

bool Collector::
stop() {
  if (!mHelper->mRunning) return false;
  mHelper->mRunning = false;
  mHelper->mDataReceiver->stop();
  mHelper->mDataReceiver->unblock();
  try { mHelper->mConsumerThread.join(); }
  catch (const boost::thread_interrupted&) {}
  return true;
}

boost::shared_ptr<SensorDataReceiver> Collector::
getDataReceiver() const {
  return mHelper->mDataReceiver;
}

boost::shared_ptr<MapManager> Collector::
getMapManager() const {
  return mHelper->mMapManager;
}

bool Collector::
getLatestFullSweep(int64_t& oStartTime, int64_t& oEndTime) const {
  PointDataBuffer::Ptr buf = mHelper->mMapManager->getPointData();
  std::vector<PointSet> pointSets = buf->getAll();
  if (pointSets.size() < 5) return false;

  const PointSet& lastPointSet = pointSets.back();
  float lastAngle = mHelper->
    computeAngleFromHorizontal(lastPointSet.mCloud->sensor_orientation_);
  float prevAngle = mHelper->
    computeAngleFromHorizontal
    (pointSets[pointSets.size()-5].mCloud->sensor_orientation_);
  float delta = lastAngle-prevAngle;
  float spinSign = (((delta < 0) && (-delta < Helper::kPi)) ||
                    ((delta > 0) && (delta > Helper::kPi))) ? -1 : 1;
  
  oStartTime = oEndTime = lastPointSet.mTimestamp;

  for (std::vector<PointSet>::const_reverse_iterator iter = pointSets.rbegin();
       iter != pointSets.rend(); ++iter) {
    float angle = mHelper->
      computeAngleFromHorizontal(iter->mCloud->sensor_orientation_);
    float angleDiff = spinSign*(lastAngle-angle);

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

  bool started = false;
  bool wasEverInside = false;
  for (std::vector<PointSet>::const_reverse_iterator iter = pointSets.rbegin();
       iter != pointSets.rend(); ++iter) {
    float angle = mHelper->
      computeAngleFromHorizontal(iter->mCloud->sensor_orientation_);
    if (angle > Helper::kPi) angle -= Helper::kPi;
    bool inside = (angle >= iMinAngle) && (angle <= iMaxAngle);

    if (!started) {
      if (inside) continue;
      started = true;
    }
    if (inside) {
      oStartTime = iter->mTimestamp;
      wasEverInside = true;
    }
    else {
      if (wasEverInside) break;
      oEndTime = iter->mTimestamp;
    }
  }

  return true;
}
