#ifndef CONSTRAINTAPP_H
#define CONSTRAINTAPP_H

#include <boost/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <iostream>
#include <fstream>
//#include <lcmtypes/drc_lcmtypes.h>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <forward_kinematics/treefksolverposfull_recursive.hpp>
#include <map>
#include <vector>
#include <string>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <boost/thread/thread_time.hpp>
#include <boost/optional.hpp>

class ConstraintApp
{
 public :

  ConstraintApp();
  virtual ~ConstraintApp();

  virtual bool WaitForObservations(unsigned int timeout_ms) = 0;
  virtual bool GetObservations(std::vector<double>& actualObservations,
			       std::vector<int>& observationIds) = 0;
  virtual bool GetExpectedObservations(const std::vector<double>& state,
				       const std::vector<int>& observationIds,
				       std::vector<double>& observations) = 0;
  virtual bool GetResetAndClear() = 0;
  virtual void GetCurrentStateEstimate(std::vector<double>& state) = 0;
  virtual void SetCurrentStateEstimate(const std::vector<double>& state) = 0;

  virtual void AffordanceTrackCollectionHandlerAux(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
						   const drc::affordance_track_collection_t *tracks) {
    AffordanceTrackCollectionHandler(tracks);
  }
  virtual void AffordanceTrackCollectionHandler(const drc::affordance_track_collection_t *affordance) = 0;

  virtual void AffordanceFitHandlerAux(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
				       const drc::affordance_t *affordance) {
    AffordanceFitHandler(affordance);
  }
  virtual void AffordanceFitHandler(const drc::affordance_t *affordance) = 0;

  typedef boost::optional<KDL::Frame> OptionalKDLFrame;
  static OptionalKDLFrame GetFrameFromParams(const drc::affordance_t *msg);
  static std::vector<double> FrameToVector(const KDL::Frame& frame);

 protected :
  bool shouldStop();
  void stopThreads();
  void main();
  KDL::Frame VectorToFrame(const std::vector<double>& state);  
  int lcm_handle_timeout(lcm_t* lcm, int ms);

 protected :
  boost::thread* m_mainThread;
  volatile bool m_stopThreads;
  boost::mutex m_stopThreadsMutex;

  boost::mutex m_lcmMutex;
  //lcm_t* m_lcm;
  lcm::LCM* m_lcm;

  std::ofstream m_log;
};

#endif
