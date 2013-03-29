#ifndef CONSTRAINTAPP_H
#define CONSTRAINTAPP_H

#include <boost/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <iostream>
#include <fstream>
#include <lcmtypes/drc_lcmtypes.h>
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

  static void AffordanceTrackCollectionHandlerAux(const lcm_recv_buf_t* rbuf,
						  const char* channel,
						  const drc_affordance_track_collection_t* msg,
						  void* user_data) {
    ((ConstraintApp*) user_data)->AffordanceTrackCollectionHandler(msg);
  }
  virtual void AffordanceTrackCollectionHandler(const drc_affordance_track_collection_t *msg) = 0;

  static void AffordanceFitHandlerAux(const lcm_recv_buf_t* rbuf,
				      const char* channel,
				      const drc_affordance_t* msg,
				      void* user_data) {
    ((ConstraintApp*) user_data)->AffordanceFitHandler(msg);
  }
  virtual void AffordanceFitHandler(const drc_affordance_t *msg) = 0;

 protected :
  bool shouldStop();
  void stopThreads();
  void main();
  KDL::Frame GetFrameFromParams(const drc_affordance_t *msg);
  KDL::Frame VectorToFrame(const std::vector<double>& state);
  std::vector<double> FrameToVector(const KDL::Frame& frame);
  int lcm_handle_timeout(lcm_t* lcm, int ms);

 protected :
  boost::thread* m_mainThread;
  volatile bool m_stopThreads;
  boost::mutex m_stopThreadsMutex;

  boost::mutex m_lcmMutex;
  lcm_t* m_lcm;

  std::ofstream m_log;
};

#endif
