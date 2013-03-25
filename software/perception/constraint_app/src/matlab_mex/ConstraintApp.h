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
#include "PointObservation.h"

class ConstraintApp
{
public:
  typedef std::vector<double> JointVector;
  class Configuration {
  public:
    KDL::Frame base_expressedIn_world;
    JointVector joints;
  };
  class Link {
  public:
  Link(const KDL::Vector& lb, const int _id) : link_expressedIn_base(lb), id(_id) {}
    KDL::Vector link_expressedIn_base;
    int id;
  };
  typedef std::map<std::string, Link> LinkMap;
  typedef PointObservation Observation;
  typedef std::map<std::string, Observation> ObservationMap;

  ConstraintApp();
  virtual ~ConstraintApp();

  bool shouldStop() {
    bool ret;
    {
      boost::mutex::scoped_lock lock(m_stopThreadsMutex);
      ret = m_stopThreads;
    }
    return ret;
  }

  void stopThreads() {
    {
      boost::mutex::scoped_lock lock(m_stopThreadsMutex);
      m_stopThreads = true;
    }
    m_mainThread->join();
  }

  int getCounter() {
    boost::mutex::scoped_lock lock(m_counterMutex);
    return m_counter;
  }

  bool WaitForObservations(unsigned int timeout_ms);
  bool GetObservations(std::vector<double>& actualObservations,
		       std::vector<int>& observationIds);
  bool GetExpectedObservations(const std::vector<double>& state,
			       const std::vector<int>& observationIds,
			       std::vector<double>& observations);
  bool GetResetAndClear();
  void GetCurrentStateEstimate(std::vector<double>& state);
  void SetCurrentStateEstimate(const std::vector<double>& state);

  static void AffordanceTrackCollectionHandlerAux(const lcm_recv_buf_t* rbuf,
						      const char* channel,
						      const drc_affordance_track_collection_t* msg,
						      void* user_data) {
    ((ConstraintApp*) user_data)->AffordanceTrackCollectionHandler(msg);
  }
  void AffordanceTrackCollectionHandler(const drc_affordance_track_collection_t *msg);

  static void AffordanceFitHandlerAux(const lcm_recv_buf_t* rbuf,
				      const char* channel,
				      const drc_affordance_t* msg,
				      void* user_data) {
    ((ConstraintApp*) user_data)->AffordanceFitHandler(msg);
  }
  void AffordanceFitHandler(const drc_affordance_t *msg);

protected:
  boost::thread* m_mainThread;
  volatile bool m_stopThreads;
  boost::mutex m_stopThreadsMutex;

  boost::mutex m_counterMutex;
  volatile int m_counter;

  boost::mutex m_lcmMutex;
  lcm_t* m_lcm;

  std::ofstream m_log;

  boost::mutex m_dataMutex;
  boost::condition_variable m_dataCondition;
  Configuration m_currentEstimate;
  LinkMap m_currentLinks;
  ObservationMap m_currentObservations;
  bool m_wasReset;

  int m_nextLinkId;

 protected:
  void main();
  int lcm_handle_timeout(lcm_t* lcm, int ms);
  KDL::Frame GetFrameFromParams(const drc_affordance_t *msg);
  KDL::Frame VectorToFrame(const std::vector<double>& state);
  std::vector<double> FrameToVector(const KDL::Frame& frame);
};

#endif
