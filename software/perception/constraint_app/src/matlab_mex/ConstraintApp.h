#ifndef CONSTRAINTAPP_H
#define CONSTRAINTAPP_H

#include <boost/thread.hpp>
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
    KDL::Frame link_expresedIn_base;
  };
  typedef std::map<std::string, Link> LinkMap;

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

  Configuration m_currentEstimate;
  LinkMap m_currentLinks;

 protected:
  void main();
  int lcm_handle_timeout(lcm_t* lcm, int ms);
  KDL::Frame GetFrameFromParams(const drc_affordance_t *msg);
};

#endif
