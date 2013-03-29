#ifndef CONSTRAINTAPP_RB_UKF_H
#define CONSTRAINTAPP_RB_UKF_H

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
#include "ConstraintApp.h"

class ConstraintApp_RB_UKF : public ConstraintApp
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
  typedef std::map<int, Link> LinkMap;
  typedef PointObservation Observation;
  typedef std::map<int, Observation> ObservationMap;

  ConstraintApp_RB_UKF();
  virtual ~ConstraintApp_RB_UKF();

  virtual bool WaitForObservations(unsigned int timeout_ms);
  virtual bool GetObservations(std::vector<double>& actualObservations,
			       std::vector<int>& observationIds);
  virtual bool GetExpectedObservations(const std::vector<double>& state,
				       const std::vector<int>& observationIds,
				       std::vector<double>& observations);
  virtual bool GetResetAndClear();
  virtual void GetCurrentStateEstimate(std::vector<double>& state);
  virtual void SetCurrentStateEstimate(const std::vector<double>& state);

  virtual void AffordanceTrackCollectionHandler(const drc_affordance_track_collection_t *msg);
  virtual void AffordanceFitHandler(const drc_affordance_t *msg);

 protected:
  boost::mutex m_dataMutex;
  boost::condition_variable m_dataCondition;
  Configuration m_currentEstimate;
  LinkMap m_currentLinks;
  ObservationMap m_currentObservations;
  bool m_wasReset;

  int m_nextLinkId;
};

#endif
