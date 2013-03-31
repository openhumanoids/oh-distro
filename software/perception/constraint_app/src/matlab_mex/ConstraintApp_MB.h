#ifndef CONSTRAINTAPP_MB_H
#define CONSTRAINTAPP_MB_H

#include <boost/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <iostream>
#include <fstream>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <forward_kinematics/treefksolverposfull_recursive.hpp>
#include <map>
#include <vector>
#include <string>
#include "ConstraintApp.h"
#include "Affordance.h"

class ConstraintApp_MB : public ConstraintApp
{
 public:
  ConstraintApp_MB();
  virtual ~ConstraintApp_MB();

  virtual bool WaitForObservations(unsigned int timeout_ms) { return true; }
  virtual bool GetObservations(std::vector<double>& actualObservations,
			       std::vector<int>& observationIds) { return true; }
  virtual bool GetExpectedObservations(const std::vector<double>& state,
				       const std::vector<int>& observationIds,
				       std::vector<double>& observations) { return true; }
  virtual bool GetResetAndClear() { return false; }
  virtual void GetCurrentStateEstimate(std::vector<double>& state) {}
  virtual void SetCurrentStateEstimate(const std::vector<double>& state) {}
  virtual void AffordanceTrackCollectionHandler(const drc::affordance_track_collection_t *msg);
  virtual void AffordanceFitHandler(const drc::affordance_t *msg);

 protected:
  boost::mutex m_dataMutex;

  typedef std::pair<int, Affordance::Ptr> AffordancePtrMapPair;
  typedef std::map<int, Affordance::Ptr> AffordancePtrMap;
  AffordancePtrMap m_affordances;

};

#endif
