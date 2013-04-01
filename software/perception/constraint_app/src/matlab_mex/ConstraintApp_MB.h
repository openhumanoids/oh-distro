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
#include "PointObservation.h"

class ConstraintApp_MB : public ConstraintApp
{
 public:
  typedef Affordance::StateVector StateVector;
  typedef std::vector<double> ObservationVector;
  typedef std::vector<int> IdVector;

  ConstraintApp_MB();
  virtual ~ConstraintApp_MB();

  virtual bool WaitForObservations(unsigned int timeout_ms);
  virtual bool GetObservations(ObservationVector& actualObservations,
			       IdVector& observationIds);
  virtual bool GetExpectedObservations(const StateVector& state,
				       const IdVector& observationIds,
				       ObservationVector& observations);
  virtual bool GetResetAndClear();
  virtual void GetCurrentStateEstimate(StateVector& state);
  virtual void SetCurrentStateEstimate(const StateVector& state);
  virtual void AffordanceTrackCollectionHandler(const drc::affordance_track_collection_t *msg);
  virtual void AffordanceFitHandler(const drc::affordance_t *msg);
  virtual int GetStateSize();

 protected:
  boost::mutex m_dataMutex;
  boost::condition_variable m_dataCondition;

  Affordance::Ptr m_affordance;
  int m_affordanceUID;
  bool m_wasReset;

  StateVector m_currentState;

  typedef PointObservation Observation;
  typedef std::map<int, Observation> ObservationMap;
  ObservationMap m_currentObservations;
};

#endif
