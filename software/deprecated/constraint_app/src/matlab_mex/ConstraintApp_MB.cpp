#include "ConstraintApp_MB.h"
#include <boost/format.hpp>
#include "mytreefksolverpos_recursive.hpp"

ConstraintApp_MB::ConstraintApp_MB() : ConstraintApp(), m_wasReset(true)
{
  m_log << "ConstraintApp_MB::ConstraintApp_MB: built at " << __DATE__ << " " << __TIME__ << std::endl;
}

ConstraintApp_MB::~ConstraintApp_MB() 
{
  m_log << "ConstraintApp_MB::~ConstraintApp_MB" << std::endl;
}

void ConstraintApp_MB::AffordanceTrackCollectionHandler(const drc::affordance_track_collection_t *msg)
{
  boost::mutex::scoped_lock lock(m_dataMutex);

  /*
  if ( msg->uid != m_affordanceUID ) {
    m_log << "WARNING: received track collection for uid=" << msg->uid 
	  << ", but expecting for uid=" << m_affordanceUID 
	  << ".  ignoring tracks." << std::endl;
    return;
  }
  */

  m_log << "got a new track collection for affordance with uid=" << msg->uid << std::endl;

  KDL::Tree& tree(m_affordance->GetTree());
  const KDL::SegmentMap& segmentMap(tree.getSegments());

  for ( int i = 0; i < msg->ntracks; i++ ) {
    const drc::affordance_track_t* track = &msg->tracks[i];
    m_log << "  track: segment=" << track->segment << ", id=" << track->id << std::endl;

    KDL::Vector track_expressedIn_world(track->position.x, track->position.y, 
					track->position.z);

    std::string segmentName(track->segment); 
    KDL::SegmentMap::const_iterator segmentIter(segmentMap.find(segmentName));
    if ( segmentIter == segmentMap.end() ) {
      m_log << "ERROR: " << segmentName << " was not found in tree" << std::endl;
      continue;
    }
    const KDL::Segment& parentSegment(segmentIter->second.segment);
    
    //this is an existing observation
    ObservationMap::iterator existingObs(m_currentObservations.find(segmentName));
    Observation newobs(track_expressedIn_world);
    if ( existingObs == m_currentObservations.end() ) {
      //we have not yet observed this link, so add it
      m_currentObservations.insert(std::pair<std::string, Observation>(segmentName, newobs));
      m_log << "  new observation for " << segmentName << std::endl
	    << "    world pose: " << track_expressedIn_world << std::endl;
    } else {
      //we've observed, and have not yet processed. just update the observation
      existingObs->second = newobs;
      m_log << "  repeat observation for " << segmentName << std::endl
	    << "    world pose: " << track_expressedIn_world << std::endl;
    }
  }

  m_dataCondition.notify_all();
}

void ConstraintApp_MB::AffordanceFitHandler(const drc::affordance_plus_t *msg)
{
  boost::mutex::scoped_lock lock(m_dataMutex);

  Affordance::Ptr msgAffordance(new Affordance(msg->aff.otdf_type, m_log));
  Affordance::StateVector msgState;
  if ( msgAffordance->GetStateFromMsg(msg->aff, msgState) ) {
    m_affordance = msgAffordance;
    m_fastFKSolver = boost::shared_ptr<FastFKSolver>(new FastFKSolver(m_affordance->m_tree));
    m_affordanceUID = msg->aff.uid;
    m_wasReset = true;
    m_currentState = msgState;
    m_prevFit = *msg; //this is a deep copy in c++ version of lcm
    m_log << "ConstraintApp_MB::AffordanceFitHandler: updated new affordance and state" << std::endl;
    m_affordance->PrintState(msgState);
  } else {
    m_log << "ConstraintApp_MB::AffordanceFitHandler: unable to create new afforance due to invalid state" << std::endl;
  }
}

bool ConstraintApp_MB::GetResetAndClear()
{
  boost::mutex::scoped_lock lock(m_dataMutex);
  bool ret = m_wasReset;
  m_wasReset = false;
  return ret;
}

void ConstraintApp_MB::GetCurrentStateEstimate(StateVector& state) 
{ 
  boost::mutex::scoped_lock lock(m_dataMutex);
  state = m_currentState;
}

void ConstraintApp_MB::SetCurrentStateEstimate(const StateVector& state)
{
  boost::mutex::scoped_lock lock(m_dataMutex);
  m_currentState = state;

  PublishFitMessage();
}

bool ConstraintApp_MB::WaitForObservations(unsigned int timeout_ms)
{
  m_log << "ConstraintApp_MB::WaitForObservations, timeout = " << timeout_ms << std::endl;

  boost::mutex::scoped_lock lock(m_dataMutex);

  if ( !m_currentObservations.empty() ) return true;

  boost::system_time timeout = boost::get_system_time() + boost::posix_time::milliseconds(timeout_ms);
  m_dataCondition.timed_wait(lock, timeout);

  return !m_currentObservations.empty();
}

bool ConstraintApp_MB::GetObservations(ObservationVector& actualObservations,
				       IdVector& observationIds)
{
  boost::mutex::scoped_lock lock(m_dataMutex);

  //m_log << "ConstraintApp_MB::GetObservations" << std::endl;

  actualObservations.clear();
  observationIds.clear();

  actualObservations.reserve(m_currentObservations.size()*3);
  observationIds.reserve(m_currentObservations.size());

  for ( ObservationMap::const_iterator iter = m_currentObservations.begin(); 
	iter != m_currentObservations.end(); ++iter ) {
    actualObservations.push_back(iter->second.obs_expressedIn_world[0]);
    actualObservations.push_back(iter->second.obs_expressedIn_world[1]);
    actualObservations.push_back(iter->second.obs_expressedIn_world[2]);

    observationIds.push_back(iter->first);

    //m_log << "   added observation for id " << iter->first << std::endl
    //	  << "      obs: " << iter->second.obs_expressedIn_world << std::endl;
  }

  m_currentObservations.clear();

  return true;
}

/*
bool ConstraintApp_MB::GetExpectedObservations(const StateVector& state,
					       const IdVector& observationIds,
					       ObservationVector& observations)
{
  boost::mutex::scoped_lock lock(m_dataMutex);

  m_log << "ConstraintApp_MB::GetExpectedObservations" << std::endl;

  observations.clear();
  observations.reserve(observationIds.size()*3);

  KDL::JntArray joints(state.size());
  m_affordance->DecodeState(state, joints);

  MyTreeFkSolverPos_recursive fk(m_affordance->m_tree);
  MyTreeFkSolverPos_recursive::SegmentToPoseMap map;
  int fkret = fk.JntToCart(joints, map);
  if ( fkret < 0 ) {
    std::cout << "ConstraintApp_HCA::GetExpectedObservations: error while getting forward kinematics." << std::endl;
    return false;
  }

  for ( int i = 0; i < observationIds.size(); i++ ) {
    MyTreeFkSolverPos_recursive::SegmentToPoseMap::const_iterator iter = map.find(observationIds[i]);
    if ( iter == map.end() ) {
      std::cout << "ConstraintApp_HCA::GetExpectedObservations: unable to find pose for segment " 
		<< observationIds[i] << std::endl;
      return false;
    } 
    const KDL::Frame& frame(iter->second);

    observations.push_back(frame.p[0]);
    observations.push_back(frame.p[1]);
    observations.push_back(frame.p[2]);
  }

  return true;
}
*/

bool ConstraintApp_MB::GetExpectedObservations(const StateVector& state,
					       const IdVector& observationIds,
					       ObservationVector& observations)
{
  boost::mutex::scoped_lock lock(m_dataMutex);

  //m_log << "ConstraintApp_MB::GetExpectedObservations" << std::endl;

  observations.clear();
  observations.reserve(observationIds.size()*3);

  KDL::JntArray joints(state.size());
  m_affordance->DecodeState(state, joints);
  m_fastFKSolver->setJointPositions(joints);

  //KDL::Tree& tree(m_affordance->GetTree());
  //const KDL::SegmentMap& segmentMap(tree.getSegments());

  for ( IdVector::const_iterator iter = observationIds.begin(); 
	iter != observationIds.end(); ++iter ) {

    std::string trackSegmentName(*iter);
    /*
    KDL::SegmentMap::const_iterator obsIter(segmentMap.find(trackSegmentName));
    if ( obsIter == segmentMap.end() ) {
      m_log << "ERROR: unable to find corresponding segment: " << *iter << std::endl;
      return false;
    }

    KDL::Frame track_expressedIn_world;
    if ( !m_affordance->GetSegmentExpressedInWorld(trackSegmentName, state, 
						   track_expressedIn_world) ) {
      m_log << "ERROR: unable to decode current state. not processing any further expected observations." << std::endl;
      return false;
    }
    */
    KDL::Frame track_expressedIn_world;
    if ( !m_fastFKSolver->getFrame(*iter, track_expressedIn_world) ) {
      m_log << "ERROR: unable to decode current state. not processing any further expected observations." << std::endl;
      return false;
    }

    /*
    m_log << "old_output: " << track_expressedIn_world << std::endl
	  << "new_output: " << track_expressedIn_world2 << std::endl;
    */

    observations.push_back(track_expressedIn_world.p[0]);
    observations.push_back(track_expressedIn_world.p[1]);
    observations.push_back(track_expressedIn_world.p[2]);

    /*
    m_log << "   added expected observation for " << trackSegmentName << std::endl
	  << "      exp: " << track_expressedIn_world.p << std::endl;
    */
  }
  return true;
}

int ConstraintApp_MB::GetStateSize() 
{
  return m_currentState.size();
}

void ConstraintApp_MB::PublishFitMessage()
{
  drc::affordance_plus_t newMsg = m_prevFit;
  drc::affordance_t& affordance(newMsg.aff);

  affordance.aff_store_control = drc::affordance_t::UPDATE;
  affordance.origin_xyz[0] = m_currentState[0];
  affordance.origin_xyz[1] = m_currentState[1];
  affordance.origin_xyz[2] = m_currentState[2];
  affordance.origin_rpy[0] = m_currentState[3];
  affordance.origin_rpy[1] = m_currentState[4];
  affordance.origin_rpy[2] = m_currentState[5];

  std::vector<std::string> jointNames;
  m_affordance->GetJointNames(jointNames);

  assert(affordance.nstates == jointNames.size()-6);
  
  for(int i = 6; i < jointNames.size(); i++ ) {
    affordance.states[i-6] = m_currentState[i];
    affordance.state_names[i-6] = jointNames[i];
  }

  m_lcm->publish("AFFORDANCE_FIT", &newMsg);
}

bool ConstraintApp_MB::GetJacobian(const StateVector& state, 
				   const IdVector& observationIds,
				   ConstraintApp_MB::Jacobian& jacobian, 
				   int method /*= 1*/)
{
  const StateVector& eulerState(state);

  // the KDL jacobian is (observation twists) x (delta euler state & delta joint angles)
  // our Jacobiain is (observation positions) x (delta euler state & delta joint angles)
  jacobian.resize(observationIds.size()*3, eulerState.size()); 

  for ( int i = 0; i < observationIds.size(); i++ ) {
    KDL::Jacobian thisObservationJacobian;
    if ( method == 1 ) {
      if ( !m_affordance->GetSegmentJacobianExpressedInWorld(observationIds[i],
							    eulerState,
							    thisObservationJacobian) ) {
	std::cout << "ERROR: GetJacobian: unable to find " 
		  << observationIds[i] << " segment" << std::endl;
	return false;
      }
    } else if ( method == 2 ) {
      if ( !m_affordance->GetSegmentJacobianExpressedInWorld2(observationIds[i],
							     eulerState,
							     thisObservationJacobian) ) {
	std::cout << "ERROR: GetJacobian: unable to find " 
		  << observationIds[i] << " segment" << std::endl;
	return false;
      }
    } else {
      std::cout << "ERROR: GetJacobian: invalid method=" << method << std::endl;
      return false;
    }

    //the first 3 rows contain the position jacobian
    jacobian.middleRows<3>(i*3) = thisObservationJacobian.data.topRows(3);
  }

  return true;
}
