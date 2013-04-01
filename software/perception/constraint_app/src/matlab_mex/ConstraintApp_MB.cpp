#include "ConstraintApp_MB.h"
#include <boost/format.hpp>

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

  if ( msg->uid != m_affordanceUID ) {
    m_log << "WARNING: received track collection for uid=" << msg->uid 
	  << ", but expecting for uid=" << m_affordanceUID << ".  ignoring tracks." << std::endl;
    return;
  }

  m_log << "got a new track collection for affordance with uid=" << msg->uid << std::endl;

  KDL::Tree& tree(m_affordance->GetTree());
  const KDL::SegmentMap& segmentMap(tree.getSegments());

  for ( int i = 0; i < msg->ntracks; i++ ) {
    const drc::affordance_track_t* track = &msg->tracks[i];
    m_log << "  track: segment=" << track->segment << ", id=" << track->id << std::endl;

    std::string trackSegmentName(boost::str(boost::format("%i") % track->id));
    KDL::Vector track_expressedIn_world(track->position.x, track->position.y, track->position.z);

    std::string parentSegmentName(track->segment); 
    KDL::SegmentMap::const_iterator parentIter(segmentMap.find(parentSegmentName));
    if ( parentIter == segmentMap.end() ) {
      m_log << "ERROR: an observation has a parent of " << parentSegmentName 
	    << ", but this parent could not be found in the tree, ignoring this observation" << std::endl;
      continue;
    }
    const KDL::Segment& parentSegment(parentIter->second.segment);
    
    KDL::SegmentMap::const_iterator obsIter(segmentMap.find(trackSegmentName));
    if ( obsIter == segmentMap.end() ) {
      //this is a new observation, determine where it is relative to the parent segment and add it to the tree
      KDL::Frame parent_expressedIn_world;
      if ( !m_affordance->GetSegmentExpressedInWorld(parentSegmentName, m_currentState, parent_expressedIn_world) ) {
	m_log << "ERROR: unable to decode current state. ignoring all observations." << std::endl;
	return;
      }
      KDL::Vector track_expressedIn_parent(parent_expressedIn_world.Inverse() * track_expressedIn_world);
      KDL::Segment newSegment(trackSegmentName, KDL::Joint(KDL::Joint::None),
			      KDL::Frame(track_expressedIn_parent) );
      tree.addSegment(newSegment, track->segment);

      m_log << "-----------------------------------" << std::endl
	    << "  adding segment \"" << trackSegmentName << "\" to the tree, with a parent of \"" 
	    << parentSegmentName << "\"" << std::endl
	    << "  parent_expressedIn_world: " << parent_expressedIn_world << std::endl
	    << "   track_expressedIn_world: " << track_expressedIn_world << std::endl
	    << "  track_expressedIn_parent: " << track_expressedIn_parent << std::endl
	    << "  new tree: " << std::endl;
      m_affordance->PrintKdlTree();
    } else {
      //this is an existing observation
      ObservationMap::iterator existingObs(m_currentObservations.find(track->id));
      Observation newobs(track_expressedIn_world);
      if ( existingObs == m_currentObservations.end() ) {
	//we have not yet observed this link, so add it
	m_currentObservations.insert(std::pair<int, Observation>(track->id, newobs));
	m_log << "  new observation for segment " << parentSegmentName << ", id=" << track->id << std::endl
	      << "    world pose: " << track_expressedIn_world << std::endl;
      } else {
	//we've observed, and have not yet processed. just update the observation
	existingObs->second = newobs;
	m_log << "  repeat observation for segment " << parentSegmentName << ", id=" << track->id << std::endl
	      << "    world pose: " << track_expressedIn_world << std::endl;
      }
    }
  }

  m_dataCondition.notify_all();
}

void ConstraintApp_MB::AffordanceFitHandler(const drc::affordance_t *msg)
{
  boost::mutex::scoped_lock lock(m_dataMutex);

  Affordance::Ptr msgAffordance(new Affordance(m_log, *msg));
  Affordance::StateVector msgState;
  if ( msgAffordance->GetStateFromMsg(*msg, msgState) ) {
    m_affordance = msgAffordance;
    m_affordanceUID = msg->uid;
    m_wasReset = true;
    m_currentState = msgState;
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

  m_log << "ConstraintApp_MB::GetObservations" << std::endl;

  actualObservations.clear();
  observationIds.clear();

  actualObservations.reserve(m_currentObservations.size()*3);
  observationIds.reserve(m_currentObservations.size());

  for ( ObservationMap::const_iterator iter = m_currentObservations.begin(); iter != m_currentObservations.end(); ++iter ) {
    actualObservations.push_back(iter->second.obs_expressedIn_world[0]);
    actualObservations.push_back(iter->second.obs_expressedIn_world[1]);
    actualObservations.push_back(iter->second.obs_expressedIn_world[2]);

    observationIds.push_back(iter->first);

    m_log << "   added observation for id " << iter->first << std::endl
	  << "      obs: " << iter->second.obs_expressedIn_world << std::endl;
  }

  m_currentObservations.clear();

  return true;
}

bool ConstraintApp_MB::GetExpectedObservations(const StateVector& state,
					       const IdVector& observationIds,
					       ObservationVector& observations)
{
  boost::mutex::scoped_lock lock(m_dataMutex);

  m_log << "ConstraintApp_MB::GetExpectedObservations" << std::endl;

  observations.clear();
  observations.reserve(observationIds.size()*3);

  KDL::Tree& tree(m_affordance->GetTree());
  const KDL::SegmentMap& segmentMap(tree.getSegments());

  for ( IdVector::const_iterator iter = observationIds.begin(); iter != observationIds.end(); ++iter ) {

    std::string trackSegmentName(boost::str(boost::format("%i") % *iter));

    KDL::SegmentMap::const_iterator obsIter(segmentMap.find(trackSegmentName));
    if ( obsIter == segmentMap.end() ) {
      m_log << "ERROR: unable to find corresponding segment for id " << *iter << std::endl;
      return false;
    }

    KDL::Frame track_expressedIn_world;
    if ( !m_affordance->GetSegmentExpressedInWorld(trackSegmentName, state, track_expressedIn_world) ) {
      m_log << "ERROR: unable to decode current state. not processing any further expected observations." << std::endl;
      return false;
    }

    //TODO: calculate the gradient

    observations.push_back(track_expressedIn_world.p[0]);
    observations.push_back(track_expressedIn_world.p[1]);
    observations.push_back(track_expressedIn_world.p[2]);

    m_log << "   added expected observation for " << trackSegmentName << std::endl
	  << "      exp: " << track_expressedIn_world.p << std::endl;
  }
  return true;
}

int ConstraintApp_MB::GetStateSize() 
{
  return m_currentState.size();
}
