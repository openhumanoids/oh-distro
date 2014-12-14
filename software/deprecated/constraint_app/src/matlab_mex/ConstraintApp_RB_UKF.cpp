#include "ConstraintApp_RB_UKF.h"

//input: "AFFORDANCE_TRACK_COLLECTION" has incoming 3d points from sudeep
//       includes a "segment" string which is the "id" and a 3d point to track for each "affordance"

//input: "AFFORDANCE_FIT" a new affordance_t() type. this includes (x,y,z,r,p,w) as params.  
//       might contain other joints too, but we don't track them yet.  when this arrives, it's a reset to our system.

//output: "AFFORDANCE_JOINT_ANGLES" affordance_t() type.  our best estimate of the (x,y,z,r,p,w)

// when a new AFFORDANCE_FIT arrives, remove all existing relative poses.
// when a AFFORDANCE_TRACK_COLLECTION arrives...
//    if a track has been seen before, then this is a new observation
//    if a track has not been seen before, then the position of this (x,y,z) relative to the current base pose is the new pose used for expected observations

// when a AFFORDANCE_TRACK_COLLECTION
//    add noise to the current best estimate of pose
//    perform the update step.  use the relative poses as expected observations, use new data as new observations.


ConstraintApp_RB_UKF::ConstraintApp_RB_UKF() : ConstraintApp(), m_wasReset(true), m_nextLinkId(0)
{
}

ConstraintApp_RB_UKF::~ConstraintApp_RB_UKF() 
{
  m_log << "ConstraintApp_RB_UKF::~ConstraintApp_RB_UKF" << std::endl;
}

void ConstraintApp_RB_UKF::AffordanceTrackCollectionHandler(const drc::affordance_track_collection_t *msg)
{
  m_log << "got a new track collection" << std::endl;

  boost::mutex::scoped_lock lock(m_dataMutex);

  for ( int i = 0; i < msg->ntracks; i++ ) {
    const drc::affordance_track_t* track = &msg->tracks[i];

    int trackId(track->id);
    KDL::Vector track_expressedIn_world(track->position.x, track->position.y, track->position.z);
    KDL::Vector track_expressedIn_base(m_currentEstimate.base_expressedIn_world.Inverse() * track_expressedIn_world);
    
    if ( m_currentLinks.find(trackId) == m_currentLinks.end() ) {
      //this track is a new one
      m_currentLinks.insert(std::pair<int, Link>(trackId, Link(track_expressedIn_base, m_nextLinkId++)));
      m_log << "  creating a new link for id \"" << trackId << "\"" << std::endl
	    << "    world pose: " << track_expressedIn_world << std::endl
	    << "     base pose: " << track_expressedIn_base << std::endl;
    } else {
      //this track is already known, just add this report as an observation
      ObservationMap::iterator existingObs(m_currentObservations.find(trackId));
      Observation newobs(track_expressedIn_world);
      if ( existingObs == m_currentObservations.end() ) {
	//we have not yet observed this link, so add it
	m_currentObservations.insert(std::pair<int, Observation>(trackId, newobs));
	m_log << "  new observation for segment \"" << trackId << "\"" << std::endl
	      << "    world pose: " << track_expressedIn_world << std::endl;
      } else {
	//we've observed, and have not yet processed. just update the observation
	existingObs->second = newobs;
	m_log << "  repeat observation for id \"" << trackId << "\"" << std::endl
	      << "    world pose: " << track_expressedIn_world << std::endl;
      }
    }
  }

  m_dataCondition.notify_all();

  /*
  m_log << "sent a new (empty) estimate of joint angles" << std::endl;
  drc_affordance_joint_angles_t aja;
  aja.utime = msg->utime;
  aja.uid = msg->uid;
  drc_affordance_joint_angles_t_publish(m_lcm, "AFFORDANCE_JOINT_ANGLES", &aja);
  */
}

void ConstraintApp_RB_UKF::AffordanceFitHandler(const drc::affordance_t *msg)
{
  m_log << "got a new affordance fit" << std::endl;

  boost::mutex::scoped_lock lock(m_dataMutex);
  boost::optional<KDL::Frame> msgFrame(GetFrameFromParams(msg));
  if ( !msgFrame ) {
    m_log << "ERROR: unable to extract base pose from msg" << std::endl;
    return;
  }
  m_currentEstimate.base_expressedIn_world = *msgFrame;
  m_currentLinks.clear();
  m_currentObservations.clear();
  m_wasReset = true;

  m_log << "  reset the base pose to " << std::endl
	<< m_currentEstimate.base_expressedIn_world << std::endl;
}

bool ConstraintApp_RB_UKF::WaitForObservations(unsigned int timeout_ms)
{
  m_log << "ConstraintApp_RB_UKF::WaitForObservations, timeout = " << timeout_ms << std::endl;

  boost::mutex::scoped_lock lock(m_dataMutex);

  if ( !m_currentObservations.empty() ) return true;

  boost::system_time timeout = boost::get_system_time() + boost::posix_time::milliseconds(timeout_ms);
  m_dataCondition.timed_wait(lock, timeout);

  return !m_currentObservations.empty();
}

bool ConstraintApp_RB_UKF::GetExpectedObservations(const std::vector<double>& state,
						   const std::vector<int>& observationIds,
						   std::vector<double>& observations)
{
  boost::mutex::scoped_lock lock(m_dataMutex);

  m_log << "ConstraintApp_RB_UKF::GetExpectedObservations" << std::endl;

  observations.clear();
  observations.reserve(observationIds.size()*3);

  for ( std::vector<int>::const_iterator iter = observationIds.begin(); iter != observationIds.end(); ++iter ) {

    //find the id; TODO: need to make this a search tree
    LinkMap::const_iterator linkIter;
    for ( linkIter = m_currentLinks.begin(); linkIter != m_currentLinks.end(); ++linkIter ) {
      if ( linkIter->second.id == *iter ) break;
    }

    if ( linkIter == m_currentLinks.end() ) {
      m_log << "ERROR: unable to find corresponding link " << *iter;
      return false;
    }

    KDL::Frame base_expressedIn_world(VectorToFrame(state));
    KDL::Vector expected = base_expressedIn_world * linkIter->second.link_expressedIn_base;
    observations.push_back(expected[0]);
    observations.push_back(expected[1]);
    observations.push_back(expected[2]);

    m_log << "   added expected observation for " << linkIter->first << ", id=" << *iter << std::endl
	  << "      exp: " << expected << std::endl;
  }
  return true;
}

bool ConstraintApp_RB_UKF::GetObservations(std::vector<double>& actualObservations,
					   std::vector<int>& observationIds)
{
  boost::mutex::scoped_lock lock(m_dataMutex);

  m_log << "ConstraintApp_RB_UKF::GetObservations" << std::endl;

  actualObservations.clear();
  observationIds.clear();

  actualObservations.reserve(m_currentObservations.size()*3);
  observationIds.reserve(m_currentObservations.size());

  for ( ObservationMap::const_iterator iter = m_currentObservations.begin(); iter != m_currentObservations.end(); ++iter ) {
    actualObservations.push_back(iter->second.obs_expressedIn_world[0]);
    actualObservations.push_back(iter->second.obs_expressedIn_world[1]);
    actualObservations.push_back(iter->second.obs_expressedIn_world[2]);

    LinkMap::const_iterator linkIter = m_currentLinks.find(iter->first);
    if ( linkIter == m_currentLinks.end() ) {
      m_log << "ERROR: unable to find a corresponding link for observation " << iter->first << std::endl;
      return false;
    }

    observationIds.push_back(linkIter->second.id);

    m_log << "   added observation for id " << iter->first << std::endl
	  << "      obs: " << iter->second.obs_expressedIn_world << std::endl
	  << "    my id: " << linkIter->second.id << std::endl;
  }

  m_currentObservations.clear();

  return true;
}

bool ConstraintApp_RB_UKF::GetResetAndClear()
{
  boost::mutex::scoped_lock lock(m_dataMutex);
  bool ret = m_wasReset;
  m_wasReset = false;
  return ret;
}

void ConstraintApp_RB_UKF::GetCurrentStateEstimate(std::vector<double>& state)
{
  boost::mutex::scoped_lock lock(m_dataMutex);
  state = FrameToVector(m_currentEstimate.base_expressedIn_world);
}

void ConstraintApp_RB_UKF::SetCurrentStateEstimate(const std::vector<double>& state)
{
  boost::mutex::scoped_lock lock(m_dataMutex);
  m_currentEstimate.base_expressedIn_world = VectorToFrame(state);
}
