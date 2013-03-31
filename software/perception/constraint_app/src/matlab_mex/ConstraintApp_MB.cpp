#include "ConstraintApp_MB.h"

ConstraintApp_MB::ConstraintApp_MB() : ConstraintApp()
{
  m_log << "ConstraintApp_MB::ConstraintApp_MB: built at " << __DATE__ << " " << __TIME__ << std::endl;
}

ConstraintApp_MB::~ConstraintApp_MB() 
{
  m_log << "ConstraintApp_MB::~ConstraintApp_MB" << std::endl;
}

void ConstraintApp_MB::AffordanceTrackCollectionHandler(const drc::affordance_track_collection_t *msg)
{
  m_log << "got a new track collection for affordance with uid=" << msg->uid << std::endl;

  boost::mutex::scoped_lock lock(m_dataMutex);

  AffordancePtrMap::iterator iter = m_affordances.find(msg->uid);
  if ( iter == m_affordances.end() ) {
    m_log << "ERROR: received track points for an affordance uid=" << msg->uid 
	  << " for which no fit (model) has been received" << std::endl;
    return;
  }
  Affordance::Ptr affordance(iter->second);
  
  for ( int i = 0; i < msg->ntracks; i++ ) {
    const drc::affordance_track_t* track = &msg->tracks[i];
    m_log << "  track: segment=" << track->segment << ", id=" << track->id << std::endl;
  }

  //TODO: if a track has been seen before, add this as a new observation for that track

  //TODO: if a track has not been seen before, add a new link in the tree, relative to the sepcified segment

  //TODO: when a new affordance fit is received, it should get a new unique identifier (perhaps for refernce by the matlab code

  /*
  for ( int i = 0; i < msg->ntracks; i++ ) {
    drc_affordance_track_t* track = &msg->tracks[i];

    if ( track->segment == "" ) {
      m_log << "ERROR: received a segment name of \"" << track->segment 
	    << "\" but I can only handle a single segment, segment must be = \"\"" << std::endl;
      continue;
    }

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
  */
  /*
  m_log << "sent a new (empty) estimate of joint angles" << std::endl;
  drc_affordance_joint_angles_t aja;
  aja.utime = msg->utime;
  aja.uid = msg->uid;
  drc_affordance_joint_angles_t_publish(m_lcm, "AFFORDANCE_JOINT_ANGLES", &aja);
  */
}

void ConstraintApp_MB::AffordanceFitHandler(const drc::affordance_t *msg)
{
  boost::mutex::scoped_lock lock(m_dataMutex);

  AffordancePtrMap::iterator iter = m_affordances.find(msg->uid);

  if ( iter == m_affordances.end() ) {
    m_log << "got a new affordance fit" << std::endl;  
    //new afforandce
    m_affordances.insert(AffordancePtrMapPair(msg->uid, Affordance::Ptr(new Affordance(m_log, *msg))));
  } else {
    // a new fit for an existing affordance
    iter->second = Affordance::Ptr(new Affordance(m_log, *msg));
  }
}
