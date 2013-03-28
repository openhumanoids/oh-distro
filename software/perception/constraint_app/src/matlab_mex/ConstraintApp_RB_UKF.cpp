#include "ConstraintApp_RB_UKF.h"
#include <stdio.h>
#include <inttypes.h>
#include <sys/select.h>
typedef int SOCKET;
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <boost/thread/thread_time.hpp>

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

//NB: use tail -f cam.log to view output from the ConstraintApp_RB_UKF::main thread
ConstraintApp_RB_UKF::ConstraintApp_RB_UKF() : m_stopThreads(false), m_counter(0), 
				 m_log("/tmp/cam.log", std::ios::trunc|std::ios::binary),  
				 m_wasReset(true), m_nextLinkId(0)
{
  
  m_lcm = lcm_create(NULL);

  drc_affordance_track_collection_t_subscribe(m_lcm, "AFFORDANCE_TRACK_COLLECTION",
					      AffordanceTrackCollectionHandlerAux, this);
  drc_affordance_t_subscribe(m_lcm, "AFFORDANCE_FIT",
			     AffordanceFitHandlerAux, this);

  m_mainThread = new boost::thread(boost::bind(&ConstraintApp_RB_UKF::main, this));
}

ConstraintApp_RB_UKF::~ConstraintApp_RB_UKF() 
{
  stopThreads();
  delete m_mainThread;

  lcm_destroy(m_lcm);

  m_log << "ConstraintApp_RB_UKF::~ConstraintApp_RB_UKF" << std::endl;
}

void ConstraintApp_RB_UKF::main()
{
  while ( !shouldStop() ) {
    {
      boost::mutex::scoped_lock lock(m_lcmMutex);
      lcm_handle_timeout(m_lcm, 500);
    }
  }
}

int ConstraintApp_RB_UKF::lcm_handle_timeout(lcm_t* lcm, int ms)
{
    // setup the LCM file descriptor for waiting.
    SOCKET lcm_fd = lcm_get_fileno(lcm);
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(lcm_fd, &fds);

    // wait a limited amount of time for an incoming message
    struct timeval timeout = {
        ms / 1000,           // seconds
        (ms % 1000) * 1000   // microseconds
    };
    int status = select(lcm_fd + 1, &fds, 0, 0, &timeout);
    if(status > 0 && FD_ISSET(lcm_fd, &fds)) {
        lcm_handle(lcm);
        return 1;
    }

    // no messages
    return 0;
}

void ConstraintApp_RB_UKF::AffordanceTrackCollectionHandler(const drc_affordance_track_collection_t *msg)
{
  m_log << "got a new track collection" << std::endl;

  boost::mutex::scoped_lock lock(m_dataMutex);

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

  /*
  m_log << "sent a new (empty) estimate of joint angles" << std::endl;
  drc_affordance_joint_angles_t aja;
  aja.utime = msg->utime;
  aja.uid = msg->uid;
  drc_affordance_joint_angles_t_publish(m_lcm, "AFFORDANCE_JOINT_ANGLES", &aja);
  */
}

void ConstraintApp_RB_UKF::AffordanceFitHandler(const drc_affordance_t *msg)
{
  m_log << "got a new affordance fit" << std::endl;

  boost::mutex::scoped_lock lock(m_dataMutex);
  m_currentEstimate.base_expressedIn_world = GetFrameFromParams(msg);
  m_currentLinks.clear();
  m_currentObservations.clear();
  m_wasReset = true;

  m_log << "  reset the base pose to " << std::endl
	<< m_currentEstimate.base_expressedIn_world << std::endl;
}

KDL::Frame ConstraintApp_RB_UKF::GetFrameFromParams(const drc_affordance_t *msg)
{
  std::string names[] = { "x", "y", "z", "roll", "pitch", "yaw" };
  double xyzrpw[6];
  int found = 0;

  for ( int i = 0; i < msg->nparams; i++ ) {
    for ( int j = 0; j < 6; j++ ) {
      if ( msg->param_names[i] == names[j] ) {
	xyzrpw[j] = msg->params[i];
	found++;
      }
    }
  }

  if ( found != 6 ) {
    m_log << "unable to find all pose parameters" << std::endl;
    return KDL::Frame();
  }

  KDL::Frame ret;
  ret.p[0] = xyzrpw[0];
  ret.p[1] = xyzrpw[1];
  ret.p[2] = xyzrpw[2];
  ret.M = KDL::Rotation::RPY(xyzrpw[3], xyzrpw[4], xyzrpw[5]);
  return ret;
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

KDL::Frame ConstraintApp_RB_UKF::VectorToFrame(const std::vector<double>& state)
{
  KDL::Frame res;
  
  if ( state.size() != 6 ) {
    m_log << "ERROR: improper number of elements while setting state" << std::endl;
    return KDL::Frame();
  }

  res.p[0] = state[0];
  res.p[1] = state[1];
  res.p[2] = state[2];

  res.M = KDL::Rotation::RPY(state[3], state[4], state[5]);

  return res;
}

std::vector<double> ConstraintApp_RB_UKF::FrameToVector(const KDL::Frame& frame)
{
  std::vector<double> state;
  state.reserve(6);

  state.push_back(frame.p[0]);
  state.push_back(frame.p[1]);
  state.push_back(frame.p[2]);
  double r,p,w;
  frame.M.GetRPY(r,p,w);
  state.push_back(r);
  state.push_back(p);
  state.push_back(w);

  return state;
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
