#include "ConstraintApp.h"
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

//NB: use tail -f cam.log to view output from the ConstraintApp::main thread
ConstraintApp::ConstraintApp() : m_stopThreads(false), m_counter(0), m_log("/tmp/cam.log", std::ios::trunc|std::ios::binary)
{
  
  m_lcm = lcm_create(NULL);

  drc_affordance_track_collection_t_subscribe(m_lcm, "AFFORDANCE_TRACK_COLLECTION",
					      AffordanceTrackCollectionHandlerAux, this);
  drc_affordance_t_subscribe(m_lcm, "AFFORDANCE_FIT",
			     AffordanceFitHandlerAux, this);

  m_mainThread = new boost::thread(boost::bind(&ConstraintApp::main, this));
}

ConstraintApp::~ConstraintApp() 
{
  stopThreads();
  delete m_mainThread;

  lcm_destroy(m_lcm);
}

void ConstraintApp::main()
{
  while ( !shouldStop() ) {
    {
      boost::mutex::scoped_lock lock(m_lcmMutex);
      lcm_handle_timeout(m_lcm, 500);
    }
  }
}

int ConstraintApp::lcm_handle_timeout(lcm_t* lcm, int ms)
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

void ConstraintApp::AffordanceTrackCollectionHandler(const drc_affordance_track_collection_t *msg)
{
  m_log << "got a new track collection" << std::endl;

  boost::mutex::scoped_lock lock(m_dataMutex);

  for ( int i = 0; i < msg->ntracks; i++ ) {
    drc_affordance_track_t* track = &msg->tracks[i];
    
    std::string segmentName(track->segment);
    KDL::Vector track_expressedIn_world(track->position.x, track->position.y, track->position.z);
    KDL::Vector track_expressedIn_base(m_currentEstimate.base_expressedIn_world.Inverse() * track_expressedIn_world);
    
    if ( m_currentLinks.find(segmentName) == m_currentLinks.end() ) {
      //this track is a new one
      m_currentLinks.insert(std::pair<std::string, Link>(segmentName, Link(track_expressedIn_base)));
      m_log << "  creating a new link for segment \"" << segmentName << "\"" << std::endl
	    << "    world pose: " << track_expressedIn_world << std::endl
	    << "     base pose: " << track_expressedIn_base << std::endl;
    } else {
      //this track is already known, just add this report as an observation
      ObservationMap::iterator existingObs(m_currentObservations.find(segmentName));
      Observation newobs(track_expressedIn_world);
      if ( existingObs == m_currentObservations.end() ) {
	//we have not yet observed this link, so add it
	m_currentObservations.insert(std::pair<std::string, Observation>(segmentName, newobs));
	m_log << "  new observation for segment \"" << segmentName << "\"" << std::endl
	      << "    world pose: " << track_expressedIn_world << std::endl;
      } else {
	//we've observed, and have not yet processed. just update the observation
	existingObs->second = newobs;
	m_log << "  repeat observation for segment \"" << segmentName << "\"" << std::endl
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

void ConstraintApp::AffordanceFitHandler(const drc_affordance_t *msg)
{
  m_log << "got a new affordance fit" << std::endl;

  boost::mutex::scoped_lock lock(m_dataMutex);
  m_currentEstimate.base_expressedIn_world = GetFrameFromParams(msg);
  m_currentLinks.clear();
  m_currentObservations.clear();

  m_log << "  reset the base pose to " << std::endl
	<< m_currentEstimate.base_expressedIn_world << std::endl;
}

KDL::Frame ConstraintApp::GetFrameFromParams(const drc_affordance_t *msg)
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

bool ConstraintApp::WaitForObservations(unsigned int timeout_ms)
{
  m_log << "ConstraintApp::WaitForObservations, timeout = " << timeout_ms << std::endl;

  boost::mutex::scoped_lock lock(m_dataMutex);

  if ( !m_currentObservations.empty() ) return true;

  boost::system_time timeout = boost::get_system_time() + boost::posix_time::milliseconds(timeout_ms);
  m_dataCondition.timed_wait(lock, timeout);

  return !m_currentObservations.empty();
}

bool ConstraintApp::GetObservations(std::vector<double>& expectedObservations, std::vector<double>& actualObservations)
{
  boost::mutex::scoped_lock lock(m_dataMutex);

  m_log << "ConstraintApp::GetObservations" << std::endl;

  actualObservations.clear();
  actualObservations.reserve(m_currentObservations.size()*3);
  expectedObservations.clear();
  expectedObservations.reserve(m_currentObservations.size()*3);

  for ( ObservationMap::const_iterator iter = m_currentObservations.begin(); iter != m_currentObservations.end(); ++iter ) {
    actualObservations.push_back(iter->second.obs_expressedIn_world[0]);
    actualObservations.push_back(iter->second.obs_expressedIn_world[1]);
    actualObservations.push_back(iter->second.obs_expressedIn_world[2]);

    LinkMap::const_iterator linkIter = m_currentLinks.find(iter->first);
    if ( linkIter == m_currentLinks.end() ) {
      m_log << "ERROR: unable to find a corresponding link for observation " << iter->first << std::endl;
      return false;
    }

    KDL::Vector expected = m_currentEstimate.base_expressedIn_world * linkIter->second.link_expressedIn_base;
    expectedObservations.push_back(expected[0]);
    expectedObservations.push_back(expected[1]);
    expectedObservations.push_back(expected[2]);

    m_log << "   added observation for " << iter->first << std::endl
	  << "      obs: " << iter->second.obs_expressedIn_world << std::endl
	  << "      exp: " << expected << std::endl;
  }

  m_currentObservations.clear();

  return true;
}
