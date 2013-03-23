#include "ConstraintApp.h"
#include <stdio.h>
#include <inttypes.h>
#include <sys/select.h>
typedef int SOCKET;

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

  m_log << "sent a new (empty) estimate of joint angles" << std::endl;
  drc_affordance_joint_angles_t aja;
  aja.utime = msg->utime;
  aja.uid = msg->uid;
  drc_affordance_joint_angles_t_publish(m_lcm, "AFFORDANCE_JOINT_ANGLES", &aja);
}

void ConstraintApp::AffordanceFitHandler(const drc_affordance_t *msg)
{
  m_log << "got a new affordance fit" << std::endl;

  m_currentEstimate.base_expressedIn_world = GetFrameFromParams(msg);
  m_currentLinks.clear();

  m_log << "  reset the base pose to " << m_currentEstimate.base_expressedIn_world << std::endl;
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
