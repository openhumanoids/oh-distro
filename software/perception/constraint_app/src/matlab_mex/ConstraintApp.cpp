#include "ConstraintApp.h"
#include <stdio.h>
#include <inttypes.h>
#include <sys/select.h>
typedef int SOCKET;

//NB: use tail -f cam.log to view output from the ConstraintApp_RB_UKF::main thread
ConstraintApp::ConstraintApp() : m_stopThreads(false), m_log("/tmp/cam.log", std::ios::trunc|std::ios::binary) 
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

  m_log << "ConstraintApp::~ConstraintApp" << std::endl;
}

void ConstraintApp::main() {
  while ( !shouldStop() ) {
    {
      boost::mutex::scoped_lock lock(m_lcmMutex);
      lcm_handle_timeout(m_lcm, 500);
    }
  }
}

int ConstraintApp::lcm_handle_timeout(lcm_t* lcm, int ms) {
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

bool ConstraintApp::shouldStop() 
{
  bool ret;
  {
    boost::mutex::scoped_lock lock(m_stopThreadsMutex);
    ret = m_stopThreads;
  }
  return ret;
}

void ConstraintApp::stopThreads() 
{
  {
    boost::mutex::scoped_lock lock(m_stopThreadsMutex);
    m_stopThreads = true;
  }
  m_mainThread->join();
}


KDL::Frame ConstraintApp::VectorToFrame(const std::vector<double>& state)
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

std::vector<double> ConstraintApp::FrameToVector(const KDL::Frame& frame)
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
