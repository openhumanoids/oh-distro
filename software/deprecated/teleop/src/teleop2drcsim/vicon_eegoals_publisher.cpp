#include "vicon_kinematics.hpp"
#include <sys/time.h>
#include <time.h>
#include <lcmtypes/teleop.hpp>
#include <string>

using namespace vicon_kinematics; //??

// Scaling factor to convert from vicon units to gazebo units (mm to m)
static const double VICON_TO_GAZEBO_SCALAR = 1.0e-3;

/**
* Basic Memory Checker - dies if can't malloc
* @param ptr the pointer to check
*/
void checkmem(void* ptr)
{
  if (!ptr)
  {
  printf("Out of Memory\n");
  exit(1);
  }
}



/**
* Convert a double[3] of XYZ coordinates to a point3d_t;
* Also divides by the scalar factor
* @param xyz the double array
* @return the point3d_t
*/
point3d_t create_marker (const double xyz[3])
{
  point3d_t pt;
  pt.x = xyz[0] * VICON_TO_GAZEBO_SCALAR;
  pt.y = xyz[1] * VICON_TO_GAZEBO_SCALAR;
  pt.z = xyz[2] * VICON_TO_GAZEBO_SCALAR;
  return pt;
}




  /**
  * Utility class to contain all the callbacks for various kinematic solving methods
  * Also publishes the system goals via a ViconKinematicsSolver
  */
class Handler 
{
  public:
    typedef void(Handler::*Callback)(const lcm::ReceiveBuffer*, const std::string&, const teleop::fourmarkers_t*);

    Handler ()
    {
    // Create the LCM to use to publish the goals
    lcm::LCM* lcmptr = new lcm::LCM();
    checkmem(lcmptr);
    m_lcm = boost::shared_ptr<lcm::LCM> (new lcm::LCM());

    // Create the constraint solver / goal publisher
    m_vicon_solver = new ViconKinematicsSolver(m_lcm);
    checkmem(m_vicon_solver);

    // Wait until the solver has gotten an update from the system
    // and unsubscribed the lcm channel
    // NOTE: comment this line out for unit testing
    while(-1 == m_lcm->handle());

    m_functionmap.insert(std::make_pair("simple",&Handler::SimpleApproach));
    m_functionmap.insert(std::make_pair("angle",&Handler::AngleApproach));
    }
    ~Handler()
    {
    free(m_vicon_solver);
    }

    // Various Approaches

    /**
    * Publishes goals given the positions of the markers
    * @param rbuf the raw bytes and timestamp of the received message
    * @param chan the channel name
    * @param msg the struct
    */
    void SimpleApproach(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const teleop::fourmarkers_t* msg)
    {
    fourMarkers_t markers;
    markers.LEndeffector = create_marker(msg->Segment_LeftHand_c);
    markers.REndeffector = create_marker(msg->Segment_RightHand_c);
    markers.LShoulder = create_marker(msg->Segment_LeftShoulder_c);
    markers.RShoulder = create_marker(msg->Segment_RightShoulder_c);
    m_vicon_solver->publish_ee_goals_given_four_vicon_markers(markers);
    }

    /**
    * Publishes goals given the angles between the markers
    * @param rbuf the raw bytes and timestamp of the received message
    * @param chan the channel name
    * @param msg the struct
    */
    void AngleApproach(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const teleop::fourmarkers_t* msg)
    {
      /*  atlasJointAngles_t joint_angles;
      // I have no idea if these are right...
      joint_angles.LShoulderYaw=msg->left_shoulder[0];
      joint_angles.LShoulderPitch=msg->left_shoulder[1];
      joint_angles.LShoulderRoll=msg->left_shoulder[2];
      joint_angles.LElbowPitch=msg->left_hand[0];
      joint_angles.LWristYaw=msg->left_hand[1];
      joint_angles.LWristRoll=msg->left_hand[2];
      joint_angles.RShoulderYaw=msg->right_shoulder[0];
      joint_angles.RShoulderPitch=msg->right_shoulder[1];
      joint_angles.RShoulderRoll=msg->right_shoulder[2];
      joint_angles.RElbowPitch=msg->right_hand[0];
      joint_angles.RWristYaw=msg->right_hand[1];
      joint_angles.RWristRoll=msg->right_hand[2];
      joint_angles.NeckYaw=0;// ignore neck angles, they can be zero
      joint_angles.NeckPitch=0;
      m_vicon_solver->publish_ee_goals_given_vicon_angles(joint_angles);*/
    }

    /**
    * Prints out the existing methods
    * Could have made it return a std::vector, but :P
    */
    void printMethods() const
    {
      for(std::map<std::string,Callback>::const_iterator i = m_functionmap.begin(); i != m_functionmap.end(); ++i)
      {
        printf("%s\n",i->first.c_str());
      }
    }

    /**
    * Finds the appropriate method given the user's desired approach
    * @param approachName the user's input
    * @return the proper method or NULL if not found
    */
    Callback lookupApproach(std::string approachName)
    {
      for(std::map<std::string,Callback>::const_iterator i = m_functionmap.begin(); i != m_functionmap.end(); ++i)
      {
        if (i->first == approachName)
        {
          return i->second;
        }
      }
      return NULL;
    }

  private: 
    boost::shared_ptr<lcm::LCM> m_lcm;
    ViconKinematicsSolver* m_vicon_solver;
    std::map<std::string,Callback> m_functionmap;
  };

  /**
  * Main
  * Listens to the "vicon4markers" LCM channel for segment data
  * Then publishes the ee_goals based on position, angle, or whatever
  */
  int main(int argc, char ** argv)
  {
    // Create the receiving LCM channel
    lcm::LCM lcm;
    // Check for bad LCM initialization 
    if(!lcm.good())
    {
      return 1;
    }

    // Parse the command line, register the callback
    Handler handlerObject;
    if (argc < 2)
    {
      printf("No approach type name given. Defaulting to SimpleApproach\n");
      //printf("Pass the argument 'list' to list all the currently-implemented approaches\n");
      lcm.subscribe("vicon4markers", &Handler::SimpleApproach, &handlerObject);
    }
    else
    {
      char* name = argv[1];
      // Can add lowercasing here if needed
      if (strcmp(name, "list") == 0)
      {
        printf("Currently-implemented approaches:\n");
        handlerObject.printMethods();
        return 0;
      }
      else
      {
        Handler::Callback callback = handlerObject.lookupApproach(name);
        if (callback)
        {
          printf("Found approach %s\n",name);
          lcm.subscribe("vicon4markers", callback, &handlerObject);
        }
        else
        {
        printf("Invalid approach type name.\n");
        printf("Currently-implemented approaches:\n");
        handlerObject.printMethods();
        return 1;
      }
    }
  }

  // Run the handler until an error is hit
  while(!lcm.handle())
  {
    printf("\n\nMAIN reading lcm vicon\n\n");
  }

  return 0;
}// end main



