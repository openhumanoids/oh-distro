#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>

#include <ConciseArgs>
using namespace std;

int main(int argc, char ** argv) {
  int uid=-1; // by default dont track anything
  int tracker_id=0; // id of this tracker instance. Plane=0, otherwise above that
  string tracker_type=""; // what type of tracker to have
  int plane_uid=-1; // id of this tracker instance. Plane=0, otherwise above that
  std::cout << "tracker types: stop, plane, color, histogram, feature, icp\n"; 
  ConciseArgs opt(argc, (char**)argv);
  opt.add(tracker_id, "t", "tracker","Tell Tracker ID ...");
  opt.add(tracker_type, "y", "type",".. use Tracker Type");
  opt.add(uid, "i", "uid",".. to track Affordance ID");
  opt.add(plane_uid, "p", "plane",".. relative to Plane Affordance ID");
  opt.parse();  
  std::cout << "uid: " << uid << "\n";    
  std::cout << "tracker_id: " << tracker_id << "\n";    
  std::cout << "tracker_type: " << tracker_type << "\n";    
  std::cout << "plane_uid: " << plane_uid << "\n";    

  lcm::LCM lcm("");
  if(!lcm.good())
    return 1;

  drc::tracker_command_t msg;
  msg.utime = 0;
  msg.uid = uid;
  msg.tracker_id = tracker_id;
  if(tracker_type == "stop"){
    msg.tracker_type = drc::tracker_command_t::STOP;
  }else if(tracker_type == "plane"){
    msg.tracker_type = drc::tracker_command_t::PLANE;
  }else if (tracker_type == "color"){
    msg.tracker_type = drc::tracker_command_t::COLOR;
  }else if(tracker_type == "histogram"){
    msg.tracker_type = drc::tracker_command_t::HISTOGRAM;
  }else if(tracker_type == "feature"){
    msg.tracker_type = drc::tracker_command_t::FEATURE;
  }else if(tracker_type == "icp"){
    msg.tracker_type = drc::tracker_command_t::ICP;
  }else{
    cout << "tracker not understood\n";
    exit(-1); 
  }
  msg.plane_uid = plane_uid;

  lcm.publish("TRACKER_COMMAND", &msg);
  std::cout << "TRACKER_COMMAND sent" << endl << "============================" << endl;
  return 0;
}