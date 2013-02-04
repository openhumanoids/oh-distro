// Basic tool to create to independent LCM communities on one machine
// With messages bridged according to the contents of the "network" config block
//
// Message size is reported in bytes:
// A bot_core_pose_t message is 144 bytes:
// 16 double * 8bytes = 128
// utime = long = 4
// fingerprint = long = 4
// fingerprint_base = long = 4
// 4 missing bytes = 4 (probably the channel OR that the fingerprint has be double counted here)

#include "network-bridge.h"

#include <ConciseArgs>

using namespace boost; 
using namespace std;

const std::string KMCLApp::B2R_CHANNEL = "TUNNEL_BASE_TO_ROBOT";
const std::string KMCLApp::R2B_CHANNEL = "TUNNEL_ROBOT_TO_BASE";


KMCLApp::KMCLApp(lcm_t* robot_lcm,lcm_t* base_lcm,
                 std::string task,
                 bool base_only,
                 bool bot_only):
      robot_lcm(robot_lcm), base_lcm(base_lcm),
      task(task),base_only(base_only),bot_only(bot_only){
    verbose=false;
  current_utime=0;
  
  bw_init_utime = 0;
  bw_cumsum_robot2base = 0;
  bw_cumsum_base2robot = 0;
  
  //bot_param = bot_param_new_from_file("/home/mfallon/drc/software/config/drc_robot.cfg");
  bot_param = bot_param_new_from_server(robot_lcm, 0);
  
  if (bot_param == NULL) {
    std::cerr << "Couldn't get bot param from file " << std::endl;
    exit(-1);
  }
  
  string direction = "robot_to_base";  
  robot2base_subscription =  parse_direction(task, direction, 1);
  std::cout << robot2base_subscription<< " robot2base_subscription string\n";
  std::cout << "========\n";  
  
  direction = "base_to_robot";
  base2robot_subscription =  parse_direction(task, direction, 0);
  std::cout << base2robot_subscription << " base2robot_subscription string\n";
  std::cout << "========\n";  
  
  for (size_t i=0; i< resendlist_.size() ; i++){
      cout << resendlist_[i].channel << " "
          << resendlist_[i].max_freq << " "
          << resendlist_[i].last_utime << " "
          << resendlist_[i].robot2base << "\n";
  }
  cout <<"Config has been read\n=========================\n"; 
}



// Parse the message channels to be sent in a particular direction:
std::string KMCLApp::parse_direction(string task, string direction, bool direction_bool){
  string subscription_string ="";
  
  char channels_key[512];  
  char frequency_list[512];  
  sprintf(channels_key, "network.%s.%s.channels",task.c_str() , direction.c_str() );
  sprintf(frequency_list, "network.%s.%s.frequency",task.c_str() , direction.c_str() );
  std::cout << channels_key << "\n";
  
  // Read the keys:  
  char **fnames = bot_param_get_str_array_alloc(bot_param, channels_key);
  std::vector <string> channels;
  for (int j = 0; fnames && fnames[j]!=NULL; j++) {
    channels.push_back(fnames[j]);
  } 
  int n_channels = channels.size();
  double frequencys[n_channels];
  bot_param_get_double_array_or_fail(bot_param,frequency_list,frequencys,channels.size());
  for (size_t j = 0; j < n_channels ; j++) {
    std::cout << j << ": " <<channels[j] << " at " << frequencys[j] << " Hz\n";
    addResend( Resend(channels[j], frequencys[j],   direction_bool, 0) );
    if (j==0){
      subscription_string = channels[j];
    }else{
      subscription_string += "|" + channels[j];
    }
  }  
  return subscription_string;
}

// Determine if a specific message is to be sent or not:
bool KMCLApp::determine_resend_from_list(std::string channel, int64_t msg_utime, bool &robot2base){
 for (size_t i=0; i < resendlist_.size() ; i++){
   if ( resendlist_[i].channel.compare(channel) == 0){
     if (resendlist_[i].max_freq ==0){
       robot2base = resendlist_[i].robot2base;
       // cout << "always send " << channel << " message\n";
       return true;
     }
     double elapsed_time = (msg_utime - resendlist_[i].last_utime)/1E6;
     if (elapsed_time > 1.0/resendlist_[i].max_freq){
       resendlist_[i].last_utime = msg_utime;
       robot2base = resendlist_[i].robot2base;
       return true;
     }else{
       return false;
     }
   }
 }
 return false;
}

// The clock/robot_utime message is used to timing:
void utime_handler(const lcm_recv_buf_t* rbuf, const char* channel,
                                const drc_utime_t* msg, void* user_data){
  KMCLApp* app = (KMCLApp*) user_data;
  app->set_current_utime(msg->utime);
//  std::cout << "got current time\n";
}

int main (int argc, char ** argv) {
  string task = "driving";
  bool bot_only = false;
  bool base_only = false;
  
  ConciseArgs opt(argc, (char**)argv);
  opt.add(task, "t", "task","Task: driving, walking, manipulation");
  opt.add(bot_only, "r", "robotonly", "If true, do not LCM connect base side.");
  opt.add(base_only, "b", "baseonly", "If true, do not LCM connect robot side.");
  opt.parse();
  std::cout << "task: " << task << "\n";
  std::cout << "robot only: " << bot_only << "\n";
  std::cout << "base only: " << base_only << "\n";

  lcm_t* robot_lcm = 0;
  lcm_t* base_lcm = 0;
  robot_lcm= lcm_create(NULL);//"udpm://239.255.12.67:1267?ttl=1");  
  base_lcm = lcm_create("udpm://239.255.12.68:1268?ttl=1");  
  
  KMCLApp* app= new KMCLApp(robot_lcm,base_lcm,task,base_only,bot_only);
  boost::thread_group thread_group;

  // Subscribe to robot time and use that to key the publishing of all messages in both directions:
  drc_utime_t_subscribe(app->robot_lcm, "ROBOT_UTIME", utime_handler, app);
  
  thread_group.create_thread(boost::bind(robot2base, boost::ref( *app)));
  thread_group.create_thread(boost::bind(base2robot, boost::ref( *app)));
  thread_group.join_all();  
  return 0;
}
