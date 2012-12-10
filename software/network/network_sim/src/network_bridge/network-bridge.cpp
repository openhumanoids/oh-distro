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

#include <boost/thread.hpp>
#include <lcm/lcm.h>

#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <lcmtypes/bot_core.h>
#include <lcmtypes/drc_lcmtypes.h>
#include <ConciseArgs>

using namespace boost; 
using namespace std;

// Structure of which channels to resent and when
struct Resend{
  Resend(std::string channel, double max_freq, bool robot2base, int64_t last_utime):
    channel(channel), max_freq(max_freq), robot2base(robot2base), last_utime(last_utime) {}
  std::string channel; // .. LCM channel
  double max_freq; // max freq of transmission
  int64_t last_utime; // last utime of transmission
  bool robot2base; // true r2b | false b2r
};

///////////////////////////////////////////////////////////////
class KMCLApp{
  public:
    KMCLApp(lcm_t* robot_lcm, lcm_t* base_lcm,
	 std::string task);
    
    ~KMCLApp(){
    }
    
    lcm_t* robot_lcm;
    lcm_t* base_lcm;
    bool verbose;
    BotParam * bot_param;
    
    string robot2base_subscription;
    string base2robot_subscription;
    
    // the time we started counting the BW:
    int64_t bw_init_utime;
    // Size in ??? of the culumative bandwidth this period:
    int bw_cumsum_base2robot;
    int bw_cumsum_robot2base;
    
    void addResend( Resend resent_in ){  resendlist_.push_back(resent_in);    }
    bool determine_resend_from_list(std::string channel, int64_t msg_utime, bool &robot2base);    

    void set_current_utime(int64_t current_utime_in){
        boost::mutex::scoped_lock lock(guard);
        current_utime = current_utime_in;
    }    
    
    int64_t get_current_utime(){
      boost::mutex::scoped_lock lock(guard);
      return current_utime;
    }
    
    std::string parse_direction(string task, string direction, bool direction_bool);    
private:
    std::string task;
    std::vector<Resend> resendlist_;    
    
    int64_t current_utime;   
    boost::mutex guard;
};  

KMCLApp::KMCLApp(lcm_t* robot_lcm,lcm_t* base_lcm,
      std::string task):
      robot_lcm(robot_lcm), base_lcm(base_lcm),
      task(task){
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
       cout << "always send " << channel << " message\n";
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

// message-type ignorant callback:
// All traffic passes through here
void generic_handler (const lcm_recv_buf_t *rbuf, const char *channel, void *user_data)
{
  KMCLApp* app = (KMCLApp*) user_data;
  if (app->verbose)
        printf ("%.3f Channel %-20s size %d\n", rbuf->recv_utime / 1000000.0,
                channel, rbuf->data_size);
        
  // Keep Bandwidth Stats:
  double elapsed_time = (double) (app->get_current_utime() - app->bw_init_utime)/1E6 ;
  double bw_window = 1.0; // bw window in seconds
  if ( elapsed_time  > 1.0  ){
    // 1024 is also used in bot spy:
    double bw_base2robot =  app->bw_cumsum_base2robot /(1024.0* elapsed_time );
    double bw_robot2base=  app->bw_cumsum_robot2base / (1024.0* elapsed_time );
    cout << "r "<<  bw_robot2base << " <----------> "<< bw_base2robot << " b [kB/s] w="<< bw_window << "sec\n";
     
    drc_bandwidth_stats_t stats;
    stats.utime = app->get_current_utime();
    stats.previous_utime = app->bw_init_utime;
    stats.bytes_from_robot = app->bw_cumsum_robot2base;
    stats.bytes_to_robot = app->bw_cumsum_base2robot;
    drc_bandwidth_stats_t_publish(app->robot_lcm, "BW_STATS", &stats);
    drc_bandwidth_stats_t_publish(app->base_lcm, "BW_STATS", &stats);
     
    app->bw_init_utime = app->get_current_utime();
    app->bw_cumsum_robot2base = 0;
    app->bw_cumsum_base2robot = 0;
  }
        
  // Determine if the message should be dropped or sent (and then send)
  bool robot2base=true;
  if (app->determine_resend_from_list(channel, app->get_current_utime(), robot2base )  ) {  
    if (robot2base){
      app->bw_cumsum_robot2base += rbuf->data_size;
      lcm_publish (app->base_lcm, channel, rbuf->data, rbuf->data_size); 
      if (app->verbose)
        cout << "R2B " <<app->get_current_utime()<< "| "<<channel <<"\n";
    }else{
      app->bw_cumsum_base2robot += rbuf->data_size;
      lcm_publish (app->robot_lcm, channel, rbuf->data, rbuf->data_size); 
      if (app->verbose)
        cout << "B2R " <<app->get_current_utime()<< "| "<<channel <<"\n";
    }
  }        
}

void robot2base(KMCLApp& app ) { 
  // Subscribe to robot time and use that to key the publishing of all messages in both directions:
  drc_utime_t_subscribe(app.robot_lcm, "ROBOT_UTIME", utime_handler, &app);
  lcm_subscribe (app.robot_lcm, app.robot2base_subscription.c_str() , generic_handler, &app);
  cout << "robot to base subscribed\n";
  while (1)
      lcm_handle(app.robot_lcm);    
}

void base2robot(KMCLApp& app) { 
  sleep(1); // ... not necessary just for clarity
  lcm_subscribe (app.base_lcm, app.base2robot_subscription.c_str(), generic_handler, &app);
  cout << "base to robot subscribed\n";
  while (1)
    lcm_handle(app.base_lcm);
}


int main (int argc, char ** argv) {
  string task = "driving";
  ConciseArgs opt(argc, (char**)argv);
  opt.add(task, "t", "task","Task: driving, walking, manipulation");
  opt.parse();
  std::cout << "task: " << task << "\n";
  
  lcm_t * robot_lcm= lcm_create(NULL);//"udpm://239.255.12.67:1267?ttl=1");  
  lcm_t * base_lcm= lcm_create("udpm://239.255.12.68:1268?ttl=1");  
  
  KMCLApp* app= new KMCLApp(robot_lcm,base_lcm,task);
  boost::thread_group thread_group;
  thread_group.create_thread(boost::bind(robot2base, boost::ref( *app)));
  thread_group.create_thread(boost::bind(base2robot, boost::ref( *app)));
  thread_group.join_all();  
  return 0;
}
