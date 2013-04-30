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
#include <path_util/path_util.h>
#include "goby/common/logger.h"

#include <ConciseArgs>

using namespace boost; 
using namespace std;

const std::string KMCLApp::B2R_CHANNEL = "TUNNEL_BASE_TO_ROBOT";
const std::string KMCLApp::R2B_CHANNEL = "TUNNEL_ROBOT_TO_BASE";

    


KMCLApp::KMCLApp(boost::shared_ptr<lcm::LCM> &robot_lcm, boost::shared_ptr<lcm::LCM> &base_lcm,
                 const CommandLineConfig& cl_cfg):
    robot_lcm(robot_lcm), base_lcm(base_lcm),
    cl_cfg(cl_cfg){
    
    current_utime=0;
  
    bw_init_utime = 0;
    bw_cumsum_robot2base = 0;
    bw_cumsum_base2robot = 0;
  
    fprintf(stderr,"Reading config from file\n");
    std::string config_path = std::string(getConfigPath()) +'/' + std::string(cl_cfg.config_file);
    bot_param = bot_param_new_from_file(config_path.c_str());
    if (bot_param == NULL) {
        std::cerr << "Couldn't get bot param from file " << config_path << std::endl;
        exit(-1);
    }
  
    //bot_param = bot_param_new_from_file("/home/mfallon/drc/software/config/drc_robot.cfg");
    /*if (bot_only){
      cout << "getting params from robot url\n";
      bot_param = bot_param_new_from_server(robot_lcm->getUnderlyingLCM(), 0);
      }else{ // if bot is not true, then get it from base
      cout << "getting params from base url\n";
      bot_param = bot_param_new_from_server(base_lcm->getUnderlyingLCM(), 0);
      }*/

    if (bot_param == NULL) {
        std::cerr << "Couldn't get bot param from file " << std::endl;
        exit(-1);
    }
  
    string direction = "robot_to_base";  
    robot2base_subscription =  parse_direction(cl_cfg.task, direction, 1);
    std::cout << robot2base_subscription<< " is robot2base_subscription string\n";
    std::cout << "========\n";  
  
    direction = "base_to_robot";
    base2robot_subscription =  parse_direction(cl_cfg.task, direction, 0);
    std::cout << base2robot_subscription << " is base2robot_subscription string\n";
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
            subscription_string += "$|^" + channels[j];
        }
    }  
    return subscription_string;
}

// Determine if a specific message is to be sent or not:
bool KMCLApp::determine_resend_from_list(std::string channel, int64_t msg_utime, bool &robot2base, int msg_bytes, bool* on_demand /* = 0 */){
    if(on_demand) *on_demand = false;
    for (size_t i=0; i < resendlist_.size() ; i++){
        if ( resendlist_[i].channel.compare(channel) == 0){
            if (resendlist_[i].max_freq ==0){
                robot2base = resendlist_[i].robot2base;
                resendlist_[i].queued_msgs++;
                resendlist_[i].queued_bytes= resendlist_[i].queued_bytes + msg_bytes;
                // cout << "always send " << channel << " message\n";
                return true;
            }
            else if (resendlist_[i].max_freq < 0 ){ // on demand
                robot2base = resendlist_[i].robot2base;
                resendlist_[i].queued_msgs++;
                resendlist_[i].queued_bytes= resendlist_[i].queued_bytes + msg_bytes;
                if(on_demand) *on_demand = true;
                return true;
            }
            double elapsed_time = (msg_utime - resendlist_[i].last_utime)/1E6;
            if (elapsed_time > 1.0/resendlist_[i].max_freq){
                resendlist_[i].last_utime = msg_utime;
                robot2base = resendlist_[i].robot2base;
                resendlist_[i].queued_msgs++;
                resendlist_[i].queued_bytes= resendlist_[i].queued_bytes + msg_bytes;
                return true;
            }else{
                return false;
            }
        }
    }
    return false;
}


std::string KMCLApp::print_resend_list(){
    stringstream ss;
  
    //cout << "0123456789012345678901234567890123456789\n";
    ss << "   CHANNEL NAME | MSGS |    BYTES |      AVG\n";
    for (size_t i=0; i < resendlist_.size() ; i++){
        Resend r = resendlist_[i];
        char buffer [500];
        float bytes_per_msg =0;
        if (r.queued_msgs > 0)
            bytes_per_msg = (float) (r.queued_bytes/r.queued_msgs);
      
        sprintf (buffer, "%15s | %4d | %8d | %8.0f", r.channel.substr(0,15).c_str(), 
                 r.queued_msgs, r.queued_bytes, bytes_per_msg );
    
        ss << buffer << "\n";
    }
    return ss.str();
}

void KMCLApp::send_resend_list(){
    drc::bandwidth_stats_t stats;
    stats.utime = get_current_utime();
    stats.previous_utime = bw_init_utime;
    stats.bytes_from_robot = bw_cumsum_robot2base;
    stats.bytes_to_robot = bw_cumsum_base2robot;
  
    for (size_t i=0; i < resendlist_.size() ; i++){
        Resend r = resendlist_[i];
        stats.channels.push_back( r.channel);
        stats.queued_msgs.push_back( r.queued_msgs);
        stats.queued_bytes.push_back( r.queued_bytes);
    }
    stats.num_channels = stats.channels.size();
    robot_lcm->publish("BW_STATS", &stats);  
    base_lcm->publish("BW_STATS", &stats);  
}


// The clock/robot_utime message is used to timing:
void KMCLApp::utime_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
                            const  drc::utime_t* msg){
    //KMCLApp* app = (KMCLApp*) user_data;
    set_current_utime(msg->utime);
//  std::cout << "got current time\n";
}

int main (int argc, char ** argv) {
    string task = "driving";

    CommandLineConfig cl_cfg;
    cl_cfg.bot_only = true;
    cl_cfg.base_only = false;
    cl_cfg.role = "robot";
    cl_cfg.config_file ="drc_robot.cfg";
    cl_cfg.enable_gui = false;
    cl_cfg.verbose = false;
  
    ConciseArgs opt(argc, (char**)argv);
    opt.add(cl_cfg.task, "t", "task","Task: driving, walking, manipulation");
    opt.add(cl_cfg.config_file, "c", "config_file", "Config Filename");
    opt.add(cl_cfg.role, "r", "role", "Role: robot, base or both (local system mode)");
    opt.add(cl_cfg.enable_gui, "g", "gui", "Enable NCurses GUI (for debugging)");
    opt.add(cl_cfg.verbose, "v", "verbose", "Enable verbose terminal output");
  
    opt.parse();
    std::cout << "task: " << cl_cfg.task << "\n";
    std::cout << "config_file: " << cl_cfg.config_file<< "\n";
    std::cout << "role: " << cl_cfg.role << "\n";
    if (cl_cfg.role=="robot"){
        cl_cfg.bot_only = true;
        cl_cfg.base_only = false;
    } else if (cl_cfg.role=="base"){
        cl_cfg.bot_only = false;
        cl_cfg.base_only = true;
    } else if (cl_cfg.role=="both"){
        cl_cfg.bot_only = false;
        cl_cfg.base_only = false;
    } else{
        std::cout << "mode not understood\n";
        exit(-1);
    }  
    std::cout << "robot only: " << cl_cfg.bot_only << "\n";
    std::cout << "base only: " << cl_cfg.base_only << "\n";

    char* lcm_url_robot;
    lcm_url_robot = getenv ( "LCM_URL_DRC_ROBOT" );
    
    if (lcm_url_robot!=NULL){
        printf ("The lcm_url_robot is: %s\n",lcm_url_robot);
    }else{
        std::cout << "LCM_URL_DRC_ROBOT environment variable has not been set." << std::endl;
        exit(-1);
    }

    char* lcm_url_base;
    lcm_url_base = getenv ( "LCM_URL_DRC_BASE" );
    if (lcm_url_base!=NULL){
        printf ("The lcm_url_base is: %s\n",lcm_url_base);      
    }else{
        std::cout << "LCM_URL_DRC_BASE environment variable has not been set." << std::endl;
        exit(-1);
    }




    boost::shared_ptr<lcm::LCM> robot_lcm(new lcm::LCM( lcm_url_robot )  );
    if(!robot_lcm->good()){
        std::cerr <<"ERROR: lcm is not good()" <<std::endl;
    }
    boost::shared_ptr<lcm::LCM> base_lcm(new lcm::LCM( lcm_url_base )  );
    if(!base_lcm->good()){
        std::cerr <<"ERROR: lcm is not good()" <<std::endl;
    }

  
    KMCLApp* app= new KMCLApp(robot_lcm,base_lcm,cl_cfg);
    boost::thread_group thread_group;

    // Subscribe to robot time and use that to key the publishing of all messages in both directions:
    if(cl_cfg.base_only)
        app->base_lcm->subscribe("ROBOT_UTIME", &KMCLApp::utime_handler, app);      
    else
        app->robot_lcm->subscribe("ROBOT_UTIME", &KMCLApp::utime_handler, app);      
  
    thread_group.create_thread(boost::bind(robot2base, boost::ref( *app)));
    thread_group.create_thread(boost::bind(base2robot, boost::ref( *app)));
    thread_group.join_all();  
    return 0;
}
