#ifndef NetworkBridge20130204
#define NetworkBridge20130204

#include <boost/thread.hpp>

#include <lcm/lcm-cpp.hpp>
//#include <lcmtypes/bot_core.h>
#include <lcmtypes/drc_lcmtypes.hpp>

#include <bot_param/param_client.h>
#include <bot_param/param_util.h>

class KMCLApp;
// thread for base to robot
void base2robot(KMCLApp& app);
// thread for robot to base
void robot2base(KMCLApp& app);

//void utime_handler(const lcm_recv_buf_t* rbuf, const char* channel,
//                   const drc::utime_t* msg, void* user_data);


// Structure of which channels to resent and when
struct Resend{
  Resend(std::string channel, double max_freq, bool robot2base, int64_t last_utime):
    channel(channel), max_freq(max_freq), last_utime(last_utime), robot2base(robot2base), queued_msgs(0), queued_bytes(0) {}
    std::string channel; // .. LCM channel
    double max_freq; // max freq of transmission
    int64_t last_utime; // last utime of transmission
    bool robot2base; // true r2b | false b2r
  
  int queued_msgs; // number of queued messaged
  int queued_bytes; // sum of the total number of LCM bytes of this message type queued for transmission ... used to determine outgoing bandwidth [added by mfallon Feb 2013]

};


struct CommandLineConfig
{
    std::string task;
    bool base_only;
    bool bot_only;
    std::string config_file;
    bool enable_gui;
    std::string role;
    bool verbose;
    bool file_log;
    std::string log_path;
};


///////////////////////////////////////////////////////////////
class KMCLApp{
  public:
    KMCLApp(boost::shared_ptr<lcm::LCM> &robot_lcm, boost::shared_ptr<lcm::LCM> &base_lcm,
            const CommandLineConfig& cl_cfg);
    
    ~KMCLApp(){
    }
    
    boost::shared_ptr<lcm::LCM> robot_lcm;
    boost::shared_ptr<lcm::LCM> base_lcm;
    const CommandLineConfig& cl_cfg;
     //lcm_t* robot_lcm;
    //lcm_t* base_lcm;
    
    void utime_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
                   const  drc::utime_t* msg); 
    void reset_stats_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
                   const drc::utime_t* msg); 
    
    BotParam * bot_param;
    
    std::string robot2base_subscription;
    std::string base2robot_subscription;
    
    // the time we started counting the BW:
    int64_t bw_init_utime;
    // Size in ??? of the culumative bandwidth this period:
    int bw_cumsum_base2robot;
    int bw_cumsum_robot2base;
    
    void addResend( Resend resent_in ){  resendlist_.push_back(resent_in);    }
    bool determine_resend_from_list(std::string channel, int64_t msg_utime, bool &robot2base, int msg_bytes, bool* on_demand = 0);    

    void set_current_utime(int64_t current_utime_in){
        boost::mutex::scoped_lock lock(guard);
        current_utime = current_utime_in;
    }    
    
    int64_t get_current_utime();
    
    void set_reset_usage_stats(bool reset_usage_stat_in){
      reset_usage_stats = reset_usage_stat_in; 
    }
    bool get_reset_usage_stats()
    { return reset_usage_stats; }
    
    
    const std::vector<Resend>& resendlist() 
    { return resendlist_; }

    std::vector<Resend>* mutable_resendlist() 
    { return &resendlist_; }

    std::string print_resend_list();
    void send_resend_list();
    
    std::string parse_direction(std::string task, std::string direction, bool direction_bool);    
    
    static const std::string B2R_CHANNEL;
    static const std::string R2B_CHANNEL;
  private:
    std::vector<Resend> resendlist_;    
    
    bool reset_usage_stats;
    int64_t current_utime;   
    boost::mutex guard;
};

#endif
