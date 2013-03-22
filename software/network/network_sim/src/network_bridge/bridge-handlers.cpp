#include "network-bridge.h"

#include <iostream>

using namespace boost; 
using namespace std;

// implementation of this handler for the
// "drc-network-bridge"
// traffic is passed immediately to the other lcm community
// and does not traverse the traffic shaper
void generic_handler (const lcm_recv_buf_t *rbuf, const char *channel, void *user_data)
{
  KMCLApp* app = (KMCLApp*) user_data;
  if (app->cl_cfg.verbose)
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
     
    drc::bandwidth_stats_t stats;
    stats.utime = app->get_current_utime();
    stats.previous_utime = app->bw_init_utime;
    stats.bytes_from_robot = app->bw_cumsum_robot2base;
    stats.bytes_to_robot = app->bw_cumsum_base2robot;
    app->robot_lcm->publish("BW_STATS", &stats);
    app->base_lcm->publish("BW_STATS", &stats);
     
    app->bw_init_utime = app->get_current_utime();
    app->bw_cumsum_robot2base = 0;
    app->bw_cumsum_base2robot = 0;
  }
        
  // Determine if the message should be dropped or sent (and then send)
  bool robot2base=true;
  if (app->determine_resend_from_list(channel, app->get_current_utime(), robot2base, rbuf->data_size  )  ) {  
    if (robot2base){
      app->bw_cumsum_robot2base += rbuf->data_size;
      lcm_publish (app->base_lcm->getUnderlyingLCM(), channel, rbuf->data, rbuf->data_size); 
      if (app->cl_cfg.verbose)
        cout << "R2B " <<app->get_current_utime()<< "| "<<channel <<"\n";
    }else{
      app->bw_cumsum_base2robot += rbuf->data_size;
      lcm_publish (app->robot_lcm->getUnderlyingLCM(), channel, rbuf->data, rbuf->data_size); 
      if (app->cl_cfg.verbose)
        cout << "B2R " <<app->get_current_utime()<< "| "<<channel <<"\n";
    }
  }        
}


void robot2base(KMCLApp& app ) { 
  lcm_subscribe (app.robot_lcm->getUnderlyingLCM(), app.robot2base_subscription.c_str(), generic_handler, &app);
  cout << "robot to base subscribed\n";
  while (1)
      lcm_handle(app.robot_lcm->getUnderlyingLCM());    
}

void base2robot(KMCLApp& app) { 
  sleep(1); // ... not necessary just for clarity
  lcm_subscribe (app.base_lcm->getUnderlyingLCM(), app.base2robot_subscription.c_str(), generic_handler, &app);
  cout << "base to robot subscribed\n";
  while (1)
    lcm_handle(app.base_lcm->getUnderlyingLCM());
}

