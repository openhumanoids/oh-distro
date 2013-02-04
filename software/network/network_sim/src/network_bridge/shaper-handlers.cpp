#include "network-bridge.h"

#include <iostream>

using namespace boost; 
using namespace std;

enum Direction { ROBOT2BASE = true, BASE2ROBOT = false };

// implementation of this handler for the
// "drc-network-shaper"
// traffic is passed to the lcm channel "TUNNEL_BASE_TO_ROBOT" or "TUNNEL_ROBOT_TO_BASE"
// which will then be taken up by bot-lcm-tunnel for transmission over the
// traffic controlled link
void outgoing_handler(const lcm_recv_buf_t *rbuf, const char *channel, void *user_data, Direction direction)
{
    KMCLApp* app = (KMCLApp*) user_data;
    
    // Determine if the message should be dropped or sent (and then send)
    bool unused;
    if (app->determine_resend_from_list(channel, app->get_current_utime(), unused))
    {
        drc_shaper_msg_t msg;
        msg.channel = strdup(channel);
        msg.data_size = rbuf->data_size;
        msg.data = (uint8_t *)malloc(msg.data_size);
        memcpy(msg.data,rbuf->data,msg.data_size);

        switch(direction)
        {
            case ROBOT2BASE:
                app->bw_cumsum_robot2base += rbuf->data_size;
                drc_shaper_msg_t_publish(app->robot_lcm, KMCLApp::R2B_CHANNEL.c_str(), &msg);
                cout << "sending: R2B " <<app->get_current_utime()<< "| "<<channel <<"\n";
                break;
                
            case BASE2ROBOT:
                app->bw_cumsum_base2robot += rbuf->data_size;
                drc_shaper_msg_t_publish(app->base_lcm, KMCLApp::B2R_CHANNEL.c_str(), &msg);
                cout << "sending: B2R " <<app->get_current_utime()<< "| "<<channel <<"\n";
                break;
        }        
    }        
}

void base_outgoing_handler(const lcm_recv_buf_t *rbuf, const char *channel, void *user_data)
{
    outgoing_handler(rbuf, channel, user_data, BASE2ROBOT);
}

void robot_outgoing_handler(const lcm_recv_buf_t *rbuf, const char *channel, void *user_data)
{
    outgoing_handler(rbuf, channel, user_data, ROBOT2BASE);    
}

void base_incoming_handler(const lcm_recv_buf_t* rbuf, const char* channel,
                           const drc_shaper_msg_t* msg, void* user_data)
{
    KMCLApp* app = (KMCLApp*) user_data;
    lcm_publish(app->base_lcm, msg->channel, msg->data, msg->data_size);
    cout << "received: R2B " <<app->get_current_utime()<< "| "<< msg->channel <<"\n";
}

void robot_incoming_handler(const lcm_recv_buf_t* rbuf, const char* channel,
                            const drc_shaper_msg_t* msg, void* user_data)
{
    KMCLApp* app = (KMCLApp*) user_data;
    lcm_publish(app->robot_lcm, msg->channel, msg->data, msg->data_size);    
    cout << "received: B2R " <<app->get_current_utime()<< "| "<< msg->channel <<"\n";
}

void robot2base(KMCLApp& app ) { 
  if(!app.base_only)
  {
      // outgoing 
      lcm_subscribe (app.robot_lcm, app.robot2base_subscription.c_str(), robot_outgoing_handler, &app);
      // incoming
      drc_shaper_msg_t_subscribe (app.robot_lcm, KMCLApp::B2R_CHANNEL.c_str(), robot_incoming_handler, &app);
      cout << "robot to base subscribed\n";
  }
  
  while (1)
      lcm_handle(app.robot_lcm);    
}

void base2robot(KMCLApp& app) { 
  sleep(1); // ... not necessary just for clarity

  if(!app.bot_only)
  {
      // outgoing
      lcm_subscribe (app.base_lcm, app.base2robot_subscription.c_str(), base_outgoing_handler, &app);
      // incoming
      drc_shaper_msg_t_subscribe (app.base_lcm, KMCLApp::R2B_CHANNEL.c_str(), base_incoming_handler, &app);
      cout << "base to robot subscribed\n";
  }
  
  while (1)
    lcm_handle(app.base_lcm);
}

