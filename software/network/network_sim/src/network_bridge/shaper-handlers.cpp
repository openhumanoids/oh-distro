#include "network-bridge.h"

#include <iostream>
#include <queue>
#include <map>

#include <boost/bimap.hpp>
#include <boost/circular_buffer.hpp>

#include "goby/acomms/modemdriver/udp_driver.h"
#include "goby/acomms/connect.h"
#include "goby/acomms/bind.h"
#include "goby/acomms/amac.h"

using namespace boost; 
using namespace std;

enum Node { BASE = 1, ROBOT = 2};

struct QueuedMessage
{
    std::string channel;
    std::vector<char> data;
};

bool operator< (const drc_shaper_msg_t& a, const drc_shaper_msg_t& b)
{
    return (a.priority == b.priority) ?
        a.fragment > b.fragment : // higher fragment has lower priority
        a.priority < b.priority;
}


class DRCShaper
{
public:
    DRCShaper(KMCLApp& app, Node node);
    void run();

private:
    void outgoing_handler(const lcm_recv_buf_t *rbuf, const char *channel, void *user_data);
    void udp_data_receive(const goby::acomms::protobuf::ModemTransmission& msg);
    void data_request_handler( goby::acomms::protobuf::ModemTransmission* msg);

    friend void lcm_outgoing_handler(const lcm_recv_buf_t *rbuf, const char *channel, void *user_data);
private:
    KMCLApp& app_;
    boost::asio::io_service udp_service_;
    boost::shared_ptr<goby::acomms::UDPDriver> udp_driver_;
    goby::acomms::MACManager mac_;

    Node node_;
    Node partner_;
    lcm_t* lcm_;

    // maps channel to ring buffer
    std::map<std::string, boost::circular_buffer<QueuedMessage> > queues_;

    // maps channel to message number
    std::map<std::string, int> message_num_;

    // maps channel number to channel name
    boost::bimap<std::string, int> channel_id_;

    int last_send_type_;

    int largest_id_;
    
    std::priority_queue<drc_shaper_msg_t> send_queue_;

    std::map<std::string, std::vector<drc_shaper_msg_t> > receive_queue_;
};

void lcm_outgoing_handler(const lcm_recv_buf_t *rbuf, const char *channel, void *user_data)
{
    ((DRCShaper*) user_data)->outgoing_handler(rbuf, channel, user_data);
}


//
// from network_bridge.h
//
void robot2base(KMCLApp& app)
{
    if(app.base_only == app.bot_only)
        throw(std::runtime_error("Must choose only one of base only (-b) or robot only (-r) when running drc_network_shaper"));
    
    if(!app.base_only)
    {
        DRCShaper robot_shaper(app, ROBOT);
        robot_shaper.run();
    }
}

void base2robot(KMCLApp& app) { 
    sleep(1); // ... not necessary just for clarity

    if(!app.bot_only)
    {
        DRCShaper base_shaper(app, BASE);
        base_shaper.run();
    }
}

//
// definition of DRCShaper
//

DRCShaper::DRCShaper(KMCLApp& app, Node node)
    : app_(app),
      node_(node),
      partner_(node == BASE ? ROBOT : BASE),
      lcm_(node == BASE ? app.base_lcm : app.robot_lcm),
      last_send_type_(0),
      largest_id_(0)
{

    const std::vector<Resend>& resendlist = app.resendlist();

    int current_id = 0;
    for(int i = 0, n = resendlist.size(); i < n; ++i)
    {
        if(!channel_id_.left.count(resendlist[i].channel))
        {
            std::cout << "Mapping: " << resendlist[i].channel << " to id: " << current_id << std::endl;
            channel_id_.insert(boost::bimap<std::string, int>::value_type(resendlist[i].channel, current_id));
            ++current_id;
        }
    }
    largest_id_ = current_id-1;
    
    
    
    // outgoing 
    lcm_subscribe (lcm_,
                   node_ == BASE ?  app.base2robot_subscription.c_str() : app.robot2base_subscription.c_str(),
                   lcm_outgoing_handler, this);

    cout << "subscribed" << std::endl;

    udp_driver_.reset(new goby::acomms::UDPDriver(&udp_service_));

    int max_frame_size = bot_param_get_int_or_fail(app.bot_param, "network.udp_frame_size_bytes");
    {
        goby::acomms::protobuf::DriverConfig cfg;
        cfg.set_modem_id(node_);

        char* robot_host = bot_param_get_str_or_fail(app.bot_param, "network.robot.udp_host");
        char* base_host = bot_param_get_str_or_fail(app.bot_param, "network.base.udp_host");
        int robot_port = bot_param_get_int_or_fail(app.bot_param, "network.robot.udp_port");
        int base_port = bot_param_get_int_or_fail(app.bot_param, "network.base.udp_port");
        
        cfg.SetExtension(UDPDriverConfig::max_frame_size, max_frame_size);
        
        UDPDriverConfig::EndPoint* local_endpoint =
            cfg.MutableExtension(UDPDriverConfig::local);
        local_endpoint->set_port(node == BASE ? base_port : robot_port);

        UDPDriverConfig::EndPoint* remote_endpoint =
            cfg.MutableExtension(UDPDriverConfig::remote);

        remote_endpoint->set_ip(node == BASE ? robot_host : base_host);
        remote_endpoint->set_port(node == BASE ? robot_port : base_port);
        
        goby::acomms::connect(&udp_driver_->signal_receive, this, &DRCShaper::udp_data_receive);
        goby::acomms::connect(&udp_driver_->signal_data_request, this, &DRCShaper::data_request_handler);

        std::cout << "Starting UDP driver with configuration: " << cfg.ShortDebugString() << std::endl;
        udp_driver_->startup(cfg);
        free(robot_host);
        free(base_host);
    }
        
    {
        int target_rate_bps = bot_param_get_int_or_fail(app.bot_param, "network.target_rate_bps");

        // add slots as part of cfg
        goby::acomms::protobuf::MACConfig cfg;
        cfg.set_modem_id(node_);
        cfg.set_type(goby::acomms::protobuf::MAC_FIXED_DECENTRALIZED);

        goby::acomms::protobuf::ModemTransmission slot;
        slot.set_src(node_);
        slot.set_type(goby::acomms::protobuf::ModemTransmission::DATA);
        slot.set_slot_seconds(static_cast<double>(max_frame_size)*8/target_rate_bps);
        cfg.add_slot()->CopyFrom(slot);

        goby::acomms::bind(mac_, *udp_driver_);
        std::cout << "Starting MAC with configuration: " << cfg.ShortDebugString() << std::endl;
        mac_.startup(cfg);
    }
}


void DRCShaper::outgoing_handler(const lcm_recv_buf_t *rbuf, const char *channel, void *user_data)
{
    // Determine if the message should be dropped or sent (and then send)
    bool unused;
    if (app_.determine_resend_from_list(channel, app_.get_current_utime(), unused))
    {
        QueuedMessage msg;
        msg.channel = channel;
        msg.data.assign((uint8_t*)rbuf->data, (uint8_t*)rbuf->data + rbuf->data_size);
        
        cout << "queueing: " << app_.get_current_utime() << " | "
             << channel << " | " << rbuf->data_size << " bytes\n";

        std::map<std::string, boost::circular_buffer<QueuedMessage> >::iterator q_it =
            queues_.find(channel);
        
        // if this is the first time we've seen this channel, create a new circular buffer for it
        if(q_it == queues_.end())
        {
            // TODO: make circular buffer size configurable (currently 1, which isn't much of a buffer)
            queues_.insert(std::make_pair(channel, boost::circular_buffer<QueuedMessage>(1, 1, msg)));
            
        }
        else
        {
            // add it to the end of the buffer
            q_it->second.push_back(msg);
        }
        
    }
}

void DRCShaper::data_request_handler( goby::acomms::protobuf::ModemTransmission* msg)
{
    // TODO: check that send_queue has highest priority item

    if(send_queue_.empty())
    {
        int starting_send_type = last_send_type_;

        while(1)
        {
            ++last_send_type_;
            if(last_send_type_ > largest_id_)
                last_send_type_ = 0;
            
//            std::cout << "Checking channel id: " << last_send_type_ << std::endl;
            
//            std::cout << "Name: " << channel_id_.right.at(last_send_type_) << std::endl;
            
            std::map<std::string, boost::circular_buffer<QueuedMessage> >::iterator it = queues_.find(channel_id_.right.at(last_send_type_));
            
            if(it != queues_.end() && !it->second.empty())
            {
                QueuedMessage& qmsg = it->second.back();

                int pos = 0;
                int fragment = 0;
                while(pos < qmsg.data.size())
                {
                    // TODO: get the correct overhead (in bytes)
                    int overhead = 23;
                    
                    int data_size = std::min(
                        static_cast<int>(msg->max_frame_bytes()-overhead),
                        static_cast<int>(qmsg.data.size()-pos));
                    
                    drc_shaper_msg_t msg_frag;
                    msg_frag.channel = channel_id_.left.at(qmsg.channel);
                    msg_frag.data_size = data_size;
                    msg_frag.data = (uint8_t *)malloc(data_size);
                    msg_frag.fragment = (fragment++);
                    msg_frag.priority = 1;
                    memcpy(msg_frag.data,&qmsg.data[0]+pos,data_size);
                    pos += data_size;
                    msg_frag.is_last_fragment = (pos >= qmsg.data.size());
                    send_queue_.push(msg_frag);
                }
                
                it->second.pop_back();
                break;
            }
            
            // no data at all
            if(last_send_type_ == starting_send_type) return;
        }
    }
    
    
    msg->set_dest(partner_);
    msg->set_ack_requested(false);
    
    char buffer[msg->max_frame_bytes()];
    int size = drc_shaper_msg_t_encode(buffer, 0, msg->max_frame_bytes(), &send_queue_.top());

    send_queue_.pop();
    msg->add_frame(&buffer, size);

//    std::cout << "Data request msg (with data): " << msg->DebugString() << std::endl;
}

void DRCShaper::udp_data_receive(const goby::acomms::protobuf::ModemTransmission& msg)
{
//    cout << "received: " << msg.DebugString() << std::endl;
    
    drc_shaper_msg_t lcm_msg;
    drc_shaper_msg_t_decode(msg.frame(0).data(), 0, msg.max_frame_bytes(), &lcm_msg);

     cout << "received: " << app_.get_current_utime() << " | "
          << channel_id_.right.at(lcm_msg.channel) << " | "
          << lcm_msg.fragment << " | "
          << (bool)lcm_msg.is_last_fragment << " | "
          << drc_shaper_msg_t_encoded_size(&lcm_msg) << " bytes\n";
    

     if(lcm_msg.is_last_fragment)
     {
         if(lcm_msg.fragment == 0)
             lcm_publish(lcm_, channel_id_.right.at(lcm_msg.channel).c_str(),
                 lcm_msg.data, lcm_msg.data_size);
     }
     else
     {
         receive_queue_[channel_id_.right.at(lcm_msg.channel)].push_back(lcm_msg);
     }     
}

void DRCShaper::run()
{
    while (1)
    {
        int lcm_fd = lcm_get_fileno(lcm_);
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(lcm_fd, &fds);
        
        // wait a limited amount of time for an incoming message
        struct timeval timeout = { 
            0, // seconds
            1000  // microseconds
        };
        int status = select(lcm_fd + 1, &fds, 0, 0, &timeout);
        
        if(0 == status) {
            // no messages
            udp_driver_->do_work(); 
            mac_.do_work();
        } else if(FD_ISSET(lcm_fd, &fds)) {
            // LCM has events ready to be processed.
            lcm_handle(lcm_);
        }
    }
}
