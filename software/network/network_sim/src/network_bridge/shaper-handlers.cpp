#include <iostream>
#include <queue>
#include <map>

#include <boost/bimap.hpp>
#include <boost/circular_buffer.hpp>

#include "goby/acomms/modemdriver/udp_driver.h"
#include "goby/acomms/connect.h"
#include "goby/acomms/bind.h"
#include "goby/acomms/amac.h"

#include "network-bridge.h"
#include "shaper-packet.pb.h"

#include "ldpc/ldpc_wrapper.h"

using namespace boost; 
using namespace std;

enum { RECEIVE_MODULUS = 16 };    
enum { MIN_NUM_FRAGMENTS_FOR_FEC = 3 };

enum Node { BASE = 1, ROBOT = 2};



struct MessageQueue
{
    MessageQueue(const std::string& ch,
                 int buffer_size,
                 const std::vector<char>& initial_msg)
        : message_count(0),
          channel(ch),
          messages(buffer_size, 1, initial_msg)
        {
            
        }

    int message_count;
    std::string channel;
    boost::circular_buffer< std::vector<char> > messages;
};



std::string DebugStringNoData(const drc::ShaperPacket& a)
{
    std::string s = a.ShortDebugString();
    return s.substr(0, s.find("data:")) + " data.size(): " + goby::util::as<std::string>(a.data().length());    
}

template<typename Char>
unsigned int xor_cs(const std::vector<Char>& data)
{
    unsigned char cs = 0;
    for(typename std::vector<Char>::const_iterator it = data.begin(), end = data.end();
        it != end; ++it)
        cs ^= *it;
    return cs;
}

// rounds down to the closest highest power of 2
unsigned int floor_multiple_sixteen(unsigned int v)
{
    unsigned int lg = (v / 16)*16;
    return lg;
}

void check_rc(int rc)
{
  if(rc == -1)
    std::cout << "Error: previous operation failed!" << std::endl;
}

namespace drc
{    
    bool operator< (const ShaperPacket& a, const ShaperPacket& b)
    {
        return (a.priority() == b.priority()) ?
            a.fragment() > b.fragment() : // higher fragment has lower priority
            a.priority() < b.priority();
    }

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

    // wrap message number into the [0, RECEIVE_MODULUS) space
    void receive_mod(int& i)
        {
            if(i < 0) i += RECEIVE_MODULUS;
            if(i >= RECEIVE_MODULUS) i -= RECEIVE_MODULUS;
        }
    void try_decode( std::map<int, drc::ShaperPacket>& received_frags);


    friend void lcm_outgoing_handler(const lcm_recv_buf_t *rbuf, const char *channel, void *user_data);
private:
    KMCLApp& app_;
    boost::asio::io_service udp_service_;
    boost::shared_ptr<goby::acomms::UDPDriver> udp_driver_;
    goby::acomms::MACManager mac_;

    Node node_;
    Node partner_;
    boost::shared_ptr<lcm::LCM> lcm_;

    // maps channel to ring buffer
    std::map<std::string, MessageQueue > queues_;

    // maps channel number to channel name
    boost::bimap<std::string, int> channel_id_;

    int last_send_type_;

    int largest_id_;
    
    std::priority_queue<drc::ShaperPacket> send_queue_;

    // maps channel id to (map of message_number to (map of fragment number to fragment))
    std::map<int, std::map<int, std::map<int, drc::ShaperPacket> > > receive_queue_;

    double fec_;
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
      largest_id_(0),
      fec_(1.5) // TODO: make configurable or auto-adapt
{
    assert(floor_multiple_sixteen(1025) == 1024);


    
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
    lcm_subscribe(lcm_->getUnderlyingLCM(),
                  node_ == BASE ?  app.base2robot_subscription.c_str() : app.robot2base_subscription.c_str(), lcm_outgoing_handler, this);

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
    if (app_.determine_resend_from_list(channel, app_.get_current_utime(), unused, rbuf->data_size))
    {
        std::vector<char> data((uint8_t*)rbuf->data, (uint8_t*)rbuf->data + rbuf->data_size);
        
        std::map<std::string, MessageQueue>::iterator q_it = queues_.find(channel);
        
        // if this is the first time we've seen this channel, create a new circular buffer for it
        if(q_it == queues_.end())
        {
            // TODO: make circular buffer size configurable (currently 1, which isn't much of a buffer)
            queues_.insert(std::make_pair(channel, MessageQueue(channel, 1, data)));
        }
        else
        {
            ++q_it->second.message_count;
            receive_mod(q_it->second.message_count);

            // add it to the end of the buffer
            q_it->second.messages.push_back(data);
        }    

        
        cout << "queueing: " << app_.get_current_utime() << " | "
             << channel << " #" << queues_.find(channel)->second.message_count << " | " << rbuf->data_size << " bytes *" << xor_cs(data) << std::endl;
        // cout << app_.print_resend_list(); // print info sent in BW Stats msg
        app_.send_resend_list();
        app_.bw_cumsum_robot2base += rbuf->data_size;

    }
}

void DRCShaper::data_request_handler(goby::acomms::protobuf::ModemTransmission* msg)
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
            
            std::map<std::string, MessageQueue >::iterator it =
                queues_.find(channel_id_.right.at(last_send_type_));
            
            if(it != queues_.end() && !it->second.messages.empty())
            {
                std::vector<char>& qmsg = it->second.messages.back();

                // int pos = 0;
                // int fragment = 0;

                // TODO: get the correct overhead (in bytes)
                int overhead = 31;
                
                int payload_size = msg->max_frame_bytes()-overhead;

                drc::ShaperPacket msg_frag;
                int16_t fragment = 0;
                msg_frag.set_channel(channel_id_.left.at(it->second.channel));
                msg_frag.set_message_number(it->second.message_count);
                msg_frag.set_message_size(qmsg.size());
                msg_frag.set_priority(1);

                if(qmsg.size() <= payload_size)
                {
                    msg_frag.set_data(&qmsg[0], qmsg.size());
                    msg_frag.set_fragment(0);
                    msg_frag.set_is_last_fragment(true);
                    send_queue_.push(msg_frag);
                }
                else
                {                
		    payload_size = floor_multiple_sixteen(msg->max_frame_bytes()-overhead);
                    while(qmsg.size() / payload_size < MIN_NUM_FRAGMENTS_FOR_FEC)
		      payload_size = floor_multiple_sixteen(payload_size-1);

                    if(payload_size == 0)
                        throw(std::runtime_error("udp_frame_size_bytes is too small for LDPC error correction, use a larger value"));
                    
                    ldpc_enc_wrapper encoder(reinterpret_cast<uint8_t*>(&qmsg[0]),
                                             qmsg.size(),
                                             payload_size,
                                             fec_);
                    bool enc_done = false;
                    while (!enc_done)
                    {
                        std::vector<uint8_t> buffer(payload_size);
                        enc_done = encoder.getNextPacket(&buffer[0], &fragment);
                        msg_frag.set_data(&buffer[0], payload_size);
                        msg_frag.set_fragment(fragment);
                        msg_frag.set_is_last_fragment(enc_done);
                        send_queue_.push(msg_frag);
                    }                    
                }
		it->second.messages.pop_back();
		break;
            }
            
            
            // no data at all
            if(last_send_type_ == starting_send_type) return;
        }
    }
    
    msg->set_dest(partner_);
    msg->set_ack_requested(false);
    
    send_queue_.top().SerializeToString(msg->add_frame());
    //std::cout << "Sending: " << DebugStringNoData(send_queue_.top()) << std::endl;
    // std::cout << "Data size: " << send_queue_.top().data().size() << std::endl;
    // std::cout << "Encoded size: " << msg->frame(0).size() << std::endl;
    
    send_queue_.pop();

//    std::cout << "Data request msg (with data): " << msg->DebugString() << std::endl;
}

void DRCShaper::udp_data_receive(const goby::acomms::protobuf::ModemTransmission& msg)
{    
    drc::ShaperPacket packet;
    packet.ParseFromString(msg.frame(0));

    cout << "received: " << app_.get_current_utime() << " | "
         << DebugStringNoData(packet) << std::endl;    
    
    receive_queue_[packet.channel()][packet.message_number()][packet.fragment()] = packet;
    if(packet.is_last_fragment())
    {
        if(packet.fragment() == 0)
        {
            cout << "publishing: " << app_.get_current_utime() << " | "
                 << channel_id_.right.at(packet.channel()) << " #" << packet.message_number() << " | " << packet.data().size() << " bytes" << std::endl;
	    std::vector<unsigned char> buffer(packet.data().size());
	    for(std::string::size_type i = 0, n = packet.data().size(); i < n; ++i)
	      buffer[i] = packet.data()[i];
	    
	    check_rc(lcm_publish(lcm_->getUnderlyingLCM(), channel_id_.right.at(packet.channel()).c_str(),
				 &buffer[0], packet.data().size()));

	    receive_queue_[packet.channel()][packet.message_number()].clear();
        }
        else
        {
            // try to reconstruct
            std::cout << "Trying to reconstruct message with last fragment: " << DebugStringNoData(packet) << std::endl;
	    
	    //            std::cout << "Checking " << RECEIVE_MODULUS/2 << " assumed oldest message numbers" << std::endl;
            
            std::map<int, std::map<int, drc::ShaperPacket> >& this_queue = receive_queue_[packet.channel()];

            int begin = packet.message_number()-3*RECEIVE_MODULUS/4;
            receive_mod(begin);
            int end = packet.message_number()-RECEIVE_MODULUS/4;
            receive_mod(end);
            
            for(int i = begin; i != end; )
            {
	                      std::cout << "Check #" << i << ": ";

                std::map<int, std::map<int, drc::ShaperPacket> >::iterator it = this_queue.find(i);
                if(it == this_queue.end() || it->second.empty())
		  {
		    	    std::cout << "Ok, packet has been processed." << std::endl;
		  }
		    else
                {
                    try_decode(it->second);
                    it->second.clear();
                }
                
                ++i;
                receive_mod(i);
            }

            std::map<int, drc::ShaperPacket>& received_frags = this_queue[packet.message_number()];
            try_decode(received_frags);            
        }
        
    }
}


void DRCShaper::try_decode(std::map<int, drc::ShaperPacket>& received_frags)
{
    const drc::ShaperPacket& front = received_frags.begin()->second;
    ldpc_dec_wrapper decoder(front.message_size(), front.data().size(), fec_);
    
    for(std::map<int, drc::ShaperPacket>::iterator it = received_frags.begin(),
            end = received_frags.end(); it != end; ++it)
    {
        int dec_done = decoder.processPacket(reinterpret_cast<uint8_t*>(&it->second.mutable_data()->at(0)), it->second.fragment());
        if (dec_done != 0)
        {
            if (dec_done == 1)
            {
                std::vector<unsigned char> buffer(front.message_size());
                decoder.getObject(&buffer[0]);


                cout << "publishing: " << app_.get_current_utime() << " | "
                     << channel_id_.right.at(front.channel()) << " #" << front.message_number() << " | " << buffer.size() << " bytes *" << xor_cs(buffer) << std::endl;
                
                check_rc(lcm_publish(lcm_->getUnderlyingLCM(), channel_id_.right.at(front.channel()).c_str(),
				     &buffer[0], buffer.size()));
                received_frags.clear();
                return;
            }
            else
            {
                std::cout << "Error: ldpc got all the sent packets, but couldn't reconstruct... this shouldn't happen!" << std::endl;
                received_frags.clear();
                return;
            }
        }
    }
    
    int expected = ceil(front.message_size()*fec_ / front.data().size());
    int received = received_frags.size();
    std::cout << "Not enough packets received to decode: received " << received << " of " << expected << std::endl;    
}


void DRCShaper::run()
{
    while (1)
    {
        int lcm_fd = lcm_get_fileno(lcm_->getUnderlyingLCM());
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
            lcm_handle(lcm_->getUnderlyingLCM());
        }
    }
}
