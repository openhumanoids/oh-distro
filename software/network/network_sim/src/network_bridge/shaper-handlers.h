#ifndef SHAPERHANDLERS20130506H
#define SHAPERHANDLERS20130506H

#include "network-bridge.h"
#include "shaper-packet.pb.h"
#include "ldpc/ldpc_wrapper.h"
#include "custom-codecs.h"

enum { RECEIVE_MODULUS = 16 };    
enum { MIN_NUM_FRAGMENTS_FOR_FEC = 3 };


struct MessageQueue
{
MessageQueue(const std::string& ch,
             int buffer_size,
             const std::vector<unsigned char>& initial_msg,
             bool is_on_demand = false)
: message_count(0),
        channel(ch),
        messages(buffer_size, 1, initial_msg),
        on_demand(is_on_demand)
        {
            
        }

    int message_count;
    std::string channel;
    boost::circular_buffer< std::vector<unsigned char> > messages;
    bool on_demand;
};


struct DataUsage
{
DataUsage() : queued_msgs(0),
        queued_bytes(0),
        sent_bytes(0),
        received_bytes(0) 
        { }
        
    int queued_msgs; // number of queued messaged
    int queued_bytes; // sum of the total number of LCM bytes of this message type queued for transmission

    int sent_bytes; // sum of the total number of bytes of this message type transmitted. Does *not* include overhead of UDP or IPv4, so it should be an accurate count of the bytes counted by DARPA
    int received_bytes; // sum of the total number of bytes of this message type received. Does *not* include overhead of UDP or IPv4. If packets are dropped, this may be less than the total sent.
};


inline std::string DebugStringNoData(const drc::ShaperPacket& a)
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
inline unsigned int floor_multiple_sixteen(unsigned int v)
{
    unsigned int lg = (v / 16)*16;
    return lg;
}

inline void check_rc(int rc)
{
    if(rc == -1)
        goby::glog.is(goby::common::logger::WARN) && goby::glog << group("publish") << "Error: LCM publish failed!" << std::endl;
}

namespace drc
{    
    inline bool operator< (const ShaperPacket& a, const ShaperPacket& b)
    {
        return (a.header().priority() == b.header().priority()) ?
            a.header().fragment() > b.header().fragment() : // higher fragment has lower priority
            a.header().priority() < b.header().priority();
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
    void publish_receive(const std::string& channel,
                         int message_number,
                         std::vector<unsigned char>& lcm_data);
    void data_request_handler( goby::acomms::protobuf::ModemTransmission* msg);

    void on_demand_handler(const lcm::ReceiveBuffer *rbuf,
                           const std::string &channel,
                           const drc::shaper_data_request_t *msg)
    {
        goby::glog.is(goby::common::logger::VERBOSE) && goby::glog << "On demand request for channel: " << msg->channel
                                 << std::endl;

        std::map<std::string, MessageQueue >::iterator it =
            queues_.find(msg->channel);

        if(!fill_send_queue(it, msg->priority))
            goby::glog.is(goby::common::logger::WARN) && goby::glog << "Failed to fill request for channel: " << msg->channel
                                  << std::endl;
    }
    
    
    bool fill_send_queue(std::map<std::string, MessageQueue >::iterator it,
                         int priority = 1);
    
    // wrap message number into the [0, RECEIVE_MODULUS) space
    void receive_mod(int& i)
    {
        if(i < 0) i += RECEIVE_MODULUS;
        if(i >= RECEIVE_MODULUS) i -= RECEIVE_MODULUS;
    }
    void try_decode( std::map<int, drc::ShaperPacket>& received_frags);

    void post_bw_stats();

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

    // maps channel number to data usage
    std::map<int, DataUsage> data_usage_;

    int last_send_type_;

    int largest_id_;

    int max_frame_size_;
    
    std::priority_queue<drc::ShaperPacket> send_queue_;

    // maps channel name to special purpose enCODer/DECoder.
    std::map<std::string, boost::shared_ptr<CustomChannelCodec> > custom_codecs_;
    
    class ReceiveMessageParts
    {
      public:
      ReceiveMessageParts()
          : decoder_done_(false)
        {  }
        
        void add_fragment(const drc::ShaperPacket& fragment);
        bool decoder_done() { return decoder_done_; }
        void get_decoded(std::vector<unsigned char>* buffer)
        {
            decoder_->getObject(&(*buffer)[0]);
        }

        int expected()
        {
            if(fragments_.size())
                return ceil(fragments_.begin()->second.header().message_size()*fec_ / 
                            fragments_.begin()->second.data().size());
            else
                return -1;
        }
        
        
        std::map<int, drc::ShaperPacket>::size_type size() { return fragments_.size(); }
        
        
      private:
        bool decoder_done_;
        boost::shared_ptr<ldpc_dec_wrapper> decoder_;
        std::map<int, drc::ShaperPacket> fragments_;
    };
    
    // maps channel id to (map of message_number to MessageParts)
    std::map<int, std::map<int,  ReceiveMessageParts> > receive_queue_;    
    
    static double fec_;

    int channel_buffer_size_;
    goby::acomms::DCCLCodec* dccl_;    
};

#endif
