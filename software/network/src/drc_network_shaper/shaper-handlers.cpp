#include <iostream>
#include <queue>
#include <map>
#include <unistd.h>
#include <cstdio>

#include <boost/bimap.hpp>
#include <boost/circular_buffer.hpp>

#include "goby/common/logger.h"
#include "goby/common/time.h"
#include "goby/acomms/connect.h"
#include "goby/acomms/bind.h"
#include "goby/acomms/amac.h"
#include "goby/acomms/dccl.h"

#include "shaper-handlers.h"
#include "procman-codecs.h"
#include "robot-state-codecs.h"
#include "footstep-plan-codecs.h"
#include "manip-plan-codecs.h"
#include "lzma-codec.h"

using namespace boost; 
using namespace std;

using goby::glog;
using namespace goby::common::logger;



double DRCShaper::fec_ = 1;


void lcm_outgoing_handler(const lcm_recv_buf_t *rbuf, const char *channel, void *user_data)
{
    ((DRCShaper*) user_data)->outgoing_handler(rbuf, channel, user_data);
}


//
// from network_bridge.h
//
void robot2base(DRCShaperApp& app)
{
    if(app.cl_cfg.base_only == app.cl_cfg.bot_only)
        throw(std::runtime_error("Must choose only one of base only (-b) or robot only (-r) when running drc_network_shaper"));
    
    if(!app.cl_cfg.base_only)
    {
        DRCShaper robot_shaper(app, ROBOT);
        robot_shaper.run();
    }
}

void base2robot(DRCShaperApp& app)
{
    if(!app.cl_cfg.bot_only)
    {
        DRCShaper base_shaper(app, BASE);
        base_shaper.run();
    }
}

//
// definition of DRCShaper
//

DRCShaper::DRCShaper(DRCShaperApp& app, Node node)
    : app_(app),
      node_(node),
      partner_(node == BASE ? ROBOT : BASE),
      lcm_(node == BASE ? app.base_lcm : app.robot_lcm),
      last_send_type_(0),
      largest_id_(0),
      dccl_(goby::acomms::DCCLCodec::get()),
      timer_(io_),
      work_(io_),
      next_slot_t_(goby::common::goby_time()),
      latency_ms_(0),
      expected_packet_loss_percent_(0),
      full_header_overhead_(0),
      fast_mode_(false)
{   
    assert(floor_multiple_sixteen(1025) == 1024);
    dccl_->add_id_codec<DRCEmptyIdentifierCodec>("drc_header_codec");    
    dccl_->set_id_codec("drc_header_codec");    
    
    glog.set_name("drc-network-shaper");
    glog.add_group("ch-push", goby::common::Colors::blue);
    glog.add_group("ch-pop", goby::common::Colors::magenta);
    glog.add_group("fragment-push", goby::common::Colors::yellow);
    glog.add_group("fragment-pop", goby::common::Colors::cyan);
    glog.add_group("tx", goby::common::Colors::blue);
    glog.add_group("rx", goby::common::Colors::magenta);
    glog.add_group("publish", goby::common::Colors::green);
    glog.add_group("rx-cleanup", goby::common::Colors::lt_green);

    goby::glog.add_stream(static_cast<goby::common::logger::Verbosity>(goby::common::protobuf::GLogConfig::VERBOSE), &std::cout);
    
    if(app.cl_cfg.enable_gui)
        goby::glog.enable_gui();

    
    if(app.cl_cfg.file_log)
    {
        // debug log
        using namespace boost::posix_time;

        std::string goby_debug_file_name = app_.cl_cfg.log_path + "/drc-network-shaper-" + (node_ == BASE ? "base-" : "robot-") + app.cl_cfg.id + "-" + to_iso_string(second_clock::universal_time()) + ".txt";

        std::string goby_debug_latest_symlink = app_.cl_cfg.log_path + "/drc-network-shaper-" + (node_ == BASE ? "base-" : "robot-") + app.cl_cfg.id + "-latest.txt";
        remove(goby_debug_latest_symlink.c_str());
        
        flog_.open(goby_debug_file_name.c_str());        
        
        if(!flog_.is_open())
        {
            std::cerr << "Failed to open requested debug log file: " << goby_debug_file_name << ". Check value and permissions on --logpath" << std::endl;
            exit(EXIT_FAILURE);
        }

        if(symlink(goby_debug_file_name.c_str(), goby_debug_latest_symlink.c_str()) != 0)
        {
            std::cerr << "Failed to create symlink: " << goby_debug_latest_symlink<< std::endl;
            exit(EXIT_FAILURE);
        }


        goby::glog.add_stream(static_cast<goby::common::logger::Verbosity>(goby::common::protobuf::GLogConfig::VERBOSE),
                              &flog_);

        dccl::dlog.connect(dccl::logger::INFO_PLUS, &flog_, true);

        // data usage log
	open_usage_log();
    }    
    

    if(app.cl_cfg.verbose)
    {
        goby::glog.add_stream(static_cast<goby::common::logger::Verbosity>(goby::common::protobuf::GLogConfig::VERBOSE), &std::cout);
        dccl::dlog.connect(dccl::logger::INFO_PLUS, &std::cout, true);
    }
    else
    {
        goby::glog.add_stream(static_cast<goby::common::logger::Verbosity>(goby::common::protobuf::GLogConfig::WARN), &std::cout);
        dccl::dlog.connect(dccl::logger::WARN_PLUS, &std::cout, true);
    }
    

    goby::acomms::DCCLFieldCodecManager::add<DRCPresenceBitStringCodec, google::protobuf::FieldDescriptor::TYPE_STRING>("presence_bit");
    goby::acomms::DCCLFieldCodecManager::add<DRCPresenceBitNumericFieldCodec<goby::int32> >("presence_bit");
    goby::acomms::DCCLFieldCodecManager::add<DRCPresenceBitNumericFieldCodec<goby::int64> >("presence_bit");
    goby::acomms::DCCLFieldCodecManager::add<DRCPresenceBitNumericFieldCodec<goby::uint32> >("presence_bit");
    goby::acomms::DCCLFieldCodecManager::add<DRCPresenceBitNumericFieldCodec<goby::uint64> >("presence_bit");
    goby::acomms::DCCLFieldCodecManager::add<DRCPresenceBitNumericFieldCodec<float> >("presence_bit");
    goby::acomms::DCCLFieldCodecManager::add<DRCPresenceBitNumericFieldCodec<double> >("presence_bit");
    goby::acomms::DCCLFieldCodecManager::add<DRCPresenceBitEnumFieldCodec>("presence_bit");

    std::string config_prefix = "network.";
    if(!app.cl_cfg.id.empty())
        config_prefix += std::string(app.cl_cfg.id + ".");
    
    
    bool disable_pmd_custom_codecs = bot_param_get_boolean_or_fail(app.bot_param, std::string(config_prefix + "disable_pmd_custom_codecs").c_str());
    if (!disable_pmd_custom_codecs){    
        load_pmd_custom_codecs();
    }

    
    bool disable_ers_custom_codecs = bot_param_get_boolean_or_fail(app.bot_param, std::string(config_prefix + "disable_ers_custom_codecs").c_str());

    if (!disable_ers_custom_codecs){    
        int add_joint_efforts = false;
        bot_param_get_boolean(app.bot_param, std::string(config_prefix + "add_ers_joint_efforts").c_str(), &add_joint_efforts);

        double key_frame_period = 5;
        bot_param_get_double(app.bot_param, std::string(config_prefix + "key_frame_period").c_str(), &key_frame_period);
        
        goby::glog.is(goby::common::logger::VERBOSE) && goby::glog << "Key frame period for EST_ROBOT_STATE: " << key_frame_period << std::endl;
        
        load_ers_custom_codecs(add_joint_efforts, key_frame_period);
    }
    
    bool disable_robot_plan_custom_codecs = bot_param_get_boolean_or_fail(app.bot_param, std::string(config_prefix + "disable_robot_plan_custom_codecs").c_str());
    if (!disable_robot_plan_custom_codecs){    
        load_robot_plan_custom_codecs();
    }

    fast_mode_ = bot_param_get_boolean_or_fail(app.bot_param,
                                               std::string(config_prefix + "fast_mode").c_str());
    
    
    load_lzma_custom_codecs();
    
    dccl_->validate<drc::ShaperHeader>();
    
    goby::glog.is(goby::common::logger::VERBOSE) && goby::glog << *dccl_ << std::endl;

    drc::ShaperHeader full_header;
    full_header.set_channel(0);
    full_header.set_priority(0);
    full_header.set_message_number(0);
    full_header.set_fragment(0);
    full_header.set_is_last_fragment(false);
    full_header.set_message_size(0);
    DRCEmptyIdentifierCodec::currently_decoded_id = dccl_->id<drc::ShaperHeader>();
    
    if(fast_mode_)
    {
        // +1 for header size byte
        full_header_overhead_ = full_header.ByteSize() + 1;
    }
    else
    {
        full_header_overhead_ = dccl_->size(full_header);
        
        std::string encoded_header;
        
        dccl_->encode(&encoded_header, full_header);
        assert(encoded_header.size() == full_header_overhead_);
    }
    
    
    const std::vector<Resend>& resendlist = app.resendlist();
    
    int current_id = 0;
    for(int i = 0, n = resendlist.size(); i < n; ++i)
    {
        if(!channel_id_.left.count(resendlist[i].channel))
        {
            glog.is(VERBOSE) && glog << "Mapping: " << resendlist[i].channel << " to id: " << current_id << std::endl;
            channel_id_.insert(boost::bimap<std::string, int>::value_type(resendlist[i].channel, current_id));
            buffer_sizes_.insert(std::make_pair(current_id, resendlist[i].buffer_size));
            priority_[resendlist[i].priority].push_back(current_id);
            
            
            if(node_ == ROBOT)
            {
                if(resendlist[i].robot2base)
                    sent_data_usage_.insert(std::make_pair(current_id, DataUsage()));
                else
                    received_data_usage_.insert(std::make_pair(current_id, DataUsage()));
            }
            else
            {
                if(resendlist[i].robot2base)
                    received_data_usage_.insert(std::make_pair(current_id, DataUsage()));
                else
                    sent_data_usage_.insert(std::make_pair(current_id, DataUsage()));
            }
            
            ++current_id;
        }
    }


    largest_id_ = current_id-1;
    
    
    // outgoing 
    std::string subscription = node_ == BASE ?  app.base2robot_subscription : app.robot2base_subscription;
    lcm_subscribe(lcm_->getUnderlyingLCM(), subscription.c_str(), lcm_outgoing_handler, this);

    // on_demand
    lcm_->subscribe("SHAPER_DATA_REQUEST", &DRCShaper::on_demand_handler, this);
    
    glog.is(VERBOSE) && glog << "subscribed to: [" << subscription << "]" << std::endl;
    udp_driver_.reset(new DRCUDPDriver(&udp_service_));
    
    max_frame_size_ = bot_param_get_int_or_fail(app.bot_param, std::string(config_prefix + "udp_frame_size_bytes").c_str());
    {
        goby::acomms::protobuf::DriverConfig cfg;
        cfg.set_modem_id(node_);

        char* robot_host = bot_param_get_str_or_fail(app.bot_param,
                                                     std::string(config_prefix + "robot.udp_host").c_str());
        char* base_host = bot_param_get_str_or_fail(app.bot_param,
                                                    std::string(config_prefix + "base.udp_host").c_str());
        int robot_port = bot_param_get_int_or_fail(app.bot_param,
                                                   std::string(config_prefix + "robot.udp_port").c_str());
        int base_port = bot_param_get_int_or_fail(app.bot_param,
                                                  std::string(config_prefix + "base.udp_port").c_str());
        
        cfg.SetExtension(UDPDriverConfig::max_frame_size, max_frame_size_);
        
        UDPDriverConfig::EndPoint* local_endpoint =
            cfg.MutableExtension(UDPDriverConfig::local);
        local_endpoint->set_port(node == BASE ? base_port : robot_port);

        UDPDriverConfig::EndPoint* remote_endpoint =
            cfg.MutableExtension(UDPDriverConfig::remote);

        remote_endpoint->set_ip(node == BASE ? robot_host : base_host);
        remote_endpoint->set_port(node == BASE ? robot_port : base_port);
        
        goby::acomms::connect(&udp_driver_->signal_receive, this, &DRCShaper::udp_data_receive);
        goby::acomms::connect(&udp_driver_->signal_data_request, this, &DRCShaper::data_request_handler);

        glog.is(VERBOSE) && glog << "Starting UDP driver with configuration: " << cfg.ShortDebugString() << std::endl;
        udp_driver_->startup(cfg);
        free(robot_host);
        free(base_host);
    }

    fallback_target_rate_bps_ = bot_param_get_int_or_fail(app.bot_param, std::string(config_prefix + "target_rate_bps").c_str());
    target_rate_bps_ = fallback_target_rate_bps_;
    
    fallback_seconds_ = 60;
    bot_param_get_int(app.bot_param, std::string(config_prefix + "fallback_seconds").c_str(), &fallback_seconds_);

    disallow_rate_change_seconds_ = 10;
    bot_param_get_int(app.bot_param, std::string(config_prefix +"disallow_rate_change_seconds").c_str(), &disallow_rate_change_seconds_);
    
    bool use_new_timer = true;
    if(!use_new_timer)
    {

        // add slots as part of cfg
        goby::acomms::protobuf::MACConfig cfg;
        cfg.set_modem_id(node_);
        cfg.set_type(goby::acomms::protobuf::MAC_FIXED_DECENTRALIZED);

        goby::acomms::protobuf::ModemTransmission slot;
        slot.set_src(node_);
        slot.set_type(goby::acomms::protobuf::ModemTransmission::DATA);
        slot.set_slot_seconds(static_cast<double>(max_frame_size_)*8/target_rate_bps_);
        cfg.add_slot()->CopyFrom(slot);

        goby::acomms::bind(mac_, *udp_driver_);
        glog.is(VERBOSE) && glog << "Starting MAC with configuration: " << cfg.ShortDebugString() << std::endl;
        mac_.startup(cfg);
    }
    else
    {
        timer_.expires_at(next_slot_t_);
        timer_.async_wait(boost::bind(&DRCShaper::begin_slot, this, _1));
        // std::cout << "timer expires at: " << next_slot_t_ << std::endl;
        
    }


    const int latency_map_size = 2;
    int latency_keys[latency_map_size];
    int latency_keys_size = bot_param_get_int_array(app.bot_param, std::string(config_prefix + "latency").c_str(), latency_keys, latency_map_size);
    int throughput[latency_map_size];
    int throughput_values_size = bot_param_get_int_array(app.bot_param, std::string(config_prefix + "throughput_bps").c_str(), throughput, latency_map_size);
    
    if(latency_keys_size != throughput_values_size)
    {
        std::cerr << config_prefix << "latency/throughput_bps pairs improperly configured" << std::endl;
        exit(EXIT_FAILURE);
    }
    glog.is(VERBOSE) && glog << "Read in latency/throughput pairs: " << std::endl;
    for(int i = 0, n = latency_keys_size; i < n; ++i)
    {
        glog.is(VERBOSE) && glog << latency_keys[i] << " ms: " << throughput[i] << " bps" << std::endl;
        latency_throughput_[latency_keys[i]] = throughput[i];
    }
    
    

    
    expected_packet_loss_percent_ = bot_param_get_int_or_fail(app.bot_param, std::string(config_prefix + "expected_packet_loss_percent").c_str());
    fec_ = 1/(1-(double)expected_packet_loss_percent_/100);
    glog.is(VERBOSE) && glog << "Forward error correction: " << fec_ << std::endl;
}

void DRCShaper::open_usage_log()
{
    using namespace boost::posix_time;

    std::string data_usage_file_name = app_.cl_cfg.log_path + "/drc-network-shaper-data-usage-" + (node_ == BASE ? "base-" : "robot-") + to_iso_string(second_clock::universal_time()) + ".csv";

    data_usage_log_.open(data_usage_file_name.c_str());
    if(!data_usage_log_.is_open())
    {
	std::cerr << "Failed to open requested CSV data log file: " << data_usage_file_name << ". Check value and permissions on --logpath" << std::endl;
	exit(EXIT_FAILURE);
    }

    if(data_usage_log_.is_open())
    {
        // Cache header strings in case or reset commands:
        if (header_string_.str().empty()){
	  header_string_ << "UTIME";
	  for(boost::bimap<std::string, int>::left_const_iterator it = channel_id_.left.begin(), end = channel_id_.left.end(); it != end; ++it)
	  {
	      bool robot2base = (node_ == ROBOT) ? sent_data_usage_.count(it->second) : received_data_usage_.count(it->second);
	      header_string_ << "," << it->first << (robot2base ? 0 : 1);
	  }
	  header_string_ << std::endl;
	}
	data_usage_log_ << header_string_.str();
    }
}





void DRCShaper::outgoing_handler(const lcm_recv_buf_t *rbuf, const char *channel, void *user_data)
{
    
    // Determine if the message should be dropped or sent (and then send)
    bool unused;
    bool on_demand = false;
    if (app_.determine_resend_from_list(channel, app_.get_current_utime(), unused, rbuf->data_size, &on_demand))
    {
        sent_data_usage_[channel_id_.left.at(channel)].queued_msgs++;
        sent_data_usage_[channel_id_.left.at(channel)].queued_bytes += rbuf->data_size;
        
        std::vector<unsigned char> data((uint8_t*)rbuf->data, (uint8_t*)rbuf->data + rbuf->data_size);
        if(custom_codecs_.count(channel))
        {
            std::vector<unsigned char> lcm_data = data;
            glog.is(VERBOSE) && glog << group("ch-push") << "running custom encoder: " << app_.get_current_utime() << " | " << channel  << " | " << std::endl;
            if(!custom_codecs_[channel]->encode(lcm_data, &data))
            {
                glog.is(VERBOSE) && glog << group("ch-push") << "custom encoder returns false, discarding message." << std::endl;
                return;
            }

            // for debugging
            if(custom_codecs_[channel]->has_loopback_channel())
            {
                std::vector<unsigned char> loopback_data = data;
                publish_receive(custom_codecs_[channel]->loopback_channel(), 0, loopback_data);
            }
        }
        
        std::map<std::string, MessageQueue>::iterator q_it = queues_.find(channel);
        
        // if this is the first time we've seen this channel, create a new circular buffer for it
        if(q_it == queues_.end())
        {
            int buffer_size = on_demand ? 1 : buffer_sizes_[channel_id_.left.at(channel)];
            glog.is(VERBOSE) && glog << group("ch-push") << "creating queue for [" << channel << "] with buffer size: " << buffer_size << std::endl;

            queues_.insert(std::make_pair(channel, MessageQueue(channel, buffer_size, data, on_demand)));

        }
        else
        {
            // add it to the end of the buffer
            q_it->second.messages.push_back(data);
        }

        
        glog.is(VERBOSE) && glog << group("ch-push") << "queueing: " << app_.get_current_utime()
                                 << " | " << channel  << " | "
                                 << "qsize: " << queues_.find(channel)->second.messages.size()
                                 << " | " << data.size() << " bytes *" << xor_cs(data)
                                 << " | on_demand: " << std::boolalpha << on_demand 
                                 << std::endl;
    }
}

void DRCShaper::data_request_handler(goby::acomms::protobuf::ModemTransmission* msg)
{
    // TODO: check that send_queue has highest priority item
    
    // highest priority first
    for(std::map<int, std::list<int> >::reverse_iterator p_it = priority_.rbegin(),
            end = priority_.rend(); p_it != end; ++p_it)
    {
        int highest_existing_priority = send_queue_.empty() ? -1 : send_queue_.top().header().priority();
//        glog.is(VERBOSE) && glog << group("tx") << "Current highest priority is: " << highest_existing_priority << std::endl;        

        // no point to keep checking, we've got something good enough already
        if(p_it->first <= highest_existing_priority)
            break;
        
        std::list<int>& ids = p_it->second;

        for(std::list<int>::iterator iit = ids.begin(),
                iend = ids.end(); iit != iend; ++iit)
        {
            std::map<std::string, MessageQueue >::iterator q_it =
                queues_.find(channel_id_.right.at(*iit));

//            glog.is(VERBOSE) && glog << group("tx") << "Checking priority: " << p_it->first
//                                     << ", Checking id: " << *iit << std::endl;
            
            if(q_it != queues_.end() && !q_it->second.on_demand) // don't queue up on_demand queues here
            {
                if(fill_send_queue(q_it, p_it->first))
                {
                    // move this id to the end so it's the last one next time
                    ids.splice(ids.end(), ids, iit);
                    break;
                }
            }
        }
        
        
    }

    // no data at all
    if(send_queue_.empty())
    {
        next_slot_t_ += boost::posix_time::milliseconds(1);
        timer_.expires_at(next_slot_t_);
        timer_.async_wait(boost::bind(&DRCShaper::begin_slot, this, _1));
        return;
    }
    
    msg->set_dest(partner_);
    msg->set_ack_requested(false);

    std::string* frame = msg->add_frame();
    std::string header;
    drc::ShaperHeader header_msg = send_queue_.top().header();

    if(fast_mode_)
    {
        *frame = std::string(1, header_msg.ByteSize() & 0xff);
        *frame += header_msg.SerializeAsString();
    }
    else
    {
        dccl_->encode(frame, header_msg);
    }
    
        
    (*frame) += send_queue_.top().data();
    
    int encoded_size = msg->frame(0).size();
    glog.is(VERBOSE) && glog << group("tx") << "Sending: " << DebugStringNoData(send_queue_.top()) << std::endl;
    glog.is(VERBOSE) && glog << group("tx") << "Data size: " << send_queue_.top().data().size() << std::endl;
    glog.is(VERBOSE) && glog << group("tx") << "Encoded size: " << encoded_size << std::endl;

    const int UDP_HEADER_BYTES = 20;
    const int ETHERNET_HEADER_BYTES = 14;
    
    next_slot_t_ += boost::posix_time::microseconds(((encoded_size+UDP_HEADER_BYTES+ETHERNET_HEADER_BYTES)*8.0)/target_rate_bps_*1e6);
    timer_.expires_at(next_slot_t_);
    timer_.async_wait(boost::bind(&DRCShaper::begin_slot, this, _1));
    
    sent_data_usage_[send_queue_.top().header().channel()].sent_bytes += msg->frame(0).size() + UDP_HEADER_BYTES;
    
    send_queue_.pop();

//    glog.is(VERBOSE) && glog << "Data request msg (with data): " << msg->DebugString() << std::endl;
}

            
bool DRCShaper::fill_send_queue(std::map<std::string, MessageQueue >::iterator it,
                                int priority /* = 1 */)
{
    if(it != queues_.end() && !it->second.messages.empty())
    {
        std::vector<unsigned char>& qmsg = it->second.messages.front();

        int payload_size = max_frame_size_-full_header_overhead_;

        drc::ShaperPacket msg_frag;
        drc::ShaperHeader* msg_head = msg_frag.mutable_header();
        
        int16_t fragment = 0;
        msg_head->set_channel(channel_id_.left.at(it->second.channel));
        msg_head->set_priority(priority);
        
        if(qmsg.size() <= payload_size)
        {
            msg_frag.set_data(&qmsg[0], qmsg.size());
            msg_head->set_is_last_fragment(true);
            for(int i = 0, n = round(fec_); i < n; ++i)
                send_queue_.push(msg_frag);
        }
        else
        {
            msg_head->set_message_size(qmsg.size());
            ++it->second.message_count;
            receive_mod(it->second.message_count);
            msg_head->set_message_number(it->second.message_count);

            payload_size = floor_multiple_sixteen(max_frame_size_-full_header_overhead_);
            while(qmsg.size() / payload_size < MIN_NUM_FRAGMENTS_FOR_FEC)
                payload_size = floor_multiple_sixteen(payload_size-1);

            if(payload_size == 0)
                throw(std::runtime_error("udp_frame_size_bytes is too small for LDPC error correction, use a larger value"));
                    
            ldpc_enc_wrapper encoder(reinterpret_cast<uint8_t*>(&qmsg[0]),
                                     qmsg.size(),
                                     payload_size,
                                     fec_);
            bool enc_done = false;
            std::vector<drc::ShaperPacket> encoded_fragments;
            while (!enc_done)
            {
                std::vector<uint8_t> buffer(payload_size);
                enc_done = encoder.getNextPacket(&buffer[0], &fragment);
                msg_frag.set_data(&buffer[0], payload_size);
                msg_head->set_fragment(fragment);
                msg_head->set_is_last_fragment(false);
                encoded_fragments.push_back(msg_frag);
            }              
            std::sort(encoded_fragments.begin(), encoded_fragments.end());
            encoded_fragments.at(0).mutable_header()->set_is_last_fragment(true);

            for(std::vector<drc::ShaperPacket>::const_reverse_iterator it = encoded_fragments.rbegin(),
                    end = encoded_fragments.rend(); it != end; ++it)
            {
                glog.is(VERBOSE) && glog << group("fragment-push") << "pushing fragment to send: " << DebugStringNoData(*it) << std::endl;
                send_queue_.push(*it);
            }
        }

        glog.is(VERBOSE) && glog << group("ch-pop") << "dequeueing: " << app_.get_current_utime() << " | "
                                 << it->second.channel << " msg #" << it->second.message_count << " | " << "qsize: " << queues_.find(it->second.channel)->second.messages.size() << " | " << qmsg.size() << " bytes *" << xor_cs(qmsg) << std::endl;

        it->second.messages.pop_front();
        return true;
    }
    else
        return false;        
}
        



void DRCShaper::udp_data_receive(const goby::acomms::protobuf::ModemTransmission& msg)
{
    try
    {
        
        drc::ShaperPacket packet;

        DRCEmptyIdentifierCodec::currently_decoded_id = dccl_->id<drc::ShaperHeader>();


        if(fast_mode_)
        {
            unsigned header_size = (msg.frame(0)[0] & 0xff);
            packet.mutable_header()->ParseFromString(msg.frame(0).substr(1, header_size+1));
            packet.set_data(msg.frame(0).substr(header_size+1));
        }
        else
        {
            dccl_->decode(msg.frame(0), packet.mutable_header());
            packet.set_data(msg.frame(0).substr(dccl_->size(packet.header())));
        }
        
        received_data_usage_[packet.header().channel()].received_bytes += msg.frame(0).size();


        glog.is(VERBOSE) && glog << group("rx") <<  "received: " << app_.get_current_utime() << " | " << DebugStringNoData(packet) << std::endl;


        
        if(packet.header().is_last_fragment()
           && !packet.header().has_fragment())
        {
            std::vector<unsigned char> buffer(packet.data().size());
            for(std::string::size_type i = 0, n = packet.data().size(); i < n; ++i)
                buffer[i] = packet.data()[i];

            publish_receive(channel_id_.right.at(packet.header().channel()),
                            packet.header().message_number(),
                            buffer);
        }
        else
        {
            std::map<int, ReceiveMessageParts>& this_queue = receive_queue_[packet.header().channel()];
            // do cleanup on oldest 1/2 messages
            int begin = packet.header().message_number()-3*RECEIVE_MODULUS/4;
            receive_mod(begin);
            int end = packet.header().message_number()-RECEIVE_MODULUS/4;
            receive_mod(end);
            
            for(int i = begin; i != end; )
            {
                //glog.is(VERBOSE) && glog << "Cleanup on Message #" << i << ": " << std::endl;

                std::map<int, ReceiveMessageParts>::iterator it = this_queue.find(i);
            
                if(it == this_queue.end() || it->second.size() == 0)
                {
                    // glog.is(VERBOSE) && glog << "Ok, packet has been processed." << std::endl;
                }
                else
                {
                    int expected = it->second.expected();
                    int received = it->second.size();
                    if(it->second.decoder_done())
                    {                
                        glog.is(VERBOSE) &&
                            glog << group("rx-cleanup") << "Received enough fragments to decode message " << i << " from channel: "
                                 << channel_id_.right.at(packet.header().channel()) << " - received " << received
                                 << " of " << expected << std::endl;
                    }
                    else
                    {
                        glog.is(WARN) &&
                            glog <<  group("rx-cleanup") <<  "Not enough fragments received for message " << i << " from channel: "
                                 << channel_id_.right.at(packet.header().channel()) << " - received " << received
                                 << " of " << expected << std::endl;
                    }
                    this_queue.erase(it);
                }

                ++i;
                receive_mod(i);
            }

            ReceiveMessageParts& message_parts = this_queue[packet.header().message_number()];
            if(message_parts.decoder_done())
            {
                // already done and published, just add for statistics
                if(!message_parts.add_fragment(packet))
                    this_queue.erase(packet.header().message_number());
            }
            else
            {
                if(!message_parts.add_fragment(packet))
                    this_queue.erase(packet.header().message_number());
                else if(message_parts.decoder_done())
                {
                    std::vector<unsigned char> buffer(packet.header().message_size());
                    message_parts.get_decoded(&buffer);
                    publish_receive(channel_id_.right.at(packet.header().channel()),
                                    packet.header().message_number(),
                                    buffer);
                }
            }
        }    
    }
    catch(std::exception& e)
    {
        glog.is(WARN) && glog << "Failed to handle incoming UDP message. Reason: " << e.what() << std::endl;
    }
}

void DRCShaper::publish_receive(std::string channel,
                                int message_number,
                                std::vector<unsigned char>& lcm_data)
{
    
    if(custom_codecs_.count(channel))
    {
        std::vector<unsigned char> transmit_data = lcm_data;
        glog.is(VERBOSE) && glog << group("ch-pop") << "running custom decoder: " << app_.get_current_utime() << " | " << channel  << " | " << std::endl;
        if(!custom_codecs_[channel]->decode(&lcm_data, transmit_data))
        {
            glog.is(VERBOSE) && glog << group("ch-pop") << "custom decoder returns false, discarding message." << std::endl;
            return;
        }
    }

    // non-standard publish for this
    if(channel == "EST_ROBOT_STATE")
    {
        drc::robot_state_t robot_state;
        if(robot_state.decode(&lcm_data[0], 0, lcm_data.size()) != -1)
        {
            drc::utime_t t;
            t.utime = robot_state.utime;            
            lcm_->publish("ROBOT_UTIME", &t);
        }
        

        // static int64_t newest_ers_utime = 0;

        // if(newest_ers_utime > robot_state.utime)
        // {
        //     glog.is(WARN) && glog << group("ch-pop") << "EST_ROBOT_STATE message received with older time than latest ... discarding message." << std::endl;
        //     return;
        // }
        // else
        // {
        //     newest_ers_utime = robot_state.utime;
        // }
    }
    
    glog.is(VERBOSE) && glog << group("publish")
                             << "publishing: " << app_.get_current_utime()
                             << " | " << channel
                             << " #" << message_number << " | "
                             << lcm_data.size() << " bytes *" << xor_cs(lcm_data) << std::endl;

    check_rc(lcm_publish(lcm_->getUnderlyingLCM(), channel.c_str(), 
                         &lcm_data[0], lcm_data.size()));
}



void DRCShaper::post_bw_stats()
{

   if (app_.get_reset_usage_stats() ){
      std::cout << "Received request to reset the stats. Zeroing all transmission counts\n";
      data_usage_log_.close();
      
      for (std::map<int, DataUsage>::iterator it = sent_data_usage_.begin(),
               end = sent_data_usage_.end(); it != end; ++it)
          it->second = DataUsage();

      for (std::map<int, DataUsage>::iterator it = received_data_usage_.begin(),
               end = received_data_usage_.end(); it != end; ++it)
          it->second = DataUsage();      
      
      open_usage_log();
      app_.set_reset_usage_stats(false);
    }	  
 
    uint64_t now = goby::common::goby_time<uint64_t>();
    drc::bandwidth_stats_t stats;
    stats.utime = now;
    stats.previous_utime = app_.bw_init_utime;
    stats.sim_utime = app_.get_current_utime();	
    stats.num_sent_channels = sent_data_usage_.size();
    stats.num_received_channels = received_data_usage_.size();

    static int last_sent_bytes = 0;
    int sent_bytes = 0;
    for (std::map<int, DataUsage>::const_iterator it = sent_data_usage_.begin(),
             end = sent_data_usage_.end(); it != end; ++it)
    {
        stats.sent_channels.push_back(channel_id_.right.at(it->first));
        stats.queued_msgs.push_back(it->second.queued_msgs);
        stats.queued_bytes.push_back(it->second.queued_bytes);
        stats.sent_bytes.push_back(it->second.sent_bytes);
        sent_bytes += it->second.sent_bytes;
    }
    glog.is(VERBOSE) && glog << group("rx") << "Sent Rate: " << (sent_bytes-last_sent_bytes)*8/1.0 << " bps" << std::endl;
    glog.is(VERBOSE) && glog << group("rx") << "Target Rate: " << target_rate_bps_ << " bps" << std::endl;
    last_sent_bytes = sent_bytes;

    glog.is(VERBOSE) && glog << group("rx") << "Latency: " << latency_ms_ << " ms" << std::endl;

    static int64_t utime_last_bps_change = 0;
    if(latency_throughput_.size())
    {
        // find the closest latency to the measured value
        std::map<int, int>::iterator upper_it = latency_throughput_.lower_bound(latency_ms_);
        std::map<int, int>::iterator lower_it = upper_it;
        
        int upper_error, lower_error;
        if(upper_it == latency_throughput_.end())
            upper_it--;
        if(lower_it != latency_throughput_.begin())
            lower_it--;

        upper_error = std::abs(upper_it->first - latency_ms_);
        lower_error = std::abs(lower_it->first - latency_ms_);

        
        int new_target_rate_bps = (upper_error < lower_error) ? upper_it->second : lower_it->second;

        if(new_target_rate_bps != target_rate_bps_ && ((now - utime_last_bps_change) / 1000000 > disallow_rate_change_seconds_))
        {
            utime_last_bps_change = now;
            target_rate_bps_ = new_target_rate_bps;
        }

        if(fallback_target_rate_bps_ != target_rate_bps_ && (now - utime_last_bps_change) / 1000000 > fallback_seconds_)
        {
            glog.is(VERBOSE) && glog << group("rx") << "More than " << fallback_seconds_ << " seconds since last rate change; conservatively switching us back to fallback rate" << std::endl;        
            utime_last_bps_change = now;
            target_rate_bps_ = fallback_target_rate_bps_;
        }
        
        
        glog.is(VERBOSE) && glog << group("rx") << "Setting throughput to: " << target_rate_bps_ << " bps" << std::endl;

    }
    
    stats.utime_last_bps_change = utime_last_bps_change;
    
    stats.estimated_latency_ms = latency_ms_;
    stats.target_bps = target_rate_bps_;
    
    for (std::map<int, DataUsage>::const_iterator it = received_data_usage_.begin(),
             end = received_data_usage_.end(); it != end; ++it)
    {
        stats.received_channels.push_back(channel_id_.right.at(it->first));
        stats.received_bytes.push_back(it->second.received_bytes);
    }        
        
    lcm_->publish(node_ == BASE ? std::string("BASE_BW_STATS_" + boost::to_upper_copy(app_.cl_cfg.id)) : std::string("ROBOT_BW_STATS_" + boost::to_upper_copy(app_.cl_cfg.id)), &stats);
    app_.bw_init_utime = stats.utime;
    
    if(data_usage_log_.is_open())
    {
        data_usage_log_ << app_.get_current_utime();
        for(boost::bimap<std::string, int>::left_const_iterator it = channel_id_.left.begin(), end = channel_id_.left.end(); it != end; ++it)
        {
            int bytes = (sent_data_usage_.count(it->second)) ? sent_data_usage_[it->second].sent_bytes : received_data_usage_[it->second].received_bytes;
            data_usage_log_ << "," << bytes;
        }
        data_usage_log_ << std::endl;
    }

    drc::utime_t t;
    t.utime = now;

    if(node_ == BASE)
        lcm_->publish("BASE_UTIME", &t);

}



bool DRCShaper::ReceiveMessageParts::add_fragment(const drc::ShaperPacket& message)
{
    fragments_[message.header().fragment()] = message;
    
    if(!decoder_)
    {
        decoder_.reset(new ldpc_dec_wrapper(message.header().message_size(),
                                            message.data().size(),
                                            DRCShaper::fec_));
    }

    
    
    if(decoder_done_)
        return true;
    
    int dec_done = decoder_->processPacket(
        reinterpret_cast<uint8_t*>(&fragments_[message.header().fragment()].mutable_data()->at(0)),
        message.header().fragment());
    
    switch(dec_done)
    {
        case 0:
            if(message.header().is_last_fragment())
            {
                glog.is(WARN) && glog << group("rx") << "Error: ldpc got last sent packet, but thought it wasn't done decoding!" << std::endl;
                return false;
            }
            else
            {
                return true;
            }
        case 1:
            decoder_done_ = true;
            glog.is(VERBOSE) && glog << group("rx") << "Decoder done!" << std::endl;
            return true;
            
        default:
            glog.is(WARN) && glog << group("rx") << "Error: ldpc got all the sent packets, but couldn't reconstruct... this shouldn't happen!" << std::endl;
            return false;
    }    
}

void DRCShaper::run()
{
    double last_bw_stats_time = goby::common::goby_time<double>();
    while (1)
    {
        int lcm_fd = lcm_get_fileno(lcm_->getUnderlyingLCM());
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(lcm_fd, &fds);
        
        // wait a limited amount of time for an incoming message
        struct timeval timeout = { 
            0, // seconds
            100  // microseconds
        };
        int status = select(lcm_fd + 1, &fds, 0, 0, &timeout);
        
        if(0 == status) {
            // no messages
            udp_driver_->do_work(); 

            //mac_.do_work();
            
            io_.poll();
            
            double now = goby::common::goby_time<double>();
            if(now > last_bw_stats_time + 1)
            {
                post_bw_stats();
                last_bw_stats_time += 1;
            }
        } else if(FD_ISSET(lcm_fd, &fds)) {
            // LCM has events ready to be processed.
            lcm_handle(lcm_->getUnderlyingLCM());
        }
    }
}

// changed to use wall time
int64_t DRCShaperApp::get_current_utime()
{
    return goby::common::goby_time<goby::uint64>();
}

void DRCShaper::load_lzma_custom_codecs()
{
    custom_codecs_.insert(std::make_pair("COMMITTED_FOOTSTEP_PLAN", boost::shared_ptr<CustomChannelCodec>(new LZMACustomCodec)));
}



void DRCShaper::load_pmd_custom_codecs()
{
    custom_codecs_.insert(std::make_pair("PMD_ORDERS2", boost::shared_ptr<CustomChannelCodec>(new PMDOrdersCodec(node_))));
    custom_codecs_.insert(std::make_pair("PMD_INFO2", boost::shared_ptr<CustomChannelCodec>(new PMDInfoCodec(node_))));    


    // test pmd info diff
        {
            drc::PMDInfoDiff diff, diff_out;
            diff.set_reference_time(243);
            diff.set_utime(6000000);
            
            drc::PMDInfoDiff::PMDDeputyCmdDiff* diff_cmd = diff.add_cmds();
            diff_cmd->mutable_cmd()->set_name("FOO");
            diff_cmd->mutable_cmd()->set_group("BAR");
            diff_cmd->set_pid(263);
            diff_cmd->mutable_cmd()->set_auto_respawn(false);
            
            
            std::string bytes;
            dccl_->encode(&bytes, diff);
            
            DRCEmptyIdentifierCodec::currently_decoded_id = dccl_->id<drc::PMDInfoDiff>();
            dccl_->decode(bytes, &diff_out);
            
//        std::cout << diff.DebugString() << diff_out.DebugString() << std::endl;
            assert(diff.SerializeAsString() == diff_out.SerializeAsString());
            
        }
        

}

void DRCShaper::load_ers_custom_codecs(bool add_joint_efforts, double key_frame_period)
{
    int ers_freq = 0;
    for(int i = 0, n = app_.resendlist().size(); i < n; ++i)
    {
        if(app_.resendlist()[i].channel == "EST_ROBOT_STATE")
        {
            ers_freq = app_.resendlist()[i].max_freq;
            break;
        }
    }
    
    glog.is(VERBOSE) && glog << "EST_ROBOT_STATE frequency is: " << ers_freq << std::endl;
    
    const std::string& ers_channel = "EST_ROBOT_STATE";
    custom_codecs_.insert(std::make_pair(ers_channel, boost::shared_ptr<CustomChannelCodec>(new RobotStateCodec(ers_channel + "_COMPRESSED_LOOPBACK", ers_freq, add_joint_efforts, key_frame_period)))); // 118
    custom_codecs_[ers_channel + "_COMPRESSED_LOOPBACK"] = custom_codecs_[ers_channel];

    {
        drc::MinimalRobotState state, state_out;
        state.set_utime(1420023560e6);
        drc::TranslationVector* translation = state.mutable_pose()->mutable_translation();
        drc::RotationQuaternion* rotation = state.mutable_pose()->mutable_rotation();

        translation->set_x(23);
        translation->set_y(50);
        translation->set_z(8.4);

        rotation->set_x(0);
        rotation->set_y(1);
        rotation->set_z(0);
        rotation->set_w(0);

        state.set_l_foot_force_z(1004.5);
        state.set_r_foot_force_z(1005.5);

        state.set_l_foot_torque_x(234.6);
        state.set_r_foot_torque_x(-432.4);

        state.set_l_foot_torque_y(222.6);
        state.set_r_foot_torque_y(-1.5);
        
        std::string bytes;
        dccl_->encode(&bytes, state);
        
        DRCEmptyIdentifierCodec::currently_decoded_id = dccl_->id<drc::MinimalRobotState>();
        dccl_->decode(bytes, &state_out);
//        std::cout << state.DebugString() << std::endl;
//        std::cout << state_out.DebugString() << std::endl;
        assert(state.SerializeAsString() == state_out.SerializeAsString());
    }
}

void DRCShaper::load_robot_plan_custom_codecs()
{
    
        // test minimal robot state
    {
        drc::MinimalRobotPlan plan, plan_out;
        plan.set_utime(60000);

        plan.mutable_goal()->set_utime(1420023598e6);

        drc::TranslationVector* translation = plan.mutable_goal()->mutable_pose()->mutable_translation();
        drc::RotationQuaternion* rotation = plan.mutable_goal()->mutable_pose()->mutable_rotation();

        translation->set_x(23);
        translation->set_y(50);
        translation->set_z(8.4);
        
        rotation->set_x(0);
        rotation->set_y(1);
        rotation->set_z(0);
        rotation->set_w(0);


        plan.mutable_goal_diff()->add_utime_diff(100e3);
        plan.mutable_goal_diff()->add_utime_diff(200e3);
        plan.mutable_goal_diff()->add_utime_diff(300e3);
        drc::TranslationVectorDiff* trans_diff = plan.mutable_goal_diff()->mutable_pos_diff()->mutable_translation_diff();
        drc::RotationQuaternionDiff* rot_diff = plan.mutable_goal_diff()->mutable_pos_diff()->mutable_rotation_diff();
        

        trans_diff->add_dx(0.1);
        trans_diff->add_dy(0.2);
        trans_diff->add_dz(0.01);

        rot_diff->add_dx(0);
        rot_diff->add_dy(1);
        rot_diff->add_dz(0);
        rot_diff->add_dw(0);

        
        plan.set_left_arm_control_type(1);
        plan.set_left_leg_control_type(2);
        plan.set_right_arm_control_type(4);
        plan.set_right_leg_control_type(0);

        
//        std::cout << plan.DebugString() << std::endl;
        
        std::string bytes;
        dccl_->encode(&bytes, plan);
        
        DRCEmptyIdentifierCodec::currently_decoded_id = dccl_->id<drc::MinimalRobotPlan>();
        dccl_->decode(bytes, &plan_out);

//        std::cout << plan_out.DebugString() << std::endl;
        
        assert(plan.SerializeAsString() == plan_out.SerializeAsString());
    }

    std::string footstep_plan_channel = "WALKING_CONTROLLER_PLAN_REQUEST";
    custom_codecs_.insert(std::make_pair(footstep_plan_channel, boost::shared_ptr<CustomChannelCodec>(new FootStepPlanCodec(footstep_plan_channel + "_COMPRESSED_LOOPBACK"))));
    custom_codecs_[footstep_plan_channel + "_COMPRESSED_LOOPBACK"] = custom_codecs_[footstep_plan_channel];
    
    const std::string& supports_plan_channel = "COMMITTED_ROBOT_PLAN_WITH_SUPPORTS";
    custom_codecs_.insert(std::make_pair(supports_plan_channel, boost::shared_ptr<CustomChannelCodec>(new SupportsPlanCodec(supports_plan_channel + "_COMPRESSED_LOOPBACK")))); 
    custom_codecs_[supports_plan_channel + "_COMPRESSED_LOOPBACK"] = custom_codecs_[supports_plan_channel];
    
}


void DRCShaper::begin_slot(const boost::system::error_code& e)
{        
    // canceled the last timer
    if(e == boost::asio::error::operation_aborted) return;   

    goby::acomms::protobuf::ModemTransmission s;
    s.set_type(goby::acomms::protobuf::ModemTransmission::DATA);
    s.set_src(0);

    udp_driver_->handle_initiate_transmission(s);    
}
