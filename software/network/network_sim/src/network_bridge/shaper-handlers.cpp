#include <iostream>
#include <queue>
#include <map>

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
#include "grasp-codecs.h"

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
void robot2base(KMCLApp& app)
{
    if(app.cl_cfg.base_only == app.cl_cfg.bot_only)
        throw(std::runtime_error("Must choose only one of base only (-b) or robot only (-r) when running drc_network_shaper"));
    
    if(!app.cl_cfg.base_only)
    {
        DRCShaper robot_shaper(app, ROBOT);
        robot_shaper.run();
    }
}

void base2robot(KMCLApp& app)
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

DRCShaper::DRCShaper(KMCLApp& app, Node node)
    : app_(app),
      node_(node),
      partner_(node == BASE ? ROBOT : BASE),
      lcm_(node == BASE ? app.base_lcm : app.robot_lcm),
      last_send_type_(0),
      largest_id_(0),
      channel_buffer_size_(100),
      dccl_(goby::acomms::DCCLCodec::get())
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

    
    custom_codecs_.insert(std::make_pair("PMD_ORDERS", boost::shared_ptr<CustomChannelCodec>(new PMDOrdersCodec(node))));
    custom_codecs_.insert(std::make_pair("PMD_INFO", boost::shared_ptr<CustomChannelCodec>(new PMDInfoCodec(node))));

    //custom_codecs_.insert(std::make_pair("EST_ROBOT_STATE", boost::shared_ptr<CustomChannelCodec>(new RobotStateCodec)));

    const std::string& ers_channel = "EST_ROBOT_STATE";
    custom_codecs_.insert(std::make_pair(ers_channel, boost::shared_ptr<CustomChannelCodec>(new RobotStateCodec(ers_channel + "_COMPRESSED_LOOPBACK")))); // 118
    custom_codecs_[ers_channel + "_COMPRESSED_LOOPBACK"] = custom_codecs_[ers_channel];

    
    const std::string& footstep_plan_channel = "COMMITTED_FOOTSTEP_PLAN";
    custom_codecs_.insert(std::make_pair(footstep_plan_channel, boost::shared_ptr<CustomChannelCodec>(new FootStepPlanCodec(footstep_plan_channel + "_COMPRESSED_LOOPBACK")))); // 118
    custom_codecs_[footstep_plan_channel + "_COMPRESSED_LOOPBACK"] = custom_codecs_[footstep_plan_channel];

    const std::string& manip_plan_channel = "COMMITTED_ROBOT_PLAN";
    custom_codecs_.insert(std::make_pair(manip_plan_channel, boost::shared_ptr<CustomChannelCodec>(new ManipPlanCodec(manip_plan_channel + "_COMPRESSED_LOOPBACK")))); 
    custom_codecs_[manip_plan_channel + "_COMPRESSED_LOOPBACK"] = custom_codecs_[manip_plan_channel];

    
    const std::string& grasp_channel = "COMMITTED_GRASP";
    custom_codecs_.insert(std::make_pair(grasp_channel, boost::shared_ptr<CustomChannelCodec>(new GraspCodec(grasp_channel + "_COMPRESSED_LOOPBACK")))); 
    custom_codecs_[grasp_channel + "_COMPRESSED_LOOPBACK"] = custom_codecs_[grasp_channel];
    
    dccl_->validate<drc::ShaperHeader>();
    
    goby::glog.is(goby::common::logger::VERBOSE) && goby::glog << *dccl_ << std::endl;

    // test pmd info diff
    {
        drc::PMDInfoDiff diff, diff_out;
        diff.set_reference_time(243);
        diff.set_utime(6000000);
        
        drc::PMDInfoDiff::PMDDeputyCmdDiff* diff_cmd = diff.add_cmds();
        diff_cmd->set_name("FOO");
        diff_cmd->set_group("BAR");
        diff_cmd->set_pid(263);
        diff_cmd->set_auto_respawn(false);
        
        std::string bytes;
        dccl_->encode(&bytes, diff);

        DRCEmptyIdentifierCodec::currently_decoded_id = dccl_->id<drc::PMDInfoDiff>();
        dccl_->decode(bytes, &diff_out);

//        std::cout << diff.DebugString() << diff_out.DebugString() << std::endl;
        assert(diff.SerializeAsString() == diff_out.SerializeAsString());

    }
        

    // test minimal robot state
    {
        drc::MinimalRobotPlan plan, plan_out;
        plan.set_utime(6000000);

        plan.mutable_goal()->set_utime(0);
        drc::TranslationVector* translation = plan.mutable_goal()->mutable_origin_position()->mutable_translation();
        drc::RotationQuaternion* rotation = plan.mutable_goal()->mutable_origin_position()->mutable_rotation();

        translation->set_x(23);
        translation->set_y(50);
        translation->set_z(8.4);

        rotation->set_x(0);
        rotation->set_y(1);
        rotation->set_z(0);
        rotation->set_w(0);


        plan.mutable_goal_diff()->add_utime_diff(100);
        plan.mutable_goal_diff()->add_utime_diff(200);
        plan.mutable_goal_diff()->add_utime_diff(300);
        drc::TranslationVectorDiff* trans_diff = plan.mutable_goal_diff()->mutable_pos_diff()->mutable_translation_diff();
        drc::RotationQuaternionDiff* rot_diff = plan.mutable_goal_diff()->mutable_pos_diff()->mutable_rotation_diff();
        

        trans_diff->add_dx(0.1);
        trans_diff->add_dy(0.2);
        trans_diff->add_dz(0.01);

        rot_diff->add_dx(0);
        rot_diff->add_dy(1);
        rot_diff->add_dz(0);
        rot_diff->add_dw(0);

        
        plan.set_num_grasp_transitions(0);
        plan.set_left_arm_control_type(1);
        plan.set_left_leg_control_type(2);
        plan.set_right_arm_control_type(4);
        plan.set_right_leg_control_type(0);

//        std::cout << plan.DebugString() << std::endl;
        
        std::string bytes;
        dccl_->encode(&bytes, plan);
        
        DRCEmptyIdentifierCodec::currently_decoded_id = dccl_->id<drc::MinimalRobotPlan>();
        dccl_->decode(bytes, &plan_out);

        while(plan_out.grasp_size() > plan_out.num_grasp_transitions())
            plan_out.mutable_grasp()->RemoveLast();
        
//        std::cout << plan_out.DebugString() << std::endl;
        assert(plan.SerializeAsString() == plan_out.SerializeAsString());
    }
        

    
    
    const std::vector<Resend>& resendlist = app.resendlist();
    
    int current_id = 0;
    for(int i = 0, n = resendlist.size(); i < n; ++i)
    {
        if(!channel_id_.left.count(resendlist[i].channel))
        {
            glog.is(VERBOSE) && glog << "Mapping: " << resendlist[i].channel << " to id: " << current_id << std::endl;
            channel_id_.insert(boost::bimap<std::string, int>::value_type(resendlist[i].channel, current_id));

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

    
    if(app.cl_cfg.file_log)
    {
        // debug log
        using namespace boost::posix_time;

        std::string goby_debug_file_name = app_.cl_cfg.log_path + "/drc-network-shaper-" + (node_ == BASE ? "base-" : "robot-") + to_iso_string(second_clock::universal_time()) + ".txt";

        flog_.open(goby_debug_file_name.c_str());
        if(!flog_.is_open())
        {
            std::cerr << "Failed to open requested debug log file: " << goby_debug_file_name << ". Check value and permissions on --logpath" << std::endl;
            exit(EXIT_FAILURE);
        }

        goby::glog.add_stream(static_cast<goby::common::logger::Verbosity>(goby::common::protobuf::GLogConfig::VERBOSE),
                              &flog_);


        // data usage log
	open_usage_log();
    }    
    
    
    largest_id_ = current_id-1;
    
    
    // outgoing 
    std::string subscription = node_ == BASE ?  app.base2robot_subscription : app.robot2base_subscription;
    lcm_subscribe(lcm_->getUnderlyingLCM(), subscription.c_str(), lcm_outgoing_handler, this);

    // on_demand
    lcm_->subscribe("SHAPER_DATA_REQUEST", &DRCShaper::on_demand_handler, this);
    
    glog.is(VERBOSE) && glog << "subscribed to: [" << subscription << "]" << std::endl;
    udp_driver_.reset(new DRCUDPDriver(&udp_service_));
    
    max_frame_size_ = bot_param_get_int_or_fail(app.bot_param, "network.udp_frame_size_bytes");
    {
        goby::acomms::protobuf::DriverConfig cfg;
        cfg.set_modem_id(node_);

        char* robot_host = bot_param_get_str_or_fail(app.bot_param, "network.robot.udp_host");
        char* base_host = bot_param_get_str_or_fail(app.bot_param, "network.base.udp_host");
        int robot_port = bot_param_get_int_or_fail(app.bot_param, "network.robot.udp_port");
        int base_port = bot_param_get_int_or_fail(app.bot_param, "network.base.udp_port");
        
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
    {
        int target_rate_bps = bot_param_get_int_or_fail(app.bot_param, "network.target_rate_bps");

        // add slots as part of cfg
        goby::acomms::protobuf::MACConfig cfg;
        cfg.set_modem_id(node_);
        cfg.set_type(goby::acomms::protobuf::MAC_FIXED_DECENTRALIZED);

        goby::acomms::protobuf::ModemTransmission slot;
        slot.set_src(node_);
        slot.set_type(goby::acomms::protobuf::ModemTransmission::DATA);
        slot.set_slot_seconds(static_cast<double>(max_frame_size_)*8/target_rate_bps);
        cfg.add_slot()->CopyFrom(slot);

        goby::acomms::bind(mac_, *udp_driver_);
        glog.is(VERBOSE) && glog << "Starting MAC with configuration: " << cfg.ShortDebugString() << std::endl;
        mac_.startup(cfg);
    }

    channel_buffer_size_ = bot_param_get_int_or_fail(app.bot_param, "network.channel_buffer_size");

    double expected_packet_loss_percent = bot_param_get_int_or_fail(app.bot_param, "network.expected_packet_loss_percent");
    fec_ = 1/(1-expected_packet_loss_percent/100);
    glog.is(VERBOSE) && glog << "Forward error correction: " << fec_ << std::endl;

    if(app.cl_cfg.verbose)
        goby::glog.add_stream(static_cast<goby::common::logger::Verbosity>(goby::common::protobuf::GLogConfig::VERBOSE), &std::cout);
    else
        goby::glog.add_stream(static_cast<goby::common::logger::Verbosity>(goby::common::protobuf::GLogConfig::WARN), &std::cout);
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
            int buffer_size = on_demand ? 1 : channel_buffer_size_;
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

    if(send_queue_.empty())
    {
        int starting_send_type = last_send_type_;

        while(1)
        {
            ++last_send_type_;
            if(last_send_type_ > largest_id_)
                last_send_type_ = 0;
            
	    //            glog.is(VERBOSE) && glog << "Checking channel id: " << last_send_type_ << std::endl;          
            //            glog.is(VERBOSE) && glog << "Name: " << channel_id_.right.at(last_send_type_) << std::endl;
            
            std::map<std::string, MessageQueue >::iterator it =
                queues_.find(channel_id_.right.at(last_send_type_));

            if(it != queues_.end() && !it->second.on_demand) // don't queue up on_demand queues here
            {
                if(fill_send_queue(it))
                    break;
            }
            
            // no data at all
            if(last_send_type_ == starting_send_type) return;
        }
    }
    
    msg->set_dest(partner_);
    msg->set_ack_requested(false);

    std::string* frame = msg->add_frame();
    std::string header;
    dccl_->encode(frame, send_queue_.top().header());
    (*frame) += send_queue_.top().data();
    
    
    glog.is(VERBOSE) && glog << group("tx") << "Sending: " << DebugStringNoData(send_queue_.top()) << std::endl;
    glog.is(VERBOSE) && glog << group("tx") << "Data size: " << send_queue_.top().data().size() << std::endl;
    glog.is(VERBOSE) && glog << group("tx") << "Encoded size: " << msg->frame(0).size() << std::endl;

    sent_data_usage_[send_queue_.top().header().channel()].sent_bytes += msg->frame(0).size();
    
    send_queue_.pop();

//    glog.is(VERBOSE) && glog << "Data request msg (with data): " << msg->DebugString() << std::endl;
}

            
bool DRCShaper::fill_send_queue(std::map<std::string, MessageQueue >::iterator it,
                                int priority /* = 1 */)
{
    if(it != queues_.end() && !it->second.messages.empty())
    {
        std::vector<unsigned char>& qmsg = it->second.messages.front();
        ++it->second.message_count;
        receive_mod(it->second.message_count);

        static int overhead = dccl_->size(drc::ShaperHeader());
        int payload_size = max_frame_size_-overhead;

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
            msg_head->set_message_number(it->second.message_count);

            payload_size = floor_multiple_sixteen(max_frame_size_-overhead);
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
        dccl_->decode(msg.frame(0), packet.mutable_header());
        packet.set_data(msg.frame(0).substr(dccl_->size(packet.header())));    

        received_data_usage_[packet.header().channel()].received_bytes += msg.frame(0).size();
    
        glog.is(VERBOSE) && glog << group("rx") <<  "received: " << app_.get_current_utime() << " | "
                                 << DebugStringNoData(packet) << std::endl;    

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
                message_parts.add_fragment(packet);            
            }
            else
            {
                message_parts.add_fragment(packet);
                if(message_parts.decoder_done())
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
        
        channel = "EST_ROBOT_STATE_TX";
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
    double elapsed_time = (now - app_.bw_init_utime)/1.0e6 ;
    double bw_window = 1.0; // bw window in seconds
    if ( elapsed_time  > bw_window  ){
        drc::bandwidth_stats_t stats;
        stats.utime = now;
        stats.previous_utime = app_.bw_init_utime;
	stats.sim_utime = app_.get_current_utime();	
        stats.num_sent_channels = sent_data_usage_.size();
        stats.num_received_channels = received_data_usage_.size();

        for (std::map<int, DataUsage>::const_iterator it = sent_data_usage_.begin(),
                 end = sent_data_usage_.end(); it != end; ++it)
        {
            stats.sent_channels.push_back(channel_id_.right.at(it->first));
            stats.queued_msgs.push_back(it->second.queued_msgs);
            stats.queued_bytes.push_back(it->second.queued_bytes);
            stats.sent_bytes.push_back(it->second.sent_bytes);
            
        }
        
        for (std::map<int, DataUsage>::const_iterator it = received_data_usage_.begin(),
                 end = received_data_usage_.end(); it != end; ++it)
        {
            stats.received_channels.push_back(channel_id_.right.at(it->first));
            stats.received_bytes.push_back(it->second.received_bytes);
        }        
        
        lcm_->publish(node_ == BASE ? "BASE_BW_STATS" : "ROBOT_BW_STATS", &stats);
        app_.bw_init_utime = stats.utime;
    }
    
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
}



void DRCShaper::ReceiveMessageParts::add_fragment(const drc::ShaperPacket& message)
{
    fragments_[message.header().fragment()] = message;
    
    if(!decoder_)
    {
        decoder_.reset(new ldpc_dec_wrapper(message.header().message_size(),
                                            message.data().size(),
                                            DRCShaper::fec_));
    }

    if(decoder_done_)
        return;
    
    int dec_done = decoder_->processPacket(
        reinterpret_cast<uint8_t*>(&fragments_[message.header().fragment()].mutable_data()->at(0)),
        message.header().fragment());

    switch(dec_done)
    {
        case 0: return;
        case 1:
            decoder_done_ = true;
            glog.is(VERBOSE) && glog << group("rx") << "Decoder done!" << std::endl;
            return;
        default:
            glog.is(WARN) && glog << group("rx") << "Error: ldpc got all the sent packets, but couldn't reconstruct... this shouldn't happen!" << std::endl;
            return;
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
            1000  // microseconds
        };
        int status = select(lcm_fd + 1, &fds, 0, 0, &timeout);
        
        if(0 == status) {
            // no messages
            udp_driver_->do_work(); 
            mac_.do_work();

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
int64_t KMCLApp::get_current_utime()
{
    return goby::common::goby_time<goby::uint64>();
}
