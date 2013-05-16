#ifndef PROCMANCODECS20130514H
#define PROCMANCODECS20130514H

#include "custom-codecs.h"

#include <lcmtypes/bot_procman/orders_t.hpp>
#include <lcmtypes/bot_procman/info_t.hpp>

#include "procman-analogs.pb.h"

enum { RESEND_SECONDS = 5 };



template<typename LCMType, typename DiffType>
struct State
{
State() : need_to_send_ack_(false),
        has_last_full_(false),
        has_last_diff_(false)
        { }
    
    LCMType last_full_;
    bool has_last_full_;
    
    bool need_to_send_ack_;
    DiffType last_diff_;    
    bool has_last_diff_;
    DiffType diff_waiting_ack_;
};



template<typename LCMType, typename DiffType, typename Codec, typename OtherCodec>
    class PMDWrapperCodec : public CustomChannelCodec
    {
      public:
      PMDWrapperCodec(Node node)
          : node_(node),
            dccl_(goby::acomms::DCCLCodec::get())
            {
                dccl_->validate<DiffType>();
            }
        
        bool encode(const std::vector<unsigned char>& lcm_data, std::vector<unsigned char>* transmit_data)
        {
            using goby::glog;
            using namespace goby::common::logger;

            LCMType lcm_object;
            if(lcm_object.decode(&lcm_data[0], 0, lcm_data.size()) == -1)
                return false;
            
            drc::ProcManWrapper wrapper;

            drc::ProcManWrapper::Host host;
            if(!drc::ProcManWrapper::Host_Parse(boost::to_upper_copy(lcm_object.host), &host))
            {
                glog.is(WARN) && glog << "Warning, could not parse Host: " << lcm_object.host <<". Make sure it is defined in procman-analogs.proto" << std::endl;
                return false;
            }
            else if(host == drc::ProcManWrapper::BASE && node() == BASE)
            {
                glog.is(VERBOSE) && glog << "Not sending BASE procman messages to other nodes" << std::endl;
                return false;
            }            

            wrapper.set_host(host);
            
            State<LCMType, DiffType>& host_info = Codec::host_info_[lcm_object.host];
            
            // send an ACK
            if(host_info.need_to_send_ack_)
            {
                wrapper.set_type(drc::ProcManWrapper::ACK);
                host_info.need_to_send_ack_ = false;
            }
            // if we have nothing to compare to, or we need to signal the other side to
            // send a full message
            // send a FULL
            else if(!host_info.has_last_full_ || !OtherCodec::host_info_[lcm_object.host].has_last_full_)
            {
                if(!OtherCodec::host_info_[lcm_object.host].has_last_full_)
                    wrapper.set_request_full(true);
                    
                host_info.last_full_ = lcm_object;
                host_info.has_last_full_ = true;
                
                wrapper.set_type(drc::ProcManWrapper::FULL);
                
                wrapper.mutable_data()->resize(lcm_object.getEncodedSize());
                lcm_object.encode(&(*wrapper.mutable_data())[0], 0, wrapper.data().size());

            }
            // send a DIFF
            else
            {
                if(host_info.diff_waiting_ack_.IsInitialized())
                {
                    if(lcm_object.utime < host_info.diff_waiting_ack_.utime() + host_info.last_full_.utime + RESEND_SECONDS*1e6)
                    {
                        glog.is(VERBOSE) && glog << "Waiting for ACK for " << RESEND_SECONDS << " after last diff" << std::endl;                        
                        return false;
                    }
                    else
                    {
                        host_info.diff_waiting_ack_.Clear();
                    }
                }
                
                wrapper.set_type(drc::ProcManWrapper::DIFF);
                if(!make_diff(lcm_object, host_info.last_full_, &host_info.diff_waiting_ack_))
                    return false;
                
                if(host_info.has_last_diff_)
                {                    
                    DiffType& diff_acked = host_info.last_diff_;

                    // update time, since this is the only thing that is allowed to change without
                    // needing to send a new diff.
                    diff_acked.set_utime(host_info.diff_waiting_ack_.utime());

                    glog.is(VERBOSE) && glog << "diff_waiting_ack: " << host_info.diff_waiting_ack_.ShortDebugString() << std::endl;
                    glog.is(VERBOSE) && glog << "diff_acked: " << diff_acked.ShortDebugString() << std::endl;
                    
                    if(diff_acked.SerializeAsString() == host_info.diff_waiting_ack_.SerializeAsString())
                    {
                        glog.is(VERBOSE) && glog << "Diff is identical, so not sending" << std::endl;
                        host_info.diff_waiting_ack_.Clear();
                        return false;
                    }
                    
                }
                
                //diff.SerializeToString(wrapper.mutable_data());
                dccl_->encode(wrapper.mutable_data(), host_info.diff_waiting_ack_);
            }

            glog.is(VERBOSE) && glog << "ProcMan Message type is: "
                                     << drc::ProcManWrapper::MessageType_Name(wrapper.type())
                                     << " with host: "
                                     << drc::ProcManWrapper::Host_Name(wrapper.host()) << std::endl;
            
            transmit_data->resize(wrapper.ByteSize());
            wrapper.SerializeToArray(&(*transmit_data)[0], transmit_data->size());
            
            return true;
        }
        
        bool decode(std::vector<unsigned char>* lcm_data, const std::vector<unsigned char>& transmit_data)
        {
            using goby::glog;
            using namespace goby::common::logger;

            drc::ProcManWrapper wrapper;
            wrapper.ParseFromArray(&transmit_data[0], transmit_data.size());

            glog.is(VERBOSE) && glog << "Decoding message: " << wrapper.ShortDebugString() << std::endl;
            
            LCMType lcm_object;

            std::string host = drc::ProcManWrapper::Host_Name(wrapper.host());
            boost::to_lower(host);
            State<LCMType, DiffType>& host_info = Codec::host_info_[host];

            if(wrapper.type() == drc::ProcManWrapper::FULL)
            {    
                lcm_object.decode(&wrapper.data()[0], 0, wrapper.data().size());        

                // if we already have a full message,
                // assume this is a signal to reset the full messages
                if(wrapper.request_full())
                {
                    glog.is(VERBOSE) && glog << "Other side requested FULL message, resetting state" << std::endl;
                    OtherCodec::host_info_.erase(lcm_object.host);
                }

                host_info.last_full_ = lcm_object;
                host_info.has_last_full_ = true;
                
            }
            else if(wrapper.type() == drc::ProcManWrapper::DIFF)
            {
                if(!host_info.has_last_full_)
                    return false;
                
                DiffType diff;
                //diff.ParseFromString(wrapper.data());
                DRCEmptyIdentifierCodec::currently_decoded_id = dccl_->id<DiffType>();
                dccl_->decode(wrapper.data(), &diff);

                if(!reverse_diff(&lcm_object, host_info.last_full_, diff))
                {
                    // assume our reference is no good, so delete it and force the other side to send a new one
                    Codec::host_info_.erase(host);
                    return false;
                }
                OtherCodec::host_info_[host].need_to_send_ack_ = true;
            }
            else if(wrapper.type() == drc::ProcManWrapper::ACK)
            {
                glog.is(VERBOSE) && glog << "Received ACK for last DIFF: " <<  OtherCodec::host_info_[host].diff_waiting_ack_.ShortDebugString() << std::endl;
                if(OtherCodec::host_info_[host].diff_waiting_ack_.IsInitialized())
                {
                    OtherCodec::host_info_[host].last_diff_ = OtherCodec::host_info_[host].diff_waiting_ack_;
                    OtherCodec::host_info_[host].has_last_diff_ = true;
                    OtherCodec::host_info_[host].diff_waiting_ack_.Clear();
                }
                else
                {
                    glog.is(VERBOSE) && glog << "... we weren't expecting an ACK." << std::endl;
                }                
                return false;
            }
            
    
            lcm_data->resize(lcm_object.getEncodedSize());
            lcm_object.encode(&(*lcm_data)[0], 0, lcm_data->size());
    
            return true;
    
        }

        Node node() const { return node_; }
      protected:
        virtual bool make_diff(const LCMType& lcm_object, const LCMType& reference,
                               DiffType* diff) = 0;
        virtual bool reverse_diff(LCMType* lcm_object, const LCMType& reference,
                                  const DiffType& diff) = 0;

      private:
        Node node_;        
        goby::acomms::DCCLCodec* dccl_;
        
        
    };

class PMDInfoCodec;

class PMDOrdersCodec : public PMDWrapperCodec<bot_procman::orders_t, drc::PMDOrdersDiff, PMDOrdersCodec, PMDInfoCodec>
{
  public:
  PMDOrdersCodec(Node node)
      : PMDWrapperCodec(node)
    {
        static bool only = true;
        assert(only);
        only = false;
    }

    bool make_diff(const bot_procman::orders_t& orders, const bot_procman::orders_t& reference,
                   drc::PMDOrdersDiff* diff);
    bool reverse_diff(bot_procman::orders_t* orders, const bot_procman::orders_t& reference,
                      const drc::PMDOrdersDiff& diff);

    // maps host to latest state
    static std::map<std::string, State<bot_procman::orders_t, drc::PMDOrdersDiff> > host_info_;
};


class PMDInfoCodec : public PMDWrapperCodec<bot_procman::info_t, drc::PMDInfoDiff, PMDInfoCodec, PMDOrdersCodec>
{
  public:
  PMDInfoCodec(Node node)
      : PMDWrapperCodec(node)
    {
        static bool only = true;
        assert(only);
        only = false;
    }

    bool make_diff(const bot_procman::info_t& info, const bot_procman::info_t& reference,
                   drc::PMDInfoDiff* diff);
    bool reverse_diff(bot_procman::info_t* info, const bot_procman::info_t& reference,
                      const drc::PMDInfoDiff& diff);

    // maps host to latest info_t
    static std::map<std::string, State<bot_procman::info_t, drc::PMDInfoDiff> > host_info_;
};







#endif
