#ifndef CUSTOMCODECS20130506H
#define CUSTOMCODECS20130506H

#include <vector>
#include <map>
#include <cassert>

#include <boost/algorithm/string.hpp>

#include "procman-analogs.pb.h"

#include "goby/acomms/dccl.h"
#include "goby/acomms/dccl/dccl_field_codec_typed.h"
#include "goby/common/logger.h"

#include <lcmtypes/bot_procman/orders_t.hpp>
#include <lcmtypes/bot_procman/info_t.hpp>

enum Node { BASE = 0, ROBOT = 1};

class DRCEmptyIdentifierCodec : public goby::acomms::DCCLTypedFixedFieldCodec<goby::uint32>
{
  private:
    goby::acomms::Bitset encode()
    { return goby::acomms::Bitset(0, 0); }
        
    goby::acomms::Bitset encode(const goby::uint32& wire_value)
    { return goby::acomms::Bitset(0, 0); }
    
    goby::uint32 decode(goby::acomms::Bitset* bits)
    {
        return currently_decoded_id;
    }
    
    unsigned size() 
    { return 0; }

  public:
    static int currently_decoded_id;
};


class DRCPresenceBitStringCodec : public goby::acomms::DCCLTypedFieldCodec<std::string>
{
  private:
    goby::acomms::Bitset encode();
    goby::acomms::Bitset encode(const std::string& wire_value);
    std::string decode(goby::acomms::Bitset* bits);
    unsigned size();
    unsigned size(const std::string& wire_value);
    unsigned max_size();
    unsigned min_size();
    void validate();
};

template<typename WireType, typename FieldType = WireType>
    class DRCPresenceBitNumericFieldCodec : public goby::acomms::DCCLTypedFieldCodec<WireType, FieldType>
    {
      protected:

      virtual double max()
      { return goby::acomms::DCCLFieldCodecBase::dccl_field_options().max(); }

      virtual double min()
      { return goby::acomms::DCCLFieldCodecBase::dccl_field_options().min(); }

      virtual double precision()
      { return goby::acomms::DCCLFieldCodecBase::dccl_field_options().precision(); }
            
      virtual void validate()
      {
          goby::acomms::DCCLFieldCodecBase::require(goby::acomms::DCCLFieldCodecBase::dccl_field_options().has_min(), "missing (goby.field).dccl.min");
          goby::acomms::DCCLFieldCodecBase::require(goby::acomms::DCCLFieldCodecBase::dccl_field_options().has_max(), "missing (goby.field).dccl.max");


          // ensure given max and min fit within WireType ranges
          goby::acomms::DCCLFieldCodecBase::require(min() >= boost::numeric::bounds<WireType>::lowest(),
                                      "(goby.field).dccl.min must be >= minimum of this field type.");
          goby::acomms::DCCLFieldCodecBase::require(max() <= boost::numeric::bounds<WireType>::highest(),
                                      "(goby.field).dccl.max must be <= maximum of this field type.");
      }

      goby::acomms::Bitset encode()
      {
          return goby::acomms::Bitset(size(), 0);
      }          
          
      virtual goby::acomms::Bitset encode(const WireType& value)
      {
          WireType wire_value = value;
                
          if(wire_value < min() || wire_value > max())
              return encode();              
              
          wire_value -= min();
          wire_value *= std::pow(10.0, precision());

          wire_value = goby::util::unbiased_round(wire_value, 0);
          goby::acomms::Bitset bits(size(value), goby::util::as<unsigned long>(wire_value));
          bits <<= 1;
          bits.set(0, true); //presence bit
          return bits;
      }
          
      virtual WireType decode(goby::acomms::Bitset* bits)
      {
          if(bits->to_ulong())
          {
              bits->get_more_bits(max_size() - min_size());
              (*bits) >>= 1;
              unsigned long t = bits->to_ulong();
              WireType return_value = goby::util::unbiased_round(
                  t / (std::pow(10.0, precision())) + min(), precision());
              
              return return_value;
          }
          else
          {
              throw(goby::acomms::DCCLNullValueException());
          }              
      }


      unsigned max_size()
      {
          const unsigned PRESENCE_BIT = 1;
          return goby::util::ceil_log2((max()-min())*std::pow(10.0, precision())+1) + PRESENCE_BIT;
      }

      unsigned min_size()
      { return 1; }

      unsigned size()
      { return min_size(); }
      
      unsigned size(const WireType& wire_value)
      { return max_size(); }
      
    };


class CustomChannelCodec
{
  public:
    CustomChannelCodec() { }
    virtual ~CustomChannelCodec() { }
    
    // lcm_data is the LCM encoded message, transmit_data is a pointer to store whatever data this custom codec
    // wishes to transmit. Returns true if data is provided in transmit_data, returns false if this message is to be discarded
    virtual bool encode(const std::vector<unsigned char>& lcm_data, std::vector<unsigned char>* transmit_data) = 0;
    // undo encode. Returns false if this message cannot be decoded, true otherwise.
    virtual bool decode(std::vector<unsigned char>* lcm_data, const std::vector<unsigned char>& transmit_data) = 0;
};

template<typename LCMType, typename DiffType, typename Codec, typename OtherCodec>
    class PMDWrapperCodec : public CustomChannelCodec
    {
      public:
      PMDWrapperCodec(Node node)
          : node_(node),
            dccl_(goby::acomms::DCCLCodec::get())
            {
                static bool loaded_codecs = false;
                if(!loaded_codecs)
                {
                    goby::acomms::DCCLFieldCodecManager::add<DRCPresenceBitStringCodec, google::protobuf::FieldDescriptor::TYPE_STRING>("presence_bit");
                    goby::acomms::DCCLFieldCodecManager::add<DRCPresenceBitNumericFieldCodec<goby::int32> >("presence_bit");
                    goby::acomms::DCCLFieldCodecManager::add<DRCPresenceBitNumericFieldCodec<goby::int64> >("presence_bit");
                    goby::acomms::DCCLFieldCodecManager::add<DRCPresenceBitNumericFieldCodec<goby::uint32> >("presence_bit");
                    goby::acomms::DCCLFieldCodecManager::add<DRCPresenceBitNumericFieldCodec<goby::uint64> >("presence_bit");
                    goby::acomms::DCCLFieldCodecManager::add<DRCPresenceBitNumericFieldCodec<float> >("presence_bit");
                    goby::acomms::DCCLFieldCodecManager::add<DRCPresenceBitNumericFieldCodec<double> >("presence_bit");
                
                    loaded_codecs = true;
                }
            

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

            // if we have nothing to compare to, or we need to signal the other side to
            // send a full message
            if(!Codec::state_.count(lcm_object.host) || !OtherCodec::state_.count(lcm_object.host))
            {
                if(!OtherCodec::state_.count(lcm_object.host))
                    wrapper.set_request_full(true);
                    
                Codec::state_[lcm_object.host] = lcm_object;
                wrapper.set_type(drc::ProcManWrapper::FULL);
                
                wrapper.mutable_data()->resize(lcm_object.getEncodedSize());
                lcm_object.encode(&(*wrapper.mutable_data())[0], 0, wrapper.data().size());

            }
            else
            {
                DiffType diff;
                wrapper.set_type(drc::ProcManWrapper::DIFF);
                if(!make_diff(lcm_object, Codec::state_[lcm_object.host], &diff))
                    return false;

                //diff.SerializeToString(wrapper.mutable_data());
                dccl_->encode(wrapper.mutable_data(), diff);
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
    
            LCMType lcm_object;

            if(wrapper.type() == drc::ProcManWrapper::FULL)
            {    
                lcm_object.decode(&wrapper.data()[0], 0, wrapper.data().size());        

                // if we already have a full message,
                // assume this is a signal to reset the full messages
                if(wrapper.request_full())
                {
                    glog.is(VERBOSE) && glog << "Other side requested FULL message, resetting state" << std::endl;
                    OtherCodec::state_.erase(lcm_object.host);
                }
                
                Codec::state_[lcm_object.host] = lcm_object;

            }
            else if(wrapper.type() == drc::ProcManWrapper::DIFF)
            {
                std::string host = drc::ProcManWrapper::Host_Name(wrapper.host());
                boost::to_lower(host);
                if(!Codec::state_.count(host))
                    return false;
                
                DiffType diff;
                //diff.ParseFromString(wrapper.data());
                DRCEmptyIdentifierCodec::currently_decoded_id = dccl_->id<DiffType>();
                dccl_->decode(wrapper.data(), &diff);

                if(!reverse_diff(&lcm_object, Codec::state_[host], diff))
                {
                    // assume our reference is no good, so delete it and force the other side to send a new one
                    Codec::state_.erase(host);
                    return false;
                }
                
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
    
    // maps host to latest orders_t
    static std::map<std::string, bot_procman::orders_t> state_;    
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
    static std::map<std::string, bot_procman::info_t> state_;
};


#endif
