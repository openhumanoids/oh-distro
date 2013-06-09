#ifndef CUSTOMCODECS20130506H
#define CUSTOMCODECS20130506H

#include <vector>
#include <map>
#include <cassert>

#include <boost/algorithm/string.hpp>

#include "goby/acomms/dccl.h"
#include "goby/acomms/dccl/dccl_field_codec_typed.h"
#include "goby/common/logger.h"

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
    class DRCPresenceBitNumericFieldCodec : public goby::acomms::DCCLRepeatedTypedFieldCodec<WireType, FieldType>
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

          if(goby::acomms::DCCLFieldCodecBase::this_field()->is_repeated())
              goby::acomms::DCCLFieldCodecBase::require(goby::acomms::DCCLFieldCodecBase::dccl_field_options().has_max_repeat(), "(dccl.field).max_repeat must be set for repeated fields");

          // ensure given max and min fit within WireType ranges
          goby::acomms::DCCLFieldCodecBase::require(min() >= boost::numeric::bounds<WireType>::lowest(),
                                      "(dccl.field).min must be >= minimum of this field type.");
          goby::acomms::DCCLFieldCodecBase::require(max() <= boost::numeric::bounds<WireType>::highest(),
                                      "(dccl.field).max must be <= maximum of this field type.");
      }

      
      goby::acomms::Bitset encode_repeated(const std::vector<WireType>& wire_values)
      {
          goby::acomms::Bitset all_bits;
          for(int i = 0, n = min(wire_values.size(), goby::acomms::DCCLFieldCodecBase::dccl_field_options().max_repeat()); i < n; ++i)
          {
              WireType wire_value = wire_values[i];
                
              if(wire_value < min() || wire_value > max())
                  break;
              
              wire_value -= min();
              wire_value *= std::pow(10.0, precision());

              wire_value = goby::util::unbiased_round(wire_value, 0);
              goby::acomms::Bitset bits(single_present_field_size(),
                                        goby::util::as<unsigned long>(wire_value));
              bits.push_front(true);
              all_bits.append(bits);
          }
          // EOF symbol
          all_bits.push_back(false);
          return all_bits;
      }
      
      WireType decode(goby::acomms::Bitset* bits)
      {
          std::vector<WireType> return_vec = decode_repeated(bits);
          if(return_vec.empty())
              throw goby::acomms::DCCLNullValueException();
          else
              return return_vec.at(0);
      }

      std::vector<WireType> decode_repeated(goby::acomms::Bitset* bits)
      {
          std::vector<WireType> return_vec;
          while(bits->to_ulong())
          {
              bits->get_more_bits(single_present_field_size());
              (*bits) >>= 1;
              unsigned long t = bits->to_ulong();
              WireType return_value = goby::util::unbiased_round(
                  t / (std::pow(10.0, precision())) + min(), precision());
              
              return_vec.push_back(return_value);
              bits->resize(0);
              bits->get_more_bits(1);
          }
          return return_vec;
      }

      unsigned size_repeated(const std::vector<WireType>& wire_values)
      {
          return single_present_field_size()*wire_values.size() + min_size_repeated();
      }

      unsigned max_size_repeated()
      {
          return single_present_field_size()*goby::acomms::DCCLFieldCodecBase::dccl_field_options().max_repeat() + min_size_repeated();
      }

      unsigned min_size_repeated()
      {
          return 1;
      }
      
      int min(int a, int b)
      { return std::min(a, b); }      

      unsigned single_present_field_size() 
      { 
          const unsigned PRESENCE_BIT = 1; 
          return goby::util::ceil_log2((max()-min())*std::pow(10.0, precision())+1) + PRESENCE_BIT; 
      }
      
    };


class DRCPresenceBitEnumFieldCodec
: public DRCPresenceBitNumericFieldCodec<goby::int32, const google::protobuf::EnumValueDescriptor*>
{
  public:
    goby::int32 pre_encode(const google::protobuf::EnumValueDescriptor* const& field_value)
    {
        return field_value->index();
    }
    
    const google::protobuf::EnumValueDescriptor* post_decode(const goby::int32& wire_value)
    {
        const google::protobuf::EnumDescriptor* e = this_field()->enum_type();
        
        if(wire_value < e->value_count())
        {
            const google::protobuf::EnumValueDescriptor* return_value = e->value(wire_value);
            return return_value;
        }
        else
            throw(goby::acomms::DCCLNullValueException());
        
    }
    
    
  private:
    void validate() { }
    
    double max()
    {
        const google::protobuf::EnumDescriptor* e = this_field()->enum_type();
        return e->value_count()-1;
    }
    double min()
    { return 0; }
};


class CustomChannelCodec
{
  public:
    CustomChannelCodec(const std::string loopback_channel = "")
        : loopback_channel_(loopback_channel)
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
            goby::acomms::DCCLFieldCodecManager::add<DRCPresenceBitEnumFieldCodec>("presence_bit");
                
            loaded_codecs = true;
        }
    }
    virtual ~CustomChannelCodec() { }
    
    // lcm_data is the LCM encoded message, transmit_data is a pointer to store whatever data this custom codec
    // wishes to transmit. Returns true if data is provided in transmit_data, returns false if this message is to be discarded
    virtual bool encode(const std::vector<unsigned char>& lcm_data, std::vector<unsigned char>* transmit_data) = 0;
    // undo encode. Returns false if this message cannot be decoded, true otherwise.
    virtual bool decode(std::vector<unsigned char>* lcm_data, const std::vector<unsigned char>& transmit_data) = 0;

    const std::string& loopback_channel()
    { return loopback_channel_; }
    
    bool has_loopback_channel()
    { return !loopback_channel_.empty(); }
    
  private:
    std::string loopback_channel_;
    
};

#endif
