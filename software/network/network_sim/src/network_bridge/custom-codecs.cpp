#include "custom-codecs.h"

#include <iostream>

int DRCEmptyIdentifierCodec::currently_decoded_id = 0;


using goby::glog;
using namespace goby::common::logger;




goby::acomms::Bitset DRCPresenceBitStringCodec::encode()
{
    return goby::acomms::Bitset(min_size(), 0);
}


goby::acomms::Bitset DRCPresenceBitStringCodec::encode(const std::string& wire_value)
{
    std::string s = wire_value;
    if(s.size() > dccl_field_options().max_length())
    {
        goby::glog.is(DEBUG2) && goby::glog << group(goby::acomms::DCCLCodec::glog_encode_group()) << warn << "String " << s <<  " exceeds `dccl.max_length`, truncating" << std::endl;
        s.resize(dccl_field_options().max_length()); 
    }

    goby::acomms::Bitset bits;
    bits.from_byte_string(s);
    bits.resize(size(s));
    
    bits <<= 1;
    bits.set(0, true); // presence bit    
    
    return bits;
}

unsigned DRCPresenceBitStringCodec::size()
{
    return min_size();    
}


unsigned DRCPresenceBitStringCodec::size(const std::string& wire_value)
{
    return (wire_value.size() + 1) * goby::acomms::BITS_IN_BYTE + min_size();
}


std::string DRCPresenceBitStringCodec::decode(goby::acomms::Bitset* bits)
{
    if(bits->to_ulong())
    {
        // grabs more bits to add to the MSBs of `bits`
        bits->get_more_bits(goby::acomms::BITS_IN_BYTE);
        *bits >>= 1;
        bits->resize(goby::acomms::BITS_IN_BYTE);

        std::string out;
        while(bits->to_ulong())
        {
            out.push_back(bits->to_byte_string()[0]);
            bits->get_more_bits(goby::acomms::BITS_IN_BYTE);
            *bits >>= goby::acomms::BITS_IN_BYTE;
            bits->resize(goby::acomms::BITS_IN_BYTE);
        }
        return out;
    }
    else
    {
        throw(goby::acomms::DCCLNullValueException());
    }
}

unsigned DRCPresenceBitStringCodec::max_size()
{
    return dccl_field_options().max_length() * goby::acomms::BITS_IN_BYTE + min_size();
}

unsigned DRCPresenceBitStringCodec::min_size()
{
    return 1; // presence bit
}

void DRCPresenceBitStringCodec::validate()
{
    require(dccl_field_options().has_max_length(), "missing (goby.field).dccl.max_length");
}

