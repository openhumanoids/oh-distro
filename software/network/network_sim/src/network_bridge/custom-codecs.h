#ifndef CUSTOMCODECS20130506H
#define CUSTOMCODECS20130506H

#include <vector>
#include <map>
#include <cassert>

#include <lcmtypes/bot_procman/orders_t.hpp>
#include <lcmtypes/bot_procman/info_t.hpp>

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

class PMDOrdersCodec : public CustomChannelCodec
{
  public:
    PMDOrdersCodec()
    {
        static bool only = true;
        assert(only);
        only = false;
    }

    bool encode(const std::vector<unsigned char>& lcm_data, std::vector<unsigned char>* transmit_data);
    bool decode(std::vector<unsigned char>* lcm_data, const std::vector<unsigned char>& transmit_data);

    // maps host to latest orders_t
    static std::map<std::string, bot_procman::orders_t> orders_state_;
    
};


class PMDInfoCodec : public CustomChannelCodec
{
  public:
    PMDInfoCodec()
    {
        static bool only = true;
        assert(only);
        only = false;
    }

    bool encode(const std::vector<unsigned char>& lcm_data, std::vector<unsigned char>* transmit_data);
    bool decode(std::vector<unsigned char>* lcm_data, const std::vector<unsigned char>& transmit_data);

    // maps host to latest info_t
    static std::map<std::string, bot_procman::info_t> info_state_;
    
};


#endif
