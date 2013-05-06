#include "custom-codecs.h"

#include <iostream>

// PMD_ORDERS

bool PMDOrdersCodec::encode(const std::vector<unsigned char>& lcm_data, std::vector<unsigned char>* transmit_data)
{
    bot_procman::orders_t orders;
    if(orders.decode(&lcm_data[0], 0, lcm_data.size()) == -1)
        return false;
    
    transmit_data->resize(orders.getEncodedSize());
    orders.encode(&(*transmit_data)[0], 0, transmit_data->size());
    
    return true;
}


bool PMDOrdersCodec::decode(std::vector<unsigned char>* lcm_data, const std::vector<unsigned char>& transmit_data)
{
    bot_procman::orders_t orders;
    orders.decode(&transmit_data[0], 0, transmit_data.size());

    lcm_data->resize(orders.getEncodedSize());
    orders.encode(&(*lcm_data)[0], 0, lcm_data->size());

    return true;
}

// PMD_INFO

bool PMDInfoCodec::encode(const std::vector<unsigned char>& lcm_data, std::vector<unsigned char>* transmit_data)
{
    bot_procman::info_t info;
    if(info.decode(&lcm_data[0], 0, lcm_data.size()) == -1)
        return false;
    
    transmit_data->resize(info.getEncodedSize());
    info.encode(&(*transmit_data)[0], 0, transmit_data->size());

    return true;
}


bool PMDInfoCodec::decode(std::vector<unsigned char>* lcm_data, const std::vector<unsigned char>& transmit_data)
{
   bot_procman::info_t info;
    info.decode(&transmit_data[0], 0, transmit_data.size());

    lcm_data->resize(info.getEncodedSize());
    info.encode(&(*lcm_data)[0], 0, lcm_data->size());

    return true;
}

