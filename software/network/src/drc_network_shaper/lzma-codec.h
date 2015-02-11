#ifndef LZMACUSTOMCODECS20130210H
#define LZMACUSTOMCODECS20150210H

class LZMACustomCodec : public CustomChannelCodec
{
  public:
    LZMACustomCodec() {}
    
    // lcm_data is the LCM encoded message, transmit_data is a pointer to store whatever data this custom codec
    // wishes to transmit. Returns true if data is provided in transmit_data, returns false if this message is to be discarded
    bool encode(const std::vector<unsigned char>& lcm_data, std::vector<unsigned char>* transmit_data)
    {
        std::string encoded = CompressWithLzma(std::string(lcm_data.begin(), lcm_data.end()),
                                               6);
        transmit_data->resize(encoded.size());
        std::copy(encoded.begin(), encoded.end(), transmit_data->begin());
        return true;
    }

// undo encode. Returns false if this message cannot be decoded, true otherwise.
    bool decode(std::vector<unsigned char>* lcm_data, const std::vector<unsigned char>& transmit_data)
    {
        std::string decoded = DecompressWithLzma(std::string(transmit_data.begin(), transmit_data.end()));
        lcm_data->resize(decoded.size());
        std::copy(decoded.begin(), decoded.end(), lcm_data->begin());                      
        return true;
    }
    
    
};
#endif
