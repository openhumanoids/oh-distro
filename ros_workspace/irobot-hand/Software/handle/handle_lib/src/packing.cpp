/**
 * \file packing.cpp
 *
 * Functions for bit packing to send bytes over the wire.
 *
 * Written under government funding for ARM-H project.
 *
 * \author Ben Axelrod
 * \date   May 2012
 * \copyright Copyright iRobot Corporation, 2012
 **/

#include "../include/handle_lib/packing.hpp"

/// store a 8-bit char into a char buffer
int pack(unsigned char* buf, char c)
{
    *buf = c;
    return 1;
};

/// store a 16-bit short into a char buffer (like htons())
int pack(unsigned char* buf, short s)
{
    *buf++ = s>>8; 
    *buf++ = s;
    return 2;
};

/// store a 32-bit int into a char buffer (like htonl())
int pack(unsigned char* buf, int i)
{
    *buf++ = i>>24; 
    *buf++ = i>>16;
    *buf++ = i>>8;  
    *buf++ = i;
    return 4;
};

/// unpack a 8-bit char from a char buffer
int unpack(const unsigned char* const buf, char* c)
{
    *c = buf[0];
    return 1;
};

/// unpack a 16-bit short from a char buffer (like ntohs())
int unpack(const unsigned char* const buf, short* s)
{
    *s = (buf[0]<<8) | buf[1];
    return 2;
};

/// unpack a 32-bit int from a char buffer (like ntohl())
int unpack(const unsigned char* const buf, int* i)
{
    *i = (buf[0]<<24) | (buf[1]<<16) | (buf[2]<<8) | buf[3];
    return 4;
};

/// powerful packing for floats and doubles.  based on IEEE-754 format.
// use one of the helper functions below.
long pack754(long double f, unsigned bits, unsigned expbits)
{
    long double fnorm;
    int shift;
    long long sign, exp, significand;
    unsigned significandbits = bits - expbits - 1; // -1 for sign bit

    if (f == 0.0) 
        return 0; // get this special case out of the way
    
    if (isnan(f)) // nan
        return 0xFFFFFFFFl;

    if (isinf(f) > 0) // positive infinity
    {
        // 0 sign
        // exp all 1s
        // 0 fraction
        long ret = 0;
        for (unsigned i=0; i<expbits; i++)
            ret |= 0x1 << (bits-expbits-1+i);
        return ret;
    }

    if (isinf(f) < 0) // negative infinity
    {
        // 1 sign
        // exp all 1s
        // 0 fraction
        long ret = 0x1 << (bits-1);
        for (unsigned i=0; i<expbits; i++)
            ret |= 0x1 << (bits-expbits-1+i);
        return ret;
    }

    // check sign and begin normalization
    if (f < 0) { sign = 1; fnorm = -f; }
    else { sign = 0; fnorm = f; }

    // get the normalized form of f and track the exponent
    shift = 0;
    while(fnorm >= 2.0) { fnorm /= 2.0; shift++; }
    while(fnorm < 1.0) { fnorm *= 2.0; shift--; }
    fnorm = fnorm - 1.0;

    // calculate the binary form (non-float) of the significand data
    significand = fnorm * ((1LL<<significandbits) + 0.5f);

    // get the biased exponent
    exp = shift + ((1<<(expbits-1)) - 1); // shift + bias

    // return the final answer
    return (sign<<(bits-1)) | (exp<<(bits-expbits-1)) | significand;
};

/// powerful packing for floats and doubles.  based on IEEE-754 format.
// use one of the helper functions below.
double unpack754(uint64_t val, unsigned bits, unsigned expbits)
{
    long double result;
    long long shift;
    unsigned bias;
    unsigned significandbits = bits - expbits - 1; // -1 for sign bit

    if (val == 0)
        return 0.0;
    
    unsigned long expmask = 0;
    for (unsigned i=0; i<expbits; i++)
        expmask |= 0x1 << (bits-expbits-1+i);
    
    unsigned long fracmask = 0;
    for (int i=0; i<(int)(bits-expbits-1); i++)
        fracmask |= 0x1 << i;
    
    if ((val & expmask) == expmask && (val & fracmask) != 0u)
        return nan("");
    
    // pull the significand
    result = (val & ((1LL<<significandbits)-1)); // mask
    result /= (1LL << significandbits); // convert back to float
    result += 1.0f; // add the one back on

    // deal with the exponent
    bias = (1 << (expbits-1)) - 1;
    shift = ((val >> significandbits) & ((1LL << expbits)-1)) - bias;
    while(shift > 0) { result *= 2.0; shift--; }
    while(shift < 0) { result /= 2.0; shift++; }

    // sign it
    result *= ((val>>(bits-1)) & 1) ? -1.0: 1.0;
    
    return result;
};

/// store a 32-bit float into a char buffer
int pack(unsigned char* buf, float f)
{
    int tmp = (int)pack754(f, 32, 8);
    pack(buf, tmp);
    return 4;
};

/// unpack a 32-bit float from a char buffer
int unpack(const unsigned char* const buf, float* f)
{
    int tmp;
    unpack(buf, &tmp);
    *f = (float)unpack754(tmp, 32, 8);
    return 4;
};

