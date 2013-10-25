/**
 * \file packing.hpp
 *
 * Functions for bit packing to send bytes over the wire.
 *
 * Written under government funding for ARM-H project.
 *
 * \author Ben Axelrod
 * \date   May 2012
 * \copyright Copyright iRobot Corporation, 2012
 **/

#ifndef PACKING_HPP
#define PACKING_HPP

#include <stdint.h>
#include <math.h> //isnan



/// store a 8-bit char into a char buffer
int pack(unsigned char* buf, char c);

/// store a 16-bit short into a char buffer (like htons())
int pack(unsigned char* buf, short s);

/// store a 32-bit int into a char buffer (like htonl())
int pack(unsigned char* buf, int i);

/// unpack a 8-bit char from a char buffer
int unpack(const unsigned char* const buf, char* c);

/// unpack a 16-bit short from a char buffer (like ntohs())
int unpack(const unsigned char* const buf, short* s);

/// unpack a 32-bit int from a char buffer (like ntohl())
int unpack(const unsigned char* const buf, int* i);

/// powerful packing for floats and doubles.  based on IEEE-754 format.
// use one of the helper functions below.
long pack754(long double f, unsigned bits, unsigned expbits);

/// powerful packing for floats and doubles.  based on IEEE-754 format.
// use one of the helper functions below.
double unpack754(uint64_t val, unsigned bits, unsigned expbits);

/// store a 32-bit float into a char buffer
int pack(unsigned char* buf, float f);

/// unpack a 32-bit float from a char buffer
int unpack(const unsigned char* const buf, float* f);

#endif

