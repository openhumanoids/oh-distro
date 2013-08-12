/**
 * \file handleControl.hpp
 *
 * type definitions for the HANDLE hand
 *
 * Types to be sent over tcp to control the hand from the laptop.
 *
 * \author Ben Axelrod
 * \date   March 2012
 * \copyright Copyright iRobot Corporation, 2012
 **/

#ifndef HANDLE_CONTROL_H
#define HANDLE_CONTROL_H

#include "packing.hpp"

/// The command type for a specific motor.
enum CommandType {
    MOTOR_VELOCITY,
    MOTOR_POSITION,
    MOTOR_CURRENT,
    MOTOR_VOLTAGE
};

/// A command for a specific motor.  The motor number is designated by the 
// position of this type in a fixed size array.
// size: 12 bytes
struct MotorCommand
{
    CommandType type; 
    int value;
    bool valid;
};

/// The type that is actually sent over the wire to the palm.
// size: 68 bytes
class HandleCommand
{
public:
    MotorCommand motorCommand[5];
    bool calibrate;
    
    /// Default constructor initializes all data to benign values.
    HandleCommand()
    {
        for (int i=0; i<5; i++)
        {
            motorCommand[i].type = MOTOR_VELOCITY;
            motorCommand[i].value = 0;
            motorCommand[i].valid = false;
        }
        calibrate = false;
    };
    
    bool anyValid() const
    {
        for (int i=0; i<5; i++)
            if (motorCommand[i].valid)
                return true;
        return false;
    };
    
    int pack(unsigned char* buffer) const
    {
        unsigned char* ptr = buffer;
        for (int i=0; i<5; i++)
        {
            ptr += ::pack(ptr, (char)motorCommand[i].type);
            ptr += ::pack(ptr, motorCommand[i].value);
            ptr += ::pack(ptr, (char)motorCommand[i].valid);
        }
        ptr += ::pack(ptr, (char)calibrate);
        return (ptr-buffer);
    };
    
    int unpack(const unsigned char* const buffer)
    {
        unsigned char* ptr = (unsigned char*)buffer;
        for (int i=0; i<5; i++)
        {
            ptr += ::unpack(ptr, (char*)(&(motorCommand[i].type)));
            ptr += ::unpack(ptr, &(motorCommand[i].value));
            ptr += ::unpack(ptr, (char*)(&(motorCommand[i].valid)));
        }
        ptr += ::unpack(ptr, (char*)(&calibrate));
        return (ptr-buffer);
    };
};

#endif

