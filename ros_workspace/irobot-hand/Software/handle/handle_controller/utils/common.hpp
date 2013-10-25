/**
 * \file common.hpp
 *
 * common functions for parsing arguments
 *
 * Written under government funding for ARM-H project.
 *
 * \author Ben Axelrod
 * \date   November 2012
 * \copyright Copyright iRobot Corporation, 2012
 **/

// #include <sys/types.h>
// #include <sys/stat.h>
// #include <fcntl.h>
// #include <termios.h>
// #include <stdio.h>
#include <string.h> //strlen
// #include <strings.h>
// #include <unistd.h>
#include <stdlib.h> //atoi
// #include <signal.h>
#include <ctype.h> //isdigit tolower

#include "../../handle_lib/include/handle_lib/handleTypes.hpp"
//#include "../../handle_lib/include/handle_lib/packetParser.hpp"

// turn a single character which is a decimal into an integer
// 0-9
int ctoi(char c)
{
    return (int)c - 48;  // '0' is 48 in ascii
}

// find the first character which is a digit in a string
int findNumber(char* str)
{
    for(unsigned int i=0; i<strlen(str); i++)
        if (isdigit(str[i]))
            return i;
    return -1;
}

// convert an int (0-11) to destination and responding device masks.
// if valid board, return true.
// destination and responding undefined if return false.
bool getDestination(int board, int& destination, int& responding)
{
    switch (board)
    {
        case 0: destination = DESTINATION_PALM; responding = RESPONDING_DEVICES_PALM_BITMASK; return true; break;
        case 1: destination = DESTINATION_FINGER1_PROX; responding = RESPONDING_DEVICES_FIRST_PROX_BITMASK; return true; break;
        case 2: destination = DESTINATION_FINGER1_DIST; responding = RESPONDING_DEVICES_FIRST_DIST_BITMASK;return true;  break;
        case 3: destination = DESTINATION_FINGER2_PROX; responding = RESPONDING_DEVICES_SECOND_PROX_BITMASK; return true; break;
        case 4: destination = DESTINATION_FINGER2_DIST; responding = RESPONDING_DEVICES_SECOND_DIST_BITMASK; return true; break;
        case 5: destination = DESTINATION_FINGER3_PROX; responding = RESPONDING_DEVICES_THIRD_PROX_BITMASK; return true; break;
        case 6: destination = DESTINATION_FINGER3_DIST; responding = RESPONDING_DEVICES_THIRD_DIST_BITMASK; return true; break;
        case 7: destination = DESTINATION_MOTOR1; responding = RESPONDING_DEVICES_FIRST_MOTOR1_BITMASK; return true; break;
        case 8: destination = DESTINATION_MOTOR2; responding = RESPONDING_DEVICES_FIRST_MOTOR2_BITMASK; return true; break;
        case 9: destination = DESTINATION_MOTOR3; responding = RESPONDING_DEVICES_SECOND_MOTOR1_BITMASK; return true; break;
        case 10: destination = DESTINATION_MOTOR4; responding = RESPONDING_DEVICES_SECOND_MOTOR2_BITMASK; return true; break;
        case 11: destination = DESTINATION_TACTILE; responding = RESPONDING_DEVICES_TACTILE_BITMASK; return true; break;
        default:
            return false;
    }
}

// convert a string such as:
//  * palm
//  * F1P
//  * motor2
//  * finger3distal
//  * 11   (tactile)
//  * all  (only permitted in some situations, in which case destination and responding will be -1)
// into the proper destination and responding bit masks.
// returns true if the input was properly parsed, false if unable to parse
bool getDestination(char* name, int& destination, int& responding)
{
    // get string length
    int len = strlen(name);
    
    // string can't be empty
    if (len == 0)
        return false;
    
    // test for "All"
    if (tolower(name[0]) == 'a')
    {
        destination = -1;
        responding = -1;
        return true;
    }
    
    // test for Palm
    if (tolower(name[0]) == 'p')
    {
        destination = DESTINATION_PALM; 
        responding = RESPONDING_DEVICES_PALM_BITMASK;
        return true;
    }
    
    // test for Tactile
    if (tolower(name[0]) == 't')
    {
        destination = DESTINATION_TACTILE;
        responding = RESPONDING_DEVICES_TACTILE_BITMASK;
        return true;
    }
    
    // get first digit
    int num = findNumber(name);
    
    // -1 indicates no digit, which is an error at this point
    if (num < 0)
        return false;
    // probably can't catch, but just in case
    if (num > len-1)
        return false;
    
    // if the first character is a digit, then we assume the whole input is numeric.
    if (num == 0)
        return getDestination(atoi(name), destination, responding);
    
    if (tolower(name[0]) == 'f')
    {
        // if finger, there must be at least one character after the number to indicate proximal / distal
        if (num >= len-1)
            return false;
        
        int finger_number = ctoi(name[num]);
        
        switch (finger_number)
        {
            case 1:
                if (tolower(name[num+1]) == 'p')
                {
                    destination = DESTINATION_FINGER1_PROX; 
                    responding = RESPONDING_DEVICES_FIRST_PROX_BITMASK;
                    return true;
                }
                else if (tolower(name[num+1]) == 'd')
                {
                    destination = DESTINATION_FINGER1_DIST;
                    responding = RESPONDING_DEVICES_FIRST_DIST_BITMASK;
                    return true;
                }
                else
                    return false;
                break;
                
            case 2:
                if (tolower(name[num+1]) == 'p')
                {
                    destination = DESTINATION_FINGER2_PROX; 
                    responding = RESPONDING_DEVICES_SECOND_PROX_BITMASK;
                    return true;
                }
                else if (tolower(name[num+1]) == 'd')
                {
                    destination = DESTINATION_FINGER2_DIST;
                    responding = RESPONDING_DEVICES_SECOND_DIST_BITMASK;
                    return true;
                }
                else
                    return false;
                break;
                
            case 3:
                if (tolower(name[num+1]) == 'p')
                {
                    destination = DESTINATION_FINGER3_PROX; 
                    responding = RESPONDING_DEVICES_THIRD_PROX_BITMASK;
                    return true;
                }
                else if (tolower(name[num+1]) == 'd')
                {
                    destination = DESTINATION_FINGER3_DIST;
                    responding = RESPONDING_DEVICES_THIRD_DIST_BITMASK;
                    return true;
                }
                else
                    return false;
                break;  
                
            default:
                return false;
        }
    }
    
    if (tolower(name[0]) == 'm')
    {
        int motor_number = ctoi(name[num]);
        
        switch (motor_number)
        {
            case 1:
                destination = DESTINATION_MOTOR1;
                responding = RESPONDING_DEVICES_FIRST_MOTOR1_BITMASK;
                return true;
                break;
                
            case 2:
                destination = DESTINATION_MOTOR2;
                responding = RESPONDING_DEVICES_FIRST_MOTOR2_BITMASK;
                return true;
                break;

            case 3:
                destination = DESTINATION_MOTOR3; 
                responding = RESPONDING_DEVICES_SECOND_MOTOR1_BITMASK;
                return true;
                break;

            case 4:
                destination = DESTINATION_MOTOR4; 
                responding = RESPONDING_DEVICES_SECOND_MOTOR2_BITMASK;
                return true;
                break;
                
            default:
                return false;
                break;
        }
    }
    
    return false;
}
