/**
 * \file packetParser.h
 *
 * Functions for parsing commands coming from traffic cop microcontroller.
 *
 * Written under government funding for ARM-H project.
 *
 * \author Ben Axelrod
 * \date   March 2012
 * \copyright Copyright iRobot Corporation, 2012
 **/

#ifndef PACKET_PARSER_H
#define PACKET_PARSER_H

#include "handleTypes.hpp"
#include <unistd.h> //read close

// Define this to spew lots of data to console.
//#define DEBUG_PARSER

// Define this to enable thermal compensation on tactile sensors
#define THERMAL_COMPENSATION

#ifdef DEBUG_PARSER
#include <string.h>
#endif

/// Parse the sensor data from a motor board.
// Take data from the buffer and put it into the HandSensors type.
//
// \param buff  The buffer of data from the microcontroller.  The first byte 
//              should already be lined up with the appropriate data.
// \param sensor_mask  The bitmask of sensors requested.
// \param[out] data   The HandSensors type where to load the data.
// \param[out] valid  The booleans in this type will be set to true if there is
//                    data for that sensor. 
// \finger  The finger number.  (not motor number).  Range 0-3.  Note that 
//          despite the hand having only 3 fingers, there are 4 'fingers' here.
//          This is because there are 2 motor boards for the thumb.
//
// Returns the number of bytes parsed, or -1 on error.
int parseMotor(const unsigned char* const buff, 
               const unsigned int sensor_mask, 
               HandSensors& data,
               HandSensorsValid& valid,
               const unsigned int finger) //0 to 3
{
    if (finger > 3)
        return -1;
    
    int parsed = 0;
    
    // NOTE the order of these statements matters

    // if (sensor_mask & DATA_COLLECTION_TENSION_BITMASK)
    // {
    // #ifdef DEBUG_PARSER
    //         printf("reading cable tension data\n");
    // #endif
    //     // The 2 thumb motor boards don't have cable tension sensors.
    //     // They still return data though, the data is just junk.
    //     if (finger == 0 || finger == 1)
    //     {
    //         data.cableTension[finger].sensor1 = ((float)((buff[parsed+1] << 8) | buff[parsed])) * 0.00122;
    //         data.cableTension[finger].sensor2 = ((float)((buff[parsed+3] << 8) | buff[parsed+2])) * 0.00122;
    //         valid.cableTension[finger] = true;
    //     }
    //     parsed += 4;
    // }

    if (sensor_mask & DATA_COLLECTION_MOTORCURRENT_BITMASK)
    {
#ifdef DEBUG_PARSER
        printf("reading motor current data\n");
#endif
        data.motorCurrent[finger] = ((float)((buff[parsed+1] << 8) | buff[parsed])) / 1000.0;
        valid.motorCurrent[finger] = true;
        parsed += 2;
    }

    if (sensor_mask & DATA_COLLECTION_MOTORSTATORTEMP_BITMASK)
    {
#ifdef DEBUG_PARSER
        printf("reading motor temp data\n");
#endif
        data.motorHousingTemp[finger] = ((float)((buff[parsed+1] << 8) | buff[parsed])) / 100.0;
        valid.motorHousingTemp[finger] = true;
        parsed += 2;
    }

    if (sensor_mask & DATA_COLLECTION_MOTORVELOCITY_BITMASK)
    {
#ifdef DEBUG_PARSER
        printf("reading motor velocity data\n");
#endif
        data.motorVelocity[finger] = (buff[parsed+1] << 8) | buff[parsed];
        valid.motorVelocity[finger] = true;
        parsed += 2;
    }

    if (sensor_mask & DATA_COLLECTION_MOTORWINDINGTEMP_BITMASK)
    {
#ifdef DEBUG_PARSER
        printf("reading motor winding temp data\n");
#endif
        data.motorWindingTemp[finger] = ((float)((buff[parsed+1] << 8) | buff[parsed])) / 100.0;
        valid.motorWindingTemp[finger] = true;
        parsed += 2;
    }
    
    if (sensor_mask & DATA_COLLECTION_MOTORHALL_BITMASK)
    {
#ifdef DEBUG_PARSER
        printf("reading motor hall encoder data\n");
#endif
        data.motorHallEncoder[finger] = (buff[parsed+1] << 8) | buff[parsed];
        valid.motorHallEncoder[finger] = true;

        // Turn signed short into signed int
        //if (data.motorHallEncoder[finger] & 0x8000)
        //    data.motorHallEncoder[finger] |= 0xFFFF0000;

        parsed += 2;
    }

    // if (sensor_mask & DATA_COLLECTION_DEBUG_BITMASK)
    // {
    // }

    return parsed;
};

/// Parse the sensor data from the palm traffic cop board. 
// Take data from the buffer and put it into the HandSensors type.
//
// \param buff  The buffer of data from the microcontroller.  The first byte 
//              should already be lined up with the appropriate data.
// \param sensor_mask  The bitmask of sensors requested.
// \param[out] data   The HandSensors type where to load the data.
// \param[out] valid  The booleans in this type will be set to true if there is
//                    data for that sensor. 
//
// Returns the number of bytes parsed, or -1 on error.
int parsePalmCop(const unsigned char* const buff, 
                 const unsigned int sensor_mask, 
                 HandSensors& data,
                 HandSensorsValid& valid)
{
    int parsed = 0;
    
    if (sensor_mask & DATA_COLLECTION_FINGERROTATION_BITMASK)
    {
#ifdef DEBUG_PARSER
        printf("reading spread encoder data\n");
#endif
        data.fingerSpread = (buff[parsed+1] << 8) | buff[parsed];
        valid.fingerSpread = true;
        parsed += 2;
    }

//     if (sensor_mask & DATA_COLLECTION_MOTORCURRENT_BITMASK)
//     {
// #ifdef DEBUG_PARSER
//         printf("reading motor current data\n");
// #endif
//         // the spread motor on the traffic cop is hardcoded to be the 5th motor
//         data.motorCurrent[4] = (((float)((buff[parsed+1] << 8) | buff[parsed])) * 0.00122 - 1.25) * 0.134;
//         valid.motorCurrent[4] = true;
//         parsed += 2;
//     }
    
//     if (sensor_mask & DATA_COLLECTION_MOTORSTATORTEMP_BITMASK)
//     {
// #ifdef DEBUG_PARSER
//         printf("reading motor temp data\n");
// #endif
//         // the spread motor on the traffic cop is hardcoded to be the 5th motor
//         data.motorHousingTemp[4] = ((float)((buff[parsed+1] << 8) | buff[parsed])) / 100.0;
//         valid.motorHousingTemp[4] = true;
//         parsed += 2;
//     }
    
    if (sensor_mask & DATA_COLLECTION_EXTERNALSUPPLY_BITMASK)
    {
#ifdef DEBUG_PARSER
        printf("reading voltage data\n");
#endif
        data.voltage.volts33 = ((float)((buff[parsed+1] << 8) | buff[parsed+0])) * 0.00122 * 2.0;
        data.voltage.volts12 = ((float)((buff[parsed+3] << 8) | buff[parsed+2])) * 0.00122 * 6.0;
        data.voltage.volts48 = ((float)((buff[parsed+5] << 8) | buff[parsed+4])) * 0.00122 * 31.0;
        valid.voltage = true;
        parsed += 6;
    }
    
    if (sensor_mask & DATA_COLLECTION_AIRTEMPERATURE_BITMASK)
    {
#ifdef DEBUG_PARSER
        printf("reading air temp data\n");
#endif
        data.airTemp = ((float)((buff[parsed+1] << 8) | buff[parsed])) / 100.0;
        valid.airTemp = true;
        parsed += 2;
    }
    
    // if (sensor_mask & DATA_COLLECTION_DEBUG_BITMASK)
    // {
    // }

    return parsed;
};

/// Parse the sensor data from a proximal board. 
// Take data from the buffer and put it into the HandSensors type.
//
// \param buff  The buffer of data from the microcontroller.  The first byte 
//              should already be lined up with the appropriate data.
// \param sensor_mask  The bitmask of sensors requested.
// \param[out] data   The HandSensors type where to load the data.
// \param[out] valid  The booleans in this type will be set to true if there is
//                    data for that sensor. 
// \finger The finger number. Range 0-2.
//
// Returns the number of bytes parsed, or -1 on error.
int parseProximal(const unsigned char* const buff,
                  const unsigned int sensor_mask,
                  HandSensors& data,
                  HandSensorsValid& valid,
                  const unsigned int finger) //0 to 2
{
    if (finger > 2)
        return -1;
    
    int parsed = 0;

    if (sensor_mask & DATA_COLLECTION_ACCELERATION_BITMASK)
    {
#ifdef DEBUG_PARSER
        printf("reading acceleration data\n");
#endif
        // NOTE: the -1 is required because the sensor gives a left-handed coordinate frame.
        data.proximalAcceleration[finger].x = -1.0 * (short)((buff[parsed+1] << 8) | buff[parsed+0]) / 2048.0;
        data.proximalAcceleration[finger].y = -1.0 * (short)((buff[parsed+3] << 8) | buff[parsed+2]) / 2048.0;
        data.proximalAcceleration[finger].z = -1.0 * (short)((buff[parsed+5] << 8) | buff[parsed+4]) / 2048.0;
        
        valid.proximalAcceleration[finger] = true;
        parsed += 6;
    }
    
    // if (sensor_mask & DATA_COLLECTION_DYNAMIC_BITMASK)
    // {
    // #ifdef DEBUG_PARSER
    //         printf("reading pvdf data\n");
    // #endif
    //     data.fingerPVDF[finger].proximal[0] = ((float)((buff[parsed+1] << 8) | buff[parsed+0])) * 0.00122;
    //     data.fingerPVDF[finger].proximal[1] = ((float)((buff[parsed+3] << 8) | buff[parsed+2])) * 0.00122;
    //     data.fingerPVDF[finger].proximal[2] = ((float)((buff[parsed+5] << 8) | buff[parsed+4])) * 0.00122;
    //     valid.fingerPVDF.proximal[finger] = true;
    //     parsed += 6;
    // }
    
    if (sensor_mask & DATA_COLLECTION_DISTALJOINT_BITMASK)
    {
#ifdef DEBUG_PARSER
        printf("reading flexture joint data\n");
#endif
        data.distalJointAngle[finger].proximal[0] = ((float)((buff[parsed+1] << 8) | buff[parsed+0])) * 0.00122;
        data.distalJointAngle[finger].proximal[1] = ((float)((buff[parsed+3] << 8) | buff[parsed+2])) * 0.00122;
        data.distalJointAngle[finger].proximal[2] = ((float)((buff[parsed+5] << 8) | buff[parsed+4])) * 0.00122;
        data.distalJointAngle[finger].proximal[3] = ((float)((buff[parsed+7] << 8) | buff[parsed+6])) * 0.00122;
        valid.distalJointAngle.proximal[finger] = true;
        parsed += 8;
    }
    
    if (sensor_mask & DATA_COLLECTION_PROXIMALJOINT_BITMASK)
    {
#ifdef DEBUG_PARSER
        printf("reading proximal joint encoder data\n");
#endif
        data.proximalJointAngle[finger] = (buff[parsed+1] << 8) | buff[parsed];
        valid.proximalJointAngle[finger] = true;
        parsed += 2;
    }

    if (sensor_mask & DATA_COLLECTION_TACTILE_BITMASK)
    {
#ifdef DEBUG_PARSER
        printf("reading tactile data\n");
#endif
        for (int i=0; i<12; i++)
        {
            data.fingerTactile[finger].proximal[i] = -(float)((buff[parsed+2*i+1] << 8) | buff[parsed+2*i]);
        }
        valid.fingerTactile.proximal[finger] = true;
        parsed += 24;
    }
    
    // if (sensor_mask & DATA_COLLECTION_DEBUG_BITMASK)
    // {
    // }

    if (sensor_mask & DATA_COLLECTION_TACTILE_TEMP_BITMASK)
    {
#ifdef DEBUG_PARSER
        printf("reading tactile temperature data\n");
#endif
        for (int i=0; i<12; i++)
        {
            data.fingerTactileTemp[finger].proximal[i] = (float)((buff[parsed+2*i+1] << 8) | buff[parsed+2*i]) / -5.4 + 120.0;
        }
        valid.fingerTactileTemp.proximal[finger] = true;
        parsed += 24;
    }
    
#ifdef THERMAL_COMPENSATION
    for (int i=0; i<12; i++)
        data.fingerTactile[finger].proximal[i] -= 25.0 * data.fingerTactileTemp[finger].proximal[i] - 965.0;
#endif

    return parsed;
};

/// Parse the sensor data from a distal board. 
// Take data from the buffer and put it into the HandSensors type.
//
// \param buff  The buffer of data from the microcontroller.  The first byte 
//              should already be lined up with the appropriate data.
// \param sensor_mask  The bitmask of sensors requested.
// \param[out] data   The HandSensors type where to load the data.
// \param[out] valid  The booleans in this type will be set to true if there is
//                    data for that sensor. 
// \finger The finger number. Range 0-2.
//
// Returns the number of bytes parsed, or -1 on error.
int parseDistal(const unsigned char* const buff,
                const unsigned int sensor_mask,
                HandSensors& data,
                HandSensorsValid& valid,
                const unsigned int finger) //0 to 2
{
    if (finger > 2)
        return -1;
    
    int parsed = 0;
    
    if (sensor_mask & DATA_COLLECTION_ACCELERATION_BITMASK)
    {
#ifdef DEBUG_PARSER
        printf("reading acceleration data\n");
#endif
        // NOTE: the -1 is because the sensor actually returns a left-handed coordinate frame
        data.distalAcceleration[finger].x = -1.0 * (short)((buff[parsed+1] << 8) | buff[parsed+0]) / 2048.0;
        data.distalAcceleration[finger].y = -1.0 * (short)((buff[parsed+3] << 8) | buff[parsed+2]) / 2048.0;
        data.distalAcceleration[finger].z = -1.0 * (short)((buff[parsed+5] << 8) | buff[parsed+4]) / 2048.0;
        valid.distalAcceleration[finger] = true;
        parsed += 6;
    }
    
    // if (sensor_mask & DATA_COLLECTION_DYNAMIC_BITMASK)
    // {
    // #ifdef DEBUG_PARSER
    //         printf("reading pvdf data\n");
    // #endif
    //     data.fingerPVDF[finger].distal[0] = ((float)((buff[parsed+1] << 8) | buff[parsed+0])) * 0.00122;
    //     data.fingerPVDF[finger].distal[1] = ((float)((buff[parsed+3] << 8) | buff[parsed+2])) * 0.00122;
    //     data.fingerPVDF[finger].distal[2] = ((float)((buff[parsed+5] << 8) | buff[parsed+4])) * 0.00122;
    //     valid.fingerPVDF.distal[finger] = true;
    //     parsed += 6;
    // }
    
    if (sensor_mask & DATA_COLLECTION_DISTALJOINT_BITMASK)
    {
#ifdef DEBUG_PARSER
        printf("reading flexture joint data\n");
#endif
        data.distalJointAngle[finger].distal[0] = ((float)((buff[parsed+1] << 8) | buff[parsed+0])) * 0.00122;
        data.distalJointAngle[finger].distal[1] = ((float)((buff[parsed+3] << 8) | buff[parsed+2])) * 0.00122;
        data.distalJointAngle[finger].distal[2] = ((float)((buff[parsed+5] << 8) | buff[parsed+4])) * 0.00122;
        data.distalJointAngle[finger].distal[3] = ((float)((buff[parsed+7] << 8) | buff[parsed+6])) * 0.00122;
        valid.distalJointAngle.distal[finger] = true;
        parsed += 8;
    }
    
    if (sensor_mask & DATA_COLLECTION_TACTILE_BITMASK)
    {
#ifdef DEBUG_PARSER
        printf("reading tactile data\n");
#endif
        for (int i=0; i<10; i++)
            data.fingerTactile[finger].distal[i] = -(float)((buff[parsed+2*i+1] << 8) | buff[parsed+2*i]);
        
        valid.fingerTactile.distal[finger] = true;
        parsed += 20;
    }

    // if (sensor_mask & DATA_COLLECTION_DEBUG_BITMASK)
    // {
    // }

    if (sensor_mask & DATA_COLLECTION_TACTILE_TEMP_BITMASK)
    {
#ifdef DEBUG_PARSER
        printf("reading tactile temperature data\n");
#endif
        for (int i=0; i<10; i++)
            data.fingerTactileTemp[finger].distal[i] = (float)((buff[parsed+2*i+1] << 8) | buff[parsed+2*i]) / -5.4 + 120.0;
        valid.fingerTactileTemp.distal[finger] = true;
        parsed += 20;
    }
    
#ifdef THERMAL_COMPENSATION
    for (int i=0; i<10; i++)
        data.fingerTactile[finger].distal[i] -= 25.0 * data.fingerTactileTemp[finger].distal[i] - 965.0;
#endif

    return parsed;
};

/// Parse the sensor data from the palm tactile sensor board. 
// Take data from the buffer and put it into the HandSensors type.
//
// \param buff  The buffer of data from the microcontroller.  The first byte 
//              should already be lined up with the appropriate data.
// \param sensor_mask  The bitmask of sensors requested.
// \param[out] data   The HandSensors type where to load the data.
// \param[out] valid  The booleans in this type will be set to true if there is
//                    data for that sensor. 
//
// Returns the number of bytes parsed, or -1 on error.
int parseTactile(const unsigned char* const buff,
                 const unsigned int sensor_mask,
                 HandSensors& data,
                 HandSensorsValid& valid)
{
    int parsed = 0;
    
    // if (sensor_mask & DATA_COLLECTION_DYNAMIC_BITMASK)
    // {
    // #ifdef DEBUG_PARSER
    //         printf("reading pvdf data\n");
    // #endif
    //     data.palmPVDF[0] = ((float)((buff[parsed+1] << 8) | buff[parsed+0])) * 0.00122;
    //     data.palmPVDF[1] = ((float)((buff[parsed+3] << 8) | buff[parsed+2])) * 0.00122;
    //     data.palmPVDF[2] = ((float)((buff[parsed+5] << 8) | buff[parsed+4])) * 0.00122;
    //     data.palmPVDF[3] = ((float)((buff[parsed+7] << 8) | buff[parsed+6])) * 0.00122;
    //     valid.palmPVDF = true;
    //     parsed += 8;
    // }
    
    if (sensor_mask & DATA_COLLECTION_TACTILE_BITMASK)
    {
#ifdef DEBUG_PARSER
        printf("reading tactile data\n");
#endif
        
        for (int i=0; i<48; i++)
            data.palmTactile[i] = -(float)((buff[parsed+2*i+1] << 8) | buff[parsed+2*i]);
        
        valid.palmTactile = true;
        parsed += 48*2;
    }

    // if (sensor_mask & DATA_COLLECTION_DEBUG_BITMASK)
    // {
    // }

    if (sensor_mask & DATA_COLLECTION_TACTILE_TEMP_BITMASK)
    {
#ifdef DEBUG_PARSER
        printf("reading tactile temperature data\n");
#endif
        
        for (int i=0; i<48; i++)
            data.palmTactileTemp[i] = (float)((buff[parsed+2*i+1] << 8) | buff[parsed+2*i]) / -5.4 + 120.0;
        
        valid.palmTactileTemp = true;
        parsed += 48*2;
    }
    
#ifdef THERMAL_COMPENSATION
    for (int i=0; i<48; i++)
    {
        data.palmTactile[i] -= 25.0 * data.palmTactileTemp[i] - 965.0;
    }
#endif
    
    
    return parsed;
};

/// Read the data from the sensor payload and load it into the HandSensors 
// type.  Calls the above specialized parsers for each board type.
//
// \param buff  The buffer of data from the microcontroller. 
// \param sensor_mask  The bitmask of sensors requested.
// \param device_mask  The bitmask of responding devices.
// \param[out] data   The HandSensors type where to load the data.
// \param[out] valid  The booleans in this type will be set to true if there is
//                    data for that sensor. 
//
// Returns the number of bytes parsed, or -1 on error.
int parseData(const unsigned char* const buff, 
              int sensor_mask, 
              int device_mask,
              HandSensors& data,
              HandSensorsValid& valid)
{
#ifdef DEBUG_PARSER
    printf("parseData\n");
#endif
    int parsed = 0;
    
    // palm (traffic cop)
    if (device_mask & RESPONDING_DEVICES_PALM_BITMASK)
    {
#ifdef DEBUG_PARSER
        printf("parsing traffic cop\n");
#endif
        int tmpparsed = parsePalmCop(&(buff[parsed]), sensor_mask, data, valid);
        if (tmpparsed < 0)
        {
            printf("ERROR PARSING PALM BOARD DATA\n");
            return -1;
        }
        parsed += tmpparsed;
    }
    
    for (int i=0; i<3; i++)
    {
        // finger proximal
        if (device_mask & (1 << (2*i + 1))) //RESPONDING_DEVICES_FIRST_PROX_BITMASK)
        {
#ifdef DEBUG_PARSER
            printf("parsing finger %d proximal\n", (i+1));
#endif
            
            int tmpparsed = parseProximal(&(buff[parsed]), sensor_mask, data, valid, fingerChain_to_fingerNumber[i]);
            if (parsed < 0)
            {
                printf("ERROR PARSING FINGER %d PROXIMAL DATA\n", (i+1));
                return -1;
            }
            parsed += tmpparsed;
        }
    
        // finger distal
        if (device_mask & (1 << (2*i + 2))) //RESPONDING_DEVICES_FIRST_DIST_BITMASK)
        {
#ifdef DEBUG_PARSER
            printf("parsing finger %d distal\n", (i+1));
#endif
            int tmpparsed = parseDistal(&(buff[parsed]), sensor_mask, data, valid, fingerChain_to_fingerNumber[i]);
            if (parsed < 0)
            {
                printf("ERROR PARSING FINGER %d DISTAL DATA\n", (i+1));
                return -1;
            }
            parsed += tmpparsed;
        }
    }
    
    for (int i=0; i<4; i++)
    {
        // motor 
        if (device_mask & (1 << (i + 7))) //RESPONDING_DEVICES_FIRST_MOTOR1_BITMASK)
        {
#ifdef DEBUG_PARSER
            printf("parsing motor %d\n", (i+1));
#endif
            
            int tmpparsed = parseMotor(&(buff[parsed]), sensor_mask, data, valid, motorChain_to_fingerNumber[i]);
            if (parsed < 0)
            {
                printf("ERROR PARSING MOTOR %d DATA\n", (i+1));
                return -1;
            }
            parsed += tmpparsed;
        }
    }

    // tactile
    if (device_mask & RESPONDING_DEVICES_TACTILE_BITMASK)
    {
#ifdef DEBUG_PARSER
        printf("parsing tactile\n");
#endif
        int tmpparsed = parseTactile(&(buff[parsed]), sensor_mask, data, valid);
        if (parsed < 0)
        {
            printf("ERROR PARSING TACTILE DATA\n");
            return -1;
        }
        parsed += tmpparsed;
    }

#ifdef DEBUG_PARSER
    printf("parsing responses\n");
#endif
    for (int i=0; i<12; i++)
    {
        if (device_mask & (1 << i))
            data.responses[deviceMask_to_fingerNumber[i]] = 1;
    }
    
    return parsed;
};


/// Just like standard serial read(), but blocking for up to n chars.  
// Blocks for up to 1 second.
//
// \param fd   The serial file descriptor to use.
// \param[out] buff  Where to put the data.
// \param n    Read this number of bytes.
// \param timeout  Number of microseconds to wait for.
//
// Returns the number of bytes read, or -1 on timeout.
int blockingread(int fd, unsigned char* buff, int n, unsigned int timeout = 1000000)
{
    int rd = 0;
    unsigned int time = 0;
    while (rd < n)
    {
        //printf("%d ", time);
        
        int r = read(fd, &(buff[rd]), n-rd);
        if (r <= 0)
        {
            time += 100;
            if (time >= timeout)
                return -1;
            usleep(100);
            continue;
        }
        rd += r;
    }
    return rd;
};

/// Read a standard command response from palm traffic cop microcontroller.
//
// \param fd   The serial file descriptor.
// \param[out] len  Returns entire length of message (including initial 2 
//                  bytes for packet length).
//
// Returns error code. (0 on success, larger on error)
int readResponse(int fd, int& len)
{
    unsigned char smbuff[2];
    len = 0;
    
    // first read packet size
    int ret = blockingread(fd, smbuff, 2);
    if (ret <= 0)
        return -1;
    int packetlen = (smbuff[1] << 8) | smbuff[0];
#ifdef DEBUG_PARSER
    printf("read %d bytes\n", ret);
    printf("packet len is: %d\n", packetlen);
#endif
    unsigned char* buff = new unsigned char[packetlen+2];
    buff[0] = smbuff[0];
    buff[1] = smbuff[1];

    // now read packet
    ret = blockingread(fd, &(buff[2]), packetlen);
    if (ret <= 0)
    {
        delete[] buff;
        return -1;
    }
#ifdef DEBUG_PARSER
    printf("read packet of length: %d\n", ret);
#endif
    // verify checksum
    if (computeChecksum(buff, packetlen+2) != 0)
    {
        delete[] buff;
        return 0x11;
    }
    // todo: verify reflected command byte?
    
#ifdef DEBUG_PARSER
    for (int i=0; i<packetlen+2; i++)
        printf("%02X ", buff[i]);
    printf("\n");
#endif

    len = packetlen+2;
    ret = buff[RESPONSE_STATUSCODE_OFFSET];
    delete[] buff;
    return ret;
};


/// Read a standard command response from palm traffic cop microcontroller.
// Same as above readResponse, but does not give you the length of the read
//  message
//
// \param fd   The serial file descriptor.
//
// Returns error code. (0 on success, larger on error)
int readResponse(int fd)
{
    int len;
    return readResponse(fd, len);
};

/// Read a standard command response from palm traffic cop microcontroller.
// This version gives you the response packet to inspect.
//
// \param fd   The serial file descriptor.
// \param buff  Where to put the response.
// \param maxlen  the max length of the buffer.
// \param[out] len  Returns entire length of message (including initial 2 
//                  bytes for packet length).
//
// Returns error code. (0 on success, larger on error)
int readResponse(int fd, unsigned char* buff, int maxlen, int& len)
{
    len = 0;
    
    // first read packet size
    int ret = blockingread(fd, buff, (2<maxlen)?2:maxlen);
    if (ret < 2)
        return -1;

    int packetlen = (buff[1] << 8) | buff[0];

#ifdef DEBUG_PARSER
    printf("read %d bytes\n", ret);
    printf("packet len is: %d\n", packetlen);
#endif
    
    // now read packet
    ret = blockingread(fd, &(buff[2]), (packetlen<maxlen-2)?packetlen:maxlen-2);

#ifdef DEBUG_PARSER
    printf("read packet of length: %d\n", ret);
#endif
    
    if (ret < packetlen)
        return -1;

    // verify checksum
    if (computeChecksum(buff, packetlen+2) != 0)
        return 0x11;
    // todo: verify reflected command byte?
    
#ifdef DEBUG_PARSER
    for (int i=0; i<packetlen+2; i++)
        printf("%02X ", buff[i]);
    printf("\n");
#endif

    len = packetlen+2;
    ret = buff[RESPONSE_STATUSCODE_OFFSET];
    return ret;
};


/// Read response or parse sensor data from palm traffic cop microcontroller.
// If it is sensor data, sensorMsg will be set to true, and data and valid 
// bits will be filled in to passed in references. If it is a motor response, 
// sensorMsg will be false.
//
// \param fd  The serial file descriptor
// \param sensor_mask  The bitmask of sensors requested.
// \param[out] sensorMsg  Will be set to true if message was a sensor message.
// \param[out] data  The HandSensors type where to load the data.
// \param[out] valid  The booleans in this type will be set to true if there is
//                    data for that sensor.
// \param[out] buff  A buffer to use when recieving data.
// \param maxlen  The size of the buffer.
//
// Returns an error code, 0 for success.
int parseResponse(int fd, int sensor_mask, bool& sensorMsg, HandSensors& data, HandSensorsValid& valid, unsigned char* buff, int maxlen)
{
#ifdef DEBUG_PARSER
    printf("parseResponse\n");
#endif

    sensorMsg = false;
#ifdef DEBUG_PARSER
    printf("entering first blocking read\n");
#endif
    // first read packet size
    int ret = blockingread(fd, buff, 2);
    if (ret <= 0)
    {
#ifdef DEBUG_PARSER
        //printf("ERROR: %d: %s\n", errno, strerror( errno ) );
        // EAGAIN = 11
        // EWOULDBLOCK = 11
        // EBADF = 9
        // EFAULT = 14
        // EINTR = 4
        // EINVAL = 22
        // EINVAL = 22
        // EIO = 5
        // EISDIR = 21
#endif
        return -1;
    }
    
    // packetlen is length of packet without leading 2 size bytes
    int packetlen = (buff[1] << 8) | buff[0];

#ifdef DEBUG_PARSER
    printf("read %d bytes\n", ret);
    printf("packet len is: %d\n", packetlen);
#endif
    
    if (packetlen+2 > maxlen)
        return -2;

    // now read packet
    ret = blockingread(fd, &(buff[2]), packetlen);
    if (ret <= 0)
    {
#ifdef DEBUG_PARSER
        //printf("ERROR: %s\n", strerror( errno ) );
#endif
        return -1;
    }
#ifdef DEBUG_PARSER
    printf("read packet of length: %d\n", ret);
    printf("full packet:\n");
    for (int i=0; i<packetlen+2; i++)
        printf("%02X ", buff[i]);
    printf("\n");
#endif
    // verify checksum
    if (computeChecksum(buff, packetlen+2) != 0)
    {
        return 0x11;
    }

    if (buff[RESPONSE_REFLECTEDOPCODE_OFFSET] == DATA_COLLECTION_OPCODE)
    {
        // sensor message
        sensorMsg = true;
        
        // determine which boards responded
        int respondingDevices = (buff[RESPONSE_BROADCAST_RESPONDINGDEVICES+1] << 8) | buff[RESPONSE_BROADCAST_RESPONDINGDEVICES];
        
        // determine how large the expected reply should be
        int datalen = computeDataSize(sensor_mask, respondingDevices);
        
        // data length error
        if (datalen != packetlen-4)
        {
#ifdef DEBUG_PARSER
            printf("\n\ndata length error\n");
            printf("sensor mask: %04X\n", sensor_mask);
            printf("device mask: %04X\n", respondingDevices);
            printf("computed datalen: %d\n", datalen);
            printf("packetlen-4: %d\n", packetlen-4);
#endif
            return 0x12;
        }
        
        int parsed = parseData(&(buff[RESPONSE_BROADCAST_PAYLOAD]), sensor_mask, respondingDevices, data, valid);

        // i don't think this should happen, but just in case.
        if (parsed != datalen)
        {
#ifdef DEBUG_PARSER
            printf("\n\nparsed length error\n");
            printf("sensor mask: %04X\n", sensor_mask);
            printf("device mask: %04X\n", respondingDevices);
            printf("computed datalen: %d\n", datalen);
            printf("packetlen-4: %d\n", packetlen-4);
            printf("parsed length: %d\n", parsed);
#endif
            return 0x13;
        }

#ifdef DEBUG_PARSER        
        printf("Sensor mask: %04X\n", sensor_mask);
        printf("Device mask: %04X\n", respondingDevices);
        for (int i=0; i<packetlen+2; i++)
            printf("%02X ", buff[i]);
        printf("\n");
#endif
        // success
        return 0x00;
    }
    else
    {
        // command response
        sensorMsg = false;

#ifdef DEBUG_PARSER        
        printf("Response code: %02X\n", buff[RESPONSE_STATUSCODE_OFFSET]);
#endif
       
        // success, return error code (if any).
        ret = buff[RESPONSE_STATUSCODE_OFFSET];
        return ret;
    }
};

/// Parse a motor parameter from a response payload.
// Note that this is big-endian, despite the rest of the code being little endian.
//
// \param buff  The response payload
//
// Returns the floating point value.
float parseParameterf(unsigned char* buff)
{
    float x = 0;
    unsigned char* ptr = (unsigned char*)(&x);
    *ptr = buff[0]; ptr++;
    *ptr = buff[1]; ptr++;
    *ptr = buff[2]; ptr++;
    *ptr = buff[3];
    return x;
};

int parseParameter(unsigned char* buff)
{
    return (buff[3] << 24) | 
        (buff[2] << 16) | 
        (buff[1] << 8) | 
        (buff[0] << 0);
    // int x = 0;
    // unsigned char* ptr = (unsigned char*)(&x);
    // *ptr = buff[3]; ptr++;
    // *ptr = buff[2]; ptr++;
    // *ptr = buff[1]; ptr++;
    // *ptr = buff[0];
    // return x;
};


#endif

