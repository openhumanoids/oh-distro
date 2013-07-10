/**
 * \file annanLib.hpp
 *
 * library to read data from annan's 5 slider board
 *
 * Written under government funding for ARM-H project.
 *
 * \author Ben Axelrod
 * \date   March 2012
 * \copyright Copyright iRobot Corporation, 2012
 **/

#ifndef ANNAN_LIB_H
#define ANNAN_LIB_H

#include <handle_lib/handleControl.hpp>
#include <pthread.h>

#define ANNAN_BAUDRATE B19200
//#define MODEMDEVICE "/dev/ttyACM0"
#define ANNAN_BUFFER 500
#define READ_CHUNK 20

#define MIN_SLIDER 15
#define MAX_SLIDER 1024
#define DEADBAND 100
#define MAX_MOTOR_VELOCITY 12000
#define MAX_MOTOR_POSITION 10000
#define MAX_SPREAD_POSITION 850
#define MAX_MOTOR_CURRENT 1000 
#define MAX_MOTOR_VOLTAGE 43000  // RPM = 0.2863 * Voltage - 385.5

struct AnnanBoard
{
    int Slider[5];
    bool Button[4];
};

typedef void (*annan_cb_t)(const AnnanBoard& data);

AnnanBoard parseAnnanData(char* data);

void printAnnanData(const AnnanBoard data);

int annan_connect(const char* const device);

void* annan_thread(void*);

int annan_start(annan_cb_t callback);

int annan_stop();

HandleCommand annan_to_cmd(const AnnanBoard& data);

// convert annan slider numbers to finger numbers
// const unsigned int annanToFinger[5] = {3, 2, 0, 1, 4}; // Nick's custom mapping
const unsigned int annanToFinger[5] = {0, 1, 2, 3, 4};

#endif
