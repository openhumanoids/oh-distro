/**
 * \file controller.cpp
 *
 * Overo control code
 *
 * Written under government funding for ARM-H project.
 *
 * \author Ben Axelrod
 * \date   March 2012
 * \copyright Copyright iRobot Corporation, 2012
 **/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <signal.h>
#include <pthread.h>
#include <ctype.h> //isprint
#include <getopt.h>

#include <fcntl.h>
#include <termios.h>
#include <math.h> // round

// this is not compiled with rosmake so this is a little more awkward
#include "../../handle_lib/include/handle_lib/handleTypes.hpp"
#include "../../handle_lib/include/handle_lib/packetParser.hpp"
#include "../../handle_lib/include/handle_lib/handleControl.hpp"
//#include "../../handle_lib/include/handle_lib/handleTcp.hpp"

#define DO_THERMAL

#define ANGLE_P_CONTROL 201

#define MOTOR_MIN_RPM  500 // in units of RPM
#define MOTOR_MAX_RPM  12000 // in units of RPM
#define SPREAD_MOTOR_MAX  4095 // in units of DAC value

//how many sensor messages to wait for before declaring a motor message unsuccessful
#define CMD_WAIT_TRIES 2

// how many commands to skip when commands are the same as last time
#define MAX_SKIP 10

#define BAUDRATE B1152000
#define MODEMDEVICE "/dev/ttyS0"
#define BUFFER 1000

// parameters affecting thermal limit override
#define OVERRIDE_HISTORY    50   // size of the history buffer to store (how fast to get into override)
#define ERROR_TO_OVERRIDE   500 // position target - current position must be greater than this to trigger override
#define MAX_DELTA_NO_CHANGE 10   // the sum of deltas in the history must be smaller than this to trigger override
#define OVERRIDE_ITERATIONS 50   // how many iterations to be at 0 current (how long to be in override)

#define DEFAULT_PORT "3490"
#define DEFAULT_HZ 100
#define DEFAULT_TIMEOUT 0.0 //seconds, 0 = no timeout
#define DEFAULT_SENSOR_MASK DATA_COLLECTION_ALL_BITMASK & (~DATA_COLLECTION_EXTERNALSUPPLY_BITMASK)

#define VERSION 201

#ifdef DO_THERMAL
struct override_t
{
    int pos_history[OVERRIDE_HISTORY];
    int err_history[OVERRIDE_HISTORY];
    int history_i;
    int last_error;
    int count;
};
override_t motor_override[4];
#endif

HandSensors sensor_data;

pthread_mutex_t control_mutex = PTHREAD_MUTEX_INITIALIZER;
HandleCommand control;
HandleCommand lastcmd;
int skip_count[5];

// command line params
int hz = DEFAULT_HZ;
bool udp = false;
bool calibrate = false;
bool dumb_fingers = false;
bool dumb_finger1 = false;
bool dumb_finger2 = false;
bool dumb_finger3 = false;
bool dumb_tactile = false;
bool debug_time = false;
bool debug_motors = false;
bool debug_errors = false;
bool debug_quality = false;
bool no_tact = false;
bool no_tact_temp = false;
bool no_flex = false;
bool no_accel = false;
// debug, cable tension, and pvdf are already commented out of ALL bitmask
int sensor_mask = DEFAULT_SENSOR_MASK;
float timeout = DEFAULT_TIMEOUT;

// Gets added to the spread value to compensate for rollover if the spread 
// is started greater than 512 ticks away from zero.
int spread_adjust = 0;

int sign(float val)
{
    if (val == 0)
        return 0;
    if (val > 0)
        return 1;
    return -1;
};
int sign(int val)
{
    if (val == 0)
        return 0;
    if (val > 0)
        return 1;
    return -1;
};

/// Look through 'control', return true if any commands are valid.
// index will be moved to point to the next valid command.
// does not lock mutex.
bool anyValid(int& index)
{
    for (int i=0; i<5; i++)
    {
        //index = (index+i)%5;
        if (control.motorCommand[(index+i)%5].valid)
        {
            index = (index+i)%5;
            return true;
        }
    }
    return false;
};

volatile bool run = true;
volatile int socketfd = -1;
volatile int tcp_fd = -1;
volatile struct sockaddr addr;
volatile socklen_t addrlen;
void sigint_handler(int s)
{
    printf("SIGINT received\n");
    run = false;
    if (socketfd != -1)
    {
        close(socketfd);
        socketfd = -1;
    }
    if (tcp_fd != -1)
    {
        close(tcp_fd);
        tcp_fd = -1;
    }
};

void sigpipe_handler(int s)
{
    printf("SIGPIPE received\n");
};

// #ifndef FIRMWARE_P_CONTROL
// // bang-bang control for finger spread motor
// int spreadControl(int target)
// {
//     int error = target - sensor_data.fingerSpread;
//     if (abs(error) < SPREAD_DEADBAND)
//         return 0;
//     else if (error > 0)
//         return 1;
//     else
//         return -1;
// };
// #endif


int motorControlFingerAngle(int motor, int target)
{
    // no finger angle control for spread motor
    if (motor == 4)
        return target;
    
    int error = target - sensor_data.proximalJointAngle[motorChain_to_fingerNumber[motor]];
    return ANGLE_P_CONTROL * error;
}

// motor 0-4
// target is the target encoder count.
// return value to set motor at.  
// does software thermal control.
int motorControl(int motor, int target, bool& override)
{
    override = false;
    
    // no thermal control for spread motor (yet)
    if (motor == 4)
        return target;
    
    // thermal limit override
#ifdef DO_THERMAL

    int error = target - sensor_data.motorHallEncoder[motorChain_to_fingerNumber[motor]];

    motor_override[motor].history_i++;
    motor_override[motor].history_i = motor_override[motor].history_i % OVERRIDE_HISTORY;
    
    motor_override[motor].pos_history[motor_override[motor].history_i] = 
        sensor_data.motorHallEncoder[motorChain_to_fingerNumber[motor]];

    motor_override[motor].err_history[motor_override[motor].history_i] = error;
    
    //if (motor==1) printf("%d  %d\n", error, target);

    if (abs(error) > ERROR_TO_OVERRIDE)
    {
        int err_sum = 0;
        for (int i=0; i<OVERRIDE_HISTORY; i++)
            if (abs(motor_override[motor].err_history[i]) > ERROR_TO_OVERRIDE)
                err_sum += sign(motor_override[motor].err_history[i]);
        
        int delta_sum = 0;
        for (int i=0; i<(OVERRIDE_HISTORY-1); i++)
            if (abs(motor_override[motor].pos_history[i] - motor_override[motor].pos_history[i+1]) < MAX_DELTA_NO_CHANGE)
                delta_sum++;

        //if (motor==1) printf("             %d  %d  %d  %d\n", err_sum, delta_sum, sign(motor_override[motor].last_error), sign(target));
        
        if (abs(err_sum) >= OVERRIDE_HISTORY && delta_sum >= (OVERRIDE_HISTORY-3))
        {
            if (motor_override[motor].last_error == 0 ||
                sign(motor_override[motor].last_error) == sign(error))
            {
                //if (motor==1) printf("            OVERRIDE %d\n", motor_override[motor].count);
                
                // the actual override
                motor_override[motor].last_error = error;
                target = sensor_data.motorHallEncoder[motorChain_to_fingerNumber[motor]];
                override = true;
                
                // if we have been overridden too long, release it for a short pulse.
                if (motor_override[motor].count++ > OVERRIDE_ITERATIONS)
                {
                    //if (motor==1) printf("          DISABLE\n");
                    
                    for (int i=0; i<OVERRIDE_HISTORY; i++)
                        motor_override[motor].err_history[i] = 0;
                }
            }
        }
    }
    
    if (!override)
    {
        //if (motor==1) printf("no\n");
        motor_override[motor].count = 0;
        motor_override[motor].last_error = 0;
    }
    
#endif //end thermal overide
    
    return target;
};

// /// Set a parameter on the motor [0-3]
// // returns error code.
// //  0 = success
// // -2 = unknown motor
// // -3 = write error
// int writeMotorParameter(int fd, int motor, float value, short parameter)
// {
//     unsigned char buff[COMMAND_PACKET_LENGTH];
//     memset(buff, 0, COMMAND_PACKET_LENGTH);

//     switch (motor)
//     {
//         case 0:
//             buff[DESTINATION_HEADER_OFFSET] = DESTINATION_MOTOR1;
//             break;
//         case 1:
//             buff[DESTINATION_HEADER_OFFSET] = DESTINATION_MOTOR2;
//             break;
//         case 2:
//             buff[DESTINATION_HEADER_OFFSET] = DESTINATION_MOTOR3;
//             break;
//         case 3:
//             buff[DESTINATION_HEADER_OFFSET] = DESTINATION_MOTOR4;
//             break;
//         default:
//             return -2;
//     }
    
//     buff[COMMAND_OFFSET] = MOTOR_PARAMETER_WR_L_OPCODE | parameter;
//     setPayloadf(buff, value);
//     buff[CHECKSUM_OFFSET] = computeChecksum(buff, COMMAND_PACKET_LENGTH-1);
    
//     if (write(fd, buff, COMMAND_PACKET_LENGTH) != COMMAND_PACKET_LENGTH)
//         return -3;
    
//     return 0;
// };

/// Set the motor [0-4] to the desired speed value.
// Note that value can be negative.
// Value will be capped at 2 bytes (65535).
// returns error code.
//  0 = success
// -2 = unknown motor or command type
// -3 = write error
int writeMotor(int fd, int motor, int value, CommandType type = MOTOR_VELOCITY)
{
    if (debug_motors)
    {
        printf("writing motor %d with %d and type %d\n", motor, value, type);
    }

    unsigned char buff[COMMAND_PACKET_LENGTH];
    memset(buff, 0, COMMAND_PACKET_LENGTH);
    
    switch (motor)
    {
        case 0:
            buff[DESTINATION_HEADER_OFFSET] = DESTINATION_MOTOR1;
            break;
        case 1:
            buff[DESTINATION_HEADER_OFFSET] = DESTINATION_MOTOR2;
            break;
        case 2:
            buff[DESTINATION_HEADER_OFFSET] = DESTINATION_MOTOR3;
            break;
        case 3:
            buff[DESTINATION_HEADER_OFFSET] = DESTINATION_MOTOR4;
            break;
        case 4:
            buff[DESTINATION_HEADER_OFFSET] = PALM_CHAINADDRESS;
            break;
        default:
            return -2;
    }
    
    if (motor == 4)
        buff[COMMAND_OFFSET] = FINGER_COMMAND_OPCODE;
    else
        buff[COMMAND_OFFSET] = MOTOR_COMMAND_OPCODE;
    
    switch (type)
    {
        case MOTOR_CURRENT:
            buff[COMMAND_OFFSET] |= MOTOR_COMMAND_CURRENT;
            break;
        case MOTOR_VOLTAGE:
            buff[COMMAND_OFFSET] |= MOTOR_COMMAND_VOLTAGE;
            break;
        case MOTOR_VELOCITY:
            buff[COMMAND_OFFSET] |= MOTOR_COMMAND_VELOCITY;
            break;
        case MOTOR_POSITION:
            buff[COMMAND_OFFSET] |= MOTOR_COMMAND_POSITION;
            break;
        default:
            return -2;
    }

    if (motor == 4 && type == MOTOR_POSITION) // spread adjust
        value -= spread_adjust;
    
    if (value == 0)
    {
        if (type == MOTOR_POSITION)
            buff[COMMAND_OFFSET] |= MOTOR_COMMAND_FORWARD; // just pick a direction
        else
            buff[COMMAND_OFFSET] |= MOTOR_COMMAND_STOP;
    }
    else if (value > 0)
        buff[COMMAND_OFFSET] |= MOTOR_COMMAND_FORWARD;
    else
        buff[COMMAND_OFFSET] |= MOTOR_COMMAND_REVERSE;
    
    if (type == MOTOR_VELOCITY ||
        type == MOTOR_CURRENT ||
        type == MOTOR_VOLTAGE)
    {
        if (value < 0)
            value = -value; // make positive
        if (value > 65535)
            value = 65535; // cap
        
        if (motor == 4)
        {
            if (value > 4095)
                value = 4095;
        }
        else
        {
            if (value > MOTOR_MAX_RPM)
                value = MOTOR_MAX_RPM;
            if (value > 0 && value < MOTOR_MIN_RPM)
                value = MOTOR_MIN_RPM;
        }
    }
    else if (type == MOTOR_POSITION)
    {
        if (value > 32767)
            value = 32767; // cap
        if (value < -32768)
            value = -32768; // cap
    }
    
    value = value & 0xFFFF;
    
    if (debug_motors)
    {
        printf("value cleaned to %d\n", value);
    }
    
    setPayload(buff, value);
    buff[CHECKSUM_OFFSET] = computeChecksum(buff, COMMAND_PACKET_LENGTH-1);

    if (debug_motors)
    {
        printf("packet: ");
        for (int i=0; i<COMMAND_PACKET_LENGTH; i++)
            printf("%02X ", buff[i]);
        printf("\n");
    }
    
    
    if (write(fd, buff, COMMAND_PACKET_LENGTH) != COMMAND_PACKET_LENGTH)
        return -3;
    
    return 0;
};


void cleanup(int& fd)
{
    if (fd >= 0)
    {
        unsigned char send_buff[COMMAND_PACKET_LENGTH];
    
        // stop collection
        memset(send_buff, 0, COMMAND_PACKET_LENGTH);
        send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
        send_buff[COMMAND_OFFSET] = STOP_COLLECTION_OPCODE; // stop collection
        send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
        write(fd, send_buff, COMMAND_PACKET_LENGTH);
        usleep(2000);
    
        // stop motors
        for(int i=0; i<5; i++)
        {
            writeMotor(fd, i, 0);
            usleep(2000);
        }
    
        printf("serial: closing\n");
        close(fd);
        fd = -1;
    }
    printf("serial thread stopping\n");
    pthread_exit(NULL);
}

bool repeatSendMotor(int fd, int motor, int value, CommandType type, unsigned int tries)
{
    for (unsigned int i=0; i < tries; i++)
    {
        writeMotor(fd, motor, value, type);
        
        int rsp = readResponse(fd);
        if (rsp != 0)
        {
            printf("\nWARNING: set motor returned: %02X\n", rsp);
            usleep(2000);
        }
        else
        {
            printf(".");
            fflush(stdout);
            return true;
        }
        if (!run)
            cleanup(fd);
    }
    printf("\nERROR: set motor %d failed.\n", motor);
    return false;
}

bool repeatPollSensor(int fd, int destination, int responding, int sensor, HandSensors& tmpdata, HandSensorsValid& tmpvalid, int tries = 5)
{
    unsigned char recv_buff[BUFFER];
    unsigned char send_buff[COMMAND_PACKET_LENGTH];
    
    // create poll message
    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    send_buff[DESTINATION_HEADER_OFFSET] = destination;
    send_buff[COMMAND_OFFSET] = DATA_COLLECTION_OPCODE;
    setPayload(send_buff, sensor);
    send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH);
    
    for (int i=0; i < tries; i++)
    {
        write(fd, send_buff, COMMAND_PACKET_LENGTH);
                
        int readlen = 0;
        int code = readResponse(fd, recv_buff, BUFFER, readlen);
        if (code != 0)
        {
            printf("\npoll sensor code error: %02X\n", code);
            usleep(2000);
        }
        else
        {
            int length = computeDataSize(sensor, responding);
            if (length != readlen-5)
            {
                printf("\npoll sensor read length error.  got %d, should be %d\n", readlen-5, length);
                usleep(2000);
            }
            else
            {
                int parsed = parseData(&(recv_buff[RESPONSE_PAYLOAD_OFFSET]), sensor, responding, tmpdata, tmpvalid);
                if (parsed != length)
                {
                    printf("\npoll sensor parsed length error. got %d, should be %d\n", parsed, length);
                    usleep(2000);
                }
                else
                {
                    return true;
                }
            }
        }
        
        if (!run)
            cleanup(fd);
    }
    
    // printf("\nERROR: poll sensor failed.\n");
    return false;
}

// helper wrapper for repeatPollSensor()
// NOTE that finger number is the software numbering
bool repeatPollFinger(int fd, int finger, int sensor, HandSensors& tmpdata, HandSensorsValid& tmpvalid, int tries = 5)
{
    int destination = 0;
    int responding = 0;
    switch(finger)
    {
        // note the finger number to finger chain conversion here
        case 0:
            destination = DESTINATION_FINGER2_PROX;
            responding = RESPONDING_DEVICES_SECOND_PROX_BITMASK;
            break;
        case 1:
            destination = DESTINATION_FINGER1_PROX;
            responding = RESPONDING_DEVICES_FIRST_PROX_BITMASK;
            break;
        case 2:
            destination = DESTINATION_FINGER3_PROX;
            responding = RESPONDING_DEVICES_THIRD_PROX_BITMASK;
            break;
    }
    
    return repeatPollSensor(fd, 
                            destination, //note: this is hardware numbering
                            responding,
                            sensor,
                            tmpdata, 
                            tmpvalid, 
                            tries);
}

bool repeatGenericCommand(int fd, unsigned char* send_buff, int tries = 5)
{
    for (int i=0; i < tries; i++)
    {
        write(fd, send_buff, COMMAND_PACKET_LENGTH);
        
        int rsp = readResponse(fd);
        if (rsp != 0)
        {
            printf("\nWARNING: command returned: %02X\n", rsp);
            usleep(2000);
        }
        else
        {
            printf(".");
            fflush(stdout);
            return true;
        }
        
        if (!run)
            cleanup(fd);  
    }
    
    printf("\nERROR: command failed.\n");
    return false;
}


void* serial_thread(void*)
{
    printf("serial thread started\n");
    
    int fd; // serial file descriptor

    printf("opening serial port\n");
    fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd <0 ) 
    {
        printf("ERROR: cannot open serial port\n");
        perror(MODEMDEVICE); 
        pthread_exit(NULL);
        exit(-1); 
    }
    printf("serial port open\n");
    
    printf("configuring serial port\n");
    struct termios options;
    tcgetattr(fd, &options);
    cfmakeraw(&options);
    cfsetspeed(&options, BAUDRATE);
    options.c_cflag |= (CLOCAL | CREAD);
    
    // 8 data bits, no parity, 1 stop bit
    options.c_cflag &= ~PARENB; // no parity
    options.c_cflag &= ~CSTOPB; // 1 stop bit, not 2
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;     // 8 data bits

    options.c_cflag &= ~CRTSCTS; // no CTS/RTS hardware flow control
    options.c_cc[VTIME]    = 10;  // 0.5 second timeout
    options.c_cc[VMIN]     = 1;  // block for 1 char

    tcflush(fd, TCIOFLUSH); // flush both directions
    tcsetattr(fd, TCSANOW, &options);
    printf("serial port configured\n");
    
    unsigned char send_buff[COMMAND_PACKET_LENGTH];
    
    while (run)
    {
        // do initial calibration routine
        while (calibrate)
        {
            printf("calibrating\n");
            
            pthread_mutex_lock( &control_mutex );
            calibrate = false;
            for (int i=0; i<5; i++)
            {
                control.motorCommand[i].valid = false;
                lastcmd.motorCommand[i].valid = false;
            }
            pthread_mutex_unlock( &control_mutex );
            
            // stop collection
            memset(send_buff, 0, COMMAND_PACKET_LENGTH);
            send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
            send_buff[COMMAND_OFFSET] = STOP_COLLECTION_OPCODE; // stop collection
            //setPayload(send_buff, 0); // no payload
            send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
            if (!repeatGenericCommand(fd, send_buff, 10))
            {
                printf("ERROR: Not calibrating because could not stop collection\n");
                break;
            }
            
            // stop motors
            // sanity check: make sure all motors are responding.
            bool allgood = true;
            for(int i=0; i<5; i++)
            {
                printf("stopping motor %d\n", i);
                if (!repeatSendMotor(fd, i, 0, MOTOR_VELOCITY, 5))
                    allgood = false;
            }
            
            if (!allgood)
            {
                printf("ERROR: Not calibrating because motor unresponsive\n");
                break;
            }

            // get initial finger angles            
            HandSensors tmpdata;
            HandSensorsValid tmpvalid;
            
            // sanity check: make sure the proximal joint sensors are returning
            // valid data.
            bool allvalid = true;
            for (int finger=0; finger<3; finger++)
            {
                if (!repeatPollFinger(fd, 
                                      finger, //note: this is software numbering
                                      DATA_COLLECTION_PROXIMALJOINT_BITMASK,
                                      tmpdata, 
                                      tmpvalid, 
                                      5))
                    allgood = false;
                    
                if (!tmpvalid.proximalJointAngle[finger])
                    allvalid = false;
            }

            if (!allgood || !allvalid)
            {
                printf("ERROR: Not calibrating because finger unresponsive\n");
                break;
            }
            
            // sanity check: make sure proximal joint sensors are in range.
            // if not, then the fingers will need to be calibrated with the
            // low-level utility.
            bool inrange = true;
            for (int finger=0; finger<3; finger++)
            {
                if ( tmpdata.proximalJointAngle[finger] < -20 ||
                     tmpdata.proximalJointAngle[finger] > 450)
                    inrange = false;
            }
            
            if (!inrange)
            {
                printf("ERROR: Not calibrating because finger angle out of range\n");
                break;
            }
            
            // for each finger:
            bool breakcal = false;
            for (int finger=0; finger<3; finger++)
            {
                printf("finger %d\n", finger);
                
                if (tmpdata.proximalJointAngle[finger] < 100 || finger == 2)
                {
                    // close finger a little
                    printf("closing\n");
                    //repeatSendMotor(fd, fingerNumber_to_motorChain[finger], 1000, MOTOR_POSITION, 5);

                    if (!repeatSendMotor(fd, fingerNumber_to_motorChain[finger], 1500, MOTOR_VELOCITY, 5))
                    {
                        printf("ERROR: stopping calibration routine. motor communication error\n");
                        breakcal = true;
                        break;
                    }

                    int timeout_iter = 9000;
                    int target = 100;
                    
                    // open antagonist tendon for F3 if needed
                    if (finger == 2)
                    {
                        printf("opening antagonist\n");
                        if (!repeatSendMotor(fd, fingerNumber_to_motorChain[3], -1500, MOTOR_VELOCITY, 5))
                        {
                            printf("ERROR: stopping calibration routine. motor communication error\n");
                            breakcal = true;
                            break;
                        }
                        
                        target = 400;
                        timeout_iter = (400-tmpdata.proximalJointAngle[finger]) * 70 + 2000;
                    }
 
                    int iter = 0;
                    printf("max iter= %d\n", timeout_iter);
                    do
                    {
                        repeatPollFinger(fd, 
                                         finger, //note: this is software numbering
                                         DATA_COLLECTION_PROXIMALJOINT_BITMASK,
                                         tmpdata, 
                                         tmpvalid, 
                                         1);
                        if (iter % 1000 == 0)
                            printf("iter %5d  angle %3d\n", iter, tmpdata.proximalJointAngle[finger]);
                    }
                    while (run && 
                           iter++ < timeout_iter && 
                           tmpdata.proximalJointAngle[finger] < target );
                    if (!run)
                        cleanup(fd);
                    if (iter > timeout_iter)
                    {
                        printf("ERROR: stopping calibration routine due to timeout\n");
                        breakcal = true;
                        break;
                    }
                    printf("ending iter= %d\n", iter);
                    
                    // stop motor
                    printf("stopping\n");
                    if (!repeatSendMotor(fd, fingerNumber_to_motorChain[finger], 0, MOTOR_VELOCITY, 5))
                    {
                        printf("ERROR: stopping calibration routine. motor communication error\n");
                        breakcal = true;
                        break;
                    }
                    if (finger == 2)
                    {
                        printf("stopping antagonist\n");
                        if (!repeatSendMotor(fd, fingerNumber_to_motorChain[3], 0, MOTOR_VELOCITY, 5))
                        {
                            printf("ERROR: stopping calibration routine. motor communication error\n");
                            breakcal = true;
                            break;
                        }
                        
                        usleep(100000);
                        
                        printf("closing antagonist\n");
                        if (!repeatSendMotor(fd, fingerNumber_to_motorChain[3], 1500, MOTOR_VELOCITY, 5))
                        {
                            printf("ERROR: stopping calibration routine. motor communication error\n");
                            breakcal = true;
                            break;
                        }
                        
                        iter = 0;
                        timeout_iter += 5000;
                        printf("max iter= %d\n", timeout_iter);
                        do
                        {
                            repeatPollFinger(fd, 
                                             finger, //note: this is software numbering
                                             DATA_COLLECTION_PROXIMALJOINT_BITMASK,
                                             tmpdata, 
                                             tmpvalid, 
                                             1);
                            if (iter % 1000 == 0)
                                printf("iter %5d  angle %3d\n", iter, tmpdata.proximalJointAngle[finger]);
                        }
                        while (run && 
                               iter++ < timeout_iter && 
                               tmpdata.proximalJointAngle[finger] > 390 );
                        if (!run)
                            cleanup(fd);
                        if (iter > timeout_iter)
                        {
                            printf("ERROR: stopping calibration routine due to timeout\n");
                            breakcal = true;
                            break;
                        }
                        printf("ending iter= %d\n", iter);
                        
                        printf("stopping antagonist\n");
                        if (!repeatSendMotor(fd, fingerNumber_to_motorChain[3], 0, MOTOR_VELOCITY, 5))
                        {
                            printf("ERROR: stopping calibration routine. motor communication error\n");
                            breakcal = true;
                            break;
                        }
                        
                        printf("calibrating antagonist\n");
                        memset(send_buff, 0, COMMAND_PACKET_LENGTH);
                        send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_MOTOR1; // note the finger number to motor chain conversion here
                        send_buff[COMMAND_OFFSET] = CALIBRATION_OPCODE;
                        setPayload(send_buff, DATA_COLLECTION_MOTORHALL_BITMASK);
                        send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH);
                        if (!repeatGenericCommand(fd, send_buff, 10))
                        {
                            printf("ERROR: stopping calibration routine. calibration failed\n");
                            breakcal = true;
                            break;
                        }
                    }
                    
                    // wait a bit
                    usleep(100000);
                }
                else
                {
                    printf("skipping finger close step due to finger angle\n");
                }
                
                // get new finger angle
                repeatPollFinger(fd, 
                                 finger, //note: this is software numbering
                                 DATA_COLLECTION_PROXIMALJOINT_BITMASK,
                                 tmpdata, 
                                 tmpvalid, 
                                 5);
                
                int timeout_iter = tmpdata.proximalJointAngle[finger] * 70 + 2000;
                
                // open at a slow velocity
                printf("opening\n");
                
                if (!repeatSendMotor(fd, fingerNumber_to_motorChain[finger], -1500, MOTOR_VELOCITY, 5))
                {
                    printf("ERROR: stopping calibration routine. motor communication error\n");
                    breakcal = true;
                    break;
                }

                // loop poll and monitor proximal angle to go to zero
                int iter = 0;
                printf("max iter= %d\n", timeout_iter);
                do
                {
                    repeatPollFinger(fd, 
                                     finger, //note: this is software numbering
                                     DATA_COLLECTION_PROXIMALJOINT_BITMASK,
                                     tmpdata, 
                                     tmpvalid, 
                                     1);
                    if (iter % 1000 == 0)
                        printf("iter %5d  angle %3d\n", iter, tmpdata.proximalJointAngle[finger]);
                }
                while (run && 
                       iter++ < timeout_iter && 
                       tmpdata.proximalJointAngle[finger] > 2);
                if (!run)
                    cleanup(fd);
                if (iter >= timeout_iter)
                {
                    printf("ERROR: stopping calibration routine due to timeout\n");
                    breakcal = true;
                    break;
                }
                printf("ending iter= %d\n", iter);

                // stop motor
                printf("stopping\n");
                if (!repeatSendMotor(fd, fingerNumber_to_motorChain[finger], 0, MOTOR_VELOCITY, 5))
                {
                    printf("ERROR: stopping calibration routine. motor communication error\n");
                    breakcal = true;
                    break;
                }
                
                int destination = 0;
                switch(finger)
                {
                    // note the finger number to motor chain conversion here
                    case 0: destination = DESTINATION_MOTOR4; break;
                    case 1: destination = DESTINATION_MOTOR2; break;
                    case 2: destination = DESTINATION_MOTOR3; break;
                }
                    
                // calibrate hall
                printf("calibrating\n");
                memset(send_buff, 0, COMMAND_PACKET_LENGTH);
                send_buff[DESTINATION_HEADER_OFFSET] = destination;
                send_buff[COMMAND_OFFSET] = CALIBRATION_OPCODE;
                setPayload(send_buff, DATA_COLLECTION_MOTORHALL_BITMASK);
                send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH);
                repeatGenericCommand(fd, send_buff, 10);
                // NOTE: this command pretty much always times-out
                
                // wait a bit
                usleep(100000);
            } //end for each finger
            if (breakcal)
                break;
            
            // make sure fingers are near zero before doing spread adjust.
            if ( abs(tmpdata.proximalJointAngle[0]) < 20 &&
                 abs(tmpdata.proximalJointAngle[1]) < 20)
            {
                printf("doing spread adjust.");
            
                // open spread with velocity command
                printf("opening spread\n");
                repeatSendMotor(fd, 4, -4095, MOTOR_VELOCITY, 5);
                
                // wait 3 seconds
                printf("waiting\n");
                sleep(3);
                
                repeatPollSensor(fd, 
                                 DESTINATION_PALM, 
                                 RESPONDING_DEVICES_PALM_BITMASK, 
                                 DATA_COLLECTION_FINGERROTATION_BITMASK, 
                                 tmpdata, 
                                 tmpvalid, 
                                 5);
                
                printf("read spread value = %d\n", tmpdata.fingerSpread);
                
                // if value too close to -1023 then adjust.
                if (tmpdata.fingerSpread < -512)
                    spread_adjust = 1023;
                
                printf("setting spread_adjust to %d\n", spread_adjust);
                
                // set spread to new zero
                repeatSendMotor(fd, 4, 0, MOTOR_POSITION, 5);
                
                usleep(1000000);
            }
            else
            {
                printf("ERROR: not calibrating spread because of finger angles\n");
            }
            
            printf("\ndone calibrating\n");
         
            pthread_mutex_lock( &control_mutex );
            calibrate = false;
            for (int i=0; i<5; i++)
            {
                control.motorCommand[i].valid = false;
                lastcmd.motorCommand[i].valid = false;
            }
            pthread_mutex_unlock( &control_mutex );
        } //end while (calibrate)
        
        calibrate = false;
        for (int b=0; b<12; b++)
            sensor_data.responseHistory[b] = 0;
        for (int i=0; i<5; i++)
            sensor_data.motorError[i] = 0;
        
        // stop motors
        for(int i=0; i<5; i++)
            repeatSendMotor(fd, i, 0, MOTOR_VELOCITY, 5);
        
        if (no_tact)
            sensor_mask = sensor_mask & (~DATA_COLLECTION_TACTILE_BITMASK);
        if (no_tact_temp)
            sensor_mask = sensor_mask & (~DATA_COLLECTION_TACTILE_TEMP_BITMASK);
        if (no_flex)
            sensor_mask = sensor_mask & (~DATA_COLLECTION_DISTALJOINT_BITMASK);        
        if (no_accel)
            sensor_mask = sensor_mask & (~DATA_COLLECTION_ACCELERATION_BITMASK);        

        printf("Sensor mask: 0x%04X\n", sensor_mask);
        
        unsigned char chain_mask = ALL_CHAINS_CHAINMASK;
        if (dumb_fingers)
        {
            chain_mask = chain_mask & (~FINGER_1_CHAINMASK);
            chain_mask = chain_mask & (~FINGER_2_CHAINMASK);
            chain_mask = chain_mask & (~FINGER_3_CHAINMASK);
        }
        if (dumb_finger1)
            chain_mask = chain_mask & (~FINGER_2_CHAINMASK); //NOTE: hw/sw finger mapping
        if (dumb_finger2)
            chain_mask = chain_mask & (~FINGER_1_CHAINMASK); //NOTE: hw/sw finger mapping
        if (dumb_finger3)
            chain_mask = chain_mask & (~FINGER_3_CHAINMASK);
        if (dumb_tactile)
            chain_mask = chain_mask & (~TACTILE_CHAINMASK);

        printf("Chain mask: 0x%02X\n", chain_mask);
        
        printf("configuring hardware ");

        //   
        // set sample period
        //
        memset(send_buff, 0, COMMAND_PACKET_LENGTH);
        send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
        send_buff[COMMAND_OFFSET] = SET_SAMPLE_PERIOD_OPCODE; // set sample period
        setPayload(send_buff, (unsigned int)round(1000000.0/(float)hz));
        send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
        if (!repeatGenericCommand(fd, send_buff, 5))
            exit(3);
    
        //
        // set sample args
        //
        memset(send_buff, 0, COMMAND_PACKET_LENGTH);
        send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
        send_buff[COMMAND_OFFSET] = SET_SAMPLE_ARGS_OPCODE; // set sample args
        setPayload(send_buff, sensor_mask);
        send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
        if (!repeatGenericCommand(fd, send_buff, 5))
            exit(3);
        
        //
        // start collection
        //
        memset(send_buff, 0, COMMAND_PACKET_LENGTH);
        send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
        send_buff[COMMAND_OFFSET] = START_COLLECTION_OPCODE; // start collection
        //setPayload(send_buff, 0); // no payload
        send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
        if (!repeatGenericCommand(fd, send_buff, 5))
            exit(3);

        //
        // set chain mask
        //
        memset(send_buff, 0, COMMAND_PACKET_LENGTH);
        send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
        send_buff[COMMAND_OFFSET] = SET_CHAIN_MASK_OPCODE;
        send_buff[PAYLOAD_OFFSET] = chain_mask; // 1 byte payload
        send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
        if (!repeatGenericCommand(fd, send_buff, 5))
            exit(3);
        
        printf("\nDONE CONFIGURING HARDWARE\n");
    
        if (debug_time)
        {
            printf("\nloop  sensor  motor  errors  palm  f1p  f1d  f2p  f2d  f3p  f3d  m1  m2  m3  m4  tactile\n");
        }
    
        unsigned char recv_buff[BUFFER];
        HandSensors tmpdata;
        HandSensorsValid valid;
        int count = 0;
        int command_index = 0;
        // int spreadcount = 0;
        // int spreadreal = 0;
    
        //
        // variables for debug time
        //
        timeval tv_now;
        timeval tv_last_motor;
        timeval tv_last_sensor;
        timeval tv_last_loop;
        gettimeofday(&tv_now, NULL);
        gettimeofday(&tv_last_motor, NULL);
        gettimeofday(&tv_last_sensor, NULL);
        gettimeofday(&tv_last_loop, NULL);
        int motor_time_buff[100];
        int sensor_time_buff[100];
        int board_time_buff[12][100];
        timeval tv_last_board[12];
        int board_time_index[12];
        int loop_time_buff[100];
        int motor_time_index = 1;
        int sensor_time_index = 1;
        int loop_time_index = 1;
        // bool motor_time_disp = false;
        // bool sensor_time_disp = false;
        int errorcount = 0;
    
        for (int b=0; b<12; b++)
        {
            gettimeofday(&tv_last_board[b], NULL);
            board_time_index[b] = 1;
        
            for (int i=0; i<100; i++)
                board_time_buff[b][i] = 0;
        }
    
        for (int i=0; i<100; i++)
        {
            motor_time_buff[i] = 0;
            sensor_time_buff[i] = 0;
            loop_time_buff[i] = 0;
        }
        
        //
        // variables for response history
        //
        int loop_count = 0; // same as count, but count can be decremented when motor command fails
        int responsesArray[12][100];
        for (int b=0; b<12; b++)
            for (int i=0; i<100; i++)
                responsesArray[b][i] = 0;
        
        int last_motor_response[5];
        for (int b=0; b<5; b++)
            last_motor_response[b] = 0;
        
        // Yes, this is a second while(run) loop.  This pattern makes it easy 
        // to break out of the inner loop to return to calibration code at the
        // top of the outer loop.
        while (run)
        {
            pthread_mutex_lock( &control_mutex );
            
            if (calibrate)
            {
                pthread_mutex_unlock( &control_mutex );
                break;
            }

            loop_count++;

            if (count++ % 2 == 0 && anyValid(command_index))
            {
                if (debug_motors)
                {
                    printf("CMD: ");
                    for (int i=0; i<5; i++)
                    {
                        if (control.motorCommand[i].valid)
                            printf("%05d ", control.motorCommand[i].value);
                        else
                            printf("_____ ");
                    }
                    printf("\n");

                    printf("LST: ");
                    for (int i=0; i<5; i++)
                    {
                        if (lastcmd.motorCommand[i].valid)
                            printf("%05d ", lastcmd.motorCommand[i].value);
                        else
                            printf("_____ ");
                    }
                    printf("\n");
            
                    printf("CNT: ");
                    for (int i=0; i<5; i++)
                        printf("%05d ", skip_count[i]);
                    printf("\n");

                    printf("TYP: ");
                    for (int i=0; i<5; i++)
                    {
                        if (i == command_index)
                        {
                            if (control.motorCommand[i].type == MOTOR_POSITION)
                                printf("PPPPP ");
                            else if (control.motorCommand[i].type == MOTOR_VELOCITY)
                                printf("RRRRR ");
                            else if (control.motorCommand[i].type == MOTOR_CURRENT)
                                printf("CCCCC ");
                            else if (control.motorCommand[i].type == MOTOR_VOLTAGE)
                                printf("VVVVV ");
                            else
                                printf("***** ");
                        }
                        else
                            printf("_____ ");
                    }
                    printf("\n"); 
                }

                control.motorCommand[command_index].valid = false;
                MotorCommand command = control.motorCommand[command_index];
                pthread_mutex_unlock( &control_mutex );
            
                int set = 0;
                bool override = false;
                if (command.type == MOTOR_POSITION)
                {
                    set = motorControl(command_index, command.value, override); // does thermal protection
                    if (override)
                    {
                        set = 0;
                        command.type = MOTOR_VELOCITY;
                        command.valid = true;
                    }
                }
                else if (command.type == FINGER_ANGLE)
                {
                    set = motorControlFingerAngle(command_index, command.value); // does P control
                    command.type = MOTOR_VELOCITY;
                }
                else
                    set = command.value;
            
                if (debug_motors)
                {
                    printf("AAT: ");            
                    for (int i=0; i<5; i++)
                    {
                        if (i < 4)
                            printf("%05d ", sensor_data.motorHallEncoder[motorChain_to_fingerNumber[i]]);
                        else
                            printf("%05d ", sensor_data.fingerSpread);
                    }
                    printf("\n"); 
            
                    printf("SET: ");
                    for (int i=0; i<5; i++)
                    {
                        if (i == command_index)
                            printf("%05d ", set);
                        else
                            printf("_____ ");
                    }
                    printf("\n"); 
                }

                bool skip = false;
                if (lastcmd.motorCommand[command_index].valid && 
                    lastcmd.motorCommand[command_index].type == command.type && 
                    lastcmd.motorCommand[command_index].value == set)
                {
                    // this command is the same as the last
                
                    skip_count[command_index]++;
                
                    if (skip_count[command_index] < MAX_SKIP)
                    {
                        // actually skip
                    
                        if (debug_motors)
                        {
                            printf("SKIPPING\n");
                        }
                    
                        skip = true;
                        count--; // so we stay in motor command mode
                    }
                }
            
                if (!skip)
                {
                    skip_count[command_index] = 0;
                
                    if (debug_motors)
                    {
                        printf("SETTING\n");
                    }
            
                    lastcmd.motorCommand[command_index].valid = true;
                    lastcmd.motorCommand[command_index].type = command.type;
                    lastcmd.motorCommand[command_index].value = set;
            
                    int trynum = 0;
                    int ret = 0;
                
                    ret = writeMotor(fd, command_index, set, command.type);

                    // error writing to device, don't listen for response, just put command back in queue
                    if (ret != 0)
                    {
                        printf("\nwrite error: %02X\n", ret);
                        trynum = CMD_WAIT_TRIES+1;
                    }

                    // listen for up to CMD_WAIT_TRIES incoming messages to hear expected response
                    while (trynum++ <= CMD_WAIT_TRIES)
                    {
                        //printf("listen response try %d\n", trynum);
                
                        tmpdata.reset();
                        valid.reset();
                        bool sensorMsg;
                        int code = parseResponse(fd, sensor_mask, sensorMsg, tmpdata, valid, recv_buff, BUFFER);
                        if (code == 0) // no error
                        {
                            if (debug_motors)
                            {
                                printf("&");
                            }
                            if (sensorMsg)
                            {
                                if (debug_time)
                                {
                                    gettimeofday(&tv_now, NULL);
                                    sensor_time_buff[sensor_time_index++ % 100] = 
                                        1000000*(tv_now.tv_sec - tv_last_sensor.tv_sec) + (tv_now.tv_usec - tv_last_sensor.tv_usec);
                                    tv_last_sensor = tv_now;
                                    //sensor_time_disp = true;
                                
                                    int respondingDevices = (recv_buff[RESPONSE_BROADCAST_RESPONDINGDEVICES+1] << 8) | 
                                        recv_buff[RESPONSE_BROADCAST_RESPONDINGDEVICES];
                                    for (int i=0; i<12; i++)
                                    {
                                        int mask = 0x1 << i;
                                        if (respondingDevices & mask)
                                        {
                                            board_time_buff[i][board_time_index[i]++ % 100] = 
                                                1000000*(tv_now.tv_sec - tv_last_board[i].tv_sec) + (tv_now.tv_usec - tv_last_board[i].tv_usec);
                                            tv_last_board[i] = tv_now;  
                                        }
                                    }
                                }
                        
                                sensor_data.update(tmpdata, valid);
                                sensor_data.fingerSpread += spread_adjust;
                                for (int b=0; b<12; b++)
                                {
                                    sensor_data.responseHistory[b] -= responsesArray[b][loop_count % 100];
                                    responsesArray[b][loop_count % 100] = sensor_data.responses[b];
                                    sensor_data.responseHistory[b] += sensor_data.responses[b];
                                }
                                for (int i=0; i<5; i++)
                                    sensor_data.motorError[i] = last_motor_response[i];
                                
                                if (tcp_fd >= 0)
                                {
                                    HandPacket packet(sensor_data);
                                    int len = packet.pack(recv_buff);
                                    if (udp)
                                    {
                                        if (sendalludp(tcp_fd, recv_buff, len, (const struct sockaddr*)&addr, addrlen) == -1)
                                            tcp_fd = -1;
                                    }
                                    else
                                    {
                                        if (sendall(tcp_fd, recv_buff, len, 0) == -1)
                                            tcp_fd = -1;
                                    }
                                }
                            }
                            else
                            {
                                if (debug_motors)
                                {
                                    printf("@");
                                }
                                
                                last_motor_response[motorChain_to_fingerNumber[command_index]] = 0;
                                
                                // Got motor command response
                                if (debug_time)
                                {
                                    gettimeofday(&tv_now, NULL);
                                    motor_time_buff[motor_time_index++ % 100] = 
                                        1000000*(tv_now.tv_sec - tv_last_motor.tv_sec) + (tv_now.tv_usec - tv_last_motor.tv_usec);
                                    tv_last_motor = tv_now;
                                    //motor_time_disp = true;

                                    int respondingDevices = 0;
                                    switch (command_index)
                                    {
                                        case 0:
                                            respondingDevices = RESPONDING_DEVICES_FIRST_MOTOR1_BITMASK;
                                            break;
                                        case 1:
                                            respondingDevices = RESPONDING_DEVICES_FIRST_MOTOR2_BITMASK;
                                            break;
                                        case 2:
                                            respondingDevices = RESPONDING_DEVICES_SECOND_MOTOR1_BITMASK;
                                            break;
                                        case 3:
                                            respondingDevices = RESPONDING_DEVICES_SECOND_MOTOR2_BITMASK;
                                            break;
                                    }
                                    
                                    for (int i=0; i<12; i++)
                                    {
                                        int mask = 0x1 << i;
                                        if (respondingDevices & mask)
                                        {
                                            board_time_buff[i][board_time_index[i]++ % 100] = 
                                                1000000*(tv_now.tv_sec - tv_last_board[i].tv_sec) + (tv_now.tv_usec - tv_last_board[i].tv_usec);
                                            tv_last_board[i] = tv_now;  
                                        }
                                    }
                                }
                                // if we ever implement a GET type message, this is where we would parse it.
                                break;
                            }
                        }
                        else // error
                        {
                            last_motor_response[motorChain_to_fingerNumber[command_index]] = code;
                            if (debug_errors)
                            {
                                printf("code error: %02X while waiting for motor FM%d (device %d) response\n", 
                                       code, 
                                       motorChain_to_fingerNumber[command_index]+1, 
                                       7+command_index);
                            }
                            if (sensorMsg)
                            {
                                // do nothing, keep waiting for response.
                                // sometimes sensor messages can return an error code if there is a data length issue.
                            }
                            else
                            {
                                // got an error from our command
                                // break loop with error
                                trynum = CMD_WAIT_TRIES+1;
                                break;
                            }
                        }
                    }
            
                    if (trynum >= CMD_WAIT_TRIES) // either did not hear a response, or got an error
                    {
                        if (debug_time)
                        {
                            errorcount++;
                        }
                        
                        // only report if code not specified above
                        if (last_motor_response[motorChain_to_fingerNumber[command_index]] == 0)
                            last_motor_response[motorChain_to_fingerNumber[command_index]] = 20;

                        if (debug_errors)
                        {
                            printf("no response or error when commanding motor %d\n", command_index);
                        }
                        // put back in queue to try again
                        pthread_mutex_lock( &control_mutex );
                        lastcmd.motorCommand[command_index].valid = false;
                        if (!control.motorCommand[command_index].valid) // make sure newer data is not already set
                            control.motorCommand[command_index].valid = true;
                        pthread_mutex_unlock( &control_mutex );
                    }
                }
            
                command_index++;
                command_index = command_index % 5;
            }
            else // only listen for sensor data
            {
                pthread_mutex_unlock( &control_mutex );
            
                tmpdata.reset();
                valid.reset();
                bool sensorMsg;
                int code = parseResponse(fd, sensor_mask, sensorMsg, tmpdata, valid, recv_buff, BUFFER);
                if (code == 0) // no error
                {
                    if (sensorMsg)
                    {
                        if (debug_motors)
                        {
                            printf("$");
                        }
                        if (debug_time)
                        {
                            gettimeofday(&tv_now, NULL);
                            sensor_time_buff[sensor_time_index++ % 100] = 
                                1000000*(tv_now.tv_sec - tv_last_sensor.tv_sec) + (tv_now.tv_usec - tv_last_sensor.tv_usec);
                            tv_last_sensor = tv_now;
                            //sensor_time_disp = true;

                            int respondingDevices = (recv_buff[RESPONSE_BROADCAST_RESPONDINGDEVICES+1] << 8) | 
                                recv_buff[RESPONSE_BROADCAST_RESPONDINGDEVICES];
                            for (int i=0; i<12; i++)
                            {
                                int mask = 0x1 << i;
                                if (respondingDevices & mask)
                                {
                                    board_time_buff[i][board_time_index[i]++ % 100] = 
                                        1000000*(tv_now.tv_sec - tv_last_board[i].tv_sec) + (tv_now.tv_usec - tv_last_board[i].tv_usec);
                                    tv_last_board[i] = tv_now;  
                                }
                            }
                        }

                        sensor_data.update(tmpdata, valid);
                        sensor_data.fingerSpread += spread_adjust;
                        for (int b=0; b<12; b++)
                        {
                            sensor_data.responseHistory[b] -= responsesArray[b][loop_count % 100];
                            responsesArray[b][loop_count % 100] = sensor_data.responses[b];
                            sensor_data.responseHistory[b] += sensor_data.responses[b];
                        }
                        for (int i=0; i<5; i++)
                            sensor_data.motorError[i] = last_motor_response[i];

                        if (tcp_fd >= 0)
                        {
                            HandPacket packet(sensor_data);
                            int len = packet.pack(recv_buff);
                            if (udp)
                            {
                                if (sendalludp(tcp_fd, recv_buff, len, (const struct sockaddr*)&addr, addrlen) == -1)
                                    tcp_fd = -1;
                            }
                            else
                            {
                                if (sendall(tcp_fd, recv_buff, len, 0) == -1)
                                    tcp_fd = -1;
                            }
                        }
                    }
                    else
                    {
                        if (debug_errors)
                        {
                            printf("unwarranted response: expecting sensor message\n");
                        }
                    }
                }
                else
                {
                    if (debug_time)
                    {
                        errorcount++;
                    }
                    if (debug_errors)
                    {
                        printf("code error: %02X while waiting for sensor message\n", code);
                    }
                }
            }
        
            if (debug_time)
            {
                gettimeofday(&tv_now, NULL);
                loop_time_buff[loop_time_index++ % 100] = 
                    1000000*(tv_now.tv_sec - tv_last_loop.tv_sec) + (tv_now.tv_usec - tv_last_loop.tv_usec);
                tv_last_loop = tv_now;
        
                if (loop_time_index % 100 == 0)
                {
                    double laverage = 0;
                    for (int i=0; i<100; i++)
                        laverage += loop_time_buff[i];
                    laverage /= 100.0;
                
                    double saverage = 0;
                    for (int i=0; i<100; i++)
                        saverage += sensor_time_buff[i];
                    saverage /= 100.0;
                
                    double maverage = 0;
                    for (int i=0; i<100; i++)
                        maverage += motor_time_buff[i];
                    maverage /= 100.0;
                
                    printf("%.2f  %.2f  %.2f  %d  ", 1000000.0/laverage, 1000000.0/saverage, 1000000.0/maverage, errorcount);
                
                    for (int b=0; b<12; b++)
                    {
                        double average = 0;
                        for (int i=0; i<100; i++)
                            average += board_time_buff[b][i];
                        average /= 100.0;
                        printf("%.2f  ", 1000000.0/average);
                    }
                    printf("\n");
                
                    errorcount = 0;
                }
            }
            
            if (debug_quality)
            {
                if (loop_count % 100 == 0)
                {
                    for (int b=0; b<12; b++)
                        printf("%3d ", sensor_data.responseHistory[b]);
                    printf(" : ");
                    for (int b=0; b<5; b++)
                        printf("%3d ", sensor_data.motorError[b]);
                    printf("\n");
                }
            }
            
        } //end inner while(run)
    } // end outer while(run)
    
    pthread_mutex_lock( &control_mutex );
    for (int i=0; i<5; i++)
    {
        control.motorCommand[i].valid = false;
        lastcmd.motorCommand[i].valid = false;
    }
    pthread_mutex_unlock( &control_mutex );
    
    cleanup(fd);
    return NULL;
};


int init_socket(const char* const port)
{
    int sockfd;
    struct addrinfo hints, *servinfo, *p;
    int yes=1;
    
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    if (udp)
        hints.ai_socktype = SOCK_DGRAM;
    else
        hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE; // use my IP

    int rv = getaddrinfo(NULL, port, &hints, &servinfo);
    if (rv != 0) {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
        return -1;
    }
        
    // loop through all the results and bind to the first we can
    for(p = servinfo; p != NULL; p = p->ai_next) {
        if ((sockfd = socket(p->ai_family, p->ai_socktype,
                             p->ai_protocol)) == -1) {
            perror("server: socket");
            continue;
        }

        if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes,
                       sizeof(int)) == -1) {
            perror("setsockopt SO_REUSEADDR");
            return -1;
        }

        if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &yes,
                       sizeof(int)) == -1) {
            perror("setsockopt SO_BROADCAST");
            return -1;
        }

        if (timeout > 0)
        {
            struct timeval tv;
            tv.tv_sec = (int)timeout;
            tv.tv_usec = (timeout - (int)timeout) * 1000000;
            if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval)) == -1)
            {
                perror("setsockopt SO_RCVTIMEO");
                return -1;
            }
        }
        
        if (bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
            close(sockfd);
            perror("server: bind");
            continue;
        }

        break;
    }

    if (p == NULL)  {
        fprintf(stderr, "server: failed to bind\n");
        return -1;
    }

    freeaddrinfo(servinfo); // all done with this structure

    if (!udp)
    {
        if (listen(sockfd, 10) == -1)
        {
            perror("listen");
            return -1;
        }
    }
    
    return sockfd;
}

void printUsage(FILE* stream=stdout)
{
    fprintf(stream, "usage: handle_controller [options]\n");
    fprintf(stream, "options:\n");
    fprintf(stream, "  -r, --rate, --hz <hz> Set sensor rate in hz.  Default: %d\n", DEFAULT_HZ);
    fprintf(stream, "  -p, --port <port>     Set the TCP/UDP port number.  Default: %s\n", DEFAULT_PORT);
    fprintf(stream, "  -c, --calibrate       Run a calibration routine on startup.  Default: false\n");
    fprintf(stream, "  -d, --dumb            Hand has \"dumb fingers\", don't ask for finger data.\n");
    fprintf(stream, "                        Default: false\n");
    fprintf(stream, "      --dumb_finger1    Only finger 1 is \"dumb\".  Default: false\n");
    fprintf(stream, "      --dumb_finger2    Only finger 2 is \"dumb\".  Default: false\n");
    fprintf(stream, "      --dumb_finger3    Only finger 3 is \"dumb\".  Default: false\n");
    fprintf(stream, "      --dumb_tactile    Only tactile palm is \"dumb\".  Default: false\n");
    fprintf(stream, "  -s, --sensors <mask>  Specify exactly which sensors you want to receive.\n");
    fprintf(stream, "                        This should be an ORed combination of DATA_COLLECTION_*\n");
    fprintf(stream, "                        bitfields in handleTypes.hpp.  Default: \"0x%04X\"\n", DEFAULT_SENSOR_MASK);
    fprintf(stream, "      --no_tact         Do not return any tactile sensor data.\n");
    fprintf(stream, "                        Applied after the \"--sensors\" flag.  Default: false\n");
    fprintf(stream, "      --no_tact_temp    Do not return any tactile temperature sensor data.\n");
    fprintf(stream, "                        Applied after the \"--sensors\" flag.  Default: false\n");
    fprintf(stream, "      --no_flex         Do not return any flexure sensor data.\n");
    fprintf(stream, "                        Applied after the \"--sensors\" flag.  Default: false\n");
    fprintf(stream, "      --no_accel        Do not return any accelerometer sensor data.\n");
    fprintf(stream, "                        Applied after the \"--sensors\" flag.  Default: false\n");
    fprintf(stream, "  -u, --udp             Use UDP instead of TCP.  Default: false\n");
    fprintf(stream, "  -o, --timeout <secs>  Set this control rate timeout.  Default: %f\n", DEFAULT_TIMEOUT);
    fprintf(stream, "                        <secs> is a float.  Set <= 0 to disable.\n");
    fprintf(stream, "                        Motors are stopped when timeout triggers.\n");
    fprintf(stream, "  -t --debug_time       Debug: print measured rates.  Default: false\n");
    fprintf(stream, "  -m --debug_motor      Debug: print motor info.  Default: false\n");
    fprintf(stream, "  -e --debug_error      Debug: print error codes.  Default: false\n");
    fprintf(stream, "  -q --debug_quality    Debug: print communication quality.  Default: false\n");
    fprintf(stream, "  -h, --help            Display this help message and exit.  Default: false\n");
    fprintf(stream, "  -v, --version         Display version string and exit.  Default: false\n");
}

int main(int argc, char** argv)
{
    //
    // init globals
    //
    for (int i=0; i<5; i++)
        skip_count[i] = 0;
    
    char default_port[] = DEFAULT_PORT;
    char* port = default_port;
    
#ifdef DO_THERMAL
    for (int i=0; i<4; i++)
    {
        motor_override[i].history_i = 0;
        motor_override[i].last_error = 0;
        motor_override[i].count = 0;
        for (int j=0; j<OVERRIDE_HISTORY; j++)
        {
            motor_override[i].pos_history[j] = 0;
            motor_override[i].err_history[j] = 0;
        }
        //motor_override[i].history[0] = MAX_DELTA_NO_CHANGE;
    }
#endif

    option long_options[] = {
        {"rate",  required_argument, 0, 'r' },
        {"hz",    required_argument, 0, 'r' },
        {"port",  required_argument, 0, 'p' },
        {"timeout",  required_argument, 0, 'o' },
        {"calibrate",   no_argument, 0, 'c' },
        {"dumb",        no_argument, 0, 'd' },
        {"dumb_finger1", no_argument, 0, 1001 },
        {"dumb_finger2", no_argument, 0, 1002 },
        {"dumb_finger3", no_argument, 0, 1003 },
        {"dumb_tactile", no_argument, 0, 1004 },
        {"sensors", required_argument, 0, 's' },
        {"no_tact",      no_argument, 0, 2001 },
        {"no_tact_temp", no_argument, 0, 2002 },
        {"no_flex",      no_argument, 0, 2003 },
        {"no_accel",     no_argument, 0, 2004 },
        {"udp",         no_argument, 0, 'u' },
        {"version",     no_argument, 0, 'v' },
        {"debug_time",  no_argument, 0, 't' },
        {"debug_motor", no_argument, 0, 'm' },
        {"debug_error", no_argument, 0, 'e' },
        {"debug_quality", no_argument, 0, 'q' },
        {"help",        no_argument, 0, 'h' },
        {0,             0,           0,  0  }
    };

    //
    // parse command line options
    //
    int c;
    //while ((c = getopt(argc, argv, "r:p:ctmevuhd")) != -1)
    while ((c = getopt_long(argc, argv, "r:p:o:s:ctmevuhdq", long_options, NULL)) != -1)
    {
        switch (c)
        {
            case 'r':
                hz = atoi(optarg);
                if (hz <= 0)
                {
                    fprintf(stderr, "WARNING: value '%d' is outside of range for option 'r'\n", hz);
                    fprintf(stderr, "\n");
                    printUsage(stderr);
                    exit(1);
                }
                break;
            case 'p':
                port = optarg;
                if (atoi(port)<1 || atoi(port)>65535)
                {
                    fprintf(stderr, "WARNING: value '%s' is outside of range for option 'p'\n", optarg);
                    fprintf(stderr, "\n");
                    printUsage(stderr);
                    exit(1);
                }
                break;
            case 's':
                sensor_mask = strtol(optarg, NULL, 0);
                break;
            case 2001:
                no_tact = true;
                break;
            case 2002:
                no_tact_temp = true;
                break;
            case 2003:
                no_flex = true;
                break;
            case 2004:
                no_accel = true;
                break;
            case 'o':
                timeout = atof(optarg);
                break;
            case 'c':
                calibrate = true;
                break;
            case 't':
                debug_time = true;
                break;
            case 'm':
                debug_motors = true;
                break;
            case 'e':
                debug_errors = true;
                break; 
            case 'q':
                debug_quality = true;
                break;
            case 'd':
                dumb_fingers = true;
                break;
            case 1001:
                dumb_finger1 = true;
                break;
            case 1002:
                dumb_finger2 = true;
                break;
            case 1003:
                dumb_finger3 = true;
                break;
            case 1004:
                dumb_tactile = true;
                break;
            case 'u':
                udp = true;
                break;
            case 'v':
                printf("handle_controller version: %d\n", VERSION);
                exit(0);
                break;
            case 'h':
                printUsage();
                exit(0);
                break;
            case '?':
                if (optopt == 'r' || optopt == 'p' || optopt == 'o' || optopt == 's')
                    fprintf(stderr, "Option -%c requires an argument.\n", optopt);
                else if (isprint (optopt))
                    fprintf(stderr, "Unknown option `-%c'.\n", optopt);
                else
                    fprintf(stderr, "Unknown option character `\\x%x'.\n", optopt);
                fprintf(stderr, "\n");
                printUsage(stderr);
                exit(1);
            default:
                printUsage(stderr);
                exit(2);
        }
    }
    
    printf("command line options:\n");
    printf("  hz = %d\n", hz);
    printf("  port = %s\n", port);
    printf("  udp = %d\n", udp);
    printf("  timeout = %f\n", timeout);
    printf("  calibrate = %d\n", calibrate);
    printf("  dumb fingers = %d\n", dumb_fingers);
    printf("  dumb finger 1 = %d\n", dumb_finger1);
    printf("  dumb finger 2 = %d\n", dumb_finger2);
    printf("  dumb finger 3 = %d\n", dumb_finger3);
    printf("  dumb tactile = %d\n", dumb_tactile);
    printf("  sensor mask = 0x%04X\n", sensor_mask);
    printf("  no tact = %d\n", no_tact);
    printf("  no tact temp = %d\n", no_tact_temp);
    printf("  no flex = %d\n", no_flex);
    printf("  no accel = %d\n", no_accel);
    printf("  debug time = %d\n", debug_time);
    printf("  debug motors = %d\n", debug_motors);
    printf("  debug errors = %d\n", debug_errors);
    printf("  debug quality = %d\n", debug_quality);
    
    printf("setting up signal handlers\n");
    struct sigaction sa;
    sa.sa_handler = sigint_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART; //required or segfault on CTRL-C
    if (sigaction(SIGINT, &sa, NULL) == -1) 
    {
        perror("sigaction");
        exit(1);
    }
    
    // this seems to be required even though the body of the handler is empty
    struct sigaction sa2;
    sa2.sa_handler = sigpipe_handler; // print something when we get SIGPIPE signal
    sigemptyset(&sa2.sa_mask);
    sa2.sa_flags = SA_RESTART;
    if (sigaction(SIGPIPE, &sa2, NULL) == -1) 
    {
        perror("sigaction");
        exit(1);
    }
    
    //
    // spin off new thread for sensor and motor handling
    //
    pthread_t thread;
    if (pthread_create(&thread, NULL, serial_thread, NULL))
    {
        printf("ERROR: unable to create serial thread\n");
        exit(-1);
    }
    
    socketfd = init_socket(port);
    
    while(run) 
    {
        printf("server: waiting to accept\n");
        
        if (udp)
        {
            tcp_fd = socketfd;
        }
        else
        {
            struct sockaddr_storage their_addr; // connector's address information
            socklen_t sin_size = sizeof(sockaddr_storage);
            tcp_fd = accept(socketfd, (struct sockaddr *)&their_addr, &sin_size);
            if (tcp_fd == -1) 
                continue;
        }
        
        printf("receiver: starting\n");

        unsigned char rbuff[100];
        HandleCommand cmd;
        int len = cmd.pack(rbuff); // dummy pack to get serialization length.
        
        while (tcp_fd >= 0)
        {
            int ret = 0;
            if (udp)
                ret = recvalludp(tcp_fd, rbuff, len, (struct sockaddr*)&addr, (socklen_t*)&addrlen);
            else
                ret = recvall(tcp_fd, rbuff, len);
            
            //printf("receiver: got %d bytes\n", ret);

            if (ret < 0)
            {
                tcp_fd = -1;
            }
            else 
            {
                cmd.unpack(rbuff);
                
                if (cmd.calibrate)
                {
                    // command a calibration routine, invalidate all motor commands
                    pthread_mutex_lock( &control_mutex );
                    calibrate = true;
                    for (int i=0; i<5; i++)
                    {
                        control.motorCommand[i].valid = false;
                        lastcmd.motorCommand[i].valid = false;
                    }
                    pthread_mutex_unlock( &control_mutex );
                }
                else
                {
                    pthread_mutex_lock( &control_mutex );
                    if (debug_motors)
                    {
                        printf("GOT: ");
                    }
                    for (int i=0; i<5; i++)
                    {
                        if (cmd.motorCommand[i].valid)
                        {
                            // convert finger number into motor number
                            control.motorCommand[fingerNumber_to_motorChain[i]] = cmd.motorCommand[i];
                            
                            if (debug_motors)
                            {      
                                printf("%05d ", cmd.motorCommand[i].value);
                            }
                        }
                        else
                        {
                            if (debug_motors)
                            {      
                                printf("_____ ");
                            }
                        }
                    }
                    if (debug_motors)
                    {      
                        printf("\n");
                    }
                    pthread_mutex_unlock( &control_mutex );
                }
            }
        }
        
        printf("receiver: stopping motors\n");
        pthread_mutex_lock( &control_mutex );
        for (int i=0; i<5; i++)
        {
            control.motorCommand[i].valid = true;
            control.motorCommand[i].type = MOTOR_VELOCITY;
            control.motorCommand[i].value = 0;
            lastcmd.motorCommand[i].valid = false;
        }
        pthread_mutex_unlock( &control_mutex );
        
        usleep(1000000);

        printf("receiver: invalidating motors\n");
        pthread_mutex_lock( &control_mutex );
        for (int i=0; i<5; i++)
        {
            control.motorCommand[i].valid = false;
            lastcmd.motorCommand[i].valid = false;
        }
        pthread_mutex_unlock( &control_mutex );
        
        printf("receiver: closing\n");
        
        if (!udp)
            close(tcp_fd);
    }
    
    pthread_join(thread, NULL);
    
    printf("exiting main\n");
    return 0;
}
