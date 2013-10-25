/**
 * \file stopall.c
 *
 * stop all broadcasts and motors
 *
 * Written under government funding for ARM-H project.
 *
 * \author Ben Axelrod
 * \date   March 2012
 * \copyright Copyright iRobot Corporation, 2012
 **/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>

#include "../../handle_lib/include/handle_lib/handleTypes.hpp"
#include "../../handle_lib/include/handle_lib/packetParser.hpp"

//#define BAUDRATE B2000000
//#define BAUDRATE B115200
//#define BAUDRATE B500000
#define BAUDRATE B1152000

#define MODEMDEVICE "/dev/ttyS0"

#define BUFFER 1000


volatile bool run = true;
void sigint_handler(int s)
{
    run = false;
};

/// Set the motor [0-4] to the desired speed value.
// Note that value can be negative.
// Value will be capped at 2 bytes (65535).
// returns error code.
//  0 = success
// -2 = unknown motor
// -3 = write error
int writeMotor(int fd, int motor, int value)
{
    //printf("writing motor %d with %d\n", motor, value);
    
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
        buff[COMMAND_OFFSET] = MOTOR_COMMAND_OPCODE | MOTOR_COMMAND_VELOCITY;
    
    if (value == 0)
        buff[COMMAND_OFFSET] |= MOTOR_COMMAND_STOP;
    else if (value > 0)
        buff[COMMAND_OFFSET] |= MOTOR_COMMAND_FORWARD;
    else
        buff[COMMAND_OFFSET] |= MOTOR_COMMAND_REVERSE;
    
    //if (value < 0)
    //    value = -value;
    
    //if (value > 65535)
    //    value = 65535;
    
    setPayload(buff, (short)value);
    buff[CHECKSUM_OFFSET] = computeChecksum(buff, COMMAND_PACKET_LENGTH-1);

    if (write(fd, buff, COMMAND_PACKET_LENGTH) != COMMAND_PACKET_LENGTH)
        return -3;
    
    return 0;
};

int main(int argc, char** argv)
{
    int fd; // serial file descriptor
    
    printf("setting up signal handler\n");
    struct sigaction sa;
    sa.sa_handler = sigint_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART; //required or segfault on CTRL-C
    if (sigaction(SIGINT, &sa, NULL) == -1) 
    {
        perror("sigaction");
        exit(1);
    }

    printf("trying to open\n");
    fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY | O_NDELAY); 
    if (fd <0 ) 
    {
        printf("cannot open device\n");
        perror(MODEMDEVICE); 
        exit(-1); 
    }
    printf("serial port open on file descriptor %d\n", fd);
    
    /////////////////////////////////
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

    tcflush(fd, TCIOFLUSH); // flushes both directions

    // printf("before setattr\n");
    // printf("baudrate: %u\n", BAUDRATE);
    // printf("o speed: %u\n", cfgetospeed(&options));
    // printf("i speed: %u\n", cfgetispeed(&options));

    tcsetattr(fd, TCSANOW, &options);

    // printf("after setattr\n");
    // printf("o speed: %u\n", cfgetospeed(&options));
    // printf("i speed: %u\n", cfgetispeed(&options));
    //usleep(1000000);
    //////////////////////////////

    unsigned char send_buff[COMMAND_PACKET_LENGTH];
    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    
    unsigned char recv_buff[BUFFER];
    memset(recv_buff, 0, BUFFER);
    
    //
    // stop collection
    //
    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
    send_buff[COMMAND_OFFSET] = STOP_COLLECTION_OPCODE; // stop collection
    //setPayload(send_buff, DATA_COLLECTION_MOTORHALL_BITMASK); // no payload
    send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
    write(fd, send_buff, COMMAND_PACKET_LENGTH);

    printf("Stop collection: ");
    for (int i=0; i<COMMAND_PACKET_LENGTH; i++)
        printf("%02X ", send_buff[i]);
    printf("\n");
    
    int rsp = readResponse(fd);
    printf("returned: %02X\n", rsp);
    
    if (!run)
        exit(1);
    
    for (int i=0; i<5; i++)
    {
        printf("stopping motor %d\n", i+1);
        writeMotor(fd, i, 0);
        rsp = readResponse(fd);
        printf("returned: %02X\n", rsp);
        if (!run)
            exit(1);
    }
    
    printf("closing\n");
    close(fd);
    printf("exiting\n");
    exit(0);
}
