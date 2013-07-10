/**
 * \file bootload.c
 *
 * configure microcontroller bootloaders
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

#define BAUDRATE B1152000
#define MODEMDEVICE "/dev/ttyS0"

// so we can CTRL-C the program at any time
volatile bool run = true;
void sigint_handler(int s)
{
    run = false;
};

void usage()
{
    printf("ERROR: Must pass in microcontroller id number\n");
    printf("usage: bootload <uc_id>\n");
    printf("  Where <uc_id> is in the range [0-11].\n");
    printf("  See Table 1: Microcontroller Addressing.\n");
}


int main(int argc, char** argv)
{
    int fd; // serial file descriptor
    
    if (argc != 2)
    {
        usage();
        exit(1);
    }
    
    int id = atoi(argv[1]);
    
    if (id < 0 || id > 11)
    {
        usage();
        exit(1);  
    }
    
    int device = 0;
    switch (id)
    {
        case 0: device = DESTINATION_PALM; break;
        case 1: device = DESTINATION_FINGER1_PROX; break;
        case 2: device = DESTINATION_FINGER1_DIST; break;
        case 3: device = DESTINATION_FINGER2_PROX; break;
        case 4: device = DESTINATION_FINGER2_DIST; break;
        case 5: device = DESTINATION_FINGER3_PROX; break;
        case 6: device = DESTINATION_FINGER3_DIST; break;
        case 7: device = DESTINATION_MOTOR1; break;
        case 8: device = DESTINATION_MOTOR2; break;
        case 9: device = DESTINATION_MOTOR3; break;
        case 10: device = DESTINATION_MOTOR4; break;
        case 11: device = DESTINATION_TACTILE; break;
        default:
            usage();
            exit(1);  
    }
    
    //printf("setting up signal handler\n");
    struct sigaction sa;
    sa.sa_handler = sigint_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART; //required or segfault on CTRL-C
    if (sigaction(SIGINT, &sa, NULL) == -1) 
    {
        printf("sigaction error\n");
        exit(2);
    }

    //printf("trying to open\n");
    fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY | O_NDELAY); 
    if (fd < 0) 
    {
        printf("cannot open device\n");
        close(fd);
        exit(2); 
    }
    //printf("serial port open on file descriptor %d\n", fd);
    
    struct termios options;
    tcgetattr(fd, &options);
    cfmakeraw(&options);
    cfsetspeed(&options, BAUDRATE);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB; // no parity
    options.c_cflag &= ~CSTOPB; // 1 stop bit, not 2
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;     // 8 data bits
    options.c_cflag &= ~CRTSCTS; // no CTS/RTS hardware flow control
    options.c_cc[VTIME]    = 10;  // 0.5 second timeout
    options.c_cc[VMIN]     = 1;  // block for 1 char
    tcflush(fd, TCIOFLUSH); // flushes both directions
    tcsetattr(fd, TCSANOW, &options);

    unsigned char send_buff[COMMAND_PACKET_LENGTH];
    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    
    //
    // stop collection
    //
    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
    send_buff[COMMAND_OFFSET] = STOP_COLLECTION_OPCODE; // stop collection
    //setPayload(send_buff, DATA_COLLECTION_MOTORHALL_BITMASK); // no payload
    send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
    write(fd, send_buff, COMMAND_PACKET_LENGTH);

    // printf("Stop collection: ");
    // for (int i=0; i<COMMAND_PACKET_LENGTH; i++)
    //     printf("%02X ", send_buff[i]);
    // printf("\n");
    
    int rsp = readResponse(fd);
    //printf("returned: %02X\n", rsp);
    
    if (rsp != 0)
    {
        printf("ERROR, stop collection returned: 0x%02X\n", rsp);
        close(fd);
        exit(3);
    }
    
    if (!run)
    {
        printf("stopped by user\n");
        close(fd);
        exit(1);
    }

    usleep(2000);
    
    //
    // BOOTLOAD command
    //
    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
    send_buff[COMMAND_OFFSET] = BOOTLOAD_OPCODE; // stop collection
    setPayload(send_buff, device); // payload is target micro
    send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
    write(fd, send_buff, COMMAND_PACKET_LENGTH);
    
    unsigned char response_buff[100];
    int len = 0;
    rsp = readResponse(fd, response_buff, 100, len);
    
    // printf("got %d bytes\n", len);
    // for (int i=0; i<len; i++)
    //     printf("%02X ", response_buff[i]);
    // printf("\n");
    // printf("error code: %02X\n", rsp);
    
    if (rsp != 0)
    {
        printf("ERROR, bootload command returned: 0x%02X\n", rsp);
        exit(3);
    }
    
    printf("device %d in bootload mode for 10 seconds\n", id);
    
    //printf("closing\n");
    close(fd);
    //printf("exiting\n");
    exit(0);
}
