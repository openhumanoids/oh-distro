/**
 * \file chainmask.c
 *
 * set chain mask 
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

//#define BAUDRATE B115200
#define BAUDRATE B1152000
//#define BAUDRATE B500000

#define MODEMDEVICE "/dev/ttyS0"

#define BUFFER 500



volatile bool run = true;
void sigint_handler(int s)
{
    run = false;
};


int main(int argc, char** argv)
{
    printf("Usage:\n");
    printf("  chainmask \n");
    printf("  chainmask <mask> \n");
    printf("\n");
    
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
    printf("device open\n");
    

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
    options.c_cflag |= CS8;

    options.c_cflag &= ~CRTSCTS; // no CTS/RTS hardware flow control

    options.c_cc[VTIME]    = 3;  // 0.3 second timeout
    options.c_cc[VMIN]     = 0;  // don't block for a number of chars

    tcflush(fd, TCIOFLUSH); // flushes both directions

    tcsetattr(fd, TCSANOW, &options);

    //////////////////////////////

    unsigned char send_buff[7];
    unsigned char recv_buff[BUFFER];
    char read_buff[6];

    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    memset(recv_buff, 0, BUFFER);
    memset(read_buff, 0, 6);

    int mask = 0;
    
    if (argc > 1)
    {
        mask = atoi(argv[1]);
    }
    else
    {
        printf("\n\n");
        printf(" [255] All\n");
        printf(" [  1] Tactile palm\n");
        printf(" [  2] Finger 1\n");
        printf(" [  4] Finger 2\n");
        printf(" [  8] Finger 3\n");
        printf(" [ 16] Motors 1+2\n");
        printf(" [ 32] Motors 3+4\n");
        printf("Common options:\n");
        printf(" [ 49] No Fingers\n");
        printf(" [ 48] No Fingers, no tactile palm\n");
        printf("Mask: ");
        fgets(read_buff, 6, stdin);
        mask = atoi(read_buff);
    }
    if (mask < 0 || mask > 255)
    {
        printf("UNKNOWN MASK\n");
        exit(1);
    }
    
    //destination = DESTINATION_PALM; responding=RESPONDING_DEVICES_PALM_BITMASK; break;

    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
    send_buff[COMMAND_OFFSET] = SET_CHAIN_MASK_OPCODE;
    send_buff[PAYLOAD_OFFSET] = (unsigned char)mask; // 1 byte payload
    send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
    write(fd, send_buff, COMMAND_PACKET_LENGTH);
    
    printf("\nWriting: ");
    for (int i=0; i<COMMAND_PACKET_LENGTH; i++)
        printf("%02X ", send_buff[i]);
    printf("\n\n");

    write(fd, send_buff, COMMAND_PACKET_LENGTH);
    
    int readlen = 0;
    int code = readResponse(fd, recv_buff, BUFFER, readlen);
    if (code != 0)
    {
        printf("code error: %02X\n", code);
    }
    else
    {
        printf("ok\n");
    }

    printf("closing\n");
    close(fd);
    printf("exiting\n");
    exit(0);
}
