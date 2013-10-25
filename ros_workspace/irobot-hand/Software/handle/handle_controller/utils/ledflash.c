/**
 * \file ledflash.c
 *
 * flash traffic cop led as fast as possible for debugging
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
#define BUFFER 50

volatile bool run = true;
void sigint_handler(int s)
{
    run = false;
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

    unsigned char send_buff[COMMAND_PACKET_LENGTH];
    unsigned char recv_buff[BUFFER];

    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    memset(recv_buff, 0, BUFFER);
    
    int times = 1000;
    if (argc == 2)
        times = atoi(argv[1]);
    
    for (int i=0; i<times; i++)
    {
        memset(send_buff, 0, COMMAND_PACKET_LENGTH);
        memset(recv_buff, 0, BUFFER);
        
        send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM;
        send_buff[COMMAND_OFFSET] = MOTOR_PARAMETER_WR_L_OPCODE | PARAMETER_LED;
        setPayload(send_buff, i%2);
        send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
        write(fd, send_buff, COMMAND_PACKET_LENGTH);
        
        printf("\\");
        
        int readlen = 0;
        int code = readResponse(fd, recv_buff, BUFFER, readlen);
        if (code == 0)
            printf("/");
        else
            printf("_");

        if (!run)
            break;
    }    

    printf("\n");

    printf("closing\n");
    close(fd);
    printf("exiting\n");
    exit(0);
}
