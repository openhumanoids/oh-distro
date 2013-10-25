/**
 * \file debug_poll.c
 *
 * poll for debug data with the data collection command
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
    int fd; // serial file descriptor
    
    int us_delay = 50000;
    if (argc == 2)
        us_delay = atoi(argv[1]);
    
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

    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    memset(recv_buff, 0, BUFFER);



    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
    send_buff[COMMAND_OFFSET] = STOP_COLLECTION_OPCODE; // stop collection
    //setPayload(send_buff, DATA_COLLECTION_MOTORHALL_BITMASK); // no payload
    send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
    write(fd, send_buff, COMMAND_PACKET_LENGTH);
    
    usleep(5000);
    read(fd, recv_buff, BUFFER-1);
    
    int sensor = DATA_COLLECTION_DEBUG_BITMASK;
    for (int board=0; board<12; board++)
    {
        int destination;
        printf("board %d: ", board);
        
        switch (board)
        {
            case 0: destination = DESTINATION_PALM; break;
            case 1: destination = DESTINATION_FINGER1_PROX; break;
            case 2: destination = DESTINATION_FINGER1_DIST; break;
            case 3: destination = DESTINATION_FINGER2_PROX; break;
            case 4: destination = DESTINATION_FINGER2_DIST; break;
            case 5: destination = DESTINATION_FINGER3_PROX; break;
            case 6: destination = DESTINATION_FINGER3_DIST; break;
            case 7: destination = DESTINATION_MOTOR1; break;
            case 8: destination = DESTINATION_MOTOR2; break;
            case 9: destination = DESTINATION_MOTOR3; break;
            case 10: destination = DESTINATION_MOTOR4; break;
            case 11: destination = DESTINATION_TACTILE; break;
            default:
                printf("UNKNOWN DESTINATION\n");
                exit(1);
        }
        
        memset(send_buff, 0, COMMAND_PACKET_LENGTH);
        memset(recv_buff, 0, BUFFER);
        
        send_buff[DESTINATION_HEADER_OFFSET] = destination;
        send_buff[COMMAND_OFFSET] = DATA_COLLECTION_OPCODE;
        setPayload(send_buff, sensor);
        send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH);
        write(fd, send_buff, COMMAND_PACKET_LENGTH);
                
        // printf("\nWriting: ");
        // for (int i=0; i<COMMAND_PACKET_LENGTH; i++)
        //     printf("%02X ", send_buff[i]);
        // printf("\n\n");
                
        int readlen = 0;
        int code = readResponse(fd, recv_buff, BUFFER, readlen);
        if (code != 0)
        {
            printf("ERROR: %02X", code);
        }
        else
        {
            for (int i=RESPONSE_PAYLOAD_OFFSET; i<readlen-1; i+=2)
                printf("%02X%02X ", recv_buff[i+1], recv_buff[i]);
        }
        printf("\n");
        usleep(us_delay);
    }
    
    printf("closing\n");
    close(fd);
    printf("exiting\n");
    exit(0);
}
