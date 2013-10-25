/**
 * \file parametertest.c
 *
 * get and set motor parameters
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
//#include <ctype.h>

#include "common.hpp"
#include "../../handle_lib/include/handle_lib/handleTypes.hpp"
#include "../../handle_lib/include/handle_lib/packetParser.hpp"

//#define BAUDRATE B115200
#define BAUDRATE B1152000

#define MODEMDEVICE "/dev/ttyS0"

#define BUFFER 50

volatile bool run = true;
void sigint_handler(int s)
{
    run = false;
};


void printUsage()
{
    printf("usage:\n");
    printf("  parametertest set <parameter_number> <value> <device>\n");
    printf("  parametertest get <parameter_number> <device>\n");
    printf("where:\n");
    printf("  parameter_number = 0-31 or 'all'\n");
    printf("  value = 4 bytes\n");
    printf("  device = sting name ('f2d') or number (0-11)\n"); 
    printf("\n");
    printf("Parameters:\n");
    printf(" [0] K_pTorque\n");
    printf(" [1] K_iTorque\n");
    printf(" [2] K_dTorque\n");
    printf(" [3] K_pVel\n");
    printf(" [4] K_iVel\n");
    printf(" [5] K_dVel\n");
    printf(" [6] K_pPow\n");
    printf(" [7] K_iPow\n");
    printf(" [8] K_dPow\n");
    printf(" [9] R_Tw0\n");
    printf("[10] R_th1\n");
    printf("[11] T_L+\n");
    printf("[12] T_L-\n");
    printf("[13] Tau_W\n");
    printf("[14] T_MAX\n");
    printf("[15] alpha_Cu\n");
    printf("[16] t_off\n");
    printf("[17] T_TARGET\n");
    printf("[18] RPM_MAX\n");
    printf("[19] RSV\n");
    printf("[20] V_MAX\n");
    printf("[21] K_pPos\n");
    printf("[22] PosDeadband\n");
    printf("[...] \n");
    printf("[28] Encoder Offset\n");
    printf("[29] Firmware Version\n");
    printf("[30] Device ID\n");
    printf("[31] LED\n");
}

int main(int argc, char** argv)
{
    if (argc != 4 && argc != 5)
    {
        printUsage();
        exit(1);
    }
    
    int fd; // serial file descriptor

    //printf("setting up signal handler\n");
    struct sigaction sa;
    sa.sa_handler = sigint_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART; //required or segfault on CTRL-C
    if (sigaction(SIGINT, &sa, NULL) == -1) 
    {
        perror("sigaction");
        exit(1);
    }

    //printf("trying to open\n");
    fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY | O_NDELAY); 
    if (fd <0 ) 
    {
        printf("cannot open device\n");
        perror(MODEMDEVICE); 
        exit(-1); 
    }
    //printf("device open\n");
    

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

    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
    send_buff[COMMAND_OFFSET] = STOP_COLLECTION_OPCODE; // stop collection
    //setPayload(send_buff, DATA_COLLECTION_MOTORHALL_BITMASK); // no payload
    send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
    write(fd, send_buff, COMMAND_PACKET_LENGTH);
    int rsp = readResponse(fd);
    if (rsp != 0)
        printf("WARNING: stop collection returned: %02X\n", rsp);
    
    
    // get
    if (tolower(argv[1][0]) == 'g')
    {
        int from_p = 0;
        int to_p = 0;
        
        // get 'all'
        if (tolower(argv[2][0]) == 'a')
        {
            from_p = 0;
            to_p = 32;
        }
        // get single parameter
        else
        {
            int parameter = atoi(argv[2]);
            
            if (parameter < 0 || parameter > 31)
            {
                printf("ERROR: parameter out of range\n");
                printUsage();
                close(fd);
                exit(1);
            }
            
            from_p = parameter;
            to_p = parameter+1;
        }
        
        // destination and responding device
        int destination = 0;
        int responding = 0;
        if (!getDestination(argv[3], destination, responding))
        {
            printf("ERROR: destination unknown\n");
            printUsage();
            close(fd);
            exit(1);
        }
        
        // get it
        for (int i=from_p; i<to_p; i++)
        {
            memset(send_buff, 0, COMMAND_PACKET_LENGTH);
            memset(recv_buff, 0, BUFFER);
            
            send_buff[DESTINATION_HEADER_OFFSET] = destination;
            send_buff[COMMAND_OFFSET] = MOTOR_PARAMETER_RE_L_OPCODE | i;
            //setPayload(send_buff, sensor);
            send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
            write(fd, send_buff, COMMAND_PACKET_LENGTH);
            
            // printf("sending: ");
            // for (int j=0; j<COMMAND_PACKET_LENGTH; j++)
            //     printf("%02X ", send_buff[j]);
            
            printf("Param: %d  =  ", i);
            
            int readlen = 0;
            int code = readResponse(fd, recv_buff, BUFFER, readlen);

            // printf("\n  Got: ");
            // for (int j=0; j<readlen; j++)
            //     printf("%02X ", recv_buff[j]);
            // printf("\n");
            
            if (code != 0)
            {
                printf("ERROR %02X\n", code);
            }
            else
            {
                if (i <= 22)
                {
                    float val = parseParameterf(&(recv_buff[RESPONSE_PAYLOAD_OFFSET]));
                    printf("%f\n", val);
                }
                else
                {
                    int val = parseParameter(&(recv_buff[RESPONSE_PAYLOAD_OFFSET]));
                    printf("%d\n", val);
                }
            }
        }
    }
    // set
    else if (tolower(argv[1][0]) == 's')
    {
        // set single parameter
        int parameter = atoi(argv[2]);
        
        if (parameter < 0 || parameter > 31)
        {
            printf("ERROR: parameter out of range\n");
            printUsage();
            close(fd);
            exit(1);
        }
        
        // destination and responding device
        int destination = 0;
        int responding = 0;
        if (!getDestination(argv[4], destination, responding))
        {
            printf("ERROR: destination unknown\n");
            printUsage();
            close(fd);
            exit(1);
        }
        
        // set it
        memset(send_buff, 0, COMMAND_PACKET_LENGTH);
        memset(recv_buff, 0, BUFFER);
        send_buff[DESTINATION_HEADER_OFFSET] = destination;
        send_buff[COMMAND_OFFSET] = MOTOR_PARAMETER_WR_L_OPCODE | parameter;
        
        if (parameter <= 22)
        {
            float value = atof(argv[3]);
            setPayloadf(send_buff, value);
        }
        else
        {
            int value = atoi(argv[3]);
            setPayload(send_buff, value);
        }
        
        send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
        write(fd, send_buff, COMMAND_PACKET_LENGTH);
        
        // printf("sending: ");
        // for (int i=0; i<COMMAND_PACKET_LENGTH; i++)
        //     printf("%02X ", send_buff[i]);

        printf("Param: %d  Set ", parameter);
        int readlen = 0;
        int code = readResponse(fd, recv_buff, BUFFER, readlen);

        // printf("\n  Got: ");
        // for (int i=0; i<readlen; i++)
        //     printf("%02X ", recv_buff[i]);
        // printf("\n");
        
        if (code != 0)
        {
            printf("ERROR %02X\n", code);
        }
        else
        {
            printf("ok\n");
        }
    }
    // error
    else
    {
        printf("ERROR: unknown action\n");
        printUsage();
        close(fd);
        exit(1);        
    }
    
    //printf("closing\n");
    close(fd);
    //printf("exiting\n");
    exit(0);
}
