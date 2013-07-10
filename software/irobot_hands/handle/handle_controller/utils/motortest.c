/**
 * \file motortest.c
 *
 * test low level motor control
 *
 * Written under government funding for ARM-H project.
 *
 * \author Ben Axelrod
 * \date   March 2012
 * \copyright Copyright iRobot Corporation, 2012
 **/

#include "common.hpp"

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
//#define BAUDRATE B1500000
#define BAUDRATE B1152000
//#define BAUDRATE B500000
//#define BAUDRATE B2000000

#define MODEMDEVICE "/dev/ttyS0"

#define BUFFER 500

volatile bool run = true;
void sigint_handler(int s)
{
    printf("sigint received\n");
    run = false;
};

void usage()
{
    printf("usage:\n");
    printf("  motortest <motor> <direction> <type> <ammount>  for motors 1-4\n");
    printf("  motortest <motor> 0                             to stop any motor\n");
    printf("\n");
    printf("where:\n");
    printf("  direction 0 = stop\n");
    printf("  direction 1 = reverse\n");
    printf("  direction 2 = forward\n");
    printf("\n");
    printf("  type 0 = Position\n");
    printf("  type 1 = Current\n");
    printf("  type 2 = Voltage\n");
    printf("  type 3 = Velocity\n");
    printf("\n");
    printf("  ammount = 0 to 65535\n");
}

int main(int argc, char** argv)
{
    //usage();
    
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

    // printf("before setattr\n");
    // printf("baudrate: %u\n", BAUDRATE);
    // printf("o speed: %u\n", cfgetospeed(&options));
    // printf("i speed: %u\n", cfgetispeed(&options));

    tcsetattr(fd, TCSANOW, &options);

    // printf("after setattr\n");
    // printf("o speed: %u\n", cfgetospeed(&options));
    // printf("i speed: %u\n", cfgetispeed(&options));
    
    //////////////////////////////

    unsigned char send_buff[COMMAND_PACKET_LENGTH];
    unsigned char recv_buff[BUFFER];
    char read_buff[12];
    
    int motor = 0;
    int direction = 0;
    int type = 0;
    short amount = 0;
    
    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    memset(recv_buff, 0, BUFFER);
    memset(read_buff, 0, 12);
    
    if (argc > 1)
    {
        motor = atoi(argv[1]);
    }
    else
    {
        printf("\n\n");
        printf("Motor [1-5]:");
        fgets(read_buff, 12, stdin);

        if (read_buff[0] == '\n')
        {
            usage();
            exit(0);
        }
        
        motor = atoi(read_buff);
    }
    
    if (argc > 2)
    {
        direction = atoi(argv[2]);
    }
    else
    {
        printf("\n");
        printf("0 : Stop\n");
        printf("1 : Reverse\n");
        printf("2 : Forward\n");
        printf("Direction [0-2]:");
        fgets(read_buff, 12, stdin);
        
        if (read_buff[0] == '\n')
        {
            usage();
            exit(0);
        }
        
        direction = atoi(read_buff);
    }
    
    if (direction == 0)
    {
        type = 3;
    }
    else //if (direction > 0)
    {
            if (argc > 3)
            {
                type = atoi(argv[3]);
            }
            else
            {
                printf("\n");
                printf("0 : Position\n");
                printf("1 : Current\n");
                printf("2 : Voltage\n");
                printf("3 : Velocity\n");
                printf("Type [1-3]:");
                fgets(read_buff, 12, stdin);
                
                if (read_buff[0] == '\n')
                {
                    usage();
                    exit(0);
                }

                type = atoi(read_buff);
            }
            
            if (argc > 4)
            {
                amount = (short)atoi(argv[4]);
            }
            else
            {
                printf("\n");
                //printf("Amount [0-65535]:");
                printf("Amount [-32,768 to 32,767]:");
                fgets(read_buff, 12, stdin);
                
                if (read_buff[0] == '\n')
                {
                    usage();
                    exit(0);
                }

                amount = (short)atoi(read_buff);
            }

        // else // motor == 5
        // {
        //     if (argc > 3)
        //     {
        //         amount = (unsigned short)atoi(argv[3]);
        //     }
        //     else
        //     {
        //         printf("\n");
        //         printf("Amount [0-65535]:");
        //         fgets(read_buff, 12, stdin);
                
        //         if (read_buff[0] == '\n')
        //             exit(0);
                
        //         amount = (unsigned short)atoi(read_buff);
        //     }
        // }
    }
    
    printf("\nsending:  %d  %d  %d  %d\n", motor, direction, type, amount);
    
    switch(motor)
    {
        case 1:
            send_buff[DESTINATION_HEADER_OFFSET] = MOTOR1_CHAINADDRESS | MOTOR1_CHAININDEX;
            break;
        case 2:
            send_buff[DESTINATION_HEADER_OFFSET] = MOTOR1_CHAINADDRESS | MOTOR2_CHAININDEX;
            break;
        case 3:
            send_buff[DESTINATION_HEADER_OFFSET] = MOTOR2_CHAINADDRESS | MOTOR3_CHAININDEX;
            break;
        case 4:
            send_buff[DESTINATION_HEADER_OFFSET] = MOTOR2_CHAINADDRESS | MOTOR4_CHAININDEX;
            break;
        case 5:
            send_buff[DESTINATION_HEADER_OFFSET] = PALM_CHAINADDRESS;
            break;
            //default:
            //continue;
    }
    
    if (motor == 5)
        send_buff[COMMAND_OFFSET] = FINGER_COMMAND_OPCODE;
    else
        send_buff[COMMAND_OFFSET] = MOTOR_COMMAND_OPCODE;
    
    switch (direction)
    {
        case 0:
            send_buff[COMMAND_OFFSET] |= MOTOR_COMMAND_STOP;
            break;
        case 1:
            send_buff[COMMAND_OFFSET] |= MOTOR_COMMAND_REVERSE;
            break;
        case 2:
            send_buff[COMMAND_OFFSET] |= MOTOR_COMMAND_FORWARD;
            break;
            //default:
            //continue;
    }
    
    // if (motor != 5)
    // {
        switch (type)
        {
            case 0:
                send_buff[COMMAND_OFFSET] |= MOTOR_COMMAND_POSITION;
                break;
                
            case 1:
                send_buff[COMMAND_OFFSET] |= MOTOR_COMMAND_CURRENT;
                break;
                
            case 2:
                send_buff[COMMAND_OFFSET] |= MOTOR_COMMAND_VOLTAGE;
                break;
                
            case 3:
                send_buff[COMMAND_OFFSET] |= MOTOR_COMMAND_VELOCITY;
                break;
                
                //default:
                //continue;
        }
    // }
    
    setPayload(send_buff, amount);
    send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH);
        
    printf("sending: ");
    printf("         ");
    for (int i=0; i<COMMAND_PACKET_LENGTH; i++)
        printf("%02X ", send_buff[i]);
    printf("\n");
    
    write(fd, send_buff, COMMAND_PACKET_LENGTH);
    
    ////////////////////////
    
    //int res = read(fd, recv_buff, BUFFER-1);
    int len;
    int res = readResponse(fd, recv_buff, BUFFER-1, len);
    if (res != 0)
    {
        printf("\n\n\nWARNING: read returned %d\n", res);
        printf("raw buffer: ");
        for (int i=0; i<len; i++)
            printf("%02X ", recv_buff[i]);
        printf("\n");
    }
    else
    {
        printf("ok\n");
    }

    // else if (res > 0)
    // {
    //     printf("read returned %d bytes\n", res);
    //     printf("raw buffer: ");
    //     for (int i=0; i<res; i++)
    //         printf("%02X ", recv_buff[i]);
    //     printf("\n");
    // }

    //printf("closing\n");
    close(fd);
    //printf("exiting\n");
    exit(0);
}
