/**
 * \file test_palm_tactile.c
 *
 * test palm tactile sensors
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
#include <math.h>

#include "../../handle_lib/include/handle_lib/handleTypes.hpp"
#include "../../handle_lib/include/handle_lib/packetParser.hpp"

#define BAUDRATE B1152000
#define MODEMDEVICE "/dev/ttyS0"
#define BUFFER 1000

volatile bool run = true;
void sigint_handler(int s)
{
    run = false;
};


bool tested_good[48];
int min_seen[48];
int max_seen[48];

char isGood(int i)
{
    if (tested_good[i])
        return 'O';
    return 'X';
}

void updateSensor(int i, int val, int threshold)
{
    if (val < min_seen[i])
        min_seen[i] = val;
    if (val > max_seen[i])
        max_seen[i] = val;
    if (max_seen[i]-min_seen[i] > threshold)
        tested_good[i] = true;
}


int main(int argc, char** argv)
{
    // usage: test_palm_tactile [threshold]
    // use 20 for non-infiltrated
    // use 100 for infiltrated
    
    int threshold = 2;
    if (argc > 1)
    {
        threshold = atoi(argv[1]);
    }
    
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

    options.c_cc[VTIME]    = 10;  // 0.5 second timeout
    options.c_cc[VMIN]     = 1;  // block for 1 char

    tcflush(fd, TCIOFLUSH); // flushes both directions

    tcsetattr(fd, TCSANOW, &options);

    //////////////////////////////

    unsigned char send_buff[COMMAND_PACKET_LENGTH];
    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    
    unsigned char* recv_buff = new unsigned char[BUFFER];
    memset(recv_buff, 0, BUFFER);
    
    int sensor_mask = DATA_COLLECTION_TACTILE_BITMASK;
    int destination = DESTINATION_TACTILE; 
    int responding = RESPONDING_DEVICES_TACTILE_BITMASK;

    // stop collection
    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
    send_buff[COMMAND_OFFSET] = STOP_COLLECTION_OPCODE; // stop collection
    //setPayload(send_buff, DATA_COLLECTION_MOTORHALL_BITMASK); // no payload
    send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
    write(fd, send_buff, COMMAND_PACKET_LENGTH);
    
    usleep(4000);
    read(fd, recv_buff, BUFFER-1);
    
    // create poll message
    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    send_buff[DESTINATION_HEADER_OFFSET] = destination;
    send_buff[COMMAND_OFFSET] = DATA_COLLECTION_OPCODE;
    setPayload(send_buff, sensor_mask);
    send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH);

    HandSensors* data = new HandSensors();
    HandSensorsValid* valid = new HandSensorsValid();
    int counter = 0;
    
    for (int i=0; i<48; i++)
    {
        tested_good[i] = false;
        min_seen[i] = 99999;
        max_seen[i] = -99999;
    }
    
    while (run)
    {
        write(fd, send_buff, COMMAND_PACKET_LENGTH);

        printf("\n-- %d --\n", counter++);
 
        int readlen = 0;
        int code = readResponse(fd, recv_buff, BUFFER, readlen);
        if (code != 0)
        {
            printf("code error: %02X\n", code);
        }
        else
        {
            // printf("got:  ");
            // for (int i=0; i<readlen; i++)
            //     printf("%02X ", recv_buff[i]);
            // printf("\n");
            
            int length = computeDataSize(sensor_mask, responding);
            if (length != readlen-5)
            {
                printf("\n\nread length error\n");
            }
            else
            {
                data->reset();
                valid->reset();
                int parsed = parseData(&(recv_buff[RESPONSE_PAYLOAD_OFFSET]), sensor_mask, responding, *data, *valid);
                if (parsed != length)
                {
                    printf("\n\nparsed length error\n");
                    printf("parsed %d bytes, expecting %d\n", parsed, length);
                    printf("sensor mask: %02X\n", sensor_mask);
                    printf("responding devices: %02X\n", responding);
                }
                else
                {
                    //printf("ok\n");
                    //data->printonly(*valid);
                    
                    if (valid->palmTactile)
                    {
                        for (int i=0; i<48; i++)
                            updateSensor(i, data->palmTactile[i], threshold);
                        
                        for (int i=0; i<16; i++)
                        {
                            for (int j=0; j<3; j++)
                            {
                                int id = j+3*i;
                                //printf("[%d] min:%d max:%d val:%d   ",id+1, min_seen[id], max_seen[id], data->palmTactile[id]);
                                printf("[%02d] d:%03d v:%04d   ",id+1, max_seen[id]-min_seen[id], data->palmTactile[id]);
                            }
                            printf("\n");
                        }
                        printf("\n");
                        printf("            %c\n", isGood(23));
                        printf("            %c\n", isGood(22));
                        printf("            %c\n", isGood(21));
                        printf("            %c\n", isGood(20));
                        printf("           %c %c\n", isGood(24), isGood(19));
                        printf("          %c %c %c\n", isGood(26), isGood(25), isGood(18));
                        printf("%c %c         %c         %c %c\n", isGood(47), isGood(46), isGood(27), isGood(1), isGood(0));
                        printf("%c   %c  %c  %c %c %c  %c  %c   %c\n", isGood(45), isGood(35), isGood(34), isGood(28), isGood(29), isGood(17), isGood(13), isGood(12), isGood(2));
                        printf("%c         %c   %c         %c\n", isGood(44), isGood(30), isGood(16), isGood(3));
                        printf("%c   %c  %c  %c %c %c  %c  %c   %c\n", isGood(43), isGood(36), isGood(33), isGood(32), isGood(31), isGood(15), isGood(14), isGood(11), isGood(4));
                        printf("%c   %c               %c   %c\n", isGood(42), isGood(37), isGood(10), isGood(5));
                        printf("%c                       %c\n", isGood(41), isGood(6));
                        printf("%c %c %c               %c %c %c\n", isGood(40), isGood(39), isGood(38), isGood(9), isGood(8), isGood(7));
                    }
                }
            }
        }
        
        usleep(100000);
        
    }

    delete data;
    delete valid;
    delete[] recv_buff;
    
    //
    // stop collection
    //
    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
    send_buff[COMMAND_OFFSET] = STOP_COLLECTION_OPCODE; // stop collection
    //setPayload(send_buff, 0); // no payload
    send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
    write(fd, send_buff, COMMAND_PACKET_LENGTH);
    usleep(2000);

    printf("closing\n");
    close(fd);
    printf("exiting\n");
    exit(0);
}
