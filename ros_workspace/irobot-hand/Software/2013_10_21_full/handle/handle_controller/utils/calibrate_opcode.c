/**
 * \file calibrate_opcode.c
 *
 * send the sensor calibration comand to a specific device on a specific sensor
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

#include "common.hpp"
#include "../../handle_lib/include/handle_lib/handleTypes.hpp"
#include "../../handle_lib/include/handle_lib/packetParser.hpp"

#define BAUDRATE B1152000
#define MODEMDEVICE "/dev/ttyS0"
#define BUFFER 500

volatile bool run = true;
void sigint_handler(int s)
{
    run = false;
};

void usage()
{
    printf("Usage:\n");
    printf("  calibrate_opcode \n");
    printf("  calibrate_opcode <sensor_type> \n");
    printf("  calibrate_opcode <sensor_type> <destination>\n");
    printf("\n");
    printf("sensor_type: 0-16\n");
    printf("destination: 0-11 or f2p or all\n");
    printf("\n");
}

int main(int argc, char** argv)
{
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
        close(fd);
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

    unsigned char send_buff[7];
    unsigned char recv_buff[BUFFER];
    char read_buff[6];

    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    memset(recv_buff, 0, BUFFER);
    memset(read_buff, 0, 6);

    int sensor_type = 0;
    if (argc == 1)
        usage();
    
    if (argc > 1)
    {
        sensor_type = atoi(argv[1]);
    }
    else
    {
        printf("\n\n");
        printf(" [0] All\n");
        printf(" [2] Motor Hall Encoder\n");
        printf(" [3] Motor Winding Temp\n");
        printf(" [4] Air Temp\n");
        printf(" [5] Supply Voltage\n");
        printf(" [6] Motor Velocity\n");
        printf(" [7] Motor Housing Temp\n");
        printf(" [8] Motor Current\n");
        printf(" [9] Tactile Array\n");
        printf("[10] Finger Rotation\n");
        printf("[11] Proximal Joint Angle\n");
        printf("[12] Distal Joint Angle\n");
        printf("[13] Cable Tension\n");
        printf("[14] Dynamic Sensors\n");
        printf("[15] Acceleration\n");
        printf("Sensor: ");
        fgets(read_buff, 6, stdin);
        sensor_type = atoi(read_buff);
    }
    if (sensor_type < 0 || sensor_type > 15)
    {
        printf("UNKNOWN SENSOR TYPE\n");
        close(fd);
        exit(1);
    }
    
    int sensor = 0x1 << sensor_type;

    if (sensor_type == 0)
        sensor = DATA_COLLECTION_ALL_BITMASK;
    
    
    int destination = 0;
    int responding = 0;
    if (argc > 2)
    {
        if (!getDestination(argv[2], destination, responding))
        {
            usage();
            close(fd);
            exit(1);
        }
    }
    else
    {
        printf("\n\n");
        printf(" [0] Palm\n");
        printf(" [1] Finger 1 Proximal\n");
        printf(" [2] Finger 1 Distal\n");
        printf(" [3] Finger 2 Proximal\n");
        printf(" [4] Finger 2 Distal\n");
        printf(" [5] Finger 3 (Thumb) Proximal\n");
        printf(" [6] Finger 3 (Thumb) Distal\n");
        printf(" [7] Motor 1\n");
        printf(" [8] Motor 2\n");
        printf(" [9] Motor 3\n");
        printf("[10] Motor 4\n");
        printf("[11] Tactile\n");
        printf("Board: ");
        fgets(read_buff, 6, stdin);
        if (!getDestination(read_buff, destination, responding))
        {
            usage();
            close(fd);
            exit(1);
        }
    }
    
    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
    send_buff[COMMAND_OFFSET] = STOP_COLLECTION_OPCODE; // stop collection
    //setPayload(send_buff, DATA_COLLECTION_MOTORHALL_BITMASK); // no payload
    send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
    write(fd, send_buff, COMMAND_PACKET_LENGTH);
    
    int rsp = readResponse(fd);
    //printf("returned: %02X\n", rsp);
    
    if (rsp != 0)
    {
        printf("ERROR, stop collection returned: 0x%02X\n", rsp);
        close(fd);
        exit(3);
    }
    

    // all devices
    if (destination == -1)
    {
        for (int i=0; i<12; i++)
        {
            getDestination(i, destination, responding);
            
            memset(send_buff, 0, COMMAND_PACKET_LENGTH);
            send_buff[DESTINATION_HEADER_OFFSET] = destination;
            send_buff[COMMAND_OFFSET] = CALIBRATION_OPCODE;
            setPayload(send_buff, sensor);
            send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH);
            write(fd, send_buff, COMMAND_PACKET_LENGTH);
        
            int readlen = 0;
            int code = readResponse(fd, recv_buff, BUFFER, readlen);
            printf("board: %d  code: 0x%02X\n", i, code);
        }
    }
    // single device
    else
    {
        memset(send_buff, 0, COMMAND_PACKET_LENGTH);
        send_buff[DESTINATION_HEADER_OFFSET] = destination;
        send_buff[COMMAND_OFFSET] = CALIBRATION_OPCODE;
        setPayload(send_buff, sensor);
        send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH);
        write(fd, send_buff, COMMAND_PACKET_LENGTH);
        
        int readlen = 0;
        int code = readResponse(fd, recv_buff, BUFFER, readlen);
        
        printf("code: 0x%02X\n", code);
    }
    
    //printf("closing\n");
    close(fd);
    //printf("exiting\n");
    exit(0);
}
