/**
 * \file sensor_spin.c
 *
 * test automatic sensor collection
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


int main(int argc, char** argv)
{
    // usage: sensor_spin <hz>

    int fd; // serial file descriptor
    int hz = 25;
    if (argc == 2)
        hz = atoi(argv[1]);
        
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
    
    int sensor_mask = DATA_COLLECTION_ALL_BITMASK;
    int resp;

    //
    // stop collection
    //
    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
    send_buff[COMMAND_OFFSET] = STOP_COLLECTION_OPCODE; // stop collection
    //setPayload(send_buff, 0); // no payload
    send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
    write(fd, send_buff, COMMAND_PACKET_LENGTH);

    printf("Stop collection: ");
    for (int i=0; i<COMMAND_PACKET_LENGTH; i++)
        printf("%02X ", send_buff[i]);
    printf("\n");
    
    usleep(10000);// this appears to be required.  because the read does not really block.
    while (read(fd, recv_buff, BUFFER-1) >= 0); //flush
    //tcflush(fd, TCIFLUSH);
    
    //   
    // set sample period
    //
    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
    send_buff[COMMAND_OFFSET] = SET_SAMPLE_PERIOD_OPCODE; // set sample period
    setPayload(send_buff, (unsigned int)(1000000.0/(float)hz));
    send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
    write(fd, send_buff, COMMAND_PACKET_LENGTH);

    printf("Setting sample period:  ");
    for (int i=0; i<COMMAND_PACKET_LENGTH; i++)
        printf("%02X ", send_buff[i]);
    printf("\n");

    
    //usleep(2000);// this appears to be required.  because the read does not really block.
    int len;
    resp = readResponse(fd, len);
    if (resp != 0)
    {
        printf("ERROR, error code: %02X\n", resp);
        printf("raw buffer: ");
        for (int i=0; i<20; i++)
            printf("%02X ", recv_buff[i]);
        printf("\n");
    }
    

    //
    // set sample args
    //
    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
    send_buff[COMMAND_OFFSET] = SET_SAMPLE_ARGS_OPCODE; // set sample args
    setPayload(send_buff, sensor_mask);
    send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
    write(fd, send_buff, COMMAND_PACKET_LENGTH);

    printf("Setting sample args:  ");
    for (int i=0; i<COMMAND_PACKET_LENGTH; i++)
        printf("%02X ", send_buff[i]);
    printf("\n");

    //usleep(2000);// this appears to be required.  because the read does not really block.
    
    resp = readResponse(fd, len);
    if (resp != 0)
    {
        printf("ERROR, error code: %02X\n", resp);
        printf("raw buffer:  ");
        for (int i=0; i<20; i++)
            printf("%02X ", recv_buff[i]);
        printf("\n");
    }

    //
    // start collection
    //
    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
    send_buff[COMMAND_OFFSET] = START_COLLECTION_OPCODE; // start collection
    //setPayload(send_buff, 0); // no payload
    send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
    write(fd, send_buff, COMMAND_PACKET_LENGTH);

    printf("Start collection: ");
    for (int i=0; i<COMMAND_PACKET_LENGTH; i++)
        printf("%02X ", send_buff[i]);
    printf("\n");
    
    //usleep(2000);// this appears to be required.  because the read does not really block.
    
    resp = readResponse(fd, len);
    if (resp != 0)
    {
        printf("ERROR, error code: %02X\n", resp);
        printf("raw buffer: ");
        for (int i=0; i<20; i++)
            printf("%02X ", recv_buff[i]);
        printf("\n");
    }

    // timeval tv_now;
    // timeval tv_last_motor;
    // timeval tv_last_sensor;
    // timeval tv_last_loop;
    // gettimeofday(&tv_now, NULL);
    // gettimeofday(&tv_last_motor, NULL);
    // gettimeofday(&tv_last_sensor, NULL);
    // gettimeofday(&tv_last_loop, NULL);
    // int motor_time_buff[100];
    // int sensor_time_buff[100];
    // int loop_time_buff[100];
    // int motor_time_index = 1;
    // int sensor_time_index = 1;
    // int loop_time_index = 1;
    // bool motor_time_disp = false;
    // bool sensor_time_disp = false;
    // int errorcount = 0;
    
    HandSensors* data = new HandSensors();
    HandSensors* tmpdata = new HandSensors();
    HandSensorsValid* valid = new HandSensorsValid();
    int count = 0;

    while (run)
    {
        printf("-------------------------------------------- %d\n", count++);
        
        tmpdata->reset();
        valid->reset();
        bool sensorMsg;
        int code = parseResponse(fd, sensor_mask, sensorMsg, *tmpdata, *valid, recv_buff, BUFFER);
        if (code == 0)
        {
            if (sensorMsg)
            {
                data->update(*tmpdata, *valid);
            }
        }
        else
        {
            printf("ERROR CODE: %d\n", code);
        }

        data->print();
    }
    
    delete data;
    delete tmpdata;
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
