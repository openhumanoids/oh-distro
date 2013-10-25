/**
 * \file get_sensor.c
 *
 * get a single instance of a single sensor suitable for piping
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

void printUsage()
{
        printf("Usage:\n");
        printf("  get_sensor <sensor_type> <destination>\n");
        printf("\n");
        printf("sensor_type: 0-15:\n");
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
        printf("\n");
        printf("destination: 0-11\n");
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
        printf("\n");
        printf("NOTE: Not all combinations are valid\n");
}

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        printUsage();
        exit(1);
    }

    int sensor_type = 0;
    sensor_type = atoi(argv[1]);
    if (sensor_type < 0 || sensor_type > 15)
    {
        printf("Unknown sensor\n");
        printUsage();
        exit(1);
    }
    int sensor = 0x1 << sensor_type;

    if (sensor_type == 0)
        sensor = DATA_COLLECTION_ALL_BITMASK;
    

    int board = 0;
    board = atoi(argv[2]);
    
    int destination;
    int responding;
    switch (board)
    {
        case 0: destination = DESTINATION_PALM; responding=RESPONDING_DEVICES_PALM_BITMASK; break;
        case 1: destination = DESTINATION_FINGER1_PROX; responding=RESPONDING_DEVICES_FIRST_PROX_BITMASK; break;
        case 2: destination = DESTINATION_FINGER1_DIST; responding=RESPONDING_DEVICES_FIRST_DIST_BITMASK; break;
        case 3: destination = DESTINATION_FINGER2_PROX; responding=RESPONDING_DEVICES_SECOND_PROX_BITMASK; break;
        case 4: destination = DESTINATION_FINGER2_DIST; responding=RESPONDING_DEVICES_SECOND_DIST_BITMASK; break;
        case 5: destination = DESTINATION_FINGER3_PROX; responding=RESPONDING_DEVICES_THIRD_PROX_BITMASK; break;
        case 6: destination = DESTINATION_FINGER3_DIST; responding=RESPONDING_DEVICES_THIRD_DIST_BITMASK; break;
        case 7: destination = DESTINATION_MOTOR1; responding=RESPONDING_DEVICES_FIRST_MOTOR1_BITMASK; break;
        case 8: destination = DESTINATION_MOTOR2; responding=RESPONDING_DEVICES_FIRST_MOTOR2_BITMASK; break;
        case 9: destination = DESTINATION_MOTOR3; responding=RESPONDING_DEVICES_SECOND_MOTOR1_BITMASK; break;
        case 10: destination = DESTINATION_MOTOR4; responding=RESPONDING_DEVICES_SECOND_MOTOR2_BITMASK; break;
        case 11: destination = DESTINATION_TACTILE; responding=RESPONDING_DEVICES_TACTILE_BITMASK; break;
        default:
            printf("Unknown board id\n");
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
        //printf("cannot open device\n");
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

    unsigned char send_buff[7];
    unsigned char recv_buff[BUFFER];
    char read_buff[6];

    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    memset(recv_buff, 0, BUFFER);
    memset(read_buff, 0, 6);

    
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
    setPayload(send_buff, sensor);
    send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH);
    
    // printf("\nWriting: ");
    // for (int i=0; i<COMMAND_PACKET_LENGTH; i++)
    //     printf("%02X ", send_buff[i]);
    // printf("\n\n");

    HandSensors* data = new HandSensors();
    HandSensorsValid* valid = new HandSensorsValid();
    
    //while (run)
    //{
      
    write(fd, send_buff, COMMAND_PACKET_LENGTH);
    //usleep(2000);// this appears to be required.  because the read does not really block.
        
    //printf("\n-- %d --\n", counter++);
        
    int readlen = 0;
    int code = readResponse(fd, recv_buff, BUFFER, readlen);
    if (code != 0)
    {
        // printf("code error: %02X\n", code);
    }
    else
    {
        // printf("got:  ");
        // for (int i=0; i<readlen; i++)
        //     printf("%02X ", recv_buff[i]);
        // printf("\n");
            
        int length = computeDataSize(sensor, responding);
        if (length != readlen-5)
        {
            // printf("\n\nread length error\n");
        }
        else
        {
            data->reset();
            valid->reset();
            int parsed = parseData(&(recv_buff[RESPONSE_PAYLOAD_OFFSET]), sensor, responding, *data, *valid);
            if (parsed != length)
            {
                // printf("\n\nparsed length error\n");
                // printf("parsed %d bytes, expecting %d\n", parsed, length);
                // printf("sensor mask: %02X\n", sensor);
                // printf("responding devices: %02X\n", responding);
            }
            else
            {
                // printf("ok\n");
                data->printonlyvals(*valid);
            }
        }
    }

    //usleep(500000);
        
    //} //end while

    delete data;
    delete valid;

    //tcsetattr(fd,TCSANOW, &oldtio);
    
    //printf("closing\n");
    close(fd);
    //printf("exiting\n");
    exit(0);
}
