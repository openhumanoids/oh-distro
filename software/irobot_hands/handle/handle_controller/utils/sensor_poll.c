/**
 * \file sensor_poll.c
 *
 * poll for sensor data with the data collection command
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
    printf("  sensor_poll \n");
    printf("  sensor_poll <sensor_type> \n");
    printf("  sensor_poll <sensor_type> <destination>\n");
    printf("\n");
    printf("sensor_type: 0-16\n");
    printf("destination: 0-11\n");
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

    int sensor_type = 0;
    int board = 0;
    
    if (argc > 1)
    {
        sensor_type = atoi(argv[1]);
    }
    else
    {
        printf("\n\n");
        printf(" [0] All\n");
        printf(" [1] Tactile Temp\n");
        printf(" [2] Debug\n");
        printf(" [3] Motor Hall Encoder\n");
        printf(" [4] Motor Winding Temp\n");
        printf(" [5] Air Temp\n");
        printf(" [6] Supply Voltage\n");
        printf(" [7] Motor Velocity\n");
        printf(" [8] Motor Housing Temp\n");
        printf(" [9] Motor Current\n");
        printf("[10] Tactile Array\n");
        printf("[11] Finger Rotation\n");
        printf("[12] Proximal Joint Angle\n");
        printf("[13] Distal Joint Angle\n");
        printf("[14] Cable Tension\n");
        printf("[15] Dynamic Sensors\n");
        printf("[16] Acceleration\n");
        printf("Sensor: ");
        fgets(read_buff, 6, stdin);
        sensor_type = atoi(read_buff);
    }
    if (sensor_type < 0 || sensor_type > 16)
    {
        printf("UNKNOWN SENSOR TYPE\n");
        exit(1);
    }
    
    int sensor = 0x1 << (sensor_type-1);

    if (sensor_type == 0)
        sensor = DATA_COLLECTION_ALL_BITMASK;
    //if (sensor_type == 16)
    //    sensor = DATA_COLLECTION_ACCELERATION_BITMASK | DATA_COLLECTION_TACTILE_BITMASK;

    if (argc > 2)
    {
        board = atoi(argv[2]);
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
        board = atoi(read_buff);
    }

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
            printf("UNKNOWN DESTINATION\n");
            exit(1);
    }

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
    
    printf("\nWriting: ");
    for (int i=0; i<COMMAND_PACKET_LENGTH; i++)
        printf("%02X ", send_buff[i]);
    printf("\n\n");

    HandSensors* data = new HandSensors();
    HandSensorsValid* valid = new HandSensorsValid();
    int counter = 0;
    
    while (run)
    {
        write(fd, send_buff, COMMAND_PACKET_LENGTH);
        //usleep(2000);// this appears to be required.  because the read does not really block.
        
        printf("\n-- %d --\n", counter++);
        
        int readlen = 0;
        int code = readResponse(fd, recv_buff, BUFFER, readlen);
        if (code != 0)
        {
            printf("code error: %02X\n", code);
        }
        else
        {
            printf("got:  ");
            for (int i=0; i<readlen; i++)
                printf("%02X ", recv_buff[i]);
            printf("\n");
            
            int length = computeDataSize(sensor, responding);
            if (length != readlen-5)
            {
                printf("\n\nread length error\n");
            }
            else
            {
                data->reset();
                valid->reset();
                int parsed = parseData(&(recv_buff[RESPONSE_PAYLOAD_OFFSET]), sensor, responding, *data, *valid);
                if (parsed != length)
                {
                    printf("\n\nparsed length error\n");
                    printf("parsed %d bytes, expecting %d\n", parsed, length);
                    printf("sensor mask: %02X\n", sensor);
                    printf("responding devices: %02X\n", responding);
                }
                else
                {
                    printf("ok\n");
                    data->printonly(*valid);
                }
            }
        }

        usleep(500000);
        
    } //end while

    delete data;
    delete valid;

    //tcsetattr(fd,TCSANOW, &oldtio);
    
    printf("closing\n");
    close(fd);
    printf("exiting\n");
    exit(0);
}
