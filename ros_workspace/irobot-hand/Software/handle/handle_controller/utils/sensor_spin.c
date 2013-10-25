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
#include <getopt.h> //option struct
#include <ctype.h> //isprint

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

#define DEFAULT_HZ 25
#define DEFAULT_SENSOR_MASK DATA_COLLECTION_ALL_BITMASK

void printUsage(FILE* stream=stdout)
{
    fprintf(stream, "usage: sensor_spin [options]\n");
    fprintf(stream, "options:\n");
    fprintf(stream, "  -r, --rate, --hz <hz> Set sensor rate in hz.  Default: %d\n", DEFAULT_HZ);
    fprintf(stream, "  -d, --dumb            Hand has \"dumb fingers\", don't ask for finger data.\n");
    fprintf(stream, "                        Default: false\n");
    fprintf(stream, "      --dumb_finger1    Only finger 1 is \"dumb\".  Default: false\n");
    fprintf(stream, "      --dumb_finger2    Only finger 2 is \"dumb\".  Default: false\n");
    fprintf(stream, "      --dumb_finger3    Only finger 3 is \"dumb\".  Default: false\n");
    fprintf(stream, "      --dumb_tactile    Only tactile palm is \"dumb\".  Default: false\n");
    fprintf(stream, "  -s, --sensors <mask>  Specify exactly which sensors you want to receive.\n");
    fprintf(stream, "                        This should be an ORed combination of DATA_COLLECTION_*\n");
    fprintf(stream, "                        bitfields in handleTypes.hpp.  Default: \"0x%04X\"\n", DEFAULT_SENSOR_MASK);
    fprintf(stream, "      --no_tact         Do not return any tactile sensor data.\n");
    fprintf(stream, "                        Applied after the \"--sensors\" flag.  Default: false\n");
    fprintf(stream, "      --no_tact_temp    Do not return any tactile temperature sensor data.\n");
    fprintf(stream, "                        Applied after the \"--sensors\" flag.  Default: false\n");
    fprintf(stream, "      --no_flex         Do not return any flexure sensor data.\n");
    fprintf(stream, "                        Applied after the \"--sensors\" flag.  Default: false\n");
    fprintf(stream, "      --no_accel        Do not return any accelerometer sensor data.\n");
    fprintf(stream, "                        Applied after the \"--sensors\" flag.  Default: false\n");
    fprintf(stream, "  -h, --help            Display this help message and exit.  Default: false\n");
}

// command line params
int hz = DEFAULT_HZ;
bool dumb_fingers = false;
bool dumb_finger1 = false;
bool dumb_finger2 = false;
bool dumb_finger3 = false;
bool dumb_tactile = false;
bool no_tact = false;
bool no_tact_temp = false;
bool no_flex = false;
bool no_accel = false;
int sensor_mask = DEFAULT_SENSOR_MASK;

int main(int argc, char** argv)
{
    // usage: sensor_spin <hz>

    int fd; // serial file descriptor
    
    //int hz = 25;
    //if (argc == 2)
    //    hz = atoi(argv[1]);
    
    option long_options[] = {
        {"rate",  required_argument, 0, 'r' },
        {"hz",    required_argument, 0, 'r' },
        {"dumb",        no_argument, 0, 'd' },
        {"dumb_finger1", no_argument, 0, 1001 },
        {"dumb_finger2", no_argument, 0, 1002 },
        {"dumb_finger3", no_argument, 0, 1003 },
        {"dumb_tactile", no_argument, 0, 1004 },
        {"sensors", required_argument, 0, 's' },
        {"no_tact",      no_argument, 0, 2001 },
        {"no_tact_temp", no_argument, 0, 2002 },
        {"no_flex",      no_argument, 0, 2003 },
        {"no_accel",     no_argument, 0, 2004 },
        {"help",        no_argument, 0, 'h' },
        {0,             0,           0,  0  }
    };

    //
    // parse command line options
    //
    int c;
    while ((c = getopt_long(argc, argv, "r:s:hd", long_options, NULL)) != -1)
    {
        switch (c)
        {
            case 'r':
                hz = atoi(optarg);
                if (hz <= 0)
                {
                    fprintf(stderr, "WARNING: value '%d' is outside of range for option 'r'\n", hz);
                    fprintf(stderr, "\n");
                    printUsage(stderr);
                    exit(1);
                }
                break;
            case 's':
                sensor_mask = strtol(optarg, NULL, 0);
                break;
            case 2001:
                no_tact = true;
                break;
            case 2002:
                no_tact_temp = true;
                break;
            case 2003:
                no_flex = true;
                break;
            case 2004:
                no_accel = true;
                break;
            case 'd':
                dumb_fingers = true;
                break;
            case 1001:
                dumb_finger1 = true;
                break;
            case 1002:
                dumb_finger2 = true;
                break;
            case 1003:
                dumb_finger3 = true;
                break;
            case 1004:
                dumb_tactile = true;
                break;
            case 'h':
                printUsage();
                exit(0);
                break;
            case '?':
                if (optopt == 'r' || optopt == 'p' || optopt == 'o' || optopt == 's')
                    fprintf(stderr, "Option -%c requires an argument.\n", optopt);
                else if (isprint (optopt))
                    fprintf(stderr, "Unknown option `-%c'.\n", optopt);
                else
                    fprintf(stderr, "Unknown option character `\\x%x'.\n", optopt);
                fprintf(stderr, "\n");
                printUsage(stderr);
                exit(1);
            default:
                printUsage(stderr);
                exit(2);
        }
    }
    
    printf("command line options:\n");
    printf("  hz = %d\n", hz);
    printf("  dumb fingers = %d\n", dumb_fingers);
    printf("  dumb finger 1 = %d\n", dumb_finger1);
    printf("  dumb finger 2 = %d\n", dumb_finger2);
    printf("  dumb finger 3 = %d\n", dumb_finger3);
    printf("  dumb tactile = %d\n", dumb_tactile);
    printf("  sensor mask = 0x%04X\n", sensor_mask);
    printf("  no tact = %d\n", no_tact);
    printf("  no tact temp = %d\n", no_tact_temp);
    printf("  no flex = %d\n", no_flex);
    printf("  no accel = %d\n", no_accel);

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
    setPayload(send_buff, (unsigned int)round(1000000.0/(float)hz));
    send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
    write(fd, send_buff, COMMAND_PACKET_LENGTH);

    printf("Setting sample period:  ");
    for (int i=0; i<COMMAND_PACKET_LENGTH; i++)
        printf("%02X ", send_buff[i]);
    printf("\n");

    int len = 0;
    resp = readResponse(fd, len);
    if (resp != 0)
    {
        printf("ERROR, error code: %02X\n", resp);
        printf("raw buffer: ");
        for (int i=0; i<20; i++)
            printf("%02X ", recv_buff[i]);
        printf("\n");
    }
    
    
    if (no_tact)
        sensor_mask = sensor_mask & (~DATA_COLLECTION_TACTILE_BITMASK);
    if (no_tact_temp)
        sensor_mask = sensor_mask & (~DATA_COLLECTION_TACTILE_TEMP_BITMASK);
    if (no_flex)
        sensor_mask = sensor_mask & (~DATA_COLLECTION_DISTALJOINT_BITMASK);        
    if (no_accel)
        sensor_mask = sensor_mask & (~DATA_COLLECTION_ACCELERATION_BITMASK);        
    
    printf("Sensor mask: 0x%04X\n", sensor_mask);
    
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

    unsigned char chain_mask = ALL_CHAINS_CHAINMASK;
    if (dumb_fingers)
    {
        chain_mask = chain_mask & (~FINGER_1_CHAINMASK);
        chain_mask = chain_mask & (~FINGER_2_CHAINMASK);
        chain_mask = chain_mask & (~FINGER_3_CHAINMASK);
    }
    if (dumb_finger1)
        chain_mask = chain_mask & (~FINGER_2_CHAINMASK); //NOTE: hw/sw finger mapping
    if (dumb_finger2)
        chain_mask = chain_mask & (~FINGER_1_CHAINMASK); //NOTE: hw/sw finger mapping
    if (dumb_finger3)
        chain_mask = chain_mask & (~FINGER_3_CHAINMASK);
    if (dumb_tactile)
        chain_mask = chain_mask & (~TACTILE_CHAINMASK);
    
    printf("Chain mask: 0x%02X\n", chain_mask);
    
    //
    // set chain mask
    //
    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
    send_buff[COMMAND_OFFSET] = SET_CHAIN_MASK_OPCODE;
    send_buff[PAYLOAD_OFFSET] = chain_mask; // 1 byte payload
    send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
    write(fd, send_buff, COMMAND_PACKET_LENGTH);
    
    printf("Setting chain mask:  ");
    for (int i=0; i<COMMAND_PACKET_LENGTH; i++)
        printf("%02X ", send_buff[i]);
    printf("\n");
    
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

    int responsesArray[12][100];
    for (int b=0; b<12; b++)
        for (int i=0; i<100; i++)
            responsesArray[b][i] = 0;
    
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
                for (int b=0; b<12; b++)
                {
                    data->responseHistory[b] -= responsesArray[b][count % 100];
                    responsesArray[b][count % 100] = data->responses[b];
                    data->responseHistory[b] += data->responses[b];
                }
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
