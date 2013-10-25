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
#include <vector>

#include "common.hpp"
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


void printUsage()
{
    printf("usage:\n");
    printf("  parametertest2 set <parameter_number> <value> <device>\n");
    printf("  parametertest2 get <parameter_number> <device>\n");
    printf("where:\n");
    printf("  parameter_number = 'a', 'm', 'i', or number (0-31)\n");
    printf("    'a' for all\n");
    printf("    'm' for only the float motor parameters (0-22)\n");
    printf("    'i' for only the non-motor integer parameters (28-31)\n");
    printf("    when setting a parameter, only the parameter number is supported\n");
    printf("  value = 4 byte value\n");
    printf("    interpreted as either a float or integer depending on parameter\n");
    printf("  device = 'p', 'm', 'f', 'r', 'd', 't', 'a', or number (0-11)\n");
    printf("    'p' for palm\n");
    printf("    'm' for motors\n");
    printf("    'f' for fingers (proximals and distals)\n");
    printf("    'r' for proximals\n");
    printf("    'd' for distals\n");
    printf("    't' for tactile\n");
    printf("    'a' for all\n");
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
    printf(" [9] R_w\n");
    printf("[10] R_th_W_S\n");
    printf("[11] T_L+\n");
    printf("[12] T_L-\n");
    printf("[13] Tau_W_S\n");
    printf("[14] T_MAX\n");
    printf("[15] alpha_Cu\n");
    printf("[16] t_off\n");
    printf("[17] T_TARGET\n");
    printf("[18] RPM_MAX\n");
    printf("[19] RSV\n");
    printf("[20] V_MAX\n");
    printf("[21] K_pPos\n");
    printf("[22] PosDeadband\n");
    printf("[23] R_th_S_H\n");
    printf("[24] Tau_S_H\n");
    printf("[...] \n");
    printf("[26] Spread P\n");
    printf("[27] spread deadband\n");
    printf("[28] Encoder Offset\n");
    printf("[29] Firmware Version\n");
    printf("[30] Device ID\n");
    printf("[31] LED\n");
}

std::vector<int> getParameters(const char* const argv)
{
    std::vector<int> parameters;
    
    if (tolower(argv[0]) == 'a') //all
    {
        for(int i=0; i<32; i++)
            parameters.push_back(i);
    }
    else if (tolower(argv[0]) == 'm') //motor floats
    {
        for(int i=0; i<24; i++)
            parameters.push_back(i);
    }
    else if (tolower(argv[0]) == 'i') // non-motor ints
    {
        for(int i=26; i<32; i++)
            parameters.push_back(i);
    }
    else // number
    {
        int parameter = atoi(argv);
        if (parameter >= 0 && parameter <= 31)
            parameters.push_back(parameter);
    }
    return parameters;
}

std::vector<int> getDevices(const char* const argv)
{
    std::vector<int> devices;

    if (tolower(argv[0]) == 'a') //all
    {
        for(int i=0; i<12; i++)
            devices.push_back(i);
    }
    else if (tolower(argv[0]) == 'p') //palm
    {
        devices.push_back(0);
    }
    else if (tolower(argv[0]) == 'm') //motors
    {
        for(int i=7; i<11; i++)
            devices.push_back(i);
    }
    else if (tolower(argv[0]) == 'f') //fingers
    {
        devices.push_back(1);
        devices.push_back(2);
        devices.push_back(3);
        devices.push_back(4);
        devices.push_back(5);
        devices.push_back(6);
    }
    else if (tolower(argv[0]) == 'r') //proximals
    {
        devices.push_back(1);
        devices.push_back(3);
        devices.push_back(5);
    }
    else if (tolower(argv[0]) == 'd') //distals
    {
        devices.push_back(2);
        devices.push_back(4);
        devices.push_back(6);
    }
    else if (tolower(argv[0]) == 't') //tactile
    {
        devices.push_back(11);
    }
    else // device number
    {
        int device = atoi(argv);
        if (device >= 0 && device <= 11)
            devices.push_back(device);
    }
    return devices;
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
    
    
    if (tolower(argv[1][0]) == 'g')  // get
    {
        // parameter selection
        std::vector<int> parameters = getParameters(argv[2]);
        if (parameters.empty())
        {
            printf("ERROR: parameter out of range\n");
            printUsage();
            close(fd);
            exit(1);
        }
        
        // device selection
        std::vector<int> devices = getDevices(argv[3]);
        if (devices.empty())
        {
            printf("ERROR: device out of range\n");
            printUsage();
            close(fd);
            exit(1);
        }

        // print header
        printf("      device:\n");
        printf("      ");
        for (unsigned int d=0; d<devices.size(); d++)
            printf("%d   ", devices[d]);
        printf("\n");
        
        // for each parameter
        for (unsigned int p=0; p<parameters.size(); p++)
        {
            printf("[%d]  ", parameters[p]);
            fflush(stdout);
            
            // for each device
            for (unsigned int d=0; d<devices.size(); d++)
            {
                int destination = 0;
                int responding = 0;
                if (!getDestination(devices[d], destination, responding))
                {
                    printf("ERROR: destination unknown\n");
                    printUsage();
                    close(fd);
                    exit(1);
                }
                
                // create packet
                memset(send_buff, 0, COMMAND_PACKET_LENGTH);
                memset(recv_buff, 0, BUFFER);
                send_buff[DESTINATION_HEADER_OFFSET] = destination;
                send_buff[COMMAND_OFFSET] = MOTOR_PARAMETER_RE_L_OPCODE | parameters[p];
                //setPayload(send_buff, sensor);
                send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
                
                // try 3 times
                bool success = false;
                for (int t=0; t<3; t++)
                {
                    write(fd, send_buff, COMMAND_PACKET_LENGTH);
                    int readlen = 0;
                    int code = readResponse(fd, recv_buff, BUFFER, readlen);
                    if (code == 0)
                    {
                        success = true;
                        break;
                    }
                }
                
                if (success)
                {
                    if (parameters[p] <= 24)
                    {
                        float val = parseParameterf(&(recv_buff[RESPONSE_PAYLOAD_OFFSET]));
                        printf("%5.5f  ", val);
                    }
                    else
                    {
                        int val = parseParameter(&(recv_buff[RESPONSE_PAYLOAD_OFFSET]));
                        printf("%5d  ", val);
                    }
                }
                else
                {
                    printf("XX  ");
                }
                fflush(stdout);
            }
            printf("\n");
        }
    }
    else if (tolower(argv[1][0]) == 's') // set
    {
        // parameter selection
        std::vector<int> parameters = getParameters(argv[2]);
        if (parameters.empty())
        {
            printf("ERROR: parameter out of range\n");
            printUsage();
            close(fd);
            exit(1);
        }
        if (parameters.size() > 1)
        {
            printf("ERROR: can only set 1 parameter at a time\n");
            printUsage();
            close(fd);
            exit(1); 
        }

        // device selection
        std::vector<int> devices = getDevices(argv[4]);
        if (devices.empty())
        {
            printf("ERROR: device out of range\n");
            printUsage();
            close(fd);
            exit(1);
        }

        // print header
        printf("      device:\n");
        printf("      ");
        for (unsigned int d=0; d<devices.size(); d++)
            printf("%d   ", devices[d]);
        printf("\n");
        
        // for each parameter
        printf("[%d]  ", parameters[0]);
        fflush(stdout);

        // for each device
        for (unsigned int d=0; d<devices.size(); d++)
        {
            int device = devices[d];
                
            int destination = 0;
            int responding = 0;
            if (!getDestination(device, destination, responding))
            {
                printf("ERROR: destination unknown\n");
                printUsage();
                close(fd);
                exit(1);
            }

            // create packet
            memset(send_buff, 0, COMMAND_PACKET_LENGTH);
            memset(recv_buff, 0, BUFFER);
            send_buff[DESTINATION_HEADER_OFFSET] = destination;
            send_buff[COMMAND_OFFSET] = MOTOR_PARAMETER_WR_L_OPCODE | parameters[0];
        
            if (parameters[0] <= 24)
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
            
            // try 3 times
            bool success = false;
            for (int t=0; t<3; t++)
            {
                write(fd, send_buff, COMMAND_PACKET_LENGTH);
                int readlen = 0;
                int code = readResponse(fd, recv_buff, BUFFER, readlen);
                if (code == 0)
                {
                    success = true;
                    break;
                }
            }
            
            if (success)
                printf("ok  ");
            else
                printf("XX  ");
            fflush(stdout);
        }
        printf("\n");
        
    }
    else // error
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
