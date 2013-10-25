/**
 * \file test_motor.c
 *
 * test motor
 *
 * Written under government funding for ARM-H project.
 *
 * \author Ben Axelrod
 * \date   February 2013
 * \copyright Copyright iRobot Corporation, 2013
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


bool vel_good[2];
bool temp_good[2];
bool current_good[2];
bool encoder_good[2];

int vel[2];
float temp[2];
float current[2];
short encoder[2];



char isVelGood(int i)
{
    if (vel_good[i])
        return 'O';
    return 'X';
}

char isTempGood(int i)
{
    if (temp_good[i])
        return 'O';
    return 'X';
}

char isCurrentGood(int i)
{
    if (current_good[i])
        return 'O';
    return 'X';
}

char isEncoderGood(int i)
{
    if (encoder_good[i])
        return 'O';
    return 'X';
}

bool isAllGood()
{
    for (int i=0; i<2; i++)
    {
        if (vel_good[i] && temp_good[i] && current_good[i] && encoder_good[i])
            continue;
        else
            return false;
    }
    printf("all good\n");
    return true;
}


#define MOTOR_SPEED 1000

void updateVel(int i, int val)
{
    if (val > 500 && val < 2000)
        vel_good[i] = true;
}

void updateTemp(int i, float val)
{
    if (val > 20.0 && val < 30.0)
        temp_good[i] = true;
}

void updateCurrent(int i, float val, float lowlim, float highlim=0.5)
{
    if (val > lowlim && val < highlim)
        current_good[i] = true;
}

void updateEncoder(int i, int val)
{
    if (val > 500)
        encoder_good[i] = true;
}

/// Set the motor [0-4] to the desired speed value.
// Note that value can be negative.
// Value will be capped at 2 bytes (65535).
// returns error code.
//  0 = success
// -2 = unknown motor
// -3 = write error
int writeMotor(int fd, int motor, int value)
{
    //printf("writing motor %d with %d\n", motor, value);
    
    unsigned char buff[COMMAND_PACKET_LENGTH];
    memset(buff, 0, COMMAND_PACKET_LENGTH);
    
    switch (motor)
    {
        case 0:
            buff[DESTINATION_HEADER_OFFSET] = DESTINATION_MOTOR1;
            break;
        case 1:
            buff[DESTINATION_HEADER_OFFSET] = DESTINATION_MOTOR2;
            break;
        case 2:
            buff[DESTINATION_HEADER_OFFSET] = DESTINATION_MOTOR3;
            break;
        case 3:
            buff[DESTINATION_HEADER_OFFSET] = DESTINATION_MOTOR4;
            break;
        case 4:
            buff[DESTINATION_HEADER_OFFSET] = PALM_CHAINADDRESS;
            break;
        default:
            return -2;
    }
    
    if (motor == 4)
        buff[COMMAND_OFFSET] = FINGER_COMMAND_OPCODE;
    else
        buff[COMMAND_OFFSET] = MOTOR_COMMAND_OPCODE | MOTOR_COMMAND_VELOCITY;
    
    if (value == 0)
        buff[COMMAND_OFFSET] |= MOTOR_COMMAND_STOP;
    else if (value > 0)
        buff[COMMAND_OFFSET] |= MOTOR_COMMAND_FORWARD;
    else
        buff[COMMAND_OFFSET] |= MOTOR_COMMAND_REVERSE;
    
    //if (value < 0)
    //    value = -value;
    
    //if (value > 65535)
    //    value = 65535;
    
    setPayload(buff, (short)value);
    buff[CHECKSUM_OFFSET] = computeChecksum(buff, COMMAND_PACKET_LENGTH-1);

    if (write(fd, buff, COMMAND_PACKET_LENGTH) != COMMAND_PACKET_LENGTH)
        return -3;
    
    return 0;
};


int main(int argc, char** argv)
{
    // usage: test_motor [auto stop]
    
    bool autostop = true;
    if (argc > 1)
    {
        autostop = (bool)atoi(argv[1]);
    }
    
    double currentlim = 0.08;
    if (argc > 2)
    {
        currentlim = atof(argv[2]);
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
    
    int sensor_mask = DATA_COLLECTION_ALL_BITMASK;
    int destination[2]; 
    int responding[2];
    destination[0] = DESTINATION_MOTOR1;
    destination[1] = DESTINATION_MOTOR2;
    responding[0]= RESPONDING_DEVICES_FIRST_MOTOR1_BITMASK;
    responding[1]= RESPONDING_DEVICES_FIRST_MOTOR2_BITMASK;

    // stop collection
    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
    send_buff[COMMAND_OFFSET] = STOP_COLLECTION_OPCODE; // stop collection
    //setPayload(send_buff, DATA_COLLECTION_MOTORHALL_BITMASK); // no payload
    send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
    write(fd, send_buff, COMMAND_PACKET_LENGTH);
    
    usleep(4000);
    read(fd, recv_buff, BUFFER-1);

    int loopcounter = 0;
    bool done = false;
    while (run && (!done || !autostop))
    {
        printf("\n== %d ==\n", loopcounter++);
        
        HandSensors data;
        HandSensorsValid valid;
        for (int counter=0; counter<2; counter++)
        {
            // write motor
            writeMotor(fd, counter%2, MOTOR_SPEED);
            readResponse(fd);
            //printf("returned: %02X\n", rsp);
        
            // wait
            usleep(500000);
        
            // create poll message
            memset(send_buff, 0, COMMAND_PACKET_LENGTH);
            send_buff[DESTINATION_HEADER_OFFSET] = destination[counter%2];
            send_buff[COMMAND_OFFSET] = DATA_COLLECTION_OPCODE;
            setPayload(send_buff, sensor_mask);
            send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH);
        
            write(fd, send_buff, COMMAND_PACKET_LENGTH);
        
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
            
                int length = computeDataSize(sensor_mask, responding[counter%2]);
                if (length != readlen-5)
                {
                    printf("\n\nread length error\n");
                }
                else
                {
                    int parsed = parseData(&(recv_buff[RESPONSE_PAYLOAD_OFFSET]), sensor_mask, responding[counter%2], data, valid);
                    if (parsed != length)
                    {
                        printf("\n\nparsed length error\n");
                        printf("parsed %d bytes, expecting %d\n", parsed, length);
                        printf("sensor mask: %02X\n", sensor_mask);
                        printf("responding devices: %02X\n", responding[counter%2]);
                    }
                    else
                    {
                        //printf("ok\n");
                        //data->printonly(*valid);
                    
                        //printf("motor %d\n", (counter%2)+1);
                    
                        if (valid.motorVelocity[motorChain_to_fingerNumber[counter%2]])
                            updateVel(counter%2, data.motorVelocity[motorChain_to_fingerNumber[counter%2]]);
                    
                        if (valid.motorHousingTemp[motorChain_to_fingerNumber[counter%2]])
                            updateTemp(counter%2, data.motorHousingTemp[motorChain_to_fingerNumber[counter%2]]);

                        if (valid.motorCurrent[motorChain_to_fingerNumber[counter%2]])
                            updateCurrent(counter%2, data.motorCurrent[motorChain_to_fingerNumber[counter%2]], currentlim);
                    
                        if (valid.motorHallEncoder[motorChain_to_fingerNumber[counter%2]])
                            updateEncoder(counter%2, data.motorHallEncoder[motorChain_to_fingerNumber[counter%2]]);
                    
                        // printf("vel:     %c  %d\n", isVelGood(counter%2), data.motorVelocity[motorChain_to_fingerNumber[counter%2]]);
                        // printf("temp:    %c  %f\n", isTempGood(counter%2), data.motorHousingTemp[motorChain_to_fingerNumber[counter%2]]);
                        // printf("current: %c  %f\n", isCurrentGood(counter%2), data.motorCurrent[motorChain_to_fingerNumber[counter%2]]);
                        // printf("encoder: %c  %d\n", isEncoderGood(counter%2), data.motorHallEncoder[motorChain_to_fingerNumber[counter%2]]);
                    }
                }
            }
        
            usleep(10000);

            // stop write motor
            writeMotor(fd, counter%2, 0);
            readResponse(fd);
            //printf("returned: %02X\n", rsp);
        }
        
        printf("motor:   "); for(int i=0;i<2;i++) printf("--%d--           ", i+1); printf("\n");
        printf("vel:     "); for(int i=0;i<2;i++) printf("%c  %d          ", isVelGood(i), data.motorVelocity[motorChain_to_fingerNumber[i]]); printf("\n");
        printf("temp:    "); for(int i=0;i<2;i++) printf("%c  %f    ", isTempGood(i), data.motorHousingTemp[motorChain_to_fingerNumber[i]]); printf("\n");
        printf("current: "); for(int i=0;i<2;i++) printf("%c  %f     ", isCurrentGood(i), data.motorCurrent[motorChain_to_fingerNumber[i]]); printf("\n");
        printf("encoder: "); for(int i=0;i<2;i++) printf("%c  %d          ", isEncoderGood(i), data.motorHallEncoder[motorChain_to_fingerNumber[i]]); printf("\n");
        
        done = isAllGood();
    }

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

    for (int i=0; i<2; i++)
    {
        printf("stopping motor %d\n", i+1);
        writeMotor(fd, i, 0);
        readResponse(fd);
        //printf("returned: %02X\n", rsp);
    }
    
    printf("closing\n");
    close(fd);
    printf("exiting\n");
    exit(0);
}
