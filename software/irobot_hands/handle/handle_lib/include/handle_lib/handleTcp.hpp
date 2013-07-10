/**
 * \file handleTcp.hpp
 *
 * TCP clinet interface to handle device.  Mostly written for the client 
 * (laptop) side, but includes some tcp helper functions for both sides.
 *
 * Written under government funding for ARM-H project.
 *
 * \author Ben Axelrod
 * \date   March 2012
 * \copyright Copyright iRobot Corporation, 2012
 **/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <pthread.h>
#include <signal.h>
#include <arpa/inet.h>

#include "handleTypes.hpp"
#include "packetParser.hpp"
#include "handleControl.hpp"

//#define HANDLE_PORT "3490" // the tcp port


/// The callback signature to receive sensor data.
typedef void (*handle_cb_t)(const HandPacket& data);

/// The tcp socket file descriptor
int _socketfd = -1;
sockaddr _addr;
socklen_t _addrlen;
bool _udp = false;

/// The function pointer for the sensor data callback
handle_cb_t hthread_cb = NULL;

/// The tcp listener thread
pthread_t hthread;

bool run_handle = true;

/// Connect to the server on the hand and store the socket file descriptor
// in the global variable.
// Return -1 on fail.
int handle_connect(const char* const server, const char* const port, bool udp)
{
    int sockfd;
    struct addrinfo hints, *servinfo, *p;
    int rv;
    
    _udp = udp;
    
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    if (udp)
        hints.ai_socktype = SOCK_DGRAM;
    else
        hints.ai_socktype = SOCK_STREAM;
    
    if ((rv = getaddrinfo(server, port, &hints, &servinfo)) != 0) {
        return -1;
    }

    // loop through all the results and connect to the first we can
    for(p = servinfo; p != NULL; p = p->ai_next) {
        if ((sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
            continue;
        }
        
        if (connect(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
            close(sockfd);
            continue;
        }
        
        _addr = *(p->ai_addr);
        _addrlen = p->ai_addrlen;
        
        break;
    }

    freeaddrinfo(servinfo);
    
    if (p == NULL) {
        return -1;
    }

    _socketfd = sockfd;
    return 0;
};

/// Just like regular tcp send(), but retrys until entire message is sent.
// Return -1 on failure, num bytes sent on success.
int sendall(int sockfd, const unsigned char* const msg, int len, int flags = 0, unsigned int tries = 5) 
{
    int total = 0;        // how many bytes we've sent
    unsigned int t = 0;

    while(total < len) 
    {
        int n = send(sockfd, msg+total, len-total, flags);
        if (n < 0)
            return -1;
        total += n;
        if (t++ >= tries)
            return -2;
    }
    
    return total; 
};

/// Just like regular tcp send(), but retrys until entire message is sent.
// Return -1 on failure, num bytes sent on success.
int sendalludp(int sockfd, const unsigned char* const msg, int len, 
               const struct sockaddr* addr, socklen_t addrlen,
               int flags = 0, unsigned int tries = 5) 
{
    int total = 0;        // how many bytes we've sent
    unsigned int t = 0;

    while(total < len) 
    {
        int n = sendto(sockfd, msg+total, len-total, flags, addr, addrlen);
        if (n < 0)
            return -1;
        total += n;
        if (t++ >= tries)
            return -2;
    }
    
    return total; 
};

/// Just like standard tcp recv(), but blocks for up to n bytes.  
// Will make up to 5 blocking tries to get desired number of bytes.
// Returns number of bytes read, or -1 on read error, -2 on number of tries
// error.
int recvall(int sockfd, void* buff, int len, int flags = 0, unsigned int tries = 5)
{
    int rd = 0;
    unsigned int t = 0;
    while (rd < len)
    {
        int r = recv(sockfd, ((char*)buff)+rd, len-rd, flags);
        if (r < 0)
            return -1;
        rd += r;
        if (t++ >= tries)
            return -2;
    }
    return rd;
};

/// Just like standard tcp recv(), but blocks for up to n bytes.  
// Will make up to 5 blocking tries to get desired number of bytes.
// Returns number of bytes read, or -1 on read error, -2 on number of tries
// error.
int recvalludp(int sockfd, void* buff, int n, struct sockaddr* addr, socklen_t* addrlen,
               int flags = 0, unsigned int tries = 5)
{
    int rd = 0;
    unsigned int t = 0;
    while (rd < n)
    {
        int r = recvfrom(sockfd, ((char*)buff)+rd, n-rd, flags, addr, addrlen);
        if (r < 0)
            return -1;
        rd += r;
        if (t++ >= tries)
            return -2;
    }
    return rd;
};

/// The code for the tcp listener thread.
// Pass in the tcp socket file descriptor.
void* handle_thread(void* socketfd)
{
    int fd = (int)socketfd;
    int ret = 0;
    
    unsigned char buffer[500];
    HandPacket msg;
    int len = msg.pack(buffer); // dummy pack to get serialization length.
    
    while (ret != -1 && run_handle)
    {
        ret = recvall(fd, buffer, len);
        if (ret > 0)
        {
            msg.unpack(buffer);
            if (hthread_cb)
                hthread_cb(msg);
        }
    }
    
    close(fd);
    pthread_exit(NULL);
};

/// Start the listening thread spinning.
// Pass in a callback function you want called when new data arrives.
// Returns -1 on fail.
int handle_start(handle_cb_t callback)
{
    hthread_cb = callback;
    run_handle = true;
    return pthread_create(&hthread, NULL, handle_thread, (void *)_socketfd);
};

/// Stop the tcp listening thread.  Block until thread completes.
// Returns -1 on fail.
int handle_stop()
{
    run_handle = false;
    if (_socketfd >= 0)
        close(_socketfd);
    return pthread_join(hthread, NULL);
};

/// Send command to palm.
// Returns -1 on fail.
int handle_send(HandleCommand cmd)
{
    unsigned char buff[100];
    int len = cmd.pack(buff);
    if (_udp)
        return sendalludp(_socketfd, buff, len, &_addr, _addrlen);
    else
        return sendall(_socketfd, buff, len, 0);
};

//////////////////////////////////////////

// void my_callback(const HandSensors& data)
// {
//     printf("my callback:\n");
    
//     printf("Voltage: %f %f %f\n", 
//            data.voltage.volts33, 
//            data.voltage.volts12, 
//            data.voltage.volts48);
//     printf("Hall Encoders: %d %d %d %d\n", 
//            data.motorHallEncoder[0], 
//            data.motorHallEncoder[1], 
//            data.motorHallEncoder[2], 
//            data.motorHallEncoder[3]);
// };

// int main(int argc, char *argv[])
// {
//     if (handle_connect("armH-palm-1", "3490") < 0)
//     {
//         printf("connect error\n");
//         exit(1);
//     }
    
//     handle_start(my_callback);
    
//     bool run = true;
//     while (run)
//     {
//         char read_buff[10];
//         memset(read_buff, 0, 6);
//         int motor = 0;
//         int type = 0;
//         int val = 0;
        
//         printf("\n\n");
//         printf("Motor [0-4]:");
//         fgets(read_buff, 10, stdin);
        
//         if (read_buff[0] == '\n')
//             break;
        
//         motor = atoi(read_buff);
        
//         printf("\n");
//         printf("0 : Velocity\n");
//         printf("1 : Position\n");
//         printf("Type [0-1]:");
//         fgets(read_buff, 10, stdin);
//         type = atoi(read_buff);
        
//         printf("\n");
//         printf("Amount [-65535 to 65535]:");
//         fgets(read_buff, 6, stdin);
//         val = atoi(read_buff);
        
//         if (motor >= 0 && motor < 5)
//         {
//             HandleCommand cmd;
//             cmd.motorValid[motor] = true;
//             cmd.motorCommand[motor].type = (CommandType)type;
//             cmd.motorCommand[motor].value = val;
        
//             sendall(_socketfd, (char*)&cmd, sizeof(HandleCommand), 0);
//         }
//     }
    
//     //handle_stop();
    
//     // closing the socket stops the thread
//     handle_close();
    
//     return 0;
// }
