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

#ifndef HANDLE_TCP
#define HANDLE_TCP

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

// forward declarations
void* handle_thread(void* context);
class Dexter;

/// The callback signature to receive sensor data.
typedef void (Dexter::*handle_cb_t)(const HandPacket& data);


/// Low-level C++ library for connecting to Dexter hand
//
class Dexter
{
public:
    
    /// Constructor
    Dexter()
    {
        _socketfd = -1;
        _udp = false;
        //_hthread_cb = NULL;
        _run = false; 
    };
    
    /// Destructor
    ~Dexter()
    {
    };
    
    /// Connect to the server on the hand and store the socket file descriptor
    // in the global variable.
    // Return -1 on fail.
    int connect(const char* const server, const char* const port, bool udp)
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
            
            // ::connect() uses the global, builtin function instead of member function
            if (::connect(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
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

    

    /// Start the listening thread spinning.
    // Pass in a callback function you want called when new data arrives.
    // Returns -1 on fail.
    int start() //handle_cb_t callback)
    {
        //_hthread_cb = callback;
        _run = true;

        pthread_attr_init(&_attr);
        pthread_attr_setdetachstate(&_attr, PTHREAD_CREATE_JOINABLE);
        
        return pthread_create(&_hthread, &_attr, handle_thread, (void *)this);
    };

    /// Stop the tcp listening thread.  Block until thread completes.
    // Returns -1 on fail.
    int stop()
    {
        if (_run)
        {
            _run = false;
            if (_socketfd >= 0)
            {
                close(_socketfd);
                _socketfd = -1;
            }
            pthread_attr_destroy(&_attr);
            return pthread_join(_hthread, NULL);
        }
    
        return 0;
    };

    /// Send command to palm.
    // Returns -1 on fail.
    int send(HandleCommand cmd)
    {
        unsigned char buff[100];
        int len = cmd.pack(buff);
        if (_udp)
            return sendalludp(_socketfd, buff, len, &_addr, _addrlen);
        else
            return sendall(_socketfd, buff, len, 0);
    };
    
    bool isRunning() const
    {
        return _run;
    };
    
    int getSocket() const
    {
        return _socketfd;
    };
    
    // bool hasCallback() const
    // { 
    //     return (_hthread_cb != NULL);
    // };
    
    // void callback(HandPacket msg) const
    // { 
    //     return this->*_hthread_cb(msg);
    // };

    /// The callback for sensor data from the hand.  
    void handle_cb(const HandPacket& data);
    
private:
    
    /// The tcp socket file descriptor
    int _socketfd;
    
    /// socket address info
    sockaddr _addr;
    
    /// socket address info length
    socklen_t _addrlen;
    
    /// true to use UDP instead of TCP socket
    bool _udp;
    
    // /// The function pointer for the sensor data callback
    // handle_cb_t _hthread_cb;
    
    /// The tcp/udp listener thread
    pthread_t _hthread;
    
    /// pthread attributes
    pthread_attr_t _attr;
    
    /// true when running
    bool _run; 
};


/// The code for the tcp listener thread.
// Pass in the tcp socket file descriptor.
void* handle_thread(void* context)
{
    Dexter* dexter = (Dexter*)context;
    int ret = 0;
    
    unsigned char buffer[sizeof(HandPacket)+5];
    HandPacket msg;
    int len = msg.pack(buffer); // dummy pack to get serialization length.
    
    while (ret != -1 && dexter->isRunning())
    {
        ret = recvall(dexter->getSocket(), buffer, len);
        if (ret > 0 && dexter->isRunning())
        {
            msg.unpack(buffer);
            //if (dexter->hasCallback())
            dexter->handle_cb(msg);
            //dexter->callback(msg);
        }
    }
    
    //close(fd);
    pthread_exit(NULL);
};

//////////////////////////////////////////


//
// Example program: test.cpp
//
// to compile and run:
//   cp ../../handle_lib/include/handle_lib/*.hpp .
//   cp ../../handle_lib/lib/libhandle_lib.so .
//   g++ test.cpp -L. -lhandle_lib -pthread -o test
//   ./test
//

/*

#include "handleTcp.hpp"
#include "handleControl.hpp"
#include "handleTypes.hpp"
//#include "packetParser.hpp"


void my_callback(const HandPacket& msg)
{
    printf("%f ", msg.data.airTemp);
    for (int i=0; i<5; i++)
    {
        printf("%d ", msg.data.motorHallEncoder[i]);
    }
    printf("\n");
};

int main(int argc, char *argv[])
{
    Dexter hand;
    
    if (hand.connect("192.168.40.31", "3490", false) != 0)
    {
        printf("ERROR: hand 1 connection error\n");
    }
    
    if (hand.start(my_callback) != 0)
    {
        printf("ERROR: cannot start hand 1 handler thread\n");
    }

    while (true)
    {
        printf("Press ENTER to exit\n");
        printf("Close fingers to value:");
        char read_buff[10];
        memset(read_buff, 0, 10);
        fgets(read_buff, 10, stdin);
        
        if (read_buff[0] == '\n')
            break;
        
        int val = atoi(read_buff);
        
        HandleCommand cmd;
        for (int i=0; i<3; i++)
        {
            cmd.motorCommand[i].valid = true;
            cmd.motorCommand[i].type = MOTOR_POSITION;
            cmd.motorCommand[i].value = val;
        }
        
        hand.send(cmd);
    }
    
    if (hand.stop() != 0)
    {
        printf("WARNING: cannot stop hand 1 handler thread\n");
    };
    
    return 0;
}
*/


#endif
