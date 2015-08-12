#include <stdio.h>      
#include <stdlib.h>  
#include "globalvar.h"


//global threads

pthread_t vicon_thread;

int vicon_flag;
int vicon_init_flag;
uint64_t packetCount;

//vicon variables

unsigned short servPort;
char *servIP;


int global_init()

{

vicon_flag=0;
packetCount=0;

 //servIP = "128.30.99.206"; 
 servIP = "192.168.10.1"; 	
 servPort = 800; 

return 0;
}
