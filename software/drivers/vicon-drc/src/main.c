#include <stdio.h>      /* for printf() and fprintf() */
#include <sys/socket.h> /* for socket(), connect(), send(), and recv() */
#include <arpa/inet.h>  /* for sockaddr_in and inet_addr() */
#include <stdlib.h>     /* for atoi() and exit() */
#include <string.h>     /* for memset() */
#include <unistd.h>     /* for close() */
#include <aio.h> 
#include <signal.h>
#include <pthread.h>
#include <termios.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "globalvar.h"
#include "vicon_client.h"
#include "lcm_i.h"

void *vicon_thread_handler();

int main()
{

global_init();

lcm_publish_init();
	
	while(1)
	{

	printf("in a while in main");

	parseCommandLine();
		
	}

     	pthread_join(vicon_thread, NULL);

     	exit(0);

return 0;

}

void *vicon_thread_handler()
{


	tcpip(servIP,servPort,32,0);

	startDataStream();

	tcpip_close();

	return 0;

}
