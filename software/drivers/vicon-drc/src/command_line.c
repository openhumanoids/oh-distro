#include <stdio.h>      
#include <sys/socket.h> 
#include <arpa/inet.h>  
#include <stdlib.h>     
#include <string.h>    
#include <unistd.h>    
#include <aio.h> 
#include <signal.h>
#include <pthread.h>
#include <termios.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "main.h"
#include "command_line.h"
#include "globalvar.h"
#include "vicon_client.h"

int logdata_create_flag=0;

int parseCommandLine()
{

	char arg[50];

	printf(">> ");

	scanf ( "%s", arg);  

//------------------------------------------------------------------------------------------------------//
//======================================================================================================//
//------------------------------------------------------------------------------------------------------//

	if ( strcmp( arg, "vicon" ) == 0 )
	{

		scanf ( "%s", arg);

	  	if ( strcmp( arg, "on" ) == 0 )
		{
	  			if(vicon_flag==1)
				{
				
				printf("ERROR: vicon already started!\n");
		
				}

	  			else
				{

				printf("starting vicon ...\n");
				
				pthread_create( &vicon_thread, NULL, vicon_thread_handler,NULL);
	
				vicon_flag=1;

				}
		}
		else if ( strcmp( arg, "off" ) == 0 )
		{
	  			if(vicon_flag==0)
				{

				printf("ERROR: vicon already stopped!\n");
		
				}
	  			else
				{
		
				printf("stopping vicon ...\n");

				endDataStream();
				pthread_cancel(vicon_thread);

				vicon_flag=0;

				}
		}
			else if ( strcmp( arg, "init" ) == 0 )
		{
	  			if(vicon_flag==1)
				{
				error(4);
		
				}
	  			else
				{
				
				printf("TRYING TO CONNECT...\n");
				tcpip(servIP,servPort,32,0);

				printf("Connection started...\n");
				setIOSignal(0);

				printf("AWAITING CONNECTION FROM TARSUS...\n");

				getInfoPacket();

				printf("FULLY CONNECTED!\n");
				printf("cpda\n");

				tcpip_close();

				vicon_init_flag=1;

				}
		}
		else
		{

			error(16);
		}
	}

	else if ( strcmp( arg, "help" ) == 0 )
	{
		printf("--Program Commands--\n\n");
		printf("vicon init : initializes vicon system\n");
		printf("vicon on   : turn on vicon data stream\n");
		printf("vicon off  : turn off vicon data stream\n");
	}

	else if (strcmp( arg, "flags" ) == 0 )
	{

	}
	else if (strcmp( arg, "quit" ) == 0 )
	{
		exit(0);
	}
	else
	{
		error(16);
	}



	return 0;

}

int error(int errorNumber)
	{

	switch(errorNumber)
	{
	case 1:
	printf("vicon is already enabled");
	break;
	case 2:
	printf("vicon not initialized!");
	break;
	case 3:
	printf("vicon is already disabled");
	break;
	case 4:
	printf("vicon is running, disable before initializing");
	break;
	case 5:
	printf("control is already enabled");
	break;
	case 6:
	printf("control is already disabled");
	break;
	case 7:
	printf("can't print, vicon is disabled");
	break;
	case 8:
	printf("can not log data, vicon disabled");
	break;
	case 9:
	printf("vicon data logging already enabled");
	break;
	case 10:
	printf("vicon data logging already disabled");
	break;
	case 11:
	printf("can not log data, control disabled");
	break;
	case 12:
	printf("control data logging already enabled");
	break;
	case 13:
	printf("control data logging already disabled");
	break;
	case 14:
	if(packetCount==0)
	{

	}
	else
	{
	printf("vicon has dropped a packet!");
	
	}
	break;
	case 15:
	printf("already printing!");
	break;
	case 16:
	printf("INVALID COMMAND: see 'help' command");
	break;
	case 17:
	printf("can't print, control is disabled");
	break;
	default:
	break;
	}
	printf("\n");

	return 0;

}
