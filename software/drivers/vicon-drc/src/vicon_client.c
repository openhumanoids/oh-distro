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
#include "byteconversion.h"
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "vicon_client.h"
#include "lcm_i.h"
#include "globalvar.h"
#include "structs.h"

#define RCVBUFSIZE 32

int getInfoPacket();
int parseCommandLine();
int error(int errorNumber);
float butterworthFilter(float unfiltered_val, int filter_order, int s);
void setup_io();
void DieWithError(char *errorMessage);
void sigio_func();
int handleData();

int sock;
int var2channel[18];
FILE *fp;

model_descriptor_t* models = NULL;
viconstructs_vicon_t* vicon = NULL;
channelmap_t* channelmaps = NULL;

uint64_t prevPacketCount = 0;

double cmd;
double q7;

int tcpip(char *servIP,unsigned short servPort, int bufferSize, int byteOrder)
{

	struct sockaddr_in servAddr;

	if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
		
		DieWithError("socket() failed");

		    
		memset(&servAddr, 0, sizeof(servAddr));
		    
		servAddr.sin_family      = AF_INET;
		    
		servAddr.sin_addr.s_addr = inet_addr(servIP);
		    
		servAddr.sin_port        = htons(servPort);

	    
	if (connect(sock, (struct sockaddr *) &servAddr, sizeof(servAddr)) < 0)

		DieWithError("connect() failed");

	return 0;

}


int setIOSignal(int state)
{
	if(state==1)
	{
		
	signal(SIGIO, sigio_func); 
  
	fcntl(sock, F_SETOWN, getpid());

	fcntl(sock, F_SETFL, FASYNC);

	}
	
	else
	{
		
	signal(SIGIO, sigio_func); 
  
	fcntl(sock, F_SETOWN, getpid());

	fcntl(sock, F_SETFL, 0);
		
	}
	
	return 0;
}

int getInfoPacket() //cpda: corre al hacer vicon init
{

        // Get system channel data
	long command[2];

	char tempRead[40];

	int sizeCommand;

	long header[2];

	int sizeHeader;

	long numChannels[1];

	int sizenumChannels;

	long numLetters[1];

	int sizenumLetters=4;

	char channelNames[300][40];

	sizeCommand=8;

	sizeHeader=8;

	sizenumChannels=4;

	uint8_t command_bytes[8];

	uint8_t header_bytes[8];

	uint8_t numChannels_bytes[4];

	uint8_t numLetters_bytes[4];

	command[0]=1;

	command[1]=0;

	long2bytes(command, command_bytes, sizeCommand);

	write(sock,command_bytes,sizeCommand);

	read(sock,header_bytes,sizeHeader);

	bytes2long(header_bytes, header,sizeHeader);

	read(sock,numChannels_bytes,sizenumChannels);

	bytes2long(numChannels_bytes, numChannels,sizenumChannels);

	printf("Number of Channels=%ld\n",numChannels[0]);

        // Free up, initialize linked lists
        if (models)
        {
            free_models(models);
        }
        models = malloc(sizeof(model_descriptor_t));
        models->name = NULL;
        models->markers = NULL;
        models->next = NULL;

        if (channelmaps)
        {
            free_channelmap(channelmaps);
        }
        channelmaps = malloc(sizeof(channelmap_t));
        channelmaps->channel = 0;
        channelmaps->datapos = NULL;
        channelmaps->next = NULL;

        // Print channel names, add channels to descriptor data structure
        int j;
	for(j=0;j<numChannels[0];j++)
	{

		read(sock,numLetters_bytes,sizenumLetters);

		bytes2long(numLetters_bytes, numLetters,sizenumLetters);

		read(sock,tempRead,numLetters[0]);

                int k;
		for(k=0;k<40;k++)
		{

			channelNames[j][k]=tempRead[k];
	
		}

		printf("%d Channel Name = %s\n",j,channelNames[j]);

                addChannel(channelNames[j],j,models);
                printf("\n");

		strcat(channelNames[j], "\n");

		for(k=0;k<40;k++)
		{
		tempRead[k]=0;
		}

		usleep(100);
	}

        // Generate LCM data structure
        if (vicon)
        {
            viconstructs_vicon_t_destroy(vicon);
        }
        printf("genning\n");
        vicon = genLCM(models, channelmaps);

        printf("\n");
        printf("Printing Descriptor Data Structure\n");
        printModelDescriptors(models);
        printf("Printing LCM Data Structure\n");
        printLCM(vicon);
        printf("Printing Channel Map Data Structure\n");
        printChannelMaps(channelmaps);

        return 0;
	
}

int startDataStream()
{
	
	long command[] = {3, 0};
	int sizeCommand = 0;
	
	sizeCommand=8;
	
	command[0]=3;
	command[1]=0;

	uint8_t command_bytes[8];

	long2bytes(command, command_bytes, sizeCommand);

	printf("inside int startDataStream\n");
	printf("vicon is streaming...\n");
	
	write(sock,command_bytes,sizeCommand);	
	
	setIOSignal(1);

	for(;;);

return 0;

}

int endDataStream()
{
	
	long command[2];
	int sizeCommand;
	
	sizeCommand=8;
	
	command[0]=4;
	command[1]=0;

	uint8_t command_bytes[8];

	long2bytes(command, command_bytes, sizeCommand);

	write(sock,command_bytes,sizeCommand);	
	
	setIOSignal(0);

	printf("vicon data stream has ended\n");

return 0;

}

int tcpip_close()
{
close(sock);

return 0;	
}

int handleData()
{

	static struct timeval td_start,td_end;
	static float elapsed = 0;	
	double data[500];
	uint8_t data_bytes[500*8];
	
	gettimeofday(&td_end,NULL);

	elapsed = 1000.0 * (td_end.tv_sec -td_start.tv_sec);
	
	elapsed += (td_end.tv_usec - td_start.tv_usec)/1000;
	
	gettimeofday(&td_start,NULL);

	long numValues[1];

	int sizenumValues;

	long header[2];

	int sizeHeader;

	sizeHeader=8;

	uint8_t header_bytes[8];

	sizenumValues=4;

	uint8_t numValues_bytes[4];
	
	read(sock,header_bytes,sizeHeader);

	bytes2long(header_bytes, header,sizeHeader);

	read(sock,numValues_bytes,sizenumValues);

	bytes2long(numValues_bytes, numValues,sizenumValues);

	read(sock,data_bytes,numValues[0]*8);
	
	bytes2double(data_bytes, data,numValues[0]*8);

	if((packetCount+1) != data[0] && packetCount!=0)
	{
            printf("\npacketCount+1: %d data[0]: %d\n", (int)packetCount+1, (int)data[0]);
		error(14);
                //return 1;
	}
        else if(prevPacketCount != packetCount)
        {
            printf("p");
            fflush(stdout);
        }
        else {
            printf("\nn\n");
            fflush(stdout);
        }
        prevPacketCount = packetCount;

	packetCount=data[0];

        // Update the channels
        channelmap_t* channelmap = channelmaps;
        while (channelmap)
        {
            if (channelmap->datapos)
            {
                *(channelmap->datapos)=data[channelmap->channel];
            }
            channelmap=channelmap->next;
        }


        /*
        int64_t i;
        for(i=0;i<numValues[0];i++)
        {
            printf("Channel %d: %f\n", i, ((double*)data)[i]);
        }
        */

        lcm_publish_vicon(vicon);

	return 0;
}

void DieWithError(char *errorMessage)
{
    perror(errorMessage);
    exit(1);
} 

void sigio_func()
{
handleData();

}
  
float butterworthFilter(float unfiltered_val, int filter_order, int s)
{

	float a[]={1,-1.1430,0.4128};
	float b[]={0.0675,0.1349,0.0675};

	int i;
	int n=2;
	static float xf1[3][7];
	static float yf1[3][7];
	static float xf2[3][7];
	static float yf2[3][7];


	for (i=0;i<n;i++)
	{

		xf1[i][s]=xf1[i+1][s];
		yf1[i][s]=yf1[i+1][s];
		xf2[i][s]=xf2[i+1][s];
		yf2[i][s]=yf2[i+1][s];

	}


	xf1[n][s]= unfiltered_val;
	
	yf1[n][s]=b[0]*xf1[n][s];

	for (i=1;i<(n+1);i++)
	{
 
		yf1[n][s]= yf1[n][s]+ b[i]*xf1[n-i][s]-a[i]*yf1[n-i][s];

	}

	yf1[n][s]=yf1[n][s]/a[0];

	if(filter_order==4)
	{

		unfiltered_val=yf1[n][s];

		xf2[n][s]= unfiltered_val;
	
		yf2[n][s]=b[0]*xf2[n][s];

		for (i=1;i<(n+1);i++)
		{
	 
			yf2[n][s]= yf2[n][s]+ b[i]*xf2[n-i][s]-a[i]*yf2[n-i][s];

		}

		yf2[n][s]=yf2[n][s]/a[0];

		unfiltered_val=yf2[n][s];


		return yf2[n][s];

	}

	else
	
	{

	return yf1[n][s];

	}
}

