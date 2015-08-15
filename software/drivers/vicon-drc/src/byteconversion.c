#include <stdio.h>
#include <signal.h>
#include <sys/time.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>
#include <stdint.h>
#include "byteconversion.h"



union
{
char bytesout[DOUBLESIZE];
double val;
} dbconvert;

union
{
    char bytesin[DOUBLESIZE];
    double val;
} bdconvert;

int long2bytes(long longArray[], uint8_t byteArray[], int numBytes);
int bytes2long(uint8_t byteArray[], long longArray[],int numBytes);
int bytes2double(uint8_t byteArray[], double doubleArray[],int numBytes);
int double2bytes(double doubleArray[],uint8_t byteArray[],int numBytes);
int reverseLong(uint8_t bytesIn[], int numBytes);

int double2bytes(double doubleArray[],uint8_t byteArray[],int numBytes)
 {
	int j;
	int k;
	int numVal=numBytes/DOUBLESIZE;

	for(j=0;j<numVal;j++)	
	{
		dbconvert.val=doubleArray[j];
		
	for(k=0;k<8;k++)
	{
		byteArray[j*DOUBLESIZE+k]=dbconvert.bytesout[k];
		
	}
	
	}
	 
	 
	 return 0;
 }
 
int bytes2double(uint8_t byteArray[], double doubleArray[],int numBytes)
  {
	  
	int j;
	int k;
	int numVal=numBytes/DOUBLESIZE;

	for(j=0;j<numVal;j++)	
	{
		
		
	for(k=0;k<8;k++)
	{
		bdconvert.bytesin[k]=byteArray[j*DOUBLESIZE+k];
		
	}
	
	
	doubleArray[j]=bdconvert.val;
	
	}
	  
	  
	 return 0; 
  }

int bytes2long(uint8_t byteArray[], long longArray[],int numBytes)
  {
	  
	int j;
	int k;
	int numVal=numBytes/LONGSIZE;

	//printf("%d %d\n", numBytes,numVal);
//printf("%d %d %d %d\n", byteArray[0],byteArray[1],byteArray[2],byteArray[3]);
//printf("%d %d %d %d\n", byteArray[4],byteArray[5],byteArray[6],byteArray[7]);
//printf("--------------\n");
	for(j=0;j<numVal;j++)	
	{
		longArray[j]=0;
		
	for(k=0;k<LONGSIZE;k++)
	{

		//printf("%d %d %d \n", byteArray[j*LONGSIZE+k], k, j);
		//printf("%ld \n", longArray[j]);
		
		longArray[j]=longArray[j] | (((byteArray[j*LONGSIZE+k]) << k*8) );
		
	}
	
	
	
	
	}
	 // printf("%ld %ld\n", longArray[0],longArray[1]);
	  
	 return 0; 
  }

int long2bytes(long longArray[], uint8_t byteArray[], int numBytes)
  {
	  
	int j;
	int k;
	int numVal=numBytes/LONGSIZE;

	for(j=0;j<numVal;j++)	
	{
		
		
	for(k=0;k<LONGSIZE;k++)
	{
		byteArray[j*LONGSIZE+k]= (longArray[j] >> 8*k)&0x0FF ;
		
	}
	
	
	
	
	}
	  
	  
	 return 0; 
  }

int reverseLong(uint8_t bytesIn[], int numBytes)
{

	int numLong=numBytes/LONGSIZE;
	uint8_t bytesTemp[numBytes];
	int j;
	int k;

	for(j=0;j<numBytes;j++)
	{

		bytesTemp[j]=bytesIn[j];

	}

	for(j=0;j<numLong;j++)
	{

	for(k=0;k<LONGSIZE;k++)
	{

		bytesIn[k+j*LONGSIZE]=bytesTemp[(LONGSIZE-1)-k+j*LONGSIZE];

	}

}
return 0;
}
