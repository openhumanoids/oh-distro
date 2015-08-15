#ifndef BYTECONVERSION_H_
#define BYTECONVERSION_H_

#define DOUBLESIZE 8
#define LONGSIZE 4

int long2bytes(long longArray[], uint8_t byteArray[], int numBytes);
int bytes2long(uint8_t byteArray[], long longArray[],int numBytes);
int bytes2double(uint8_t byteArray[], double doubleArray[],int numBytes);
int double2bytes(double doubleArray[],uint8_t byteArray[],int numBytes);
int reverseLong(uint8_t bytesIn[], int numBytes);
#endif /* BYTECONVERSION_H_ */
