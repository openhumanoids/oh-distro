#ifndef VICON_CLIENT_H_
#define VICON_CLIENT_H_

int endDataStream();
int getInfoPacket();
int handleData();
int setIOSignal(int state);
int tcpip(char *servIP,unsigned short servPort, int bufferSize, int byteOrder);
int startDataStream();
int tcpip_close();

#endif /* VICON_CLIENT_H_ */
