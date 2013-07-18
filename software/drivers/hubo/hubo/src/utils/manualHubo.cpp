#include <iostream>
#include <cstring>
#include <string>
#include <hubo/PracticalSocket.h>

int main(void){
  UDPSocket sock;
  //  string socket_addr = "192.168.1.91";
  string socket_addr = "localhost";
  //  int socket_port = 23456;
  int socket_port = 2010;

  int trash;
  string msg;

//   msg = "#m,MSG_DATA,0,8,1E1122334455AABBCC,***\n";
//   sock.sendTo(msg.c_str(), strlen(msg.c_str()), socket_addr, socket_port);
//   std::cout << "Sent "<<msg<<" via UDP on port "<<socket_port<<" at "<<socket_addr<<std::endl;
//   std::cout << "Any number to continue, enter 1 to quit: " << std::endl;
//   std::cin>>trash;
//   if(trash==1) return 0;
// 
//   msg = "#m,MSG_ACCEPTED,*52\n";
//   sock.sendTo(msg.c_str(), strlen(msg.c_str()), socket_addr, socket_port);
//   std::cout << "Sent "<<msg<<" via UDP on port "<<socket_port<<" at "<<socket_addr<<std::endl;
//   std::cout << "Any number to continue, enter 1 to quit: " << std::endl;
//   std::cin>>trash;
//   if(trash==1) return 0;
// 
//   msg = "#v,***\n";
//   sock.sendTo(msg.c_str(), strlen(msg.c_str()), socket_addr, socket_port);
//   std::cout << "Sent "<<msg<<" via UDP on port "<<socket_port<<" at "<<socket_addr<<std::endl;
//   std::cout << "Any number to continue, enter 1 to quit: " << std::endl;
//   std::cin>>trash;
//   if(trash==1) return 0;
// 
//   msg = "#V,myversion231,***\n";
//   sock.sendTo(msg.c_str(), strlen(msg.c_str()), socket_addr, socket_port);
//   std::cout << "Sent "<<msg<<" via UDP on port "<<socket_port<<" at "<<socket_addr<<std::endl;
//   std::cout << "Any number to continue, enter 1 to quit: " << std::endl;
//   std::cin>>trash;
//   if(trash==1) return 0;
// 
// //original:  msg = "#S,T11:09:21.6,LAT33N14.9322,LON117W27.0600,D-0.02,DG0.00,A9.71,P2.0,R3.0,TR0,TRG0,V0.00,H103.1,HR0.2,HG0.0,M00,***\n";
// // panama city:  
//   msg = "#S,T11:09:21.6,LAT30N4.9322,LON85W42.4300,D-0.02,DG0.00,A9.71,P2.0,R3.0,TR0,TRG0,V0.00,H103.1,HR0.2,HG0.0,M00,***\n";
// // monterey:
//   msg = "#S,T11:09:21.6,LAT36N36.3281,LON121W53.1985,D-0.02,DG0.00,A9.71,P2.0,R3.0,TR0,TRG0,V0.00,H103.1,HR0.2,HG0.0,M00,***\n";
//   sock.sendTo(msg.c_str(), strlen(msg.c_str()), socket_addr, socket_port);
//   std::cout << "Sent "<<msg<<" via UDP on port "<<socket_port<<" at "<<socket_addr<<std::endl;
//   std::cout << "Any number to continue, enter 1 to quit: " << std::endl;
//   std::cin>>trash;
//   if(trash==1) return 0;


  msg = "#S,T11:09:21.6,LAT36N36.3283,LON121W53.1985,D-0.02,DG0.00,A9.71,P2.0,R3.0,TR0,TRG0,V0.00,H103.1,HR0.2,HG0.0,M00,***\n";
  sock.sendTo(msg.c_str(), strlen(msg.c_str()), socket_addr, socket_port);
  std::cout << "Sent "<<msg<<" via UDP on port "<<socket_port<<" at "<<socket_addr<<std::endl;
  std::cout << "Any number to continue, enter 1 to quit: " << std::endl;
  std::cin>>trash;
  if(trash==1) return 0;
    /////////////////////////////////////////////////////////////////////

  
  msg = "#C,Depth,Altitude,4.0,***\n";
  sock.sendTo(msg.c_str(), strlen(msg.c_str()), socket_addr, socket_port);
  std::cout << "Sent "<<msg<<" via UDP on port "<<socket_port<<" at "<<socket_addr<<std::endl;
  std::cout << "Any number to continue, enter 1 to quit: " << std::endl;
  std::cin>>trash;
  if(trash==1) return 0;

  
  
//   msg = "#C,Depth,Altitude,3.0,***\n";
//   sock.sendTo(msg.c_str(), strlen(msg.c_str()), socket_addr, socket_port);
//   std::cout << "Sent "<<msg<<" via UDP on port "<<socket_port<<" at "<<socket_addr<<std::endl;
//   std::cout << "Any number to continue, enter 1 to quit: " << std::endl;
//   std::cin>>trash;
//   if(trash==1) return 0;
// 
//   msg = "#E,107,Illegal time,***\n";
//   sock.sendTo(msg.c_str(), strlen(msg.c_str()), socket_addr, socket_port);
//   std::cout << "Sent "<<msg<<" via UDP on port "<<socket_port<<" at "<<socket_addr<<std::endl;
//   std::cout << "Any number to continue, enter 1 to quit: " << std::endl;
//   std::cin>>trash;
//   if(trash==1) return 0;
// 
// //  msg = "#f,GPS, ,***\n";
//   // near-ish monterey:
//   msg = "#f,GPS,121W53.1985 36N36.3281,***\n";
//   sock.sendTo(msg.c_str(), strlen(msg.c_str()), socket_addr, socket_port);
//   std::cout << "Sent "<<msg<<" via UDP on port "<<socket_port<<" at "<<socket_addr<<std::endl;
//   std::cout << "Any number to continue, enter 1 to quit: " << std::endl;
//   std::cin>>trash;
//   if(trash==1) return 0;
// 
//   msg = "#f,ATS_USBL,22W22.222 33N33.333,***\n";
//   sock.sendTo(msg.c_str(), strlen(msg.c_str()), socket_addr, socket_port);
//   std::cout << "Sent "<<msg<<" via UDP on port "<<socket_port<<" at "<<socket_addr<<std::endl;
//   std::cout << "Any number to continue, enter 1 to quit: " << std::endl;
//   std::cin>>trash;
//   if(trash==1) return 0;
// 
//   msg = "#Z,D11/29/2010,T7:54:02.3,***\n";
//   sock.sendTo(msg.c_str(), strlen(msg.c_str()), socket_addr, socket_port);
//   std::cout << "Sent "<<msg<<" via UDP on port "<<socket_port<<" at "<<socket_addr<<std::endl;
//   std::cout << "Any number to continue, enter 1 to quit: " << std::endl;
//   std::cin>>trash;
//   if(trash==1) return 0;
// 
//   msg = "#B,C23.4,A55.5,P95.6,***\n";
//   sock.sendTo(msg.c_str(), strlen(msg.c_str()), socket_addr, socket_port);
//   std::cout << "Sent "<<msg<<" via UDP on port "<<socket_port<<" at "<<socket_addr<<std::endl;
  return 0;
}
