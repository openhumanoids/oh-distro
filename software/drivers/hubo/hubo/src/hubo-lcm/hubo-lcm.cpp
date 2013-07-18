// hubo_translator_main.cpp
// Author: Hunter McClelland
// Fall 2010, MIT CSAIL Marine Robotics Lab
// 
// This is the main() function for running mr-HuboTranslator to
// translate between the Hydroid REMUS ReCon protocol and our LCM
// 
// Purpose: The main function sets up the LCM and HUBO connections,
// parses the call, then launches 2 threads, one for each direction
// of communication.
//
// The defined class (HuboTranslator) does the grunt work of
// translating between LCM and HUBO data.  That class of the code
// draws heavily from iREMUS-4, authored by Doug Horner and Tad Masek.
//
// The code gives hints to the protected HUBO communications protocol, please
// use appropriate caution in distributing the source.
// 
// The HuboTranslator class is instantiated once for every message,
// going either direction (LCM-2-HUBO or HUBO-2-LCM)
// 
// I chose to use PracticalSocket as a C++ wrapper for my socket
// communication, PracticalSocket is GNU Licensed and comments should
// go to (the author?) Jeff_Donahoo@Baylor.edu
// http://cs.baylor.edu/~donahoo/practical/CSockets/practical/
// 
// HGM NOTES:
// To add a new message to the supported types is... medium hard.  If you
// remember to do everything, it's not so bad.  This is an attempted list
// for everything you have to do:
// 
// To add LCM-to-HUBO message:
// 1) Declare a new LCM Handler Function (up top, main)
// 2) Create a new subscription to that type/channel (main)
// 3) Add the unsubscribe function (main)
// 4) Actually code the LCM Handler Function (in the code)
// 5) Declare the EncodeAndSend function in HuboTranslator.h
// 6) Code the EncodeAndSend function (in HuboTranslator class)
// 
// To add HUBO-to-LCM message:
// 1) Declare a new ParseAndPublish function in HuboTranslator.h
// 2) Code the ParseAndPublish function (in HuboTranslator.cpp)
// 3) Add/Uncomment the switch case in ParseAndPublishRawString
//    (in HuboTranslator.cpp)

#include <pthread.h> // for threading capability
#include <unistd.h> // For getopt()
#include <iostream> // cout and cerr

#include "HuboTranslator.h"

// Constants - should be read from where johnf reads them
//#define LAT_ORIGIN 36.6052
//#define LON_ORIGIN -121.8848
// panama city BAY may 2011:
//#define LAT_ORIGIN 30.1794833
//#define LON_ORIGIN -85.74725
// panama city GULF may 2011:
#define LAT_ORIGIN 30.078450
#define LON_ORIGIN -85.706867


const int MAX_HUBO_MSG_SIZE = 1024;

string huboHost;
string localHost;
unsigned short huboPort;
unsigned short localPort;
UDPSocket huboSock;
UDPSocket localSock;

bool publishINSAsState;

// Setup/connect to LCM socket (file-global)
lcm_t *lcm;

// Delcare each one-way-translation function, to be run in separate threads
void *lcmHandler(const lcm_recv_buf_t *rbuf, const char *chan, void *data, void *castSpec);
void *lcmToHuboLooper(void *data);
void *huboToLCMLooper(void *data);

// Declare LCM Handler Functions
/*
void lcmHandleMvmtCommand    (const lcm_recv_buf_t*, const char*, const fbn_p2v_mvmt_command_t*, void*);
void lcmHandleVerRequest     (const lcm_recv_buf_t*, const char*, const fbn_p2v_ver_request_t*, void*);
void lcmHandleVerReport      (const lcm_recv_buf_t*, const char*, const fbn_p2v_ver_report_t*, void*);
void lcmHandleOverrideCommand(const lcm_recv_buf_t*, const char*, const fbn_p2v_override_t*, void*);
void lcmHandleModemCommand   (const lcm_recv_buf_t*, const char*, const fbn_p2v_modem_command_t*, void*);
void lcmHandleModemHexCommand(const lcm_recv_buf_t*, const char*, const fbn_p2v_modem_hex_command_t*, void*);
*/

void printout_usage(char *progname); // displays usage statement to screen

int main(int argc, char *argv[]){

  if ( argc != 3 ){
    printout_usage(argv[0]);
    return -1;
  }
  else {
    string user ( argv[1] );
    if (user.compare("hongkai") == 0){
      huboHost = "192.168.1.71"; // IP address of hubo
      localHost = "192.168.1.74"; // IP address of hongkai's computer
      huboPort = 1500; 
      localPort = 1500; 
    }else if (user.compare("scott") == 0){
      huboHost = "192.168.1.91"; // IP address of hubo
      localHost = "192.168.1.130"; // IP address of scott (mitpayload)
      huboPort = 1500;  
      localPort = 1500; 
    }else{
      cout<<"choose a valid option:\n";
      printout_usage(argv[0]);
      return -1;      
    }
	//what is stateflag?
    string stateflag(argv[2]);
    if(stateflag.compare("ins") == 0){
      publishINSAsState = true;
    }else if(stateflag.compare("dr") == 0){
      publishINSAsState = false;
    }else{
      cout<<"invalid option for STATEFLAG: '"<<argv[2]<<"'\n";
      printout_usage(argv[0]);
      return -1;
    }
  }

  // Configure Local Socket
  localSock.setLocalAddressAndPort(localHost,localPort);

  // create LCM connection
  lcm = lcm_create(NULL);
  if(!lcm) {
    std::cerr<<"ERROR: Failed to create LCM connection";
    return 1; 
  }

  // Setup threads
  void *data = NULL;
  pthread_t threadLCMToHubo, threadHuboToLCM;
  int rtnValL2HThread, rtnValH2LThread;

  // Begin thread execution
  rtnValL2HThread = pthread_create(&threadLCMToHubo, NULL, lcmToHuboLooper, (void*) data);
  rtnValH2LThread = pthread_create(&threadHuboToLCM, NULL, huboToLCMLooper, (void*) data);

  pthread_join(threadLCMToHubo, NULL);
  pthread_join(threadHuboToLCM, NULL);

  // Report thread failures
  if(rtnValL2HThread) {
    std::cerr<<"ERROR in LCMToHuboThread, returned value: "<<rtnValL2HThread<<std::endl;
  }
  if(rtnValH2LThread) {
    std::cerr<<"ERROR in HuboToLCMThread, returned value: "<<rtnValH2LThread<<std::endl;
  }

  // Destroy LCM, although this may not happen because ctrl-c exit
  lcm_destroy(lcm);

  return 0;
} // int main()

void printout_usage(char *progname){
  cout<<"usage: "<< progname <<" IPCONFIG STATEFLAG\n"
      <<"(IPCONFIG options: mitremus npsremus359 npsremus231 localhost huntermc mfallon)\n"
      <<"(STATEFLAG options: ins dr)\n";
}
    


void *lcmToHuboLooper(void *data){

  // Publish GPS Origin message
  HuboTranslator publish_gps_origin("#trash_string",LAT_ORIGIN, LON_ORIGIN,lcm);
  publish_gps_origin.~HuboTranslator();
  
  std::cout<<"Waiting for LCM posts... "<<"translating to port "<<huboPort<<" at "<<huboHost<<std::endl;
  while(true){ // Should this ever stop? -HGM
    // std::cout<<"calling lcm_handle()...\n";
    lcm_handle(lcm);
  }

  // May never get here
  std::cout<<"Finished listening to LCM posts\n";
  // Unsubscribe
/*
  fbn_p2v_mvmt_command_t_unsubscribe(lcm, sub1);
  fbn_p2v_override_t_unsubscribe(lcm, sub2);
  fbn_p2v_ver_report_t_unsubscribe(lcm, sub3);
  fbn_p2v_ver_request_t_unsubscribe(lcm, sub4);
  fbn_p2v_modem_command_t_unsubscribe(lcm, sub5);
  fbn_p2v_modem_hex_command_t_unsubscribe(lcm, sub6);
*/
    
  return data; // data is never used though
} // lcmToHuboLooper()


void *huboToLCMLooper(void *data){
  // std::cout<<"Hubo to LCM translator thread running\n";
  char hubo_msg[MAX_HUBO_MSG_SIZE+1];
  string hubo_string;
  bool exit_flag = false;
  string rHost = huboHost;

  std::cout<<"Waiting for incoming UDP messages... on port "<<localPort<<" at "<<localHost<<std::endl;
  while(!exit_flag){
    // std::cout<<"Waiting on recv()..."<<std::endl;

    // Listen to socket
    int respStrLen = localSock.recvFrom(hubo_msg, (int)MAX_HUBO_MSG_SIZE, rHost, huboPort);
    hubo_msg[respStrLen] = '\0';

    // std::cout<<"Received message: "<<hubo_msg<<" of length = "<<strlen(hubo_msg)<<std::endl;
    hubo_string.assign(hubo_msg);

    // When message received, put it in HuboTranslator Class
    HuboTranslator rString(hubo_string,LAT_ORIGIN,LON_ORIGIN, lcm);
    rString.publishINSAsState = publishINSAsState;

    // Publish message via HuboTranslator
    if(rString.ParseAndPublishRawString()){
      std::cerr<<"ParseAndPublishRawString returned nonzero value!\n";
      exit_flag = true;
    }
  }
  std::cout<<"Finished listening to UDP messages\n";

  return NULL;
} // huboToLCMLooper()



// **********************************************************************
// LCM Handler Functions
// **********************************************************************
/*
void lcmHandleMvmtCommand(const lcm_recv_buf_t *rbuf,
                          const char *channel,
                          const fbn_p2v_mvmt_command_t *mvmt_command,
                          void *nulldata) {
  HuboTranslator rData(mvmt_command, &huboSock, huboHost, huboPort);
  rData.publishINSAsState = publishINSAsState;
  // std::cout<<"Got data into MvmtCommand Translator\n";
  if(rData.EncodeAndSendMvmtCommandMsg(mvmt_command)) std::cerr<<"Error occured within EncodeAndSendCommandMsg\n";
}

void lcmHandleOverrideCommand(const lcm_recv_buf_t *rbuf,
                              const char *channel,
                              const fbn_p2v_override_t *override_t,
                              void *nulldata) {
  HuboTranslator rData(override_t, &huboSock, huboHost, huboPort);
  rData.publishINSAsState = publishINSAsState;
  // std::cout<<"Got data into CmdOverride Translator\n";
  if(rData.EncodeAndSendSetOverrideMsg(override_t)) std::cerr<<"Error occured within EncodeAndSendSetOverrideMsg\n";
}

void lcmHandleVerReport(const lcm_recv_buf_t *rbuf,
                        const char *channel,
                        const fbn_p2v_ver_report_t *ver_rpt,
                        void *nulldata) {
  HuboTranslator rData(ver_rpt, &huboSock, huboHost, huboPort);
  rData.publishINSAsState = publishINSAsState;
  // std::cout<<"Got data into VerReport Translator\n";
  if(rData.EncodeAndSendVerReportMsg(ver_rpt)) std::cerr<<"Error occured within EncodeAndSendVerReportMsg\n";
}

void lcmHandleVerRequest(const lcm_recv_buf_t *rbuf,
                         const char *channel,
                         const fbn_p2v_ver_request_t *ver_req,
                         void *nulldata) {
  HuboTranslator rData(ver_req, &huboSock, huboHost, huboPort);
  rData.publishINSAsState = publishINSAsState;
  // std::cout<<"Got data into VerRequest Translator\n";
  if(rData.EncodeAndSendVerRequestMsg(ver_req)) std::cerr<<"Error occured within EncodeAndSendVerRequestMsg\n";
}

void lcmHandleModemCommand(const lcm_recv_buf_t *rbuf,
                           const char *channel,
                           const fbn_p2v_modem_command_t *modem_command,
                           void *nulldata) {
  HuboTranslator rData(modem_command, &huboSock, huboHost, huboPort);
  rData.publishINSAsState = publishINSAsState;
  // std::cout<<"Got data into MvmtCommand Translator\n";
  if(rData.EncodeAndSendModemCommandMsg(modem_command)) std::cerr<<"Error occured within EncodeAndSendModemCommandMsg\n";
}

void lcmHandleModemHexCommand(const lcm_recv_buf_t *rbuf,
                              const char *channel,
                              const fbn_p2v_modem_hex_command_t *modem_hex_command,
                              void *nulldata) {
  HuboTranslator rData(modem_hex_command, &huboSock, huboHost, huboPort);
  rData.publishINSAsState = publishINSAsState;
  // std::cout<<"Got data into MvmtCommand Translator\n";
  if(rData.EncodeAndSendModemHexCommandMsg(modem_hex_command)) std::cerr<<"Error occured within EncodeAndSendModemHexCommandMsg\n";
}*/

// **********************************************************************
// End of LCM Handler Functions
// **********************************************************************
