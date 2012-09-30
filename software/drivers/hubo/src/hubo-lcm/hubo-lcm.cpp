// recon_translator_main.cpp
// Author: Hunter McClelland
// Fall 2010, MIT CSAIL Marine Robotics Lab
// 
// This is the main() function for running mr-HuboTranslator to
// translate between the Hydroid REMUS ReCon protocol and our LCM
// 
// Purpose: The main function sets up the LCM and RECON connections,
// parses the call, then launches 2 threads, one for each direction
// of communication.
//
// The defined class (HuboTranslator) does the grunt work of
// translating between LCM and RECON data.  That class of the code
// draws heavily from iREMUS-4, authored by Doug Horner and Tad Masek.
//
// The code gives hints to the protected RECON communications protocol, please
// use appropriate caution in distributing the source.
// 
// The HuboTranslator class is instantiated once for every message,
// going either direction (LCM-2-RECON or RECON-2-LCM)
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
// To add LCM-to-RECON message:
// 1) Declare a new LCM Handler Function (up top, main)
// 2) Create a new subscription to that type/channel (main)
// 3) Add the unsubscribe function (main)
// 4) Actually code the LCM Handler Function (in the code)
// 5) Declare the EncodeAndSend function in HuboTranslator.h
// 6) Code the EncodeAndSend function (in HuboTranslator class)
// 
// To add RECON-to-LCM message:
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


const int MAX_RECON_MSG_SIZE = 1024;

string reconHost;
string localHost;
unsigned short reconPort;
unsigned short localPort;
UDPSocket reconSock;
UDPSocket localSock;

bool publishINSAsState;

// Setup/connect to LCM socket (file-global)
lcm_t *lcm;

// Delcare each one-way-translation function, to be run in separate threads
void *lcmHandler(const lcm_recv_buf_t *rbuf, const char *chan, void *data, void *castSpec);
void *lcmToReconLooper(void *data);
void *reconToLCMLooper(void *data);

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
    string vehicle ( argv[1] );
    if (vehicle.compare("mitremus") == 0){
      reconHost = "192.168.1.71"; // IP address of remus (mit)
      localHost = "192.168.1.74"; // IP address of payload computer
      reconPort = 23456; // Per RECON Documentation, pdf, page 4, v1.18
      localPort = 23456; // Make sure this port agrees with REMUS.INI file
    }else if (vehicle.compare("npsremus359") == 0){
      reconHost = "192.168.1.91"; // IP address of vehicle computer (nps)
      localHost = "192.168.1.130"; // IP address of payload (mitpayload)
      reconPort = 23456;  // Per RECON Documentation, pdf, page 4, v1.18
      localPort = 23456; // Make sure this port agrees with REMUS.INI file  
    }else if (vehicle.compare("npsremus231") == 0){
      reconHost = "192.168.1.63"; // IP address of vehicle computer (nps)
      localHost = "192.168.1.130"; // IP address of payload (mitpayload)
      reconPort = 23456;  // Per RECON Documentation, pdf, page 4, v1.18
      localPort = 23456; // Make sure this port agrees with REMUS.INI file  
    }else if (vehicle.compare("localhost") == 0){
      reconHost = "localhost"; // IP address of computer which pretends to be vehicle
      localHost = "localhost"; // IP address of computer running HuboTranslator code
      reconPort = 23456;
      localPort = 2010;  
    }else if (vehicle.compare("huntermc") == 0){
      reconHost = "192.168.1.91"; // IP address of nps vehicle
      localHost = "192.168.1.68"; // IP address of computer running code
      reconPort = 23456;
      localPort = 23456;  
    }else if (vehicle.compare("mfallon") == 0){
      reconHost = "192.168.1.91"; // IP address of nps vehicle
      localHost = "192.168.1.198"; // IP address of computer running code
      reconPort = 23456;
      localPort = 23456;  
    }else{
      cout<<"choose a valid option:\n";
      printout_usage(argv[0]);
      return -1;      
    }
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
  pthread_t threadLCMToRecon, threadReconToLCM;
  int rtnValL2RThread, rtnValR2LThread;

  // Begin thread execution
  rtnValL2RThread = pthread_create(&threadLCMToRecon, NULL, lcmToReconLooper, (void*) data);
  rtnValR2LThread = pthread_create(&threadReconToLCM, NULL, reconToLCMLooper, (void*) data);

  pthread_join(threadLCMToRecon, NULL);
  pthread_join(threadReconToLCM, NULL);

  // Report thread failures
  if(rtnValL2RThread) {
    std::cerr<<"ERROR in LCMToReconThread, returned value: "<<rtnValL2RThread<<std::endl;
  }
  if(rtnValR2LThread) {
    std::cerr<<"ERROR in ReconToLCMThread, returned value: "<<rtnValR2LThread<<std::endl;
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
    


void *lcmToReconLooper(void *data){

  // SUBSCRIPTIONS GO HERE
//  fbn_p2v_mvmt_command_t_subscription_t *sub1 =
//    fbn_p2v_mvmt_command_t_subscribe(lcm, "P2V_MVMT_COMMAND", &lcmHandleMvmtCommand, NULL);

/*
  fbn_p2v_mvmt_command_t_subscription_t *sub1 =
    fbn_p2v_mvmt_command_t_subscribe(lcm, "P2V_CMD_.*", &lcmHandleMvmtCommand, NULL);
  fbn_p2v_override_t_subscription_t *sub2 =
    fbn_p2v_override_t_subscribe(lcm, "P2V_OVERRIDE", &lcmHandleOverrideCommand, NULL);
  fbn_p2v_ver_report_t_subscription_t *sub3 =
    fbn_p2v_ver_report_t_subscribe(lcm, "P2V_VER_REPORT", &lcmHandleVerReport, NULL);
  fbn_p2v_ver_request_t_subscription_t *sub4 =
    fbn_p2v_ver_request_t_subscribe(lcm, "P2V_VER_REQUEST", &lcmHandleVerRequest, NULL);
  fbn_p2v_modem_command_t_subscription_t *sub5 =
    fbn_p2v_modem_command_t_subscribe(lcm, "P2V_MODEM_COMMAND", &lcmHandleModemCommand, NULL);
  fbn_p2v_modem_hex_command_t_subscription_t *sub6 =
    fbn_p2v_modem_hex_command_t_subscribe(lcm, "P2V_MODEM_HEX_COMMAND", &lcmHandleModemHexCommand, NULL);
*/

  // Publish GPS Origin message
  HuboTranslator publish_gps_origin("#trash_string",LAT_ORIGIN, LON_ORIGIN,lcm);
  publish_gps_origin.PublishGPSToLocal(LAT_ORIGIN, LON_ORIGIN);
  publish_gps_origin.~HuboTranslator();
  
  std::cout<<"Waiting for LCM posts... "<<"translating to port "<<reconPort<<" at "<<reconHost<<std::endl;
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
} // lcmToReconLooper()


void *reconToLCMLooper(void *data){
  // std::cout<<"Recon to LCM translator thread running\n";
  char recon_msg[MAX_RECON_MSG_SIZE+1];
  string recon_string;
  bool exit_flag = false;
  string rHost = reconHost;

  std::cout<<"Waiting for incoming UDP messages... on port "<<localPort<<" at "<<localHost<<std::endl;
  while(!exit_flag){
    // std::cout<<"Waiting on recv()..."<<std::endl;

    // Listen to socket
    int respStrLen = localSock.recvFrom(recon_msg, (int)MAX_RECON_MSG_SIZE, rHost, reconPort);
    recon_msg[respStrLen] = '\0';

    // std::cout<<"Received message: "<<recon_msg<<" of length = "<<strlen(recon_msg)<<std::endl;
    recon_string.assign(recon_msg);

    // When message received, put it in HuboTranslator Class
    HuboTranslator rString(recon_string,LAT_ORIGIN,LON_ORIGIN, lcm);
    rString.publishINSAsState = publishINSAsState;

    // Publish message via HuboTranslator
    if(rString.ParseAndPublishRawString()){
      std::cerr<<"ParseAndPublishRawString returned nonzero value!\n";
      exit_flag = true;
    }
  }
  std::cout<<"Finished listening to UDP messages\n";

  return NULL;
} // reconToLCMLooper()



// **********************************************************************
// LCM Handler Functions
// **********************************************************************
/*
void lcmHandleMvmtCommand(const lcm_recv_buf_t *rbuf,
                          const char *channel,
                          const fbn_p2v_mvmt_command_t *mvmt_command,
                          void *nulldata) {
  HuboTranslator rData(mvmt_command, &reconSock, reconHost, reconPort);
  rData.publishINSAsState = publishINSAsState;
  // std::cout<<"Got data into MvmtCommand Translator\n";
  if(rData.EncodeAndSendMvmtCommandMsg(mvmt_command)) std::cerr<<"Error occured within EncodeAndSendCommandMsg\n";
}

void lcmHandleOverrideCommand(const lcm_recv_buf_t *rbuf,
                              const char *channel,
                              const fbn_p2v_override_t *override_t,
                              void *nulldata) {
  HuboTranslator rData(override_t, &reconSock, reconHost, reconPort);
  rData.publishINSAsState = publishINSAsState;
  // std::cout<<"Got data into CmdOverride Translator\n";
  if(rData.EncodeAndSendSetOverrideMsg(override_t)) std::cerr<<"Error occured within EncodeAndSendSetOverrideMsg\n";
}

void lcmHandleVerReport(const lcm_recv_buf_t *rbuf,
                        const char *channel,
                        const fbn_p2v_ver_report_t *ver_rpt,
                        void *nulldata) {
  HuboTranslator rData(ver_rpt, &reconSock, reconHost, reconPort);
  rData.publishINSAsState = publishINSAsState;
  // std::cout<<"Got data into VerReport Translator\n";
  if(rData.EncodeAndSendVerReportMsg(ver_rpt)) std::cerr<<"Error occured within EncodeAndSendVerReportMsg\n";
}

void lcmHandleVerRequest(const lcm_recv_buf_t *rbuf,
                         const char *channel,
                         const fbn_p2v_ver_request_t *ver_req,
                         void *nulldata) {
  HuboTranslator rData(ver_req, &reconSock, reconHost, reconPort);
  rData.publishINSAsState = publishINSAsState;
  // std::cout<<"Got data into VerRequest Translator\n";
  if(rData.EncodeAndSendVerRequestMsg(ver_req)) std::cerr<<"Error occured within EncodeAndSendVerRequestMsg\n";
}

void lcmHandleModemCommand(const lcm_recv_buf_t *rbuf,
                           const char *channel,
                           const fbn_p2v_modem_command_t *modem_command,
                           void *nulldata) {
  HuboTranslator rData(modem_command, &reconSock, reconHost, reconPort);
  rData.publishINSAsState = publishINSAsState;
  // std::cout<<"Got data into MvmtCommand Translator\n";
  if(rData.EncodeAndSendModemCommandMsg(modem_command)) std::cerr<<"Error occured within EncodeAndSendModemCommandMsg\n";
}

void lcmHandleModemHexCommand(const lcm_recv_buf_t *rbuf,
                              const char *channel,
                              const fbn_p2v_modem_hex_command_t *modem_hex_command,
                              void *nulldata) {
  HuboTranslator rData(modem_hex_command, &reconSock, reconHost, reconPort);
  rData.publishINSAsState = publishINSAsState;
  // std::cout<<"Got data into MvmtCommand Translator\n";
  if(rData.EncodeAndSendModemHexCommandMsg(modem_hex_command)) std::cerr<<"Error occured within EncodeAndSendModemHexCommandMsg\n";
}*/

// **********************************************************************
// End of LCM Handler Functions
// **********************************************************************
