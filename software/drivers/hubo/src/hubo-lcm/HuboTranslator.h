// HuboTranslator.h
// Author: Hunter McClelland, Fall 2010
// 
// Purpose: This defines the class to handle translation to/from
// LCM and HUBO, used with HuboTranslator.cpp

#ifndef __HuboTranslator_h__
#define __HuboTranslator_h__

#include <string>
#include <sstream> // ostringstream
#include <iostream> // cout and cerr
#include <ctime> // struct tm, strptime, and mktime
#include <lcm/lcm.h> // for lcm_t type and assoc. funcs
#include <bot_core/bot_core.h> // for bot_timestamp_now()
#include <hubo/PracticalSocket.h>
#include <time.h>

//#include "fbntypes.h"
//#include "mrtypes.h" // I think this is not used... (hgm 2011-jan)
//#include "mrlcm_obj_collection_t.h"

// added by mfallon jan2011 - common latlon converter
//#include "LatLongConverter.h"
//#include "coll_io.h"

class HuboTranslator {
 private:

  std::string hubo_msg;
  ostringstream stm;

  lcm_t *lcm;
  const void *lcm_data_ptr;
  UDPSocket *sock_ptr;
  string huboHost;
  unsigned short huboPort;

  int ParseAndPublishStateMsg(std::string *msg);
  // int ParseAndPublishADCPMsg(std::string *msg);
  int ParseAndPublishCommAckMsg(std::string *msg);
  // int ParseAndPublishCTDMsg(std::string *msg);
  // int ParseAndPublishFluorometerMsg(std::string *msg);
  int ParseAndPublishBatteryMsg(std::string *msg);
  int ParseAndPublishErrMsg(std::string *msg);
  int ParseAndPublishVerReplyMsg(std::string *msg);
  int ParseAndPublishVerRequestMsg(std::string *msg);
  // int ParseAndPublishRouteMsg(std::string *msg);
  int ParseAndPublishNavFixMsg(std::string *msg);
  // int ParseAndPublishSetStatusRespMsg(std::string *msg);
  int ParseAndPublishModemMsgTXRespMsg(std::string *msg);
  // int ParseAndPublishRawModemMsg(std::string *msg);
  int ParseAndPublishDateTimeMsg(std::string *msg);
  int ParseAndPublishUnsupportedHubo(void);

  int HuboTrans_verbose; // initialized in constructor



 public:
  // added by mfallon for gps-to-xy conversion:
  //LatLongConverter m_LatLongConverter;
  // added by mfallon for GPS fix collections:
  //coll_io m_coll_io;

  bool publishINSAsState; // if this flag is true, #S is published as BOTH
  // V2P_REMUS_STATE_INS channel and V2P_REMUS_STATE.  If false, just INS channel

  std::string rawString;

  int PublishGPSToLocal(double, double);
  int VerifyChecksum(std::string *msg);
  std::string CalcAndAppendChecksum(std::string *msg);
  int ParseAndPublishRawString();

  HuboTranslator(const std::string, double , double , lcm_t*);
  HuboTranslator(const void *lcm_msg_ptr, UDPSocket*, std::string, unsigned short);
  ~HuboTranslator();

/*  int EncodeAndSendMvmtCommandMsg(const fbn_p2v_mvmt_command_t *motion_ptr);
  int EncodeAndSendVerRequestMsg(const fbn_p2v_ver_request_t *request_ptr);
  int EncodeAndSendVerReportMsg(const fbn_p2v_ver_report_t *version_ptr);
  int EncodeAndSendSetOverrideMsg(const fbn_p2v_override_t *override_ptr);
  int EncodeAndSendModemCommandMsg(const fbn_p2v_modem_command_t *modem_cmd_ptr);
  int EncodeAndSendModemHexCommandMsg(const fbn_p2v_modem_hex_command_t *modem_hex_cmd_ptr);*/
  int SendFormattedHuboMsg(std::string *msg);
};


enum REMUSMode {
  PRE_LAUNCH,
  LAUNCHED_OVERIRDE_ACCEPTED,
  LAUNCHED_OVERIRDE_IGNORED,
  MISSION_OVER,
};

/* bool m_full_override_enabled; */
/* bool m_full_override_active; */
/* //during testing I found when depth override is */
/* //enable the flag at 0x40 is set */
/* bool m_depth_override_enabled; */
/* bool m_stay_awake; */

#endif
