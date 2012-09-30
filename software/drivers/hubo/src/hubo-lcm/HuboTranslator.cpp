// HuboTranslator.cpp
// Author: Hunter McClelland
// Fall 2010, MIT CSAIL Marine Robotics Lab
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
// 
// HGM NOTES:
// To add a new message to the supported types is... medium hard.  If you
// remember to do everything, it's not so bad.  This is an attempted list
// for everything you have to do:
// 
// To add LCM-to-RECON message:
// 1) Declare a new LCM Handler Function (in *_main.cpp)
// 2) Create a new subscription to that type/channel (in *_main.cpp)
// 3) Add the unsubscribe function (*_main.cpp)
// 4) Actually code the LCM Handler Function (in *_main.cpp)
// 5) Declare the EncodeAndSend function in HuboTranslator.h
// 6) Code the EncodeAndSend function (in HuboTranslator class)
// 
// To add RECON-to-LCM message:
// 1) Declare a new ParseAndPublish function in HuboTranslator.h
// 2) Code the ParseAndPublish function (in HuboTranslator.cpp)
// 3) Add/Uncomment the switch case in ParseAndPublishRawString
//    (in HuboTranslator.cpp)

#include "HuboTranslator.h"


// **********************************************************************
// **********************************************************************
// Definition of HuboTranslator Class
// **********************************************************************
// **********************************************************************


// Overloaded constructor, used for RECON-to-LCM communications
HuboTranslator::HuboTranslator(const string inputString, double lat_origin,
				 double lon_origin, lcm_t *lcm_ptr){
  ReconTrans_verbose = 1;

  // std::cout<<"running HuboTranslator::HuboTranslator constructor\n";
  lcm = lcm_ptr;

  rawString = inputString;
  // std::cout<<"rawString = "<<rawString<<std::endl;
  // Find Command Indicator (single char after 'hash mark', defined by protocol)
  hashPosition = rawString.find("#");
  // std::cout<<"hashPosition = "<<hashPosition<<std::endl;
  msgIndicator = rawString[hashPosition+1];
  // std::cout<<"msgIndicator = "<<msgIndicator<<std::endl;


  // Below was added by mfallon jan2011:
  //std::cout << lat_origin << " with " << lon_origin << std::endl;
//  m_LatLongConverter.m_OriginLat = lat_origin;
//  m_LatLongConverter.m_OriginLong = lon_origin;
//  m_LatLongConverter.initialise (lat_origin, lon_origin);
//  // m_LatLongConverter.m_Initialized=true;
}

// Overload the HuboTranslator constructor for LCM-to-RECON communications
HuboTranslator::HuboTranslator(const void *lcm_msg_ptr,
                                 UDPSocket *inSock_ptr,
                                 string outsockHost,
                                 unsigned short outsockPort) {
  ReconTrans_verbose = 1;

  lcm_data_ptr = lcm_msg_ptr;
  sock_ptr = inSock_ptr;
  reconHost = outsockHost;
  reconPort = outsockPort;


}


HuboTranslator::~HuboTranslator()
{
  // std::cout<<"destroyed HuboTranslator Instance\n";
  // Clear memory?  I think this is done automatically (HGM)
}

int HuboTranslator::PublishGPSToLocal(double lat_origin, double lon_origin)
{
/*
  mrlcm_gps_to_local_t gps_to_local;
  double local[3] = { 0 }; //all elements 0
  double lat_lon_el_theta[4] = { 0 }; //all elements 0
  double gps_cov[4][4] = {{0}};
  lat_lon_el_theta[0] = lat_origin;
  lat_lon_el_theta[1] = lon_origin;

  gps_to_local.utime = bot_timestamp_now();
  memcpy(gps_to_local.local, local, 3*sizeof(double));
  memcpy(gps_to_local.lat_lon_el_theta, lat_lon_el_theta, 4*sizeof(double));
  memcpy(gps_to_local.gps_cov,gps_cov, 16*sizeof(double));

  if(!mrlcm_gps_to_local_t_publish(lcm, "GPS_TO_LOCAL", &gps_to_local)) return 0;
  else return 1;
*/
return 1;
}

int HuboTranslator::VerifyChecksum(string *msg)
{
  //first remove checksum
  size_t found;
  found = msg->find_last_of("*");
  string msg_substring = msg->substr(0, found + 1);
  //recreate checksum
  CalcAndAppendChecksum(&msg_substring);

  return msg->compare(msg_substring);
}



// *************** BEGIN RECON-TO-LCM MEMBERS *********************
// ****************************************************************


int HuboTranslator::ParseAndPublishRawString()
{
  // Determine and execute appropriate method to Parse and Publish msg
  switch(msgIndicator)
  {
  case 'S':
    if(ParseAndPublishStateMsg(&rawString)) return 1;
    break;

  // case 'A':
  //   if(ParseAndPublishADCPMsg(&rawString)) return 1;
  //   break;

  // case 'c':
  //   if(ParseAndPublishCTDMsg(&rawString)) return 1;
  //   break;

  case 'C':
    if(ParseAndPublishCommAckMsg(&rawString)) return 1;
    break;

  // case 'F':
  //   if(ParseAndPublishFluorometerMsg(&rawString)) return 1;
  //   break;

  case 'B':
    if(ParseAndPublishBatteryMsg(&rawString)) return 1;
    break;

  case 'E':
    if(ParseAndPublishErrMsg(&rawString)) return 1;
    break;

  case 'V':
    if(ParseAndPublishVerReplyMsg(&rawString)) return 1;
    break;

  case 'v':
    if(ParseAndPublishVerRequestMsg(&rawString)) return 1;
    break;

  // case 'a':
  //   if(ParseAndPublishAnchorStatusMsg(&rawString)) return 1;
  //   break;

  // case 'R':
  //   if(ParseAndPublishRouteMsg(&rawString)) return 1;
  //   break;

  case 'f':
    if(ParseAndPublishNavFixMsg(&rawString)) return 1;
    break;

  // case 's':
  //   if(ParseAndPublishSetStatusRespMsg(&rawString)) return 1;
  //   break;

  case 'm':
    if(ParseAndPublishModemMsgTXRespMsg(&rawString)) return 1;
    break;

  case 'Z':
    if(ParseAndPublishDateTimeMsg(&rawString)) return 1;
    break;

  // case 'h':
  //   if(ParseAndPublishHoverAckMsg(&rawString)) return 1;
  //   break;

  // case 't':
  //   if(ParseAndPublishTerminateObjAckMsg(&rawString)) return 1;
  //   break;

  default:
    // Do something to indicate unsupported message type
    if(!(msgIndicator == 'A' || msgIndicator == 'c')){
      std::cout<<"Publishing unknown RECON msg: '"<<msgIndicator<<"'"<<std::endl;
    }
    // std::cerr<<"reconString.cpp: Passing raw message to LCM anyways"<<std::endl;

    if(ParseAndPublishUnsupportedRecon()) return 1;
    else break;
  }
  return 0;  
}

int HuboTranslator::ParseAndPublishStateMsg(string *msg)
{
  //Take the string sReply and parse it to get vehicle state
  int hours;
  int minutes;
  float seconds;
  double lat;
  int lat_deg;
  char lat_dir;
  float lat_min;
  double lon;
  int lon_deg;
  char lon_dir;
  float lon_min;
  float depth;
  float depth_goal;
  float altitude;
  float pitch;
  float roll;
  int thruster_rpm;
  int thruster_rpm_goal;
  float velocity;
  float heading;
  float heading_rate;
  float heading_goal;
  int mode;
  int curr_leg_num;

  cout << "Received this in ParseAndPublishStateMsg:\n";
  cout << *msg << "\n";

  sscanf(msg->c_str(), "#S,T%d:%d:%f,LAT%d%c%f,LON%d%c%f,D%f,DG%f,A%f,P%f,"
    "R%f,TR%d,TRG%d,V%f,H%f,HR%f,HG%f,M%X,l%d,*%*d\n", &hours, &minutes, &seconds,
         &lat_deg, &lat_dir, &lat_min, &lon_deg, &lon_dir, &lon_min, &depth,
         &depth_goal, &altitude, &pitch, &roll, &thruster_rpm,
         &thruster_rpm_goal, &velocity, &heading, &heading_rate, &heading_goal,
         &mode, &curr_leg_num);


  lat = (lat_deg + lat_min / 60.0); //lat in decimal deg
  if (lat_dir == 'S') lat = lat * -1.0;

  lon = (lon_deg + lon_min / 60.0); //lon in decimal deg
  if (lon_dir == 'W') lon = lon * -1.0;

  //Mode
  string state;
  state = "undefined";
  int state_switch = mode & 0x03;
  switch (state_switch)
  {
    case PRE_LAUNCH:
      state = "PRE_LAUNCH";
      break;
    case LAUNCHED_OVERIRDE_ACCEPTED:
      state = "LAUNCHED_OVERIRDE_ACCEPTED";
      break;
    case LAUNCHED_OVERIRDE_IGNORED:
      state = "LAUNCHED_OVERIR DE_IGNORED";
      break; //suspended for GPS fix, serious fault, etc.
    case MISSION_OVER:
      state = "MISSION_OVER";
      break;
    default:
      // Output appropriate error message, state does NOT change
      std::cerr<<"HuboTranslator.cpp: RECON reported unknown state";
  }

/*
  fbn_v2p_reconstate_t remus_statemsg_data;
  remus_statemsg_data.utime = bot_timestamp_now();

  remus_statemsg_data.mission_hours = hours;
  remus_statemsg_data.mission_minutes = minutes;
  remus_statemsg_data.mission_seconds = seconds;
  remus_statemsg_data.lat = lat;
  remus_statemsg_data.lon = lon;
  remus_statemsg_data.depth = depth;
  remus_statemsg_data.depth_goal = depth_goal;
  remus_statemsg_data.altitude = altitude;
  remus_statemsg_data.pitch = pitch;
  remus_statemsg_data.roll = roll;
  remus_statemsg_data.heading = heading;
  remus_statemsg_data.heading_goal = heading_goal;
  remus_statemsg_data.heading_rate = heading_rate;
  remus_statemsg_data.thrust_rpm = thruster_rpm;
  remus_statemsg_data.thrust_rpm_goal = thruster_rpm_goal;
  remus_statemsg_data.velocity = velocity;
  remus_statemsg_data.mode = mode;
  remus_statemsg_data.current_leg_num = curr_leg_num;
*/
  
  
  
  
  
//  double local_xy[2] = {0};
//  m_LatLongConverter.localGrid ( lat,lon,local_xy[1], local_xy[0] );
  //std::cout << local_xy[0] << " gps " <<  local_xy[1] << std::endl;
  
/*
  double x[6]={0};
  x[0] = local_xy[0];
  x[1] = local_xy[1];
  x[2] = -depth;
  x[3] = (90 - heading) * M_PI/180.0 ; // yaw
  x[4] = pitch*M_PI/180.0;// pitch
  x[5] = roll*M_PI/180.0;// roll
  vector<double> collpose(x, x+6);
  int m_Counter=0;
  m_coll_io.lcmsend_collposes(lcm, collpose,"V2P_REMUS_STATE_INS",7002,m_Counter,0,0,0);
  if(publishINSAsState){
    m_coll_io.lcmsend_collposes(lcm, collpose,"V2P_REMUS_STATE",7000,m_Counter,0,0,0);
  }
  */
  

/*
  //////////////////////////////////////////////////////////////////
  // Publish botlcm_pose_t to LCM (deprecated)
  botlcm_pose_t pose;
  pose.utime=remus_statemsg_data.utime;//bot_timestamp_now();
  // X, Y, Z values of state:
  //std::cout << local_xy[0] << " and " << local_xy[1] << std::endl;
  pose.pos[0] = local_xy[0];
  pose.pos[1] = local_xy[1];
  pose.pos[2] = -depth; // depth should be negative
  // Roll, Pitch, Yaw -> Quaterion
  double tempin[3];
  tempin[0] =(double) roll* M_PI/180.0;
  ///////// NBNBNBNBNBNBNBNBNBNBNBNBNBNBNB
  ///////// NBNBNBNBNBNBNBNBNBNBNBNBNBNBNB
  ///////// lib_bot and the lcm-viewer assumes pitch is negative down
  tempin[1] = -(double)pitch* M_PI/180.0;
  ///////// NBNBNBNBNBNBNBNBNBNBNBNBNBNBNB
  ///////// NBNBNBNBNBNBNBNBNBNBNBNBNBNBNB
  tempin[2] =   (90 - heading) * M_PI/180.0; //heading (N+clockwise) to yaw (E+anticlockwise)
  bot_roll_pitch_yaw_to_quat(tempin, pose.orientation);
  // Assign the other values to zero:
  double temp_zero[3] = {0};
  memcpy(pose.vel, temp_zero, 3*sizeof(double));
  memcpy(pose.rotation_rate, temp_zero, 3*sizeof(double));
  memcpy(pose.accel, temp_zero, 3*sizeof(double));
  pose.vel[0] = velocity;   
  botlcm_pose_t_publish (lcm,"POSE_RECONSTATE", &pose);  
  //////////////////////////////////////////////////////////////////
*/

  /*
  // not used for now at least
  // Publish mrlcm_pose_ypr_t to LCM (new version)
  mrlcm_pose_ypr_t pose_ypr;
  pose_ypr.utime=remus_statemsg_data.utime;//bot_timestamp_now();
  // X, Y, Z values of state:
  //std::cout << local_xy[0] << " and " << local_xy[1] << std::endl;
  pose_ypr.pos[0] = local_xy[0];
  pose_ypr.pos[1] = local_xy[1];
  pose_ypr.pos[2] = depth; // depth should be negative
  // Yaw, Pitch, Roll:
  pose_ypr.ypr[0] =   (90 - heading) * M_PI/180.0; //heading (N+clockwise) to yaw (E+anticlockwise)
  pose_ypr.ypr[1] = (double)pitch* M_PI/180.0;
  pose_ypr.ypr[2] =(double) roll* M_PI/180.0;
  // Assign the other values to zero:
  double temp_zero_2[3] = {0};
  memcpy(pose_ypr.vel, temp_zero_2, 3*sizeof(double));
  memcpy(pose_ypr.rotation_rate, temp_zero_2, 3*sizeof(double));
  memcpy(pose_ypr.accel, temp_zero_2, 3*sizeof(double));
  pose_ypr.vel[0] = velocity;   
  mrlcm_pose_ypr_t_publish (lcm,"POSE_YPR_RECONSTATE", &pose_ypr);  
  //////////////////////////////////////////////////////////////////
  */

/*
  if(publishINSAsState){
    if(fbn_v2p_reconstate_t_publish(lcm, "V2P_REMUS_STATE", &remus_statemsg_data)) return -1;
  }

  if(!fbn_v2p_reconstate_t_publish(lcm, "V2P_REMUS_STATE_INS", &remus_statemsg_data)) return 0;
*/

  // if (mode & 0x04)
  // {
  //   m_full_override_enabled = true;
  //   // m_Comms.Notify("REMUS_Full_Overide_Enabled", "TRUE");
  // }
  // else
  // {
  //   m_full_override_enabled = false;
  //   // m_Comms.Notify("REMUS_Full_Overide_Enabled", "FALSE");
  // }

  // if (mode & 0x08)
  // {
  //   m_full_override_active = true;
  //   // m_Comms.Notify("REMUS_Full_Overide_Active", "TRUE");
  // }
  // else
  // {
  //   m_full_override_active = false;
  //   // m_Comms.Notify("REMUS_Full_Overide_Active", "FALSE");
  // }

  // //during testing I found when depth override is enable the flag at 0x40 is set (D. Horner?)
  // if (mode & 0x40)
  // {
  //   m_depth_override_enabled = true;
  //   // m_Comms.Notify("REMUS_Depth_Overide_Enabled", "TRUE");
  // }
  // else
  // {
  //   m_depth_override_enabled = false;
  //   // m_Comms.Notify("REMUS_Depth_Overide_Enabled", "FALSE");
  // }

//  else
return 1;
}


// int HuboTranslator::ParseAndPublishADCPMsg(string *msg)
// {
//   float forward_vel;
//   float starboard_vel;
//   float vertical_vel;
//   float altitude;
//   float heading;
//   float forward_current;
//   float starboard_current;
//   float vertical_current;
//   char ADCP_look_char;

//   sscanf(msg->c_str(), "#A,F%f,S%f,V%f,A%f,H%f,CF%f,CS%f,CU%f,%c,*%*d\n",
//          &forward_vel, &starboard_vel, &vertical_vel, &altitude, &heading,
//          &forward_current, &starboard_current, &vertical_current,
//          &ADCP_look_char);

//   string ADCP_look;
//   ADCP_look = ADCP_look_char;

//   // Publish

//   return 0;
// }

int HuboTranslator::ParseAndPublishCommAckMsg(string *msg)
{
  size_t firstcomma = msg->find_first_of(",");
  size_t firstasterisk = msg->find_first_of("*");
  string ackmsg(msg->substr(firstcomma+1, firstasterisk-firstcomma-2));

  cout << "Received this in ParseAndPublishCommAckMsg:\n";
  cout << *msg << "\n";


  cout << "about to parse CommAckMsg\n";
  cout << msg << " was received\n";
  cout << ackmsg <<" was received2\n";
/*

  fbn_v2p_mvmt_response_t mvmt_resp;
  mvmt_resp.utime = bot_timestamp_now();
  mvmt_resp.ack_msg = const_cast<char*>(ackmsg.c_str());

  if(!fbn_v2p_mvmt_response_t_publish(lcm, "V2P_ACK", &mvmt_resp)) return 0;
  else*/
 return 1;
}

// int HuboTranslator::ParseAndPublishCTDMsg(string *msg)
// {
//   float temperature;
//   float conductivity;
//   float salinity;
//   float depth;
//   float soundspeed;

//   sscanf(msg->c_str(), "#c,T%f,C%f,S%f,D%f,V%f,*%*d\n", &temperature,
//          &conductivity, &salinity, &depth, &soundspeed);

//   // Publish

//   return 0;
// }

// int HuboTranslator::ParseAndPublishFluorometerMsg(string *msg)
// {
//   float gain;
//   float fluorometer;

//   sscanf(msg->c_str(), "#F,G%f,V%f,*%*d\n", &gain, &fluorometer);

//   // Publish

//   return 0;
// }

int HuboTranslator::ParseAndPublishBatteryMsg(string *msg)
{
  float capacity;
  float available;
  float percent_available;

  sscanf(msg->c_str(), "#B,C%f,A%f,P%f,*%*d\n", &capacity, &available,
         &percent_available);
/*
  // Publish
  fbn_v2p_battery_t battery_msg_data;
  battery_msg_data.utime = bot_timestamp_now();

  battery_msg_data.capacity = capacity;
  battery_msg_data.available = available;
  battery_msg_data.percent_avail = percent_available;

  if(!fbn_v2p_battery_t_publish(lcm, "V2P_BATTERY", &battery_msg_data)) return 0;
  else*/
 return 1;
}

int HuboTranslator::ParseAndPublishErrMsg(string *msg)
{
/*
  fbn_v2p_error_t error_msg;
  error_msg.utime = bot_timestamp_now();

  size_t firstcomma = msg->find_first_of(",");
  size_t secondcomma = msg->find_first_of(",",firstcomma+1);
  size_t asterisk = msg->find_first_of("*",secondcomma+1);
  // sscanf(msg->c_str(), "#E,%d,%s,*%*d\n", &(error_msg.errorcode));
  sscanf(msg->c_str(), "#E,%d", &(error_msg.errorcode));
  string errmsg(msg->substr(secondcomma+1,asterisk-secondcomma-2));
  error_msg.errorstring = const_cast<char*>(errmsg.c_str());

  if(!fbn_v2p_error_t_publish(lcm, "V2P_ERROR", &error_msg)) return 0;
  else*/
 return 1;
}

int HuboTranslator::ParseAndPublishVerReplyMsg(string *msg)
{
/*
  fbn_v2p_ver_report_t ver_report;
  ver_report.utime = bot_timestamp_now();

  // sscanf(msg->c_str(), "#V,%s,*%*d\n", ver_report.version_info);
  size_t firstcomma = msg->find_first_of(',');
  size_t lastcomma = msg->find_last_of(',');
  string versioninfo(msg->substr(firstcomma+1,lastcomma-firstcomma-1));
  ver_report.version_info = const_cast<char*>(versioninfo.c_str());

  // char jan[10] = "January  ";
  // char feb[10] = "February ";
  // char mar[10] = "March    ";
  // char apr[10] = "April    ";
  // char may[10] = "May      ";
  // char jun[10] = "June     ";
  // char jul[10] = "July     ";
  // char aug[10] = "August   ";
  // char sep[10] = "September";
  // char oct[10] = "October  ";
  // char nov[10] = "November ";
  // char dec[10] = "December ";

  // if(!strcmp(month,jan)) month_int = 1;
  // else if(!strcmp(month,feb)) month_int = 2;
  // else if(!strcmp(month,mar)) month_int = 3;
  // else if(!strcmp(month,apr)) month_int = 4;
  // else if(!strcmp(month,may)) month_int = 5;
  // else if(!strcmp(month,jun)) month_int = 6;
  // else if(!strcmp(month,jul)) month_int = 7;
  // else if(!strcmp(month,aug)) month_int = 8;
  // else if(!strcmp(month,sep)) month_int = 9;
  // else if(!strcmp(month,oct)) month_int = 10;
  // else if(!strcmp(month,nov)) month_int = 11;
  // else if(!strcmp(month,dec)) month_int = 12;
  // else month_int = -1;

  // // ostringstream build_os;
  // // build_os << year << ' ' << month << ' ' << day << ' ' << hour << ':'
  // //     << minute << ':' << second;
  // // string build(build_os.str());

  // Publish
  if(!fbn_v2p_ver_report_t_publish(lcm, "V2P_VER_REPORT", &ver_report)) return 0;
  else */
return 1;
}

int HuboTranslator::ParseAndPublishVerRequestMsg(string *msg)
{
/*
  fbn_v2p_ver_request_t ver_req;
  ver_req.utime = bot_timestamp_now();
  // no data needs to be processed

  if(!fbn_v2p_ver_request_t_publish(lcm, "V2P_VER_REQUEST", &ver_req)) return 0;
  else
*/
 return 1;

}

// int HuboTranslator::ParseAndPublishRouteMsg(string *msg)
// {
//   // No parse really, just publish

//   return 0;
// }

int HuboTranslator::ParseAndPublishNavFixMsg(string *msg)
{
/*
  if (ReconTrans_verbose){
    std::cout << "New Fix: " << *msg << endl;
  }
  
  double lat;
  int lat_deg;
  char lat_dir;
  float lat_min;
  double lon;
  int lon_deg;
  char lon_dir;
  float lon_min;

  size_t start_pos;
  size_t end_pos;
  //extract the Nav Fix Type
  start_pos = msg->find_first_of(",");
  end_pos = msg->find_first_of(",", start_pos + 1);
  string fix_type(msg->substr(start_pos + 1, end_pos - 1 - start_pos));
  string msg_substring = msg->substr(end_pos + 1);
  // there was a little bug here: mfallon
  sscanf(msg_substring.c_str(), "%d%c%f %d%c%f,*%*d\n", &lon_deg, &lon_dir,
         &lon_min, &lat_deg, &lat_dir, &lat_min);

  lat = (lat_deg + (lat_min / 60.0)); //lat in decimal deg
  if (lat_dir == 'S') lat = lat * -1.0;

  lon = (lon_deg + (lon_min / 60.0)); //lon in decimal deg
  if (lon_dir == 'W') lon = lon * -1.0;

  
  
  fbn_v2p_fix_report_t fix_report;
  fix_report.utime = bot_timestamp_now();

  fix_report.fix_type = const_cast<char*>(fix_type.c_str());
  fix_report.lat = lat;
  fix_report.lon = lon;
  
  
  int m_Counter=0;
  double local_xy[2] = {0};
*/


//  m_LatLongConverter.localGrid ( lat,lon,local_xy[1], local_xy[0] );
  //std::cout << local_xy[0] << " gps " <<  local_xy[1] << std::endl;
    
//  double x[6]={local_xy[0],local_xy[1],0,0,0,0};
//  vector<double> pose(x, x+6);
//  m_coll_io.lcmsend_collposes(lcm, pose,"V2P_FIX_UPDATE",7001,m_Counter,0,0,0);

/*
  mrlcm_system_status_t sys_stat;
  sys_stat.utime = fix_report.utime;
  string tempstr = "RECON";                       // removes a compiler warning (HGM)
  sys_stat.name = const_cast<char*>(tempstr.c_str()); // removes a compiler warning (HGM)
  sys_stat.level = 1; 
  //sprintf(strp.value ,"Fix:  | Local: ");
  char temp[400];
  sprintf(temp,"Fix: %s %f %f | Local: %f %f",fix_report.fix_type,lat,lon,local_xy[0],local_xy[1]);
  sys_stat.value = (char*) temp;
  mrlcm_system_status_t_publish(lcm,"SYSTEM_STATUS",&sys_stat);
*/
  
/*  if (ReconTrans_verbose){
    std::cout << "Local: " << local_xy[0] << ", "<< local_xy[1] << endl;
  }

  if(!fbn_v2p_fix_report_t_publish(lcm, "V2P_FIX_UPDATE", &fix_report)) return 0;
  else 
*/
return 1;
}

// int HuboTranslator::ParseAndPublishSetStatusRespMsg(string *msg)
// {
//   size_t start_pos = msg->find_first_of(",");
//   size_t end_pos = msg->find_last_of(",");
//   string msg_substring = msg->substr(start_pos + 1, end_pos - 1
//       - start_pos);

//   // Publish

//   return 0;
// }

int HuboTranslator::ParseAndPublishModemMsgTXRespMsg(string *msg){

/*
  fbn_v2p_modem_report_t modem_report;
  modem_report.utime = bot_timestamp_now();

  size_t firstcomma = msg->find_first_of(",");
  size_t secondcomma = msg->find_first_of(",",firstcomma+1);
  modem_report.source_address = const_cast<char*>((msg->substr(firstcomma+1,(secondcomma-1)-firstcomma)).c_str());

  size_t thirdcomma = msg->find_first_of(",",secondcomma+1);
  if(thirdcomma == string::npos){
    // Msg takes form #m,MSG_TEXT,*XX\n
    // So fourth, 5th, and 6th comma undefined... skip that part, and null the values
    modem_report.destination_address = 0;
    modem_report.vehicle_address = 0;
    string na = "N/A";
    modem_report.user_cmd = const_cast<char *>(na.c_str());
    modem_report.hex_data_string = const_cast<char *>(na.c_str());
  }
  else {
    size_t fourthcomma = msg->find_first_of(",",thirdcomma+1);
    size_t fifthcomma = msg->find_first_of(",",fourthcomma+1);
    
    string destination_address_str = msg->substr(secondcomma+1,(thirdcomma-1)-secondcomma);
    string vehicle_address_str = msg->substr(thirdcomma+1,(fourthcomma-1)-thirdcomma);
    string user_cmd = "1E (hardcoded,HGM)"; // TODO read the actual value (had problems with msg->substr)
    modem_report.user_cmd = const_cast<char*>(user_cmd.c_str());
    modem_report.hex_data_string = const_cast<char*>((msg->substr(fourthcomma+3,(fifthcomma-1)-(fourthcomma+2))).c_str());

    istringstream ins1, ins2;
    ins1.str(destination_address_str);
    ins1 >> modem_report.destination_address;
    ins2.str(vehicle_address_str);
    ins2 >> modem_report.vehicle_address;
  }
  
  //cout << " modem tx response: " << msg_substring << endl;

  // Publish
  if(!fbn_v2p_modem_report_t_publish(lcm, "V2P_MODEM_REPORT", &modem_report)){
    return 0;
  }
  else
*/
 return 1;
}

// int HuboTranslator::ParseAndPublishRawModemMsg(string *msg)
// {
//   size_t start_pos = msg->find_first_of(",");
//   size_t end_pos = msg->find_last_of(",");
//   string msg_substring = msg->substr(start_pos + 1, end_pos - 1
//       - start_pos);
//   //cout << " modem rx: " << msg_substring << endl;

//   // Publish

//   return 0;
// }

int HuboTranslator::ParseAndPublishDateTimeMsg(string *msg) {
/*
  fbn_v2p_time_report_t time;
  time.utime = bot_timestamp_now();

  struct tm reported_tm_struct;
  // NOTE: any frac of a second is TRUNCATED (2.8 = 2)
  strptime(msg->c_str(), "#Z,D%m/%d/%Y,T%H:%M:%S,*", &reported_tm_struct); // ignore chksum
  time_t reported_time = mktime(&reported_tm_struct);
  
  time.reported_utime = (int)reported_time * 1000000; // libbot utime is in 1/1000000 of a sec
  
  time.year_since_1900 = reported_tm_struct.tm_year;
  time.month_since_jan = reported_tm_struct.tm_mon;
  time.day = reported_tm_struct.tm_mday;
  time.hour = reported_tm_struct.tm_hour;
  time.minute = reported_tm_struct.tm_min;
  time.second = reported_tm_struct.tm_sec;

  if(!fbn_v2p_time_report_t_publish(lcm, "V2P_TIME_REPORT", &time)) return 0;
  else*/
 return 1;
}

int HuboTranslator::ParseAndPublishUnsupportedRecon()
{
/*
  size_t start_pos = rawString.find_first_of(",");
  size_t end_pos = rawString.find_last_of(",");
  string msg_indicator = rawString.substr(start_pos - 1, 1);
  string msg_substring = rawString.substr(start_pos + 1, end_pos - 1 - start_pos);

  fbn_v2p_unsupported_recon_t unsuppmsg;
  unsuppmsg.utime = bot_timestamp_now();

  unsuppmsg.command = const_cast<char*>(msg_indicator.c_str());
  unsuppmsg.data = const_cast<char*>(msg_substring.c_str());

  string channelname = "V2P_UNSUPPORTED_RECON_";
  channelname.append(&msgIndicator);

  if(!fbn_v2p_unsupported_recon_t_publish(lcm, channelname.c_str(), &unsuppmsg)) return 0;
  else
*/
 return 1;
}



// *************** BEGIN LCM-TO-RECON MEMBERS *********************
// ****************************************************************
/*
int HuboTranslator::EncodeAndSendMvmtCommandMsg(const fbn_p2v_mvmt_command_t *motion_ptr){

  // build command, example "#C,Heading,Rate,5.6,*(checksum)\n"
  stm << "#C," << motion_ptr->cmd_major_type << ",";

  if( !strcmp(motion_ptr->cmd_major_type, "Depth") ) {  // major=Depth
    stm << motion_ptr->cmd_minor_type << ",";
    stm.precision(1);

    if( (!strcmp(motion_ptr->cmd_minor_type, "Depth")) || (!strcmp(motion_ptr->cmd_minor_type, "Altitude")) ){
      stm << fixed << motion_ptr->cmd_value << ",*";
    }
    else if( !strcmp(motion_ptr->cmd_minor_type, "Triangle")){
      stm << fixed << motion_ptr->tri_min_depth << ",";
      stm << motion_ptr->tri_alt << ",";
      stm << motion_ptr->tri_rate << ",";
      stm << motion_ptr->tri_max_depth << ",*";
    }
  }

  else if( !strcmp(motion_ptr->cmd_major_type, "Heading") ) { // major=Heading
    stm << motion_ptr->cmd_minor_type << ",";

    if( (!strcmp(motion_ptr->cmd_minor_type, "Goal")) || (!strcmp(motion_ptr->cmd_minor_type, "Rate")) ){
      stm.precision(1);
      stm << fixed << motion_ptr->cmd_value << ",*";
    }
    else if( !strcmp(motion_ptr->cmd_minor_type, "Destination") ){
      // TODO: convert lat and lon to correct format
    }
  }

  else if( !strcmp(motion_ptr->cmd_major_type, "Speed") ) { // major=Speed
    if( (!strcmp(motion_ptr->cmd_minor_type, "knots")) || (!strcmp(motion_ptr->cmd_minor_type, "meters/sec")) ){
      stm.precision(1);
      stm << fixed << motion_ptr->cmd_value << " " << motion_ptr->cmd_minor_type << ",*";
    }
    else if( !strcmp(motion_ptr->cmd_minor_type, "rpm") ){
      stm << fixed << motion_ptr->cmd_value << " " << motion_ptr->cmd_minor_type << ",*";
    }
  }

  recon_msg = stm.str();
  this->CalcAndAppendChecksum(&recon_msg);
  this->SendFormattedReconMsg(&recon_msg);
  
  std::cout << "< " << recon_msg << endl;
  return 0;
}

// **UNSUPPORTED** (HGM 2010/10/28)
// int HuboTranslator::EncodeAndSendQueryMotionMsg(const mrlcm_query_remus_motion_t *query_ptr) {
//   stm << "#Q," << query_ptr->query_major << ",*";
//   recon_msg = stm.str();
//   CalcAndAppendChecksum(&recon_msg);
//   SendFormattedReconMsg(&recon_msg);
//   return 0;
// }

int HuboTranslator::EncodeAndSendVerRequestMsg(const fbn_p2v_ver_request_t *request_ptr) {
  stm << "#v,*";
  recon_msg = stm.str();
  CalcAndAppendChecksum(&recon_msg);
  SendFormattedReconMsg(&recon_msg);
  return 0;
}

int HuboTranslator::EncodeAndSendVerReportMsg(const fbn_p2v_ver_report_t *version_ptr) {
  stm << "#V," << "TBDXXX,*";
  recon_msg = stm.str();
  CalcAndAppendChecksum(&recon_msg);
  SendFormattedReconMsg(&recon_msg);
  return 0;
}

int HuboTranslator::EncodeAndSendSetOverrideMsg(const fbn_p2v_override_t *override_ptr) {
  stm << "#E," << override_ptr->override_code << ",*";
  recon_msg = stm.str();
  std::cout << "<---- RECON OVERRIDE:" << recon_msg << endl;
  CalcAndAppendChecksum(&recon_msg);
  SendFormattedReconMsg(&recon_msg);
  return 0;
}

int HuboTranslator::EncodeAndSendModemCommandMsg(const fbn_p2v_modem_command_t *modem_cmd_ptr) {
  stm << "#p," << modem_cmd_ptr->request_ack << "," << modem_cmd_ptr->overwrite_flag << ","
      << modem_cmd_ptr->nmea0183_string;
  recon_msg = stm.str();
  // DO NOT APPEND CHECKSUM TO THIS MESSAGE! (see RECON protocol)
  SendFormattedReconMsg(&recon_msg);
  return 0;
}

int HuboTranslator::EncodeAndSendModemHexCommandMsg(const fbn_p2v_modem_hex_command_t *modem_hex_cmd_ptr) {
  stm << "#m," << modem_hex_cmd_ptr->request_ack << "," << modem_hex_cmd_ptr->overwrite_flag << ","
      << modem_hex_cmd_ptr->hex_string;
  recon_msg = stm.str();
  CalcAndAppendChecksum(&recon_msg);
  SendFormattedReconMsg(&recon_msg);
  return 0;
}
*/

string HuboTranslator::CalcAndAppendChecksum(string *msg)
{
  int chksum_val = 0;
  for (int i = 0; i < (int)msg->length(); i++)
  {
    chksum_val += msg->at(i);
  }
  chksum_val = 0x0FF & chksum_val;
  ostringstream checksum;
  checksum.flags(ios::right | ios::hex | ios::uppercase);
  checksum.fill('0');
  checksum.width(2);
  checksum << chksum_val;
  *msg += checksum.str();
  return checksum.str();
}


int HuboTranslator::SendFormattedReconMsg(string *msg) {
  // Verify checksum?  Shouldn't have to, would be redundant

  // std::cout<<"DEBUG--Sending: "<<*msg<<" to reconSock\n";
  msg->append("\n");
  sock_ptr->sendTo(msg->c_str(), (int)(msg->length()), reconHost, reconPort);
  return 0;
}
