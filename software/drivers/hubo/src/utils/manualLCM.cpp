//
// LCM example program.
//

#include <string>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <sstream>

using namespace std;

#include <lcm/lcm.h>
//#include "../../lcmtypes/fbntypes.h"


// use a stringstream to separate the fields out of the line
int split_string(string msg, vector <string> &record){
  stringstream ss( msg );
  string field;
  while (getline( ss, field, ',' ))
    {
    //  std::cout << field << endl;
    // add the newly-converted field to the end of the record
    record.push_back( field );
    }
}

static void send_fbn_p2v_mvmt_command_t(lcm_t * lcm,string msg)
{
  vector <string> record;
  split_string(msg,record);
 
/*
  fbn_p2v_mvmt_command_t testdata;
  testdata.cmd_major_type =(char*) record[2].c_str();
  testdata.cmd_minor_type =(char*) record[3].c_str();
  sscanf( record[1].c_str(), "%lld",&testdata.utime); // this seems to work, but im not sure about it...
  sscanf( record[4].c_str(), "%f",&testdata.cmd_value);
  sscanf( record[5].c_str(), "%f",&testdata.tri_min_depth);
  sscanf( record[6].c_str(), "%f",&testdata.tri_alt);
  sscanf( record[7].c_str(), "%f",&testdata.tri_rate);
  sscanf( record[8].c_str(), "%f",&testdata.tri_max_depth);
  sscanf( record[9].c_str(), "%lf",&testdata.lat);
  sscanf( record[10].c_str(), "%lf",&testdata.lon);
  fbn_p2v_mvmt_command_t_publish (lcm, "P2V_MVMT_COMMAND", &testdata);
*/
  record.clear();
}

static void send_fbn_p2v_ver_request_t(lcm_t * lcm,string msg)
{
  vector <string> record;
  split_string(msg,record);

/*
  fbn_p2v_ver_request_t testdata;
  sscanf( record[1].c_str(), "%lld",&testdata.utime); // this seems to work, but im not sure about it...  
  fbn_p2v_ver_request_t_publish (lcm, "P2V_VER_REQUEST", &testdata);
*/
}

static void send_fbn_p2v_ver_report_t(lcm_t * lcm,string msg)
{
  vector <string> record;
  split_string(msg,record);

/*
  fbn_p2v_ver_report_t testdata;
  sscanf( record[1].c_str(), "%lld",&testdata.utime); // this seems to work, but im not sure about it...  
  testdata.version_info =(char*) record[2].c_str();
  fbn_p2v_ver_report_t_publish (lcm, "P2V_VER_REPORT", &testdata);
*/
}

static void send_fbn_p2v_override_t(lcm_t * lcm,string msg)
{
  vector <string> record;
  split_string(msg,record);

/*
  fbn_p2v_override_t testdata;
  sscanf( record[1].c_str(), "%lld",&testdata.utime); // this seems to work, but im not sure about it...  
  sscanf( record[2].c_str(), "%d",&testdata.override_code);
  fbn_p2v_override_t_publish (lcm, "P2V_OVERRIDE", &testdata);
*/
}

int main (int argc, char ** argv)
{
  int trash;
  lcm_t * lcm;

  lcm = lcm_create (NULL);
  if (!lcm) return 1;

  ifstream ifs( "canned_mission.txt" );
  string msg;
  while( getline( ifs, msg ) ){
    std::cout << msg << endl;
    
    if (msg.find("P2V_VER_REQUEST")!=string::npos){
      send_fbn_p2v_ver_request_t(lcm,msg);
    }else if (msg.find("P2V_OVERRIDE")!=string::npos){
      send_fbn_p2v_override_t(lcm,msg);
    }else if (msg.find("P2V_VER_REPORT")!=string::npos){
      send_fbn_p2v_ver_report_t(lcm,msg);
    }else if (msg.find("P2V_MVMT_COMMAND")!=string::npos){
      send_fbn_p2v_mvmt_command_t(lcm,msg);
    }
  }

  lcm_destroy (lcm);
  printf("done sending messages, LCM destroyed\n");
  return 0;
}
