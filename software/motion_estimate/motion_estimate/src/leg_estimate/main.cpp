#include <ConciseArgs>
#include "LegOdoWrapper.hpp"



int 
main(int argc, char ** argv){
  CommandLineConfig cl_cfg;
  cl_cfg.urdf_file = "";
  cl_cfg.param_file = "";
  cl_cfg.in_log_name = "";
  cl_cfg.out_log_name = "";
  cl_cfg.read_lcmlog = false;
  cl_cfg.begin_timestamp = -1;
  cl_cfg.end_timestamp = -1;
  cl_cfg.republish_incoming = false;
  cl_cfg.processing_rate = 1;
  
  ConciseArgs opt(argc, (char**)argv);
  opt.add(cl_cfg.urdf_file, "U", "urdf_file","urdf_file");
  opt.add(cl_cfg.param_file, "P", "param_file","param_file");
  opt.add(cl_cfg.in_log_name, "L", "in_log_name","in_log_name");
  opt.add(cl_cfg.out_log_name, "l", "out_log_name","out_log_name");
  opt.add(cl_cfg.begin_timestamp, "bt", "begin_timestamp","Run estimation from this timestamp");
  opt.add(cl_cfg.end_timestamp, "et", "end_timestamp","End estimation at this timestamp");  
  opt.add(cl_cfg.republish_incoming, "r", "republish_incoming","Republish Incoming Messages");  
  opt.add(cl_cfg.processing_rate, "pr", "processing_rate","Processing Rate from a log [0=ASAP, 1=realtime]");    
  opt.parse();
  
  std::stringstream lcmurl_in;
  if (cl_cfg.in_log_name == "" ){
    lcmurl_in << "";
  }else{
    cl_cfg.read_lcmlog = true;
    lcmurl_in << "file://" << cl_cfg.in_log_name << "?speed=" << cl_cfg.processing_rate;// + "&start_timestamp=";// + begin_timestamp;
  }
  boost::shared_ptr<lcm::LCM> lcm_subscribe(new lcm::LCM(lcmurl_in.str()) );
  
  std::stringstream lcmurl_out;  
  if (cl_cfg.out_log_name == "" ) {
    lcmurl_out << ""; // mfallon publish back to lcm if run from log
  }else{
    printf("publishing into log file: %s\n", cl_cfg.out_log_name.c_str());
    lcmurl_out << "file://" << cl_cfg.out_log_name << "?mode=w";
  } 
  boost::shared_ptr<lcm::LCM> lcm_publish(new lcm::LCM(lcmurl_out.str()) );

  if(!lcm_subscribe->good())
    return 1;
  if(!lcm_publish->good())
    return 1;

  if (lcm_publish != lcm_subscribe && cl_cfg.republish_incoming) {
    cl_cfg.republish_incoming = true;
  }else{
    // Over-rule if requesting republish
    cl_cfg.republish_incoming = false;
  }



  // launch a stand-alone app to do leg odometry
  App app(lcm_subscribe, lcm_publish, cl_cfg);
  while(0 == lcm_subscribe->handle());
  return 0;
}
