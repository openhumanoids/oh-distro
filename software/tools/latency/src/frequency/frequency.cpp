#include <iostream>     // std::cout
#include <algorithm>    // std::find
#include <iomanip>

#include <sstream>      // std::stringstream, std::stringbuf

#include "frequency.hpp"

#include <path_util/path_util.h>
#include <bot_param/param_client.h>
#include <bot_param/param_util.h>


using namespace std;

Frequency::Frequency(float window_, bool verbose_, std::string system_): window_(window_),verbose_(verbose_), system_(system_) {
  sample_period_ = (int64_t) round(window_* 1E6) ; // 0.1sec
  std::cout << sample_period_ << " is sample_period_\n";
  last_analysis_utime_ = 0;
  text_counter_ =0;
}



void Frequency::readChannels(){
  BotParam * bot_param;  
  std::string config_path = std::string(getConfigPath()) + "/drc_robot.cfg";
  std::cout << config_path <<"\n";
  bot_param = bot_param_new_from_file(config_path.c_str());
  if (bot_param == NULL) {
    std::cerr << "Couldn't get bot param from file " << config_path << std::endl;
    exit(-1);
  }

  char channels_key[512];  
  sprintf(channels_key, "frequency_tool.%s.channels", system_.c_str());
  char **fnames = bot_param_get_str_array_alloc(bot_param, channels_key);
  std::vector <string> channels;
  for (int j = 0; fnames && fnames[j]!=NULL; j++) {
    channels.push_back(fnames[j]);
    //std::cout << channels[j] << "\n";
  } 

  char channels_short_key[512];  
  sprintf(channels_short_key, "frequency_tool.%s.channels_short", system_.c_str());
  char **fnames_short = bot_param_get_str_array_alloc(bot_param, channels_short_key);
  std::vector <string> channels_short;
  for (int j = 0; fnames_short && fnames_short[j]!=NULL; j++) {
    channels_short.push_back(fnames_short[j]);
    //std::cout << channels_short[j] << "\n";
  } 

  
  for (size_t i=0; i < channels.size() ; i++){
    std::cout << channels[i] << " and " << channels_short[i] << "\n"; 
    addCounter(channels[i] , channels_short[i]);
  }
}

bool Frequency::setUtime(int64_t current_utime, std::vector<float>  &freqs){
  bool calulated_now =false;
  if (  (current_utime - last_analysis_utime_)  >=  sample_period_){
    calulated_now =true;
    
    std::map<std::string, Counter>::iterator it;
    for(it=counter_.begin(); it!=counter_.end(); it++) {
      float rate = (float) it->second.count*1E6/sample_period_ ;
      freqs.push_back(  rate   );
    }
    
    if (verbose_){
      std::stringstream ss;
      // Channels:
      if (text_counter_%10 ==0){
	for(it=counter_.begin(); it!=counter_.end(); it++) { ss  << it->second.chan_short << " | " ; }
	std::cout << ss.str() << "\n"; 
      }
      text_counter_++;
      
      // Counter
      //ss.str("");
      //for(it=counter_.begin(); it!=counter_.end(); it++) {
 	//char buffer [50];      sprintf (buffer, "%04d", it->second.count);      ss << std::string(buffer) << " | " ; 
      //}
      //std::cout << ss.str() << "counter ["<< window_ << "]\n";
      
      // Hz
      ss.str("");
      for(size_t i=0; i< freqs.size(); i++) {
	char buffer [50];      sprintf (buffer, "%04.0f", freqs[i]);      ss << std::string(buffer) << " | " ; 
      }
      std::cout << ss.str() << "rate [Hz] | "<< window_ <<" sec\n"; 
    }
    
    last_analysis_utime_ = current_utime;
    for(it=counter_.begin(); it!=counter_.end(); it++) {
      it->second.count =0;
    }
  }
  
  return calulated_now;
}

void Frequency::incrementCounter(std::string chan){
  std::map<std::string, Counter>::iterator it = counter_.find(chan);
  if(it != counter_.end()){
    counter_.find(chan)->second.count++;
  }
}


