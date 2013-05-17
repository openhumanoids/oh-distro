#ifndef AFFORDANCE_UTILS_
#define AFFORDANCE_UTILS_

#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <map>

struct Counter {
  int count;
  std::string chan_short;
} ;

class Frequency {
  public:
    Frequency (float window, bool verbose_, std::string system_);

    void readChannels();
    
    bool setUtime(int64_t current_utime, std::vector<float> &freqs);

    void incrementCounter(std::string chan);
    
    void addCounter(std::string chan, std::string chan_short){
      Counter counter;
      counter.count=0;
      counter.chan_short =chan_short;
      counter_[chan]=counter;  
    }
    
    float window_;
    int64_t sample_period_;
    std::string system_;
    
    int text_counter_;
    
    std::vector < int64_t > js_utime_;
    std::vector < int64_t > js_walltime_;
    
    int64_t last_analysis_utime_;
    
    std::map<std::string,Counter> counter_;
    
    bool verbose_;
    
    
    std::vector< std::string> getChannels(){
      std::vector< std::string> chans;
      std::map<std::string, Counter>::iterator it;
      for(it=counter_.begin(); it!=counter_.end(); it++) {
	chans.push_back( it->first );
      }
      return chans;
    }
};



#endif
