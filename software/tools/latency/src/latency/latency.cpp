#include <iostream>     // std::cout
#include <algorithm>    // std::find
#include "latency.hpp"


Latency::Latency() {
  period_ = 200;
  period_f_ = 200.0;
  
  latency_cumsum_ =0;
  latency_count_ =0;
  latency_step_cumsum_=0; 
      
}

void Latency::add_from(int64_t js_time, int64_t js_walltime){
  js_utime_.push_back(js_time);
  js_walltime_.push_back(js_walltime);
}

void Latency::add_to(int64_t jc_utime, int64_t jc_walltime, std::string message ){
  std::vector<std::int64_t>::iterator it = std::find( js_utime_.begin(), js_utime_.end(), jc_utime );
  if( it == js_utime_.end() ){
    return;
  }
  
  int idx = std::distance( js_utime_.begin(), it );
  int step_latency = js_utime_.size() - 1 - idx;
  int64_t  elapsed_walltime = jc_walltime - js_walltime_[idx]  ;
  
  js_walltime_.clear();
  js_utime_.clear();
  
  latency_cumsum_ += elapsed_walltime;
  latency_count_++;
  latency_step_cumsum_ += step_latency;
  
  if (latency_count_ % period_  == 0 ){
    std::cout << message << ": " << jc_utime 
              << " | " << ( latency_step_cumsum_ / period_f_ ) 
              << " newer msgs | " << ( latency_cumsum_ *1E-6/ period_f_)
              << " sec delay [" << latency_count_  << "]\n";
    
    latency_count_ =0;
    latency_cumsum_ = 0;
    latency_step_cumsum_ = 0;
  } 
}