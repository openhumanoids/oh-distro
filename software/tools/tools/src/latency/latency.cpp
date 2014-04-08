#include <iostream>     // std::cout
#include <algorithm>    // std::find
#include "latency.hpp"


Latency::Latency() {
  period_ = 100;
  period_f_ = 100.0;
  
  latency_cumsum_ =0;
  latency_count_ =0;
  latency_step_cumsum_=0; 
      
  verbose_=false;
  
  verbose_useful_ = false; // to print out useful verbosity 
}

void Latency::add_from(int64_t js_time, int64_t js_walltime){
  js_utime_.push_back(js_time);
  js_walltime_.push_back(js_walltime);
}

bool Latency::add_to(int64_t jc_utime, int64_t jc_walltime, std::string message, float &latency, float &new_msgs ){

  if (verbose_){
    std::cout << "marker: " <<  jc_utime << " and " << jc_walltime << "\n";  
  }  
  
  std::vector<std::int64_t>::iterator it = std::find( js_utime_.begin(), js_utime_.end(), jc_utime );
  if( it == js_utime_.end() ){
    return false;
  }
  
  int idx = std::distance( js_utime_.begin(), it );
  int step_latency = js_utime_.size() - 1 - idx;
  int64_t  elapsed_walltime = jc_walltime - js_walltime_[idx]  ;

  if (verbose_){
    for (size_t i =0; i <js_utime_.size() ; i++){
      std::cout << i << ": "<< js_utime_[i] << "\n";
    }
    for (size_t i =0; i <js_walltime_.size() ; i++){
      std::cout << i << ": "<< js_walltime_[i] << " wall\n";
    }
    std::cout << js_utime_.size() << " utimes|walltimes " << js_walltime_.size() <<  "\n";

    
    for (size_t i =0; i <js_utime_.size() ; i++){
      std::cout << i << ": "<< js_utime_[i] << " and " << js_walltime_[i] << "\n";
    }
    std::cout << elapsed_walltime << " elapsed_walltime " << step_latency << " step_latency\n\n";
  }
  
  js_walltime_.clear();
  js_utime_.clear();
  
  latency_cumsum_ += elapsed_walltime;
  latency_count_++;
  latency_step_cumsum_ += step_latency;
  
  if (latency_count_ % period_  == 0 ){
    
    new_msgs = ( float(latency_step_cumsum_) / float(latency_count_) ) ;
    latency = ( float(latency_cumsum_) *1E-3/ float(latency_count_));
    
    if (verbose_useful_)
      std::cout << message << ": " << jc_utime 
              << " | " << new_msgs
              << " newer msgs | " << latency
              << " msec delay [" << latency_count_  << "]\n";
    
    latency_count_ =0;
    latency_cumsum_ = 0;
    latency_step_cumsum_ = 0;
    
    return true;
  } 
  
  return false;
}