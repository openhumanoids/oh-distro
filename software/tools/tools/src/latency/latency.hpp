#ifndef AFFORDANCE_UTILS_
#define AFFORDANCE_UTILS_

#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>

class Latency {
  public:
    Latency (int period_);

    void add_from(int64_t js_time, int64_t js_walltime);
    bool add_to(int64_t jc_time, int64_t jc_walltime, std::string message, float &latency, float &new_msgs  );
    
    std::vector < int64_t > js_utime_;
    std::vector < int64_t > js_walltime_;
    
    
    int64_t latency_cumsum_;
    int latency_count_;
    int latency_step_cumsum_; 
    
    int period_;
    float period_f_;
    
    bool verbose_;
    bool verbose_useful_;
};



#endif
