#ifndef LATENCY_HPP_
#define LATENCY_HPP_

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

class Latency {
  public:
    Latency (int period_);

    void add_from(int64_t js_time, int64_t js_walltime, int64_t seq_id);
    bool add_to(int64_t jc_time, int64_t jc_walltime, int64_t seq_id, std::string message, float &latency, float &new_msgs  );
    
    std::vector < int64_t > js_utime_;
    std::vector < int64_t > js_walltime_;
    
    
    int64_t latency_cumsum_;
    int latency_count_;
    int latency_step_cumsum_; 
    
    int period_;
    float period_f_;
    
    bool verbose_;
    bool verbose_useful_;

    std::string tic_filename_;
    std::ofstream tic_file_;
    bool write_tics_;
    void setTicOutputFile( std::string tic_filename_in ){
      write_tics_ = true;
      tic_filename_ = tic_filename_in;

      tic_file_.open (tic_filename_in);
    }

    void closeTicOutputFile(){
      tic_file_.close();
    }

    int64_t last_seq_id_;
};



#endif
