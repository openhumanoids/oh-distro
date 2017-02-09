//
// Created by manuelli on 11/21/16.
//

#include <chrono>

namespace plan_eval_utils{

  class SimpleTimer{

  public:
    std::chrono::time_point<std::chrono::system_clock> start_time_;
    SimpleTimer(){};
    void Start(){
      start_time_ = std::chrono::system_clock::now();
    }

    std::chrono::milliseconds Elapsed(){
      std::chrono::milliseconds elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time_);

      return elapsed_time;
    }

    void PrintElapsedTime(){
      std::chrono::milliseconds elapsed_time = this->Elapsed();
      std::cout << "elapsed time in ms = " << elapsed_time.count() << std::endl;
    }
  };
}