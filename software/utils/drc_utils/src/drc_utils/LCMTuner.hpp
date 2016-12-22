//
// Created by manuelli on 12/22/16.
//

#ifndef DRC_UTILS_LCMTUNER_H
#define DRC_UTILS_LCMTUNER_H

#endif //DRC_UTILS_LCMTUNER_H

#include <sstream>
#include <thread>
#include <chrono>
#include <lcm/lcm-cpp.hpp>
#include <map>
#include <mutex>
#include <memory>


#include <iostream>       // std::cerr
#include <stdexcept>      // std::invalid_argument

// lcm types
#include "drc/lcm_tuner_status_t.hpp"
#include "drc/lcm_tuner_setter_t.hpp"

namespace LCMTuner_namespace{
    class LCMHandler {

    public:

        std::thread ThreadHandle;
        std::shared_ptr<lcm::LCM> LCMHandle;
        bool ShouldStop;

        LCMHandler()
        {
            this->ShouldStop = false;
            this->InitLCM();
        }

        void InitLCM()
        {
            this->LCMHandle = std::shared_ptr<lcm::LCM>(new lcm::LCM);
            if(!this->LCMHandle->good())
            {
                std::cout << "ERROR: lcm is not good()" << std::endl;
            }
        }

        bool WaitForLCM(double timeout)
        {
            int lcmFd = this->LCMHandle->getFileno();

            timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = timeout * 1e6;

            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(lcmFd, &fds);

            int status = select(lcmFd + 1, &fds, 0, 0, &tv);
            if (status == -1 && errno != EINTR)
            {
                printf("select() returned error: %d\n", errno);
            }
            else if (status == -1 && errno == EINTR)
            {
                printf("select() interrupted\n");
            }
            return (status > 0 && FD_ISSET(lcmFd, &fds));
        }

        void ThreadLoopWithSelect()
        {

            std::cout << "ThreadLoopWithSelect " << std::this_thread::get_id() << std::endl;

            while (!this->ShouldStop)
            {
                const double timeoutInSeconds = 0.3;
                bool lcmReady = this->WaitForLCM(timeoutInSeconds);

                if (this->ShouldStop)
                {
                    break;
                }

                if (lcmReady)
                {
                    if (this->LCMHandle->handle() != 0)
                    {
                        printf("lcm->handle() returned non-zero\n");
                        break;
                    }
                }
            }

            std::cout << "ThreadLoopWithSelect ended " << std::this_thread::get_id() << std::endl;

        }

        void ThreadLoop()
        {
            while (!this->ShouldStop)
            {
                if (this->LCMHandle->handle() != 0)
                {
                    printf("lcm->handle() returned non-zero\n");
                    break;
                }
            }
        }

        bool IsRunning()
        {
            return this->ThreadHandle.joinable();
        }

        void Start()
        {
            std::cout << "LCMHandler start... " << std::this_thread::get_id() << std::endl;
            if (this->IsRunning())
            {
                std::cout << "already running lcm thread. " << std::this_thread::get_id() << std::endl;
                return;
            }

            this->ShouldStop = false;
            this->ThreadHandle = std::thread(&LCMHandler::ThreadLoopWithSelect, this);
        }

        void Stop()
        {
            this->ShouldStop = true;
            this->ThreadHandle.join();
        }

    };

    // helper class to store a tuned variable
    class TunedVariable{
    public:
        double& val;
        const double min_val;
        const double max_val;

        TunedVariable(double & val, double min_val, double max_val): val(val), min_val(min_val), max_val(max_val){}
        void set_value(double value){
            // clip the value to the given limits before assigning it
            value = std::min(value, max_val);
            value = std::max(value, min_val);
            val = value;
        }
    };


    class LCMTuner{

    public:
        LCMHandler lcmHandler;
        std::map<std::string, std::unique_ptr<TunedVariable>> tuned_variables;
        bool should_stop = false;
        std::string publish_channel = "LCM_TUNER_STATUS";
        std::thread publish_thread_handle;
        std::string name;
        std::mutex lock;
        int publish_loop_sleep_time_in_seconds = 1;

        // constructor
        LCMTuner(std::string name){
            // record the name + the pid to get something unique
            std::ostringstream ss;
            ss << std::this_thread::get_id();
            std::string idstr = ss.str();
            this->name = name + "_" + idstr;

            this->lcmHandler.LCMHandle->subscribe("LCM_TUNER_SET_VALUES", &LCMTuner::message_handler, this);
        }

        void add_tunable_variable(std::string variable_name, double & val, double min_val, double max_val){
            lock.lock();
            //check if variable with that name already exists, if so return an error
            if (this->tuned_variables.find(variable_name) != this->tuned_variables.end()){
                std::string error_msg = "a tune variable with name " + variable_name + " already exists";
                throw std::invalid_argument(error_msg);
            }
            tuned_variables[variable_name] = std::unique_ptr<TunedVariable>(new TunedVariable(val, min_val, max_val));
            lock.unlock();
        }

        void publish_status(){
//            std::cout << "publishing LCMTuner status " << std::endl;
            lock.lock();
            int num_tuned_vars = this->tuned_variables.size();
            std::vector<std::string> var_names;
            drc::lcm_tuner_status_t msg;
            msg.timestamp = 0;
            msg.tuner_name = this->name;
            msg.num_vars = num_tuned_vars;

//            std::cout << " num tuned vars = " << num_tuned_vars << std::endl;

            for (const auto& val: this->tuned_variables){
                msg.var_names.push_back(val.first);
                msg.current_val.push_back(val.second->val);
                msg.min_val.push_back(val.second->min_val);
                msg.max_val.push_back(val.second->max_val);
            }
            lock.unlock();

//            std::cout << "about to publish msg " << std::endl;

            lcmHandler.LCMHandle->publish(this->publish_channel, &msg);
        }

        void start(){
            // start the thread loop that does publishes and timeouts and stuff
            std::cout << "starting LCMTuner publish loop" << std::endl;
            publish_thread_handle = std::thread(&LCMTuner::status_publish_loop, this);
            this->lcmHandler.Start();
        }


        // stop the lcm handler loop and the status publish loop
        void stop(){
            this->should_stop = true;
            this->lcmHandler.Stop();
            this->publish_thread_handle.join();
        }

        // publishes out the available variables, once per second
        void status_publish_loop(){
            while (!this->should_stop){
                this->publish_status();
//                std::cout << "publishing lcm tuner status" << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(this->publish_loop_sleep_time_in_seconds));
            }
        }

        // handle the message coming from the UI which sets the actual values
        void message_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                             const drc::lcm_tuner_setter_t* msg){


            std::cout << "got an lcm_tuner_setter msg " << std::endl;
            // ignore if the message is not addressed to this tuner
            if (msg->tuner_name != this->name){
                std::cout << " the name in the message doesn't match, returning" << std::endl;
                return;
            }

            lock.lock();
            for(int i = 0; i < msg->num_vars; i++){
                this->set_variable_value(msg->var_names[i], msg->values[i]);
            }
            lock.unlock();
        }

        // make sure you have the lock before calling this
        void set_variable_value(std::string variable_name, double value){
            auto it = this->tuned_variables.find(variable_name);

            if(it == this->tuned_variables.end()) {
                // we don't have this variable in our map so return
                return;
            }else{
                it->second->set_value(value);
            }
        }

    };

}
// define LCM Handler which is a helper class

