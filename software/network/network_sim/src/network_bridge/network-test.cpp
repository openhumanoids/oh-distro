#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <goby/common/time.h>

void in_robot_state_minimal(const lcm::ReceiveBuffer *rbuf,
                            const std::string &channel,
                            const drc::minimal_robot_state_t *msg,
                            void*)
{
    std::cout << "Received with time: " << msg->utime << std::endl;
}
    


int main (int argc, char* argv[])
{
    char* lcm_url_robot;
    lcm_url_robot = getenv ( "LCM_URL_DRC_ROBOT" );
    
    if (lcm_url_robot){
        printf ("The lcm_url_robot is: %s\n",lcm_url_robot);
    }
    else
    {
        std::cout << "LCM_URL_DRC_ROBOT environment variable has not been set." << std::endl;
        exit(EXIT_FAILURE);
    }

    char* lcm_url_base;
    lcm_url_base = getenv ( "LCM_URL_DRC_BASE" );
    if (lcm_url_base){
        printf ("The lcm_url_base is: %s\n",lcm_url_base);      
    }
    else
    {
        std::cout << "LCM_URL_DRC_BASE environment variable has not been set." << std::endl;
        exit(EXIT_FAILURE);
    }


    boost::shared_ptr<lcm::LCM> robot_lcm(new lcm::LCM(lcm_url_robot));
    if(!robot_lcm->good()){
        std::cerr <<"ERROR: lcm is not good()" <<std::endl;
    }
    boost::shared_ptr<lcm::LCM> base_lcm(new lcm::LCM(lcm_url_base));
    if(!base_lcm->good()){
        std::cerr <<"ERROR: lcm is not good()" <<std::endl;
    }

    // subscribe the base
    base_lcm->subscribeFunction<drc::minimal_robot_state_t>("EST_ROBOT_STATE_MINIMAL", &in_robot_state_minimal, static_cast<void*>(0));    

    while (1)
    {
        int base_fd = base_lcm->getFileno();
        int robot_fd = robot_lcm->getFileno();

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(base_fd, &fds);
        FD_SET(robot_fd, &fds);
        
        // wait a limited amount of time for an incoming message
        struct timeval timeout = { 
            1, // seconds
            0  // microseconds
        };

        int max_fd = base_fd > robot_fd ? base_fd : robot_fd;
        int status = select(max_fd + 1, &fds, 0, 0, &timeout);

        static int counter = 0;
        
        if(0 == status)
        {
            drc::minimal_robot_state_t msg;
            msg.utime = goby::common::goby_time<goby::uint64>();
            msg.origin_position = drc::position_3d_t();
            msg.num_joints = 0;
            robot_lcm->publish("EST_ROBOT_STATE_MINIMAL", &msg);
            std::cout << "Sent with time: " << msg.utime << std::endl;            

            counter++;

            if(counter % 20 == 0)
            {
                // reset stats
                drc::utime_t t;
                t.utime = goby::common::goby_time<goby::uint64>(); 
                base_lcm->publish("RESET_SHAPER_STATS", &t);
                

                // // request a state message
                // drc::shaper_data_request_t request_msg;
                // request_msg.channel = "EST_ROBOT_STATE_MINIMAL";
                // request_msg.priority = 1;
                // robot_lcm->publish("SHAPER_DATA_REQUEST", &request_msg);
            }
            
        }            
        else if(FD_ISSET(base_fd, &fds))
        {
            base_lcm->handle();
        }
        else if(FD_ISSET(robot_fd, &fds))
        {
            robot_lcm->handle();
        }
    }
    
    return 0;
}
