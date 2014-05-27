#ifndef DRCUDPModemDriver20130509H
#define DRCUDPModemDriver20130509H

#include "goby/common/time.h"

#include "goby/acomms/modemdriver/driver_base.h"
#include "goby/acomms/protobuf/udp_driver.pb.h"

#include <boost/bind.hpp>
#include <boost/asio.hpp>


class DRCUDPDriver : public goby::acomms::ModemDriverBase
{
  public:
    DRCUDPDriver(boost::asio::io_service* io_service);
    ~DRCUDPDriver();
    void startup(const goby::acomms::protobuf::DriverConfig& cfg);
    void shutdown();            
    void do_work();
    void handle_initiate_transmission(const goby::acomms::protobuf::ModemTransmission& m);

  private:
    void start_send(const goby::acomms::protobuf::ModemTransmission& msg);
    void send_complete(const boost::system::error_code& error, std::size_t bytes_transferred);
    void start_receive();
    void receive_complete(const boost::system::error_code& error, std::size_t bytes_transferred);

  private:            
    goby::acomms::protobuf::DriverConfig driver_cfg_;
    boost::asio::io_service* io_service_;
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint receiver_;
    boost::asio::ip::udp::endpoint sender_;            
    std::vector<char> receive_buffer_; 
};

#endif
