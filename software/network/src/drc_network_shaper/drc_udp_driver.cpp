#include "drc_udp_driver.h"

#include "goby/acomms/modemdriver/driver_exception.h"
#include "goby/acomms/modemdriver/mm_driver.h"
#include "goby/common/logger.h"
#include "goby/util/binary.h"

using goby::util::hex_encode;
using goby::util::hex_decode;
using goby::glog;
using namespace goby::common::logger;
using goby::common::goby_time;


const size_t UDP_MAX_PACKET_SIZE = 65507; // (16 bit length = 65535 - 8 byte UDP header -20 byte IP)


DRCUDPDriver::DRCUDPDriver(boost::asio::io_service* io_service)
    : io_service_(io_service),
      socket_(*io_service),
      receive_buffer_(UDP_MAX_PACKET_SIZE)
{
}

DRCUDPDriver::~DRCUDPDriver()
{
}


void DRCUDPDriver::startup(const goby::acomms::protobuf::DriverConfig& cfg)
{
    driver_cfg_ = cfg;
    
    
    const UDPDriverConfig::EndPoint& local = driver_cfg_.GetExtension(UDPDriverConfig::local);
    socket_.open(boost::asio::ip::udp::v4());
    socket_.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), local.port()));

    
    const UDPDriverConfig::EndPoint& remote = driver_cfg_.GetExtension(UDPDriverConfig::remote);
    boost::asio::ip::udp::resolver resolver(*io_service_);
    boost::asio::ip::udp::resolver::query query(boost::asio::ip::udp::v4(),
                                                remote.ip(),
                                                goby::util::as<std::string>(remote.port()));
    boost::asio::ip::udp::resolver::iterator endpoint_iterator = resolver.resolve(query);
    receiver_ = *endpoint_iterator;

    glog.is(DEBUG1) &&
        glog << group(glog_out_group())
             << "Receiver endpoint is: " << receiver_.address().to_string()
             << ":" << receiver_.port() << std::endl;
    
    start_receive();
    io_service_->reset();
}

void DRCUDPDriver::shutdown()
{
    io_service_->stop();
    socket_.close();
}

void DRCUDPDriver::handle_initiate_transmission(const goby::acomms::protobuf::ModemTransmission& orig_msg)
{
    // buffer the message
    goby::acomms::protobuf::ModemTransmission msg = orig_msg;
    signal_modify_transmission(&msg);

    msg.set_max_frame_bytes(driver_cfg_.GetExtension(UDPDriverConfig::max_frame_size));
    signal_data_request(&msg);

    if(!(msg.frame_size() == 0 || msg.frame(0).empty()))
        start_send(msg);
}


void DRCUDPDriver::do_work()
{
    io_service_->poll();
}

void DRCUDPDriver::start_send(const goby::acomms::protobuf::ModemTransmission& msg)
{
    // send the message
    const std::string& bytes = msg.frame(0);

    glog.is(DEBUG1) &&
        glog << group(glog_out_group())
             << "Sending hex: " << goby::util::hex_encode(bytes) << std::endl;

    socket_.async_send_to(boost::asio::buffer(bytes), receiver_, boost::bind(&DRCUDPDriver::send_complete, this, _1, _2));
}


void DRCUDPDriver::send_complete(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    if(error)
    {
        glog.is(DEBUG1) &&
            glog << group(glog_out_group()) << warn
                 << "Send error: " << error.message() << std::endl;
        return;
    }
    
    glog.is(DEBUG1) &&
        glog << group(glog_out_group())
             << "Sent " << bytes_transferred << " bytes." << std::endl;

}


void DRCUDPDriver::start_receive()
{
    socket_.async_receive_from(boost::asio::buffer(receive_buffer_), sender_, boost::bind(&DRCUDPDriver::receive_complete, this, _1, _2));
    
}

void DRCUDPDriver::receive_complete(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    if(error)
    {
        glog.is(DEBUG1) &&
            glog << group(glog_in_group()) << warn
                 << "Receive error: " << error.message() << std::endl;
        start_receive();
        return;
    }
    
    glog.is(DEBUG1) &&
        glog << group(glog_in_group())
             << "Received " << bytes_transferred << " bytes from "
             << sender_.address().to_string()
             << ":" << sender_.port() << std::endl;
    
    goby::acomms::protobuf::ModemTransmission msg;
    (*msg.add_frame()) = std::string(&receive_buffer_[0], bytes_transferred);

    msg.set_type(goby::acomms::protobuf::ModemTransmission::DATA);
    msg.set_src(0);
    msg.set_dest(0);
    signal_receive(msg);
            
    start_receive();
}
