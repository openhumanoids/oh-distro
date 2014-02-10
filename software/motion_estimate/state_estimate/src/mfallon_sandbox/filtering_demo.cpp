#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/bot_core.hpp>

#include <pointcloud_tools/pointcloud_vis.hpp>
#include <bot_frames_cpp/bot_frames_cpp.hpp>
#include <path_util/path_util.h>
#include <ConciseArgs>

#include <lcmtypes/drc/atlas_raw_imu_batch_t.hpp>

#include "visualization/collections.hpp"

#include <leg-odometry/Filter.hpp>
#include <leg-odometry/HeavyLowPassFilter.hpp>


using namespace std;


class IIRNotch{
  public:
    IIRNotch();
    
    ~IIRNotch(){
    }    
    
    void setCoeffs(int harmonic);
    // iir filter cooeffs
    Eigen::Vector3d b;
    Eigen::Vector3d a;
    
    // carry over inputs/outputs
    Eigen::VectorXd x;
    Eigen::VectorXd y;
    
    double processSample(double input);

    
  private:
};

IIRNotch::IIRNotch(){
  // defaults for 85Hz notch
  b << 0.7872805555, -1.3603002156, 0.7872805555;
  a << 1.0000000000, -1.3603002156, 0.5745611110; 
  
  x = Eigen::VectorXd(2);
  x << 0,0;
  y = Eigen::VectorXd(2);
  y << 0,0;
}
  
void IIRNotch::setCoeffs(int harmonic){
  // which 85Hz harmonic to filter
  if (harmonic==1){ //85Hz
    b << 0.7872805555, -1.3603002156, 0.7872805555;
    a << 1.0000000000, -1.3603002156, 0.5745611110;
  }else if (harmonic==2){ // 170Hz
    b << 0.6283781802, -0.6054469941,  0.6283781802;
    a << 1.0000000000, -0.6054469941,  0.2567563604;
  }else if (harmonic==3){ // 340Hz
    b << 0.3547365716, 0.3801547205, 0.3547365716;
    a << 1.0000000000,  0.3801547205, -0.2905268567;
  }
}

double IIRNotch::processSample(double input){
  bool v = false;
  
  Eigen::Vector3d x_temp ( input , x(0),  x(1) );
  Eigen::Vector3d y_temp (      0, y(0),  y(1) );
  if(v)  std::cout << input << "\n";
  if(v)  std::cout << x_temp.transpose() << " x_temp\n";
  if(v)  std::cout << y_temp.transpose() << " y_temp\n";
  if(v)  std::cout << b.transpose() << " b\n";
  if(v)  std::cout << a.transpose() << " a\n";
  
  if(v){  
    Eigen::Vector3d bit =  x_temp.cross(b);
    std::cout << bit.transpose() << " bit\n";  
  }
  
  
  double output =  (x_temp.dot(b)) -  (y_temp.dot(a));
  

  double temp_x = x(0);
  x << input , temp_x  ;
  double temp_y = y(0);
  y <<  output, temp_y ; 
  
  if(v)  std::cout << x.transpose() << " x\n";
  if(v)  std::cout << y.transpose() << " y\n\n";
  
  return output;
}


struct IMUPacket
{
  int64_t utime_raw; // raw utime in message, deltas between this utime will be very accurate
  // utime of the batch packet that contained this message: will not drift relative to other messages
  // increments at ~333Hz. created on robot machine, with bot sync
  int64_t utime_batch; 
  // Combinaton of utime_raw and utime_batch
  int64_t utime;
  // difference between utime_raw and previous measurement
  int64_t utime_delta; 

  int64_t packet_count;
  Eigen::Vector3d delta_rotation;
  Eigen::Vector3d linear_acceleration;

};

struct IMUBatch
{
  // Original Batch Packet Utime
  int64_t utime;
  // The new packets that were received with this batch
  std::vector<IMUPacket> packets; // ordered [0] oldest [end] newest
  // The old packets that were received with this batch
  std::vector<IMUPacket> packets_old; // ordered [0] oldest [end] newest
};


////////////////////////////////////////
struct CommandLineConfig
{
    int filter_type;
};


class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_);
    
    ~App(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    BotParam* botparam_;
    bot::frames* frames_cpp_;
    BotFrames* frames_;
    
    void batchHandler(const lcm::ReceiveBuffer* rbuf, 
                           const std::string& channel, const  drc::atlas_raw_imu_batch_t* msg); 
    void doFilter(IMUPacket &raw);
    const CommandLineConfig cl_cfg_;  
    
    LowPassFilter lpfilter[3];
    HeavyFiltering::HeavyLowPassFilter hlpfilter[3];
    
    // An cascade of notch filters in xyz
    IIRNotch notchfilter_x[3];
    IIRNotch notchfilter_y[3];
    IIRNotch notchfilter_z[3];
    
    int64_t last_packet_;
    int64_t last_packet_utime_;
    int counter_; // don't publish if only just started filtering

    void publishIMUPacket(IMUPacket packet, std::string channel);
};




App::App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_):
    lcm_(lcm_), cl_cfg_(cl_cfg_){
      
  lcm_->subscribe( "ATLAS_IMU_BATCH" ,&App::batchHandler,this);

  for (size_t i=0; i < 3 ; i++){
    notchfilter_x[i].setCoeffs(i+1);
    notchfilter_y[i].setCoeffs(i+1);
    notchfilter_z[i].setCoeffs(i+1);
  }
  
  last_packet_ = -1;
  last_packet_utime_ = 0;  
  counter_=0;
}

drc::atlas_raw_imu_t convertToLCMPacket(IMUPacket packet){

  drc::atlas_raw_imu_t m;
  m.utime = packet.utime;
  m.packet_count = packet.packet_count;
  m.delta_rotation[0] = packet.delta_rotation[0];
  m.delta_rotation[1] = packet.delta_rotation[1];
  m.delta_rotation[2] = packet.delta_rotation[2];

  m.linear_acceleration[0] = packet.linear_acceleration[0];
  m.linear_acceleration[1] = packet.linear_acceleration[1];
  m.linear_acceleration[2] = packet.linear_acceleration[2];
  return m;
}

IMUPacket convertFromLCMPacket(drc::atlas_raw_imu_t msg, int64_t last_packet_utime, int64_t batch_utime){
      IMUPacket raw;
      raw.utime_raw = msg.utime;
      raw.utime_batch = batch_utime; // the main incoming utime
      raw.utime_delta = raw.utime_raw - last_packet_utime;
      raw.utime = raw.utime_raw; // for now use raw timestamp

      raw.packet_count = msg.packet_count;
      raw.delta_rotation = Eigen::Vector3d(msg.delta_rotation);
      raw.linear_acceleration = Eigen::Vector3d(msg.linear_acceleration);
  return raw;
}

drc::atlas_raw_imu_batch_t convertToLCMBatch(IMUBatch batch){

  drc::atlas_raw_imu_batch_t m;
  m.utime = batch.utime;

  // Ouput the packets in the original order:
  //     newest new packet
  // 2nd newest new packet
  // ...
  //     newest old packet (that appeared in a previous message
  // 2nd newest old packet
  // ...
  
  for (int i= batch.packets.size()-1 ; i >= 0 ; i-- ){
    m.raw_imu.push_back( convertToLCMPacket( batch.packets[i] ) );
  }
  
  for (int i= batch.packets_old.size()-1 ; i >= 0 ; i-- ){
    m.raw_imu.push_back( convertToLCMPacket( batch.packets_old[i] ) );
  }
  m.num_packets = m.raw_imu.size();
  return m;
}

void App::doFilter(IMUPacket &raw){
//  std::cout <<   raw.linear_acceleration[2] << "\n";


  if (cl_cfg_.filter_type==0){
    // "light low pass filter"
    raw.linear_acceleration[0]  = (double)lpfilter[0].processSample( raw.linear_acceleration[0] );
    raw.linear_acceleration[1]  = (double)lpfilter[1].processSample( raw.linear_acceleration[1] );
    raw.linear_acceleration[2]  = (double)lpfilter[2].processSample( raw.linear_acceleration[2] );
  }else if (cl_cfg_.filter_type ==1){ // reasonable high [ass gilter
    // "heavy low pass filter"
    double scale = 1/  1.02653695819398;
    raw.linear_acceleration[0]  = scale*hlpfilter[0].processSample( raw.linear_acceleration[0] );
    raw.linear_acceleration[1]  = scale*hlpfilter[1].processSample( raw.linear_acceleration[1] );
    raw.linear_acceleration[2]  = scale*hlpfilter[2].processSample( raw.linear_acceleration[2] );
  }else if(cl_cfg_.filter_type==2){
    // notch 85Hz
    raw.linear_acceleration[0]  = notchfilter_x[0].processSample( raw.linear_acceleration[0] );
    raw.linear_acceleration[1]  = notchfilter_y[0].processSample( raw.linear_acceleration[1] );
    raw.linear_acceleration[2]  = notchfilter_z[0].processSample( raw.linear_acceleration[2] );
  }else{
    // notch 85Hz, 170, 340Hz in cascade
    raw.linear_acceleration[0]  = notchfilter_x[0].processSample( raw.linear_acceleration[0] );
    raw.linear_acceleration[1]  = notchfilter_y[0].processSample( raw.linear_acceleration[1] );
    raw.linear_acceleration[2]  = notchfilter_z[0].processSample( raw.linear_acceleration[2] );
    
    raw.linear_acceleration[0]  = notchfilter_x[1].processSample( raw.linear_acceleration[0] );
    raw.linear_acceleration[1]  = notchfilter_y[1].processSample( raw.linear_acceleration[1] );
    raw.linear_acceleration[2]  = notchfilter_z[1].processSample( raw.linear_acceleration[2] );

    raw.linear_acceleration[0]  = notchfilter_x[2].processSample( raw.linear_acceleration[0] );
    raw.linear_acceleration[1]  = notchfilter_y[2].processSample( raw.linear_acceleration[1] );
    raw.linear_acceleration[2]  = notchfilter_z[2].processSample( raw.linear_acceleration[2] );
  }

  
}




void App::batchHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_raw_imu_batch_t* msg){
  if (msg->raw_imu[0].packet_count < last_packet_){
    std::cout << "Detected time skip, resetting filter\n";
    last_packet_ = -1;
    last_packet_utime_ = 0;
    counter_ = 0;
  }
  

  int num_new = 0;
  bool verbose = false;

  IMUBatch batch;
  batch.utime = msg->utime;

  for (int i=msg->num_packets-1 ; i >= 0 ; i--){
    if (msg->raw_imu[i].packet_count > last_packet_){
      if (verbose) std::cout << "new " <<", "<<last_packet_ <<", "<< i <<", "<< msg->raw_imu[i].packet_count << "\n";

      IMUPacket raw = convertFromLCMPacket(msg->raw_imu[i], last_packet_utime_, msg->utime);
      batch.packets.push_back(raw);

      last_packet_ = msg->raw_imu[i].packet_count;
      last_packet_utime_ = msg->raw_imu[i].utime;
      num_new++;
    }else{
      // delibrately obsficate the delta field (we dont know it here)
      IMUPacket raw = convertFromLCMPacket(msg->raw_imu[i], -msg->raw_imu[i].utime, msg->utime); 
      batch.packets_old.push_back(raw);
      if (verbose) std::cout << "old " <<", "<<last_packet_ <<", "<< i <<", "<< msg->raw_imu[i].packet_count << "\n";
    }
  }

  if (verbose) std::cout << num_new << "---\n";


  for (int i=0 ; i < batch.packets.size() ; i++ ){
    drc::atlas_raw_imu_t raw_msg = convertToLCMPacket( batch.packets[i] );
    lcm_->publish("ATLAS_IMU_PACKET", &raw_msg); 
  }


  // Only filter new new packets:
  for (int i=0 ; i < batch.packets.size() ; i++ ){
    doFilter( batch.packets[i] );
  }


  // Ouput packets oldest to newest:
  for (int i=0 ; i < batch.packets.size() ; i++ ){
    drc::atlas_raw_imu_t imu_filtered_msg = convertToLCMPacket( batch.packets[i]);
    lcm_->publish("ATLAS_IMU_PACKET_FILTERED", &imu_filtered_msg);
  }


  // Don't publish the first 50 frames
  if (counter_ > 50){
    drc::atlas_raw_imu_batch_t batch_filtered_msg = convertToLCMBatch(batch);
    lcm_->publish("ATLAS_IMU_BATCH_FILTERED", &batch_filtered_msg); 
  }

  counter_++;
}


int main(int argc, char ** argv) {
  CommandLineConfig cl_cfg;
  cl_cfg.filter_type = 0; // 0 lp, 1 hlp, 2 notch
  ConciseArgs opt(argc, (char**)argv);
  opt.add(cl_cfg.filter_type, "f", "filter_type","Filter Type");
  opt.parse();  
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  App app(lcm, cl_cfg);
  cout << "Tool ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
