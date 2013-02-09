// Demonstrations application for fovis wrapper library - fovision
#include <zlib.h>
#include <lcm/lcm-cpp.hpp>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/multisense.hpp>

#include "drcvision/voconfig.hpp"
#include "drcvision/vofeatures.hpp"
#include "drcvision/voestimator.hpp"
#include "fovision.hpp"

#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds
using namespace std;

class StereoOdom{
  public:
    StereoOdom(boost::shared_ptr<lcm::LCM> &lcm_);//lcm_t* publish_lcm);
    
    ~StereoOdom(){
      free (left_buf_);
      free(right_buf_);
    }

  private:
    uint8_t* left_buf_;
    int left_buf_size_;
    uint8_t* right_buf_;
    int right_buf_size_;
    int64_t utime_cur_;

    boost::shared_ptr<lcm::LCM> lcm_;
    BotParam* botparam_;
    BotFrames* botframes_;
    voconfig::KmclConfiguration* config;

    //
    FoVision* vo_;

    //
    VoFeatures* features_;
    int64_t utime_prev_;
    uint8_t* left_buf_ref_; // copies of the reference images - probably can be extracted from fovis directly
    uint8_t* right_buf_ref_; // no used for anything currently
    int64_t ref_utime_;
    Eigen::Isometry3d ref_camera_pose_; // [pose of the camera when the reference frames changed
    bool changed_ref_frames_;

    VoEstimator* estimator_;

    void featureAnalysis();
    void updateMotion();
    void imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg);   
    void multisenseLRHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::images_t* msg);   
};    

StereoOdom::StereoOdom(boost::shared_ptr<lcm::LCM> &lcm_) : 
       lcm_(lcm_), utime_cur_(0), utime_prev_(0), 
       ref_utime_(0), changed_ref_frames_(false)
{
  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);

  // Read config from file:
  string depth_source_name ="stereo";
  config = new voconfig::KmclConfiguration(botparam_, depth_source_name);

  boost::shared_ptr<fovis::StereoCalibration> stereo_calibration_;
  stereo_calibration_ = boost::shared_ptr<fovis::StereoCalibration>(config->load_stereo_calibration());
  left_buf_size_ = stereo_calibration_->getWidth() * stereo_calibration_->getHeight();
  right_buf_size_ = left_buf_size_;
  left_buf_ = (uint8_t*) malloc(left_buf_size_);
  right_buf_ = (uint8_t*) malloc(right_buf_size_);  
  left_buf_ref_ = (uint8_t*) malloc(left_buf_size_);  
  right_buf_ref_ = (uint8_t*) malloc(right_buf_size_);  
  
  vo_ = new FoVision(lcm_ , stereo_calibration_);

  features_ = new VoFeatures(lcm_, stereo_calibration_->getWidth(), stereo_calibration_->getHeight() );
  estimator_ = new VoEstimator(lcm_ , botframes_);

  lcm_->subscribe("CAMERA",&StereoOdom::imageHandler,this);
  lcm_->subscribe("MULTISENSE_LR",&StereoOdom::multisenseLRHandler,this);
  cout <<"StereoOdom Constructed\n";
}

void StereoOdom::featureAnalysis(){
  /// Feature Output: ///////////////////////////////////////////////
  // Check we changed reference frame last iteration, if so output the set of matching inliers:
  if (changed_ref_frames_) {
    if (ref_utime_ > 0){ // skip the first null image
      if(vo_->getNumMatches() > 200){ // if less than 50 features - dont bother writing
      // was:      if(featuresA.size() > 50){ // if less than 50 features - dont bother writing
        cout << "ref frame from " << utime_prev_ << " at " << utime_cur_ <<"\n";
        features_->setFeatures(vo_->getMatches(), vo_->getNumMatches() , ref_utime_);
        features_->setImages(left_buf_ref_, right_buf_ref_);
        features_->setCameraPose( ref_camera_pose_ );
        features_->sendFeatures();
      }
    }
    changed_ref_frames_=false;
  }
  if (vo_->getChangeReferenceFrames()){ // If we change reference frame, note the change for the next iteration.
    ref_utime_ = utime_cur_;
    ref_camera_pose_ = estimator_->getCameraPose(); // publish this pose when the 
    std::copy( left_buf_ , left_buf_ + left_buf_size_  , left_buf_ref_); // Keep the image buffer to write with the features:
    std::copy( right_buf_, right_buf_+ right_buf_size_  , right_buf_ref_);
    changed_ref_frames_=true;
  }
}

void StereoOdom::updateMotion(){
  Eigen::Isometry3d delta_camera;
  Eigen::MatrixXd delta_cov;
  fovis::MotionEstimateStatusCode delta_status;
  vo_->getMotion(delta_camera, delta_cov, delta_status );
  //vo_->fovis_stats();
  estimator_->voUpdate(utime_cur_, delta_camera);
}

void StereoOdom::imageHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const  bot_core::image_t* msg){
  utime_prev_ = utime_cur_;
  utime_cur_ = msg->utime;
  memcpy(left_buf_,  msg->data.data() , msg->size/2);
  memcpy(right_buf_,  msg->data.data() + msg->size/2 , msg->size/2);
  vo_->doOdometry(left_buf_,right_buf_);
  updateMotion();
  featureAnalysis();
}

void StereoOdom::multisenseLRHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const  multisense::images_t* msg){
  utime_prev_ = utime_cur_;
  utime_cur_ = msg->utime;
  cout << "fsdfsad\n";
  cout << msg->images[0].width <<  msg->images[0].height << "\n";
  memcpy(left_buf_,  msg->images[0].data.data() , msg->images[0].size);
  memcpy(right_buf_,  msg->images[1].data.data() , msg->images[1].size);
  vo_->doOdometry(left_buf_,right_buf_);
  updateMotion();
  featureAnalysis();
}

int main(int argc, char **argv){
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  StereoOdom fo= StereoOdom(lcm);    
  while(0 == lcm->handle());
}
