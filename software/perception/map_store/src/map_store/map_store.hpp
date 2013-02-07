#ifndef map_store_HPP_
#define map_store_HPP_

#include <lcm/lcm.h>

#include <pointcloud_tools/pointcloud_lcm.hpp>
#include <pointcloud_tools/pointcloud_vis.hpp>

#include <otdf_parser/otdf_parser.h>

#include <lcmtypes/drc_lcmtypes.h>
#include <lcmtypes/bot_core.h>
#include <path_util/path_util.h>




/**
 * Represent a single image feature.
 */
struct LocalMap
{
  int map_id; // a unique incrementing counter for each map
  int64_t utime; // a time stamp - corresponds to the time of pose
  Eigen::Isometry3d pose; // this one of the camera poses from which the robot captured this map
  Eigen::Isometry3d base_pose; // the base pose, initially 0,0,0, is the grounding pose for this map

  pcl::PointCloud<PointXYZRGB>::Ptr cloud; // the raw point cloud

  // otdfs:
  std::vector< std::string > otdf_types;
  std::vector< int16_t > object_ids;
  std::vector< boost::shared_ptr<otdf::ModelInterface> > objects;
  // rgb stills
  // segmentation sequences

  // @todo what more is needed?
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

enum DumpCode { DUMP_FILE = 0, DUMP_LCM = 1, DUMP_SCREEN = 2 }; // possibly also to its own seperate log file



///////////////////////////////////////////////////////////////
class map_store{
  public:
    map_store(lcm_t* publish_lcm);
    
    ~map_store(){
    }
    
  private:
    lcm_t* publish_lcm_;

    pointcloud_lcm* pc_lcm_;
    pointcloud_vis* pc_vis_;

    vector <LocalMap> maps;

    void initialize();

    void publish_affordance_collection(LocalMap m);
    void publish_local_map(unsigned int map_id);
    void dump_maps(DumpCode code, double x_offset);



    static void seg_request_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const drc_localize_reinitialize_cmd_t* msg,
                                void* user_data){
      ((map_store *) user_data)->seg_request_handler(msg);
    }
    void seg_request_handler(const drc_localize_reinitialize_cmd_t *msg);


    static void seg_update_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const drc_localize_reinitialize_cmd_t* msg,
                                void* user_data){
      ((map_store *) user_data)->seg_update_handler(msg);
    }
    void seg_update_handler(const drc_localize_reinitialize_cmd_t *msg);

    static void dump_maps_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const drc_localize_reinitialize_cmd_t* msg,
                                void* user_data){
      ((map_store *) user_data)->dump_maps_handler(msg);
    }
    void dump_maps_handler(const drc_localize_reinitialize_cmd_t *msg);

    static void current_map_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const drc_localize_reinitialize_cmd_t* msg,
                                void* user_data){
      ((map_store *) user_data)->current_map_handler(msg);
    }
    void current_map_handler(const drc_localize_reinitialize_cmd_t *msg);

    static void pointcloud_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const drc_pointcloud2_t* msg,
                                void* user_data) {
      ((map_store *) user_data)->pointcloud_handler(msg);
    }
    void pointcloud_handler(const drc_pointcloud2_t *msg);

    void send_newmap();



};    

#endif
