#ifndef POINTCLOUD_MATH_HPP_
#define POINTCLOUD_MATH_HPP_

//  A series of math and 3D geometry utils dependent only on Eigen
// These functions may be duplicates of functions inside Eigen

#include <Eigen/Dense>
#include <Eigen/StdVector>


struct obj_cfg{
  obj_cfg(int id, std::string name, int type, bool reset):
    id(id), name(name), type(type), reset(reset) {}
  int id;
  std::string name;
  int type;
  bool reset;
};

struct ptcld_cfg{
  ptcld_cfg(int id, std::string name, int type, bool reset,
      int obj_coll, bool use_rgb, std::vector<float> rgb ):
    id(id), name(name), type(type), reset(reset),
    obj_coll(obj_coll), use_rgb(use_rgb), rgb(rgb) {}
  int id;
  std::string name;
  int type;
  bool reset;

  //int64_t ptcld_id; // id of the cloud itself
  int obj_coll; // which pose collection is it relative to | was "collection"
  //int64_t obj_id; // which object in the pose collection is it relative to | was "element_id"
  //float rgba[4];

  bool use_rgb; // use this single color [0] or use colors in the cloud itself [0]
  std::vector <float> rgb; // 3 vals 0->1
  // inline specification: http://live.boost.org/doc/libs/1_36_0/libs/assign/doc/index.html
};




//
struct Isometry3dTime{
  Isometry3dTime(int64_t utime, const Eigen::Isometry3d & pose) : utime(utime), pose(pose) {}
    int64_t utime;
    Eigen::Isometry3d pose;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



// Convert number to jet colour coordinates
// In: number between 0-->1
// Out: rgb jet colours 0->1
// http://metastine.com/2011/01/implementing-a-continuous-jet-colormap-function-in-glsl/
static inline void jet_rgb(float value,float rgb[]){
  float fourValue = (float) 4 * value;
  rgb[0]   = std::min(fourValue - 1.5, -fourValue + 4.5);
  rgb[1] = std::min(fourValue - 0.5, -fourValue + 3.5);
  rgb[2]  = std::min(fourValue + 0.5, -fourValue + 2.5);
  for (int i=0;i<3;i++){
   if (rgb[i] <0) {
     rgb[i] =0;
   }else if (rgb[i] >1){
     rgb[i] =1;
   }
  }
}

static inline void jet_rgb(float value,std::vector< float > &rgb){
  float rgb_array[3];
  jet_rgb(value,rgb_array);
  rgb.resize(3);
  rgb[0] = rgb_array[0];
  rgb[1] = rgb_array[1];
  rgb[2] = rgb_array[2];
}

// Simplification of slerp to give a fraction of a quaternion rotation
// Reimplemented from Peter Cooke's toolbox and libbot2
// q = [0.66742, 0.28604, 0.38139, 0.57208];
// q_i = [1 0 0 0];         % identity quaternion
// r = 0.1
// theta = acos(q_i*q');
// temp = (sin((1-r)*theta) * q_i + sin(r*theta) * q) / sin(theta) ;
// q_simple =  temp(1:4)/norm(temp(1:4))
// @input: r - fraction of the transform - in range [0:1]  NO CHECKING YET
// @input: q - input quaternion
// @output:q_out - quaternion equivelent to the fraction r of q 
void scale_quaternion(double r,Eigen::Quaterniond q,Eigen::Quaterniond &q_out);

// From isam/Rot3d.h
Eigen::Quaterniond euler_to_quat(double yaw, double pitch, double roll);

// Float version -  From isam/Rot3d.h
Eigen::Quaternionf euler_to_quat_f(double yaw, double pitch, double roll);

void quat_to_euler(Eigen::Quaterniond q, double& yaw, double& pitch, double& roll);

// generic double to float converter: [remove if something in eigen works instead
Eigen::Isometry3f Isometry_d2f(Eigen::Isometry3d );

void print_Isometry3d(Eigen::Isometry3d pose, std::stringstream &ss);

void print_Quaterniond(Eigen::Quaterniond r, std::stringstream &ss);



////////////////////////////////////////////////////////////////////////
// These are all depricated
typedef struct _Ptcoll_cfg
{
  int id;
  std::string name;
  bool reset;
  int64_t point_lists_id;
  int collection;
  int64_t element_id;
  int npoints;
  int type;
  //float rgba[4];
  std::vector <float> rgba;
}Ptcoll_cfg;


typedef struct _Objcoll_cfg
{
  int id;
  std::string name;
  int type;
  bool reset;
}Objcoll_cfg;

//typedef struct _Isometry3d_Time {
//  Eigen::Isometry3d p;
//  int64_t utime;
//} Isometry3d_Time;

// vs_obj_t with a quaterion
// a BotTrans with a utime
// can be converted 1-to-1 to a vs_obj_t
//typedef struct _ObjQ{
//    BotTrans p;
//    int64_t utime;
//} ObjQ;

typedef struct _Objq_coll_cfg
{
  int id;
  std::string name;
  bool reset;
  int type;  
  int nobjs;
}Objq_coll_cfg;

#endif
