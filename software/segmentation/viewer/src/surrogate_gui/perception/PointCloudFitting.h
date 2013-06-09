#include <pcl/point_types.h>
#include <iostream>
#include <Eigen/Eigen>

namespace PointCloudFitting {

  using namespace Eigen;
  using namespace std;

////////////////////////////////////////
// main function

  Affine3f pointCloutFit(pcl::PointCloud<pcl::PointXYZRGB>::Ptr modelcloud, 
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                         std::vector<float>& res_range);
                         

///////////////////////////////////////
// helper class and functions

#define ASSERT_PI(b) {if(!(b)) {std::cout << "Error on line: " << __LINE__ << " " #b << std::endl; abort();}}

template <typename T>
class Cube{
public:
  Cube(){size[0]=size[1]=size[2]=0;}
  Cube(int a, int b, int c){
    size[0]=a;
    size[1]=b;
    size[2]=c;
    data.resize(a*b*c);
  };

  T& operator() (int a,int b,int c){ 
    return data[index(a,b,c)]; 
  }
  
  const T& operator() (int a,int b,int c) const{ 
    return data[index(a,b,c)]; 
  }
  
  T& operator[] (int index) {
    ASSERT_PI(size[0]>0 && size[1]>0 && size[2]>0); 
    ASSERT_PI(index<data.size());
    return data[index]; 
  }

  const T& operator[] (int index) const {
    ASSERT_PI(size[0]>0 && size[1]>0 && size[2]>0); 
    ASSERT_PI(index<data.size());
    return data[index]; 
  }

  int index(int a, int b, int c) const{ 
    ASSERT_PI(size[0]>0 && size[1]>0 && size[2]>0); 
    ASSERT_PI(a<size[0] && b<size[1] && c<size[2]); 
    ASSERT_PI(a>=0 && b>=0 && c>=0); 
    return a*size[1]*size[2] + b*size[2] + c; 
  }

  Vector3i index(int i) const{ 
    ASSERT_PI(size[0]>0 && size[1]>0 && size[2]>0);
    int c = i%size[2];
    i/=size[2];
    int b = i%size[1];
    i/=size[1];
    int a = i;
    ASSERT_PI(a<size[0])
    return Vector3i(a,b,c);
  }
  vector<T> data;
  int size[3];  
};

void align_pts_3d(const vector<Vector3f>& pts_ref, const vector<Vector3f>& pts_cur,
                  Matrix3f& R, Vector3f& T);

void create_voxels(const vector<Vector3f>& pts, float res, float padding,
                   Cube<float>& vol, Affine3f& world_to_vol);

void point_pairs_from_dist_inds(const Cube<int>& dist_inds, const vector<Vector3f>& pts, 
                                Matrix3f R, Vector3f T,
                                vector<Vector3f>& p0, vector<Vector3f>& p1);

Affine3f optimize_pose_with_directions(const Cube<int>& dist_inds, const vector<Vector3f>& pts, Affine3f pose_init);

Affine3f align_coarse_to_fine(
          const vector<Vector3f>& pts_model, 
          const vector<Vector3f>& pts_data, 
          const vector<float>& res_range);

} //namespace PointCloudFitting
