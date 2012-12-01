#ifndef POINTCLOUD_VIS_HPP_
#define POINTCLOUD_VIS_HPP_


#include <lcm/lcm.h>
#include <iostream>


#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "pcl/ModelCoefficients.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"

#include <pcl/filters/passthrough.h>
#include "pcl/filters/conditional_removal.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/extract_indices.h"

#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/surface/convex_hull.h"
#include "pcl/surface/concave_hull.h"
#include "pcl/PolygonMesh.h"
#include "pcl/octree/octree.h"

#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/features/normal_3d.h"
#include "pcl/features/fpfh.h"
#include "pcl/registration/ia_ransac.h"

#include <bot_core/bot_core.h>

#include <vector>
#include <algorithm>

using namespace pcl;
using namespace pcl::io;

#include <pointcloud_tools/pointcloud_math.hpp>
#include <lcmtypes/pointcloud_tools.h>

// Duplicates the list in collections renderer:
static float vis_colors[] = {
    51/255.0, 160/255.0, 44/255.0,  //0
    166/255.0, 206/255.0, 227/255.0,
    178/255.0, 223/255.0, 138/255.0,//6
    31/255.0, 120/255.0, 180/255.0,
    251/255.0, 154/255.0, 153/255.0,// 12
    227/255.0, 26/255.0, 28/255.0,
    253/255.0, 191/255.0, 111/255.0,// 18
    106/255.0, 61/255.0, 154/255.0,
    255/255.0, 127/255.0, 0/255.0, // 24
    202/255.0, 178/255.0, 214/255.0,
    1.0, 0.0, 0.0, // red // 30
    0.0, 1.0, 0.0, // green
    0.0, 0.0, 1.0, // blue// 36
    1.0, 1.0, 0.0,
    1.0, 0.0, 1.0, // 42
    0.0, 1.0, 1.0,
    0.5, 1.0, 0.0,
    1.0, 0.5, 0.0,
    0.5, 0.0, 1.0,
    1.0, 0.0, 0.5,
    0.0, 0.5, 1.0,
    0.0, 1.0, 0.5,
    1.0, 0.5, 0.5,
    0.5, 1.0, 0.5,
    0.5, 0.5, 1.0,
    0.5, 0.5, 1.0,
    0.5, 1.0, 0.5,
    0.5, 0.5, 1.0
};
const int num_vis_colors = sizeof(vis_colors)/(3*sizeof(float));

class pointcloud_vis {
  public:
    pointcloud_vis (lcm_t* publish_lcm);

    // Push a colour PointCloud to LCM as a points collection
    // assumes that you want to connect it to the collection specified in Ptcoll_cfg
    //void pcdXYZRGB_to_lcm(Ptcoll_cfg ptcoll_cfg,pcl::PointCloud<pcl::PointXYZRGB> &cloud);

    
    void pose_collection_to_lcm_from_list(int id, std::vector<Isometry3dTime> & posesT);
    void pose_collection_to_lcm(obj_cfg ocfg, std::vector<Isometry3dTime> & posesT);

    void pose_to_lcm_from_list(int id,Isometry3dTime& poseT);
    void pose_to_lcm(obj_cfg ocfg, Isometry3dTime& poseT);

    void ptcld_to_lcm_from_list(int id, pcl::PointCloud<pcl::PointXYZRGB> &cloud,
            int64_t obj_id, int64_t ptcld_id);
    void ptcld_to_lcm(ptcld_cfg pcfg, pcl::PointCloud<pcl::PointXYZRGB> &cloud,
            int64_t obj_id, int64_t ptcld_id);

    void mesh_to_lcm_from_list(int id, pcl::PolygonMesh::Ptr mesh,
            int64_t obj_id, int64_t ptcld_id,
            bool sendSubset =false,const std::vector<int> &SubsetIndicies = std::vector<int>());
    void mesh_to_lcm(ptcld_cfg pcfg,pcl::PolygonMesh::Ptr mesh,
            int64_t obj_id, int64_t ptcld_id,
            bool sendSubset =false,const std::vector<int> &SubsetIndicies = std::vector<int>());

    void pointcloud2_to_lcm(pcl::PointCloud<pcl::PointXYZRGB> &cloud, std::string channel, int64_t cloud_utime);


    std::vector <obj_cfg> obj_cfg_list;
    std::vector <ptcld_cfg> ptcld_cfg_list;
  private:
    lcm_t *publish_lcm_;

};







// Read a csv file containign poses and timestamps:
void read_poses_csv(std::string poses_files, std::vector<Isometry3dTime>& poses);

// Save a PolygonMesh to PLY
// PCL's uses VTK which produces an awkward type of PLY
void savePLYFile(pcl::PolygonMesh::Ptr model,std::string fname);

// Remove all the polygons with a specific color
// this is used to remove all black polygons which represent glass in kcml
void remove_colored_polygons(pcl::PolygonMesh::Ptr meshin_ptr, std::vector<int> &color );

// Find the polygon INDICES inside a box centered around @center of 2x size @dgrid
// of the PolygonMEsh meshin_ptr
// @input: meshin_ptr - input mesh model
// @input: center - 3 element vector to center of box
// @input: dgrid - 3 elements of size of box
// @output: the polygon INDICES inside the box
void get_MeshInBoxIndices(pcl::PolygonMesh::Ptr meshin_ptr,
		   std::vector<double> &center, std::vector<double> &dgrid,
		   std::vector<int> &polygon_in_box_indices);


// Find the polygon INDICES inside a circle of size @radius
// of the PolygonMEsh meshin_ptr
// @input: meshin_ptr - input mesh model
// @input: center - 3 element vector to center of box
// @input: radius - 3 elements of size of box
// @output: the polygon INDICES inside the box
void get_MeshInCircleIndices(pcl::PolygonMesh::Ptr meshin_ptr,
		   std::vector<double> &center, double radius,
		   std::vector<int> &polygon_in_box_indices);


// Find the polygons inside a box centered around @center of 2x size @dgrid
// of the PolygonMEsh meshin_ptr
// @input: meshin_ptr - input mesh model
// @input: center - 3 element vector to center of box
// @input: dgrid - 3 elements of size of box
// @output: what lies insdie the box
void get_MeshInBox(pcl::PolygonMesh::Ptr meshin_ptr,
		   std::vector<double> &center, std::vector<double> &dgrid,
		   pcl::PolygonMesh::Ptr &minimesh_ptr);


// Merge two PolygonMesh structures into the meshA
bool merge_PolygonMesh(pcl::PolygonMesh::Ptr &meshA, pcl::PolygonMesh::Ptr meshB);

// Push a PolygonMesh to PCL. The polygonMesh is a PtCloud with a set of vertices for each mesh
// the polygon is anchored using a single obj collection specified in Ptcoll_cfg
// TODO use boost inf  for the default value clip z value
// If a subset is specified only those polygons are sent to LCM
bool PolygonMesh_to_lcm(lcm_t *lcm, Ptcoll_cfg ptcoll_cfg,pcl::PolygonMesh::Ptr mesh,
	bool clip_z = false, double clip_height = 99999999999.0,
	bool sendSubset =false,const std::vector<int> &SubsetIndicies = std::vector<int>());

// Push an OjbQ (quaterion) Collection to LCM as an Obj Collection (non-quaterion)
// assumes that you want to connect it to the collection specified in Ptcoll_cfg
//bool ObjU_to_lcm(lcm_t *lcm, Objq_coll_cfg objq_coll_cfg,std::vector<ObjQ> objq_coll);





// Push an un-coloured PointCloud to LCM as a points collection
// assumes that you want to connect it to the collection specified in Ptcoll_cfg
bool pcdXYZ_to_lcm(lcm_t *lcm, Ptcoll_cfg ptcoll_cfg,pcl::PointCloud<pcl::PointXYZ> &cloud);


void display_tic_toc(std::vector<int64_t> &tic_toc,const std::string &fun_name);

/*
typedef struct _BasicPlane
{
  int64_t utime;
  std::string name; // a unique name eg the file name
  int major; // which major plane 
  int minor; // which minor plane
  //PointT cloud; 
  pcl::PointCloud<pcl::PointXYZRGB> cloud ;
//  double coeffs[4];
   pcl::ModelCoefficients coeffs;  
  Eigen::Vector4f centroid;
  // this number of points in the original soure cloud.
  // Used for voting when combining to a final cloud
  int n_source_points; 
  
  // New members added in jun17_2011: mfallon
  EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
  Eigen::Affine3f transform000; // transform to move the plane to 0,0,0 [ie centroid=0,0,0] aligned to the x,y axis (i think)
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
  
} BasicPlane;
*/


//bool read_and_project_submaps(std::string file_path,std::string file_name_in, BasicPlane &one_plane, BasicPlane &one_plane000  );



// http://www.alecjacobson.com/weblog/?p=1527
// Acts like matlab's [Y,I] = SORT(X)
// Input:
//   unsorted  unsorted vector
// Output:
//   sorted     sorted vector, allowed to be same as unsorted
//   index_map  an index map such that sorted[i] = unsorted[index_map[i]]
template <class T>
void sort(
    std::vector<T> &unsorted,
    std::vector<T> &sorted,
    std::vector<size_t> &index_map);

// Act like matlab's Y = X[I]
// where I contains a vector of indices so that after,
// Y[j] = X[I[j]] for index j
// this implies that Y.size() == I.size()
// X and Y are allowed to be the same reference
template< class T >
void reorder(
  std::vector<T> & unordered,
  std::vector<size_t> const & index_map,
  std::vector<T> & ordered);

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

// Comparison struct used by sort
// http://bytes.com/topic/c/answers/132045-sort-get-index
template<class T> struct index_cmp
{
  index_cmp(const T arr) : arr(arr) {}
  bool operator()(const size_t a, const size_t b) const
  {
    return arr[a] < arr[b];
  }
  const T arr;
};

template <class T>
void sort(
  std::vector<T> & unsorted,
  std::vector<T> & sorted,
  std::vector<size_t> & index_map)
{
  // Original unsorted index map
  index_map.resize(unsorted.size());
  for(size_t i=0;i<unsorted.size();i++)
  {
    index_map[i] = i;
  }
  // Sort the index map, using unsorted for comparison
  sort(
    index_map.begin(),
    index_map.end(),
    index_cmp<std::vector<T>& >(unsorted));

  sorted.resize(unsorted.size());
  reorder(unsorted,index_map,sorted);
}

// This implementation is O(n), but also uses O(n) extra memory
template< class T >
void reorder(
  std::vector<T> & unordered,
  std::vector<size_t> const & index_map,
  std::vector<T> & ordered)
{
  // copy for the reorder according to index_map, because unsorted may also be
  // sorted
  std::vector<T> copy = unordered;
  ordered.resize(index_map.size());
  for(size_t i = 0; i<index_map.size();i++)
  {
    ordered[i] = copy[index_map[i]];
  }
}




#endif /* POINTCLOUD_VIS_HPP_ */
