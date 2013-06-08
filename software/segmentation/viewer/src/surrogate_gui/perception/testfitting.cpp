#include "distTransform.h"
#include <iostream>
#include <assert.h>
#include <float.h>

#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/SVD>
#include <Eigen/Eigen>
#include <bot_core/rotations.h>

using namespace std;
using namespace pcl;
using namespace Eigen;

#define ASSERT_PI(b) {if(!(b)) {cout << "Error on line: " << __LINE__ << " " #b << endl; abort();}}
#define SQ(x) ((x)*(x))

static Matrix3f ypr2rot(Vector3f ypr){
  double rpy[]={ypr[2],ypr[1],ypr[0]};
  double q[4];
  double mat[9];
  bot_roll_pitch_yaw_to_quat(rpy,q);
  int rc=bot_quat_to_matrix(q,mat);
  Matrix3d mat2(mat);
  for(int j=0;j<3;j++){
    for(int i=0;i<3;i++){
      mat2(j,i) = mat[j*3+i];
    }
  }
  return mat2.cast<float>();
}

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
  
  T& operator[] (int index) {
    ASSERT_PI(size[0]>0 && size[1]>0 && size[2]>0); 
    ASSERT_PI(index<data.size());
    return data[index]; 
  }

  int index(int a, int b, int c){ 
    ASSERT_PI(size[0]>0 && size[1]>0 && size[2]>0); 
    ASSERT_PI(a<size[0] && b<size[1] && c<size[2]); 
    ASSERT_PI(a>=0 && b>=0 && c>=0); 
    return a*size[1]*size[2] + b*size[2] + c; 
  }

  Vector3i index(int i){ 
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
                  Matrix3f& R, Vector3f& T){
  ASSERT_PI(pts_ref.size()==pts_cur.size());

  // compute mean
  Vector3f avg_ref(0,0,0), avg_cur(0,0,0);
  int N=pts_ref.size();
  for(int i=0;i<N;i++){
    avg_ref+=pts_ref[i];
    avg_cur+=pts_cur[i];
  }
  avg_ref/=N; avg_cur/=N;
  //cout << "avg: " << avg_ref.transpose() << " " <<  avg_cur.transpose() << endl;

  // sub mean
  vector<Vector3f> p_ref(N), p_cur(N);
  for(int i=0;i<N;i++){
    p_ref[i] = pts_ref[i]-avg_ref;
    p_cur[i] = pts_cur[i]-avg_cur;
    
  }

  // trans mat
  Affine3f T_ref = Affine3f::Identity();
  Affine3f T_cur = Affine3f::Identity();
  T_ref.translation() = -avg_ref; 
  T_cur.translation() = -avg_cur;

  // svd
  Matrix3f svdmat = Matrix3f::Zero();
  for(int i=0;i<N;i++){
    for(int j=0;j<N;j++){
      svdmat +=  p_ref[i]*p_cur[i].transpose();
    }
  }
  JacobiSVD<Matrix3f> svd(svdmat, ComputeFullU | ComputeFullV);
  Matrix3f u = svd.matrixU();
  Matrix3f v = svd.matrixV();
  //cout << "svdmat\n" << svdmat << endl;
  //cout << "u\n" << u << endl; 
  //cout << "v\n" << v << endl;

  Matrix3f R3 = u*v.transpose();
  Affine3f R4 = Affine3f::Identity();
  R4.linear() = R3;
  Affine3f P = T_ref.inverse()*R4*T_cur;
  
  // results
  R = P.linear();
  T = P.translation();

}

void create_voxels(const vector<Vector3f>& pts, float res, float padding,
                   Cube<float>& vol, Affine3f& world_to_vol)
{
  Vector3f pt_min, pt_max;
  pt_min = pt_max = pts[0];
  for(int i=0;i<pts.size();i++){
    pt_min = pt_min.cwiseMin(pts[i]);
    pt_max = pt_max.cwiseMax(pts[i]);
  }

  // create world to vol
  world_to_vol = Affine3f::Identity();
  world_to_vol.translation() = Vector3f(-padding,-padding,-padding);
  Affine3f scale = Affine3f::Identity();
  scale(0,0) = res;
  scale(1,1) = res;
  scale(2,2) = res;
  world_to_vol = scale*world_to_vol;
  Affine3f trans = Affine3f::Identity();
  trans.translation() = pt_min;
  world_to_vol = trans*world_to_vol;
  world_to_vol = world_to_vol.inverse();
  
  // alloc vol
  Vector3f sizef = (pt_max-pt_min)/res+2*Vector3f(padding,padding,padding)+Vector3f(1,1,1);
  Vector3i size(ceil(sizef[0]),  ceil(sizef[1]), ceil(sizef[2]));
  vol = Cube<float>(size[0], size[1], size[2]);
  //cout << size.transpose( ) << endl;
  //cout << pt_max.transpose( ) << " " << pt_min.transpose() << endl;


  // populate vol
  for(int i=0;i<pts.size();i++){
    Vector4f pts4(pts[i][0],pts[i][1],pts[i][2],1.0);
    Vector4f vol4 = world_to_vol*pts4;
    Vector3i voli(round(vol4[0]),round(vol4[1]),round(vol4[2]));
    //cout << i << " " << voli.transpose() << endl;
    //cout << i << " " << size.transpose() << endl;
    ASSERT_PI(voli[0]>=0 && voli[1]>=0 && voli[2]>=0);
    ASSERT_PI(voli[0]<size[0] && voli[1]<size[1] && voli[2]<size[2]);
    vol(voli[0],voli[1],voli[2]) = 1;
  }
}

void point_pairs_from_dist_inds(Cube<int>& dist_inds, const vector<Vector3f>& pts, 
                                Matrix3f R, Vector3f T,
                                vector<Vector3f>& p0, vector<Vector3f>& p1)
{
  for(int i=0;i<pts.size();i++){
    Vector3f pt_cur = R*pts[i]+T; 
    Vector3i pt_int(round(pt_cur[0]), round(pt_cur[1]), round(pt_cur[2]));
    bool good=true;
    for(int j=0;j<3;j++) if(pt_int[j]<0||pt_int[j]>=dist_inds.size[j]) good=false;

    if(good){
      int model_ind = dist_inds(pt_int[0],pt_int[1], pt_int[2]);
      Vector3f modelPos = dist_inds.index(model_ind).cast<float>();
      p0.push_back(modelPos);
      p1.push_back(pt_cur);
    }
  }
}

void align_coarse_to_fine(
          const vector<Vector3f>& pts_model, 
          const vector<Vector3f>& pts_data, 
          const vector<float>& res_range)
{
  /////////////////////////////
  // coarse align
  float res = res_range[0];
  float padding = 20;

  // convert model to voxels
  Cube<float> dist_xform;
  Affine3f world_to_vol;
  create_voxels(pts_model, res, padding, dist_xform, world_to_vol);

  // perform distance transform on model
  Cube<int> dist_inds(dist_xform.size[0],dist_xform.size[0],dist_xform.size[2]);
  distTransform(dist_xform.data.data(), dist_inds.data.data(), dist_xform.size[2], dist_xform.size[1], dist_xform.size[0]);
  float maxDist = SQ(dist_xform.size[0]) + SQ(dist_xform.size[1]) + SQ(dist_xform.size[2]);
  maxDist = sqrt(maxDist);
  for(int i=0;i<dist_xform.data.size();i++) dist_xform[i]/=maxDist;
  Vector3f avg_model(0,0,0);
  for(int i=0;i<pts_model.size();i++) avg_model+=pts_model[i];
  avg_model/=pts_model.size();
  //TODO pts_data_dec = decimate_points(pts_data,res);
  vector<Vector3f> pts_data_dec = pts_data;
  
  // iterate through angles
  float angle_step=30;
  //TODO: allow limit to search
  double best_error = DBL_MAX;
  Affine3f best_P;
  for(float roll=-180; roll<180; roll+=angle_step){
    cout << "Roll: " << roll << endl;
    for(float pitch=-90; pitch<90; pitch+=angle_step){
      for(float yaw=-180; yaw<180; yaw+=angle_step){
        Matrix3f R = ypr2rot(Vector3f(yaw,pitch,roll));
        //Matrix3f Rt = R.transpose();
        vector<Vector3f> pts(pts_data_dec.size());
        for(int i=0;i<pts.size();i++) pts[i] = R*pts_data_dec[i]; 
        Vector3f pts_mean(0,0,0);
        for(int i=0;i<pts.size();i++) pts_mean+=pts[i];
        pts_mean/=pts.size();
        Vector3f T=avg_model-pts_mean;
        for(int i=0;i<pts.size();i++) {
          Vector3f p = pts[i]+T;
          Vector4f p4(p[0],p[1],p[2],1);
          p4=world_to_vol*p4; 
          pts[i]=Vector3f(p4[0],p4[1],p4[2]); 
        }

        // compute point pairs and align
        vector<Vector3f> p0,p1;
        point_pairs_from_dist_inds(dist_inds, pts, Matrix3f::Identity(), Vector3f(0,0,0), p0, p1);
        Matrix3f R_opt;
        Vector3f T_opt;
        align_pts_3d(p0,p1,R_opt,T_opt);
        ASSERT_PI(!isnan(R_opt(0,0)));
        ASSERT_PI(!isnan(T_opt[0]));
        
        //cout << dist_xform.size[0] << " " << dist_xform.size[0] << " " << dist_xform.size[0] << endl;
        // compute error
        double error=0;
        for(int i=0;i<pts.size();i++){
          Vector3f pt = R_opt*pts[i]+T_opt;
          pt[0] = min<float>(max<float>(pt[0],0),dist_xform.size[0]-2);
          pt[1] = min<float>(max<float>(pt[1],0),dist_xform.size[1]-2);
          pt[2] = min<float>(max<float>(pt[2],0),dist_xform.size[2]-2);
          //cout << R_opt << endl << pts[i].transpose() << endl << T_opt.transpose() << endl;
          int ul[3] = { (int)floor(pt[0]), (int)floor(pt[1]), (int)floor(pt[2]) };
          double w[3] = {pt[0]-ul[0],pt[1]-ul[1],pt[2]-ul[2] };
          double dist = 
            dist_xform(ul[0]+0,ul[1]+0,ul[2]+0) * (1-w[0])*(1-w[1])*(1-w[2]) +
            dist_xform(ul[0]+0,ul[1]+0,ul[2]+1) * (1-w[0])*(1-w[1])*(  w[2]) +
            dist_xform(ul[0]+0,ul[1]+1,ul[2]+0) * (1-w[0])*(  w[1])*(1-w[2]) +
            dist_xform(ul[0]+0,ul[1]+1,ul[2]+1) * (1-w[0])*(  w[1])*(  w[2]) +
            dist_xform(ul[0]+1,ul[1]+0,ul[2]+0) * (w[0])  *(1-w[1])*(1-w[2]) +
            dist_xform(ul[0]+1,ul[1]+0,ul[2]+1) * (w[0])  *(1-w[1])*(  w[2]) +
            dist_xform(ul[0]+1,ul[1]+1,ul[2]+0) * (w[0])  *(  w[1])*(1-w[2]) +
            dist_xform(ul[0]+1,ul[1]+1,ul[2]+1) * (w[0])  *(  w[1])*(  w[2]);
          error+=SQ(dist);
        }
        error = sqrt(error);
        if(error<best_error){
          cout << error << " " << roll << " " << pitch  << " " << yaw << endl;
          Affine3f opt = Affine3f::Identity();
          opt.linear() = R_opt;
          opt.translation() = T_opt;
          Affine3f orig = Affine3f::Identity();
          orig.linear() = R; 
          orig.translation() = T; 
          best_P = world_to_vol.inverse()*opt*world_to_vol*orig;
          best_error = error;
        }  
      }
    }
  }
  Affine3f pose_init = best_P;


}


/* Notes
   distTransform L M N: L is the inner loop
   create_voxels: size[2] is the inner loop
   TODO: use one convention.
*/



int main(int argc, char*argv[]){
  if(argc!=3) {
    cerr << "usage: testfitting model.pcd cloud.pcd\n";
    exit(-1);
  }
  
  // load model
  PCDReader reader;
  PointCloud<pcl::PointXYZRGB>::Ptr modelcloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  int rc = reader.read(argv[1], *modelcloud);
  ASSERT_PI(rc==0);

  // load cloud
  PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  rc = reader.read(argv[2], *cloud);
  ASSERT_PI(rc==0);
  
  vector<Vector3f> modelcloudV(modelcloud->size());
  for(int i=0;i<modelcloud->size();i++) {
    modelcloudV[i][0] = modelcloud->at(i).x;
    modelcloudV[i][1] = modelcloud->at(i).y;
    modelcloudV[i][2] = modelcloud->at(i).z;
  }

  vector<Vector3f> cloudV(cloud->size());
  for(int i=0;i<cloud->size();i++) {
    cloudV[i][0] = cloud->at(i).x;
    cloudV[i][1] = cloud->at(i).y;
    cloudV[i][2] = cloud->at(i).z;
  }

  vector<float> res_range;
  res_range.push_back(0.01);
  align_coarse_to_fine(modelcloudV, cloudV, res_range);

  /*

  Cube<float> modelvol;
  Vector3i modelvolsize;
  Affine3f model_world_to_vol;
  create_voxels(modelcloudV, 0.01, 0, modelvol, model_world_to_vol);
  cout << modelvol.size[2] << " " << modelvol.size[1] << " " << modelvol.size[0] << endl;

  Cube<float> cloudvol;
  Vector3i cloudvolsize;
  Affine3f cloud_world_to_vol;
  create_voxels(cloudV, 0.01, 0, cloudvol, cloud_world_to_vol);
  cout << cloudvol.size[2] << " " << cloudvol.size[1] << " " << cloudvol.size[0] << endl;


  Cube<float> modeldist = modelvol;
  modeldist = modelvol;
  Cube<int> modelidx(modelvol.size[0], modelvol.size[1], modelvol.size[2]);
  distTransform(modeldist.data.data(), modelidx.data.data(), modelvol.size[2], modelvol.size[1], modelvol.size[0]); 
  cout << "done\n";

  FILE*fp = fopen("dump.bin", "wb");
  fwrite(modeldist.data.data(),sizeof(float),modeldist.data.size(),fp);
  fclose(fp);

  PointCloud<pcl::PointXYZRGB>::Ptr c2(new pcl::PointCloud<pcl::PointXYZRGB>());

  for(int i=0;i<cloudvol.size[0];i++){
    for(int j=0;j<cloudvol.size[1];j++){
      for(int k=0;k<cloudvol.size[2];k++){
        if(cloudvol(i,j,k)){
          PointXYZRGB pt;
          pt.x = i;
          pt.y = j;
          pt.z = k;
          c2->points.push_back(pt);
        }
      }
    }
  }
  c2->width=1;c2->height=c2->size();
  PCDWriter writer;
  writer.write("test.pcd",*c2);
                              
  */
}

