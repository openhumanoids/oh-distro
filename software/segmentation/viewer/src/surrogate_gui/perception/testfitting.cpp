#include "PointCloudFitting.h"
#include <pcl/io/pcd_io.h>

using namespace PointCloudFitting;
using namespace pcl;

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
  
  vector<float> res_range;
  res_range.push_back(0.01);
  Affine3f pose = pointCloutFit(modelcloud,cloud,res_range);

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

