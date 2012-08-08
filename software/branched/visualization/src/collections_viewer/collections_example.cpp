#include <iostream>
#include <lcm/lcm.h>

#include <isam/isam.h>
#include "visualization/viewer.hpp"
#include "visualization/pointcloud.hpp"

using namespace std;

int main(int argc, char** argv)
{
  lcm_t * lcm;

  lcm = lcm_create(NULL);
  if(!lcm)
  {
    cerr << "Failed to create lcm connection" << endl;
    return 1;
  }

  cout << "Collections example" << endl;
  Viewer viewer(lcm);
  ObjectCollection obj(1, std::string("Objects"), VS_OBJ_COLLECTION_T_POSE3D);
  LinkCollection link(2, std::string("Links"));

  // Send a reset
  // Send an object collection
  obj.add(100, isam::Pose3d(0,0,0,0,0,0));
  obj.add(101, isam::Pose3d(10,0,0,0,0,0));
  obj.add(102, isam::Pose3d(10,10,0,0,0,0));
  obj.add(103, isam::Pose3d(0,10,0,0,0,0));
  viewer.sendCollection(obj, true);

  // Send a link collection
  link.add(1, 1, 100, 1, 101);
  link.add(2, 1, 102, 1, 103);
  viewer.sendCollection(link, true);

  // Send a point collection
  PointCloudCollection pc_collection(3, std::string("Point cloud"), 1, VS_POINT3D_LIST_COLLECTION_T_POINT);
  PointCloudPtr pc = boost::make_shared<PointCloud>(100);
  pc->addPoint(10.,  1.,  1., 1., 0., 0.);
  pc->addPoint( 1., 10.,  1., 0., 1., 0.);
  pc->addPoint( 1.,  1., 10., 0., 0., 1.);
  pc_collection.add(boost::make_shared<PointCloudPtr>(pc));
  viewer.sendCollection(pc_collection, true);
  std::cout << "Send point cloud" << std::endl;

  // Send config

  lcm_destroy(lcm);
  return 0;
}

