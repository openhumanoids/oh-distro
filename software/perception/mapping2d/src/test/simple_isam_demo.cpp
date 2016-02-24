#include <isam/isam.h>
#include <iostream>
#include <vector>

#include <lcmtypes/vs_object_collection_t.h>
#include <lcmtypes/vs_point3d_list_collection_t.h>
#include <bot_core/bot_core.h>
#include <pronto_utils/pronto_math.hpp>

using namespace std;
using namespace isam;
using namespace Eigen;

lcm_t* m_lcm;

void sendCollection(vector < Pose2d_Node* > nodes,int id,string label)
{
  vs_object_collection_t objs;	
  size_t n = nodes.size();
  if (n > 1) {
    objs.id = id;
    stringstream oss;
    string mystr;
    oss << "coll" << id;
    mystr=oss.str();	  

    objs.name = (char*) label.c_str();
    objs.type = VS_OBJECT_COLLECTION_T_POSE;
    objs.reset = false;
    objs.nobjects = n;
    vs_object_t poses[n];
    for (size_t i = 0; i < n; i++) {
      Pose2d_Node* node =  nodes[i];
      isam::Pose2d  pose;
      pose = (*node).value();

      // direct Eigen Approach: TODO: test and use
      //Matrix3f m;
      //m = AngleAxisf(angle1, Vector3f::UnitZ())
      // *  * AngleAxisf(angle2, Vector3f::UnitY())
      // *  * AngleAxisf(angle3, Vector3f::UnitZ());
      Eigen::Quaterniond quat = euler_to_quat(0,0, pose.t());

      poses[i].id = i;
      poses[i].x = pose.x();
      poses[i].y = pose.y() ;
      poses[i].z = 0;
      poses[i].qw = quat.w();
      poses[i].qx = quat.x();
      poses[i].qy = quat.y();
      poses[i].qz = quat.z();
    }
    objs.objects = poses;
    vs_object_collection_t_publish(m_lcm, "OBJECT_COLLECTION", &objs);
  }
}


int main() {
  m_lcm = lcm_create(NULL);
  Slam slam;
  MatrixXd sqrtinf = 10. * Matrix<double, 3, 3>::Identity();

  /////////////
  Pose2d prior_origin(0., 0., 0.);
  Pose2d_Node a0;
  slam.add_node(&a0);
  Pose2d_Factor p_a0(&a0, prior_origin, SqrtInformation(sqrtinf));
  slam.add_factor(&p_a0);
  vector < Pose2d_Node* > pg1_nodes;
  pg1_nodes.push_back(&a0);

  /////////////
  Pose2d odo(1., 0., 0.); // A
  Pose2d_Node a1;
  slam.add_node(&a1);
  pg1_nodes.push_back(&a1);
  Pose2d_Pose2d_Factor o_a01(&a0, &a1, odo, SqrtInformation(sqrtinf));
  slam.add_factor(&o_a01);

  /////////////
  Pose2d odo3(1., 0., M_PI/2); // B
  Pose2d_Node a3;
  slam.add_node(&a3);
  pg1_nodes.push_back(&a3);
  Pose2d_Pose2d_Factor o_a23(&a1, &a3, odo3, SqrtInformation(sqrtinf));
  slam.add_factor(&o_a23);  
  
  /////////////
  Pose2d odo4(1., 0., 0.); // C
  Pose2d_Node a4;
  slam.add_node(&a4);
  pg1_nodes.push_back(&a4);
  Pose2d_Pose2d_Factor o_a34(&a3, &a4, odo4, SqrtInformation(sqrtinf));
  slam.add_factor(&o_a34);    
  
  /////////////
  Pose2d odo5(1., 0., M_PI/2); // D
  Pose2d_Node a5;
  slam.add_node(&a5);
  pg1_nodes.push_back(&a5);
  Pose2d_Pose2d_Factor o_a45(&a4, &a5, odo5, SqrtInformation(sqrtinf));
  slam.add_factor(&o_a45);  
  sendCollection(pg1_nodes,10,"Part map"); // as an example

  Pose2d odo6(1.5, 0., 0.); // E
  Pose2d_Node a6;
  slam.add_node(&a6);
  pg1_nodes.push_back(&a6);
  Pose2d_Pose2d_Factor o_a56(&a5, &a6, odo6, SqrtInformation(sqrtinf));
  slam.add_factor(&o_a56);    
  
  /////////////
  Pose2d odo7(1.5, 0., M_PI/2); // F
  Pose2d_Node a7;
  slam.add_node(&a7);
  pg1_nodes.push_back(&a7);
  Pose2d_Pose2d_Factor o_a67(&a6, &a7, odo7, SqrtInformation(sqrtinf));
  slam.add_factor(&o_a67);  
  
  /////////////
  Pose2d odo8(1., 0., 0.0); // G
  Pose2d_Node a8;
  slam.add_node(&a8);
  pg1_nodes.push_back(&a8);
  Pose2d_Pose2d_Factor o_a78(&a7, &a8, odo8, SqrtInformation(sqrtinf));
  slam.add_factor(&o_a78);  

  /////////////
  Pose2d odo9(1., 0.7, M_PI/2); // G
  Pose2d_Node a9;
  slam.add_node(&a9);
  pg1_nodes.push_back(&a9);
  Pose2d_Pose2d_Factor o_a89(&a8, &a9, odo9, SqrtInformation(sqrtinf));
  slam.add_factor(&o_a89);  

  // 3 Publish pose graphs without and loop
  slam.update();
  sendCollection(pg1_nodes,11,"Map - no loop");

  /////////////
  Pose2d odo09(0., 0., 0.0);
  Pose2d_Pose2d_Factor o_a09(&a0, &a9, odo09, SqrtInformation(sqrtinf));
  slam.add_factor(&o_a09);  

  // 6. Optimize and publish to lcm:
  slam.update();
  slam.batch_optimization();
  std::cout << "\n";
  
  sendCollection(pg1_nodes,12,"Map - loop, optimized");


  return 0;
}
