#include "sticky_hand_utils.hpp"
#include "lcmtypes/drc/desired_grasp_state_t.hpp"
#include "lcmtypes/drc/robot_urdf_t.hpp"

namespace visualization_utils
{

 //--------------------------------------------------------------------------------
  bool get_hand_palm_link_name(int hand_type,std::string &ee_name)
  {
      if(hand_type==drc::desired_grasp_state_t::SANDIA_LEFT)
          ee_name ="left_palm";    
      else if(hand_type==drc::desired_grasp_state_t::SANDIA_RIGHT)   
          ee_name ="right_palm";    
      else if(hand_type==drc::desired_grasp_state_t::IROBOT_LEFT)
          ee_name ="left_base_link";    
      else if(hand_type==drc::desired_grasp_state_t::IROBOT_RIGHT)   
          ee_name ="right_base_link";
      else if(hand_type==drc::desired_grasp_state_t::ROBOTIQ_LEFT)
          ee_name ="left_palm";    
      else if(hand_type==drc::desired_grasp_state_t::ROBOTIQ_RIGHT)   
          ee_name ="right_palm";                  
      else if(hand_type==drc::desired_grasp_state_t::INERT_LEFT)
          ee_name ="left_base_link";    
      else if(hand_type==drc::desired_grasp_state_t::INERT_RIGHT)   
          ee_name ="right_base_link";       
      else {
          cout << "unknown hand_type\n";   
          return false;
      }
      return true;
  }
  
   bool get_hand_palm_link_name_given_urdf_handtype(int hand_type,std::string &ee_name)
  {
      if(hand_type==drc::robot_urdf_t::LEFT_SANDIA)
          ee_name ="left_palm";    
      else if(hand_type==drc::robot_urdf_t::RIGHT_SANDIA)   
          ee_name ="right_palm";    
      else if(hand_type==drc::robot_urdf_t::LEFT_IROBOT)
          ee_name ="left_base_link";    
      else if(hand_type==drc::robot_urdf_t::RIGHT_IROBOT)   
          ee_name ="right_base_link";
      else if(hand_type==drc::robot_urdf_t::LEFT_ROBOTIQ)
          ee_name ="left_palm";    
      else if(hand_type==drc::robot_urdf_t::RIGHT_ROBOTIQ)   
          ee_name ="right_palm";                  
      else if(hand_type==drc::robot_urdf_t::LEFT_NONE)
          ee_name ="left_base_link";    
      else if(hand_type==drc::robot_urdf_t::RIGHT_NONE)   
          ee_name ="right_base_link";       
      else {
          cout << "unknown hand_type\n";   
          return false;
      }
      return true;
  } 
  
   bool get_hand_link_name(int hand_type,std::string &hand_link_name)
  {
      if((hand_type==drc::desired_grasp_state_t::SANDIA_LEFT)||(hand_type==drc::desired_grasp_state_t::IROBOT_LEFT)||(hand_type==drc::desired_grasp_state_t::ROBOTIQ_LEFT)||(hand_type==drc::desired_grasp_state_t::INERT_LEFT))
          hand_link_name ="l_hand";    
      else if((hand_type==drc::desired_grasp_state_t::SANDIA_RIGHT)||(hand_type==drc::desired_grasp_state_t::IROBOT_RIGHT)||(hand_type==drc::desired_grasp_state_t::ROBOTIQ_RIGHT)||(hand_type==drc::desired_grasp_state_t::INERT_RIGHT))
          hand_link_name ="r_hand";    
      else {
          cout << "unknown hand_type\n";   
          return false;
      }
      return true;
  } 
  
  bool is_left_hand(int hand_type)
  {
       if((hand_type==drc::desired_grasp_state_t::SANDIA_LEFT)||(hand_type==drc::desired_grasp_state_t::IROBOT_LEFT)||(hand_type==drc::desired_grasp_state_t::ROBOTIQ_LEFT)||(hand_type==drc::desired_grasp_state_t::INERT_LEFT))
          return true;    
      else if((hand_type==drc::desired_grasp_state_t::SANDIA_RIGHT)||(hand_type==drc::desired_grasp_state_t::IROBOT_RIGHT)||(hand_type==drc::desired_grasp_state_t::ROBOTIQ_RIGHT)||(hand_type==drc::desired_grasp_state_t::INERT_RIGHT))
          return false;  
      else {
          cout << "unknown hand_type\n";   
          return false;
      }
  }
  
  bool get_hand_name(int hand_type,std::string &hand_name)
  {
     if((hand_type==drc::desired_grasp_state_t::SANDIA_LEFT)||(hand_type==drc::desired_grasp_state_t::SANDIA_RIGHT))  
          hand_name ="sandia";    
      else if((hand_type==drc::desired_grasp_state_t::IROBOT_LEFT)||(hand_type==drc::desired_grasp_state_t::IROBOT_RIGHT))   
          hand_name ="irobot";    
      else if((hand_type==drc::desired_grasp_state_t::ROBOTIQ_LEFT)||(hand_type==drc::desired_grasp_state_t::ROBOTIQ_RIGHT))   
          hand_name ="robotiq";       
      else if((hand_type==drc::desired_grasp_state_t::INERT_LEFT)||(hand_type==drc::desired_grasp_state_t::INERT_RIGHT))   
          hand_name ="inert";   
      else {
          cout << "unknown hand_type\n";   
          return false;
      }
      return true;
  }
  //--------------------------------------------------------------------------------
  bool get_hand_urdf_file_name_and_ext(int hand_type,std::string &filename,std::string &ext)
  {
      if (hand_type  == drc::desired_grasp_state_t::SANDIA_LEFT){
        filename ="sandia_hand_left";
        ext=".urdf";
      }
      else if (hand_type  == drc::desired_grasp_state_t::SANDIA_RIGHT){
        filename ="sandia_hand_right";
        ext=".urdf";
      }
      else if (hand_type  == drc::desired_grasp_state_t::IROBOT_LEFT){
        filename ="irobot_hand_left";
        ext=".urdf";
      }
      else if (hand_type  == drc::desired_grasp_state_t::IROBOT_RIGHT){
        filename ="irobot_hand_right";
        ext=".urdf";
      }
      else if (hand_type  == drc::desired_grasp_state_t::ROBOTIQ_LEFT){
        filename ="robotiq_hand_left";
        ext=".urdf";
      }
      else if (hand_type  == drc::desired_grasp_state_t::ROBOTIQ_RIGHT){
        filename ="robotiq_hand_right";
        ext=".urdf";
      }
      else if (hand_type  == drc::desired_grasp_state_t::INERT_LEFT){
        filename ="inert_hand_left";
        ext=".urdf";
      }
      else if (hand_type  == drc::desired_grasp_state_t::INERT_RIGHT){
        filename ="inert_hand_right";
        ext=".urdf";
      }   
      else {
          cout << "unknown hand_type\n";   
          return false;
      }
      return true;
  }

//-------------------------------------------------------------------------------  
bool is_sticky_hand_condition_active(const StickyHandStruc &hand_struc,boost::shared_ptr<visualization_utils::AffordanceCollectionManager>  &affCollectionManager)
  
  { 
    if(hand_struc.is_conditional)
    {
      object_instance_map_type_::iterator obj_it = affCollectionManager->_objects.find(string(hand_struc.object_name));
      double current_dof_pos, current_dof_vel;
      obj_it->second._otdf_instance->getJointState(hand_struc.conditioned_parent_joint_name, current_dof_pos, current_dof_vel);
      bool cond=false;
      if(hand_struc.cond_type==hand_struc.GT){
        cond =  (current_dof_pos >= hand_struc.conditioned_parent_joint_val);
      }
      else if (hand_struc.cond_type==hand_struc.LT){
        cond =  (current_dof_pos <= hand_struc.conditioned_parent_joint_val);
      }
      return cond;
    }
    else {
      return true;
    }
  }

 //-------------------------------------------------------------------------------  
  void get_min_dimension(boost::shared_ptr<otdf::Geometry> &link_geom, std::string &min_dimension_tag)
  {
   
    min_dimension_tag = "XY";
    enum {SPHERE, BOX, CYLINDER, MESH, TORUS};   
    int type = link_geom->type;
//    if(type == SPHERE)
//    {
//      boost::shared_ptr<otdf::Sphere> sphere(boost::shared_dynamic_cast<otdf::Sphere>(link_geom));	
//      double radius = sphere->radius;
//    }
//    else 
    if(type == BOX)
    {
      boost::shared_ptr<otdf::Box> box(boost::shared_dynamic_cast<otdf::Box>(link_geom));
      double xDim = box->dim.x;
      double yDim = box->dim.y;
      double zDim = box->dim.z;
      if(zDim<min(xDim,yDim))
        min_dimension_tag = "Z"; 
    }      
    else if(type == CYLINDER)
    {
      boost::shared_ptr<otdf::Cylinder> cyl(boost::shared_dynamic_cast<otdf::Cylinder>(link_geom));
      double radius = cyl->radius;
      double length = cyl->length;
      if(length<radius)
       min_dimension_tag = "Z";   
    }  
    else if(type == TORUS)
    {
      boost::shared_ptr<otdf::Torus> torus(boost::shared_dynamic_cast<otdf::Torus>(link_geom));
      double innerRadius = torus->tube_radius;
      double outerRadius = torus->radius;
      if(innerRadius<outerRadius)
        min_dimension_tag = "Z";  
    }      
  
  }  
  

 //-------------------------------------------------------------------------------
  void get_user_specified_hand_approach(Eigen::Vector3d objectframe_finger_dir, Eigen::Vector3d from, Eigen::Vector3d to, boost::shared_ptr<otdf::Geometry> &link_geom, KDL::Frame &T_objectgeometry_lhand, KDL::Frame &T_objectgeometry_rhand,bool is_sandia)
  {
  
   // calculate the rotation required to rotate x axis of hand to the negative ray direction. The palm face is pointing in the -x direction for the sandia hand. 
   Eigen::Vector3d ux,uy,uz;
   //x axis on sandia hand is pointing away from the palm face
   // if we define the urdf in palm frame as base then palm is facing z positive :Will scale to other hands
   ux << 1 , 0 , 0; 
   uy << 0 , 1 , 0;
   uz << 0 , 0 , 1;
  
   Eigen::Vector3d nray = -(to - from);
   nray.normalize();  // normalize
   
     // back-track from the hit pt in the approach dir by 10*t cm
   double t = 0.08;
   Eigen::Vector3d p;
   p << to[0]+t*nray[0], to[1]+t*nray[1], to[2]+t*nray[2]; 

   Eigen::Vector3d cross_prod;
   double dot_prod;

  // no need to normalize vectors
    nray = -(to - from);
    if(nray[0]>=0){
      dot_prod =ux.dot(nray); 
      cross_prod = ux.cross(nray);
    }
    else {
      dot_prod =ux.dot(-nray); 
      cross_prod = ux.cross(-nray);
    }
 
   // supposedly handles better aaround 180deg
   double w = (dot_prod) + ux.norm()*nray.norm();
   Eigen::Vector4f quat;
   quat << w,cross_prod[0],cross_prod[1],cross_prod[2];
   quat.normalize();
   
   if (w < 0.0001) { // vectors are 180 degrees apart
    quat << 0,-ux[2],ux[1],ux[0];
    quat.normalize();
  } 
   KDL::Frame T_objectgeometry_hand;
   T_objectgeometry_hand.p[0]=p[0];
   T_objectgeometry_hand.p[1]=p[1];
   T_objectgeometry_hand.p[2]=p[2];
   T_objectgeometry_hand.M =  KDL::Rotation::Quaternion(quat[1], quat[2], quat[3], quat[0]);

  // hand pos is calculated in positive x hemisphere only.
  // if viewpoint is in negative x hemisphere, hand is flipped about Z axis.
   if(nray[0]<0){
    KDL::Frame fliphand; 
    fliphand.p[0]=0;
    fliphand.p[1]=0;
    fliphand.p[2]=0;
    fliphand.M =  KDL::Rotation::RPY(0,0,M_PI);
    T_objectgeometry_hand = T_objectgeometry_hand*fliphand;
   }
  //-------
  // Hand is now with the palm facing the object.
  
   KDL::Frame T_rotatedhand_lhand = KDL::Frame::Identity();
   KDL::Frame T_rotatedhand_rhand = KDL::Frame::Identity(); //dependent on hand model and object dimensions
   if(is_sandia)
    T_rotatedhand_lhand.p[2]=0.125;
   T_rotatedhand_rhand.p=T_rotatedhand_lhand.p;
   
   objectframe_finger_dir.normalize(); // finger dir in object frame.
 
   Eigen::Vector3d desired_finger_dir;
   
   KDL::Frame temp_frame = T_objectgeometry_hand.Inverse();
   temp_frame.p = 0*temp_frame.p;// ignore translation while dealing with direction vectors
   rotate_eigen_vector_given_kdl_frame(objectframe_finger_dir,temp_frame,desired_finger_dir);
  
   double theta = atan2(desired_finger_dir[2],desired_finger_dir[1]);
   //depending on left or right hand, sign of rotation will change.
   //std::cout<< "theta: "<< theta*(180/M_PI) << std::endl;
    
   if(is_sandia)
   { 
    T_rotatedhand_lhand.M =  KDL::Rotation::RPY(5*(M_PI/8)-theta,0,0);
    T_rotatedhand_rhand.M =  KDL::Rotation::RPY(3*(M_PI/8)-theta,0,0);
   }
   else{
    T_rotatedhand_lhand.M =  KDL::Rotation::RPY((M_PI/2)-theta,0,0);
    T_rotatedhand_rhand.M =  KDL::Rotation::RPY((M_PI/2)-theta,0,0);
   }
   T_objectgeometry_lhand = T_objectgeometry_hand *(T_rotatedhand_lhand.Inverse());//gets T_objectgeometry_rotatedhand 
   T_objectgeometry_rhand = T_objectgeometry_hand *(T_rotatedhand_rhand.Inverse());//gets T_objectgeometry_rotatedhand 
   
  } // end get_user_specified_hand_approach
  
   //-------------------------------------------------------------------------------
  void get_hand_approach(Eigen::Vector3d from, Eigen::Vector3d to, boost::shared_ptr<otdf::Geometry> &link_geom, KDL::Frame &T_objectgeometry_lhand,KDL::Frame &T_objectgeometry_rhand,bool is_sandia)
  {

   // calculate the rotation required to rotate x axis of hand to the negative ray direction. The palm face is pointing in the -x direction for the sandia hand.  
       
   Eigen::Vector3d ux,uy,uz;
   ux << 1 , 0 , 0; //x axis on sandia hand is pointing away from the palm face
   uy << 0 , 1 , 0;
   uz << 0 , 0 , 1;
  
   Eigen::Vector3d nray = -(to - from);
   nray.normalize();  // normalize
   

   // back-track from the hit pt in the approach dir by 100*t cm
   double t = 0.05;
   Eigen::Vector3d p;
   p << to[0]+t*nray[0], to[1]+t*nray[1], to[2]+t*nray[2]; 
   
   // APPROACH 1 to calculate 
    Eigen::Vector3d cross_prod;
    double dot_prod;
    if(nray[0]>=0){
      dot_prod =ux.dot(nray); 
      cross_prod = ux.cross(nray);
    }
    else {
      dot_prod =ux.dot(-nray); 
      cross_prod = ux.cross(-nray);
    }

   cross_prod.normalize(); 

   double q[4];
   double axis[3]  = {cross_prod[0],cross_prod[1],cross_prod[2]};
   
    double angle = acos(dot_prod);
  bot_angle_axis_to_quat(angle,axis,q); 
  
  // APPROACH 2
  // no need to normalize vectors
    nray = -(to - from);
    if(nray[0]>=0){
      dot_prod =ux.dot(nray); 
      cross_prod = ux.cross(nray);
    }
    else {
      dot_prod =ux.dot(-nray); 
      cross_prod = ux.cross(-nray);
    }
 
   // supposedly handles better aaround 180deg
   double w = (dot_prod) + ux.norm()*nray.norm();
   Eigen::Vector4f quat;
   quat << w,cross_prod[0],cross_prod[1],cross_prod[2];
   quat.normalize();
   
   if (w < 0.0001) { // vectors are 180 degrees apart
    quat << 0,-ux[2],ux[1],ux[0];
    quat.normalize();
  } 


   KDL::Frame T_objectgeometry_hand;
   T_objectgeometry_hand.p[0]=p[0];
   T_objectgeometry_hand.p[1]=p[1];
   T_objectgeometry_hand.p[2]=p[2];
   T_objectgeometry_hand.M =  KDL::Rotation::Quaternion(q[1], q[2], q[3], q[0]);
   T_objectgeometry_hand.M =  KDL::Rotation::Quaternion(quat[1], quat[2], quat[3], quat[0]);
   
//   std::cout << "quat(x,y,z,w):  " << q[1]<< " " <<q[2]<< " "  << q[3]<< " "  << q[0]  << std::endl;
//   std::cout << "quat(x,y,z,w):  " << quat[1]<< " " <<quat[2]<< " "  << quat[3]<< " "  << quat[0]  << std::endl;
   
   
  // hand pos is calculated in positive x hemisphere only.
  // if viewpoint is in negative x hemisphere, hand is flipped about Z axis.
   if(nray[0]<0){
    KDL::Frame fliphand; 
    fliphand.p[0]=0;
    fliphand.p[1]=0;
    fliphand.p[2]=0;
    fliphand.M =  KDL::Rotation::RPY(0,0,M_PI);
    T_objectgeometry_hand = T_objectgeometry_hand*fliphand;
   }
  //-----------
  // Hand is now with the palm facing the object.
  
  //hand orientation is dependent on whether the smallest dimension along the axis of the link geometry in geom_z_axis or perpendicular to it in geom_XY plane.
  // things like steering wheel cylinder are thin in Z axis. You want to position your hand so as to pinch the thin part of the plate. Not that by default z is the key axis for urdf geometry definitions like cyl, torus.
     std::string min_dimension_tag;
     get_min_dimension(link_geom,min_dimension_tag); 
     
     
  //  if looking down at the object from the top or looking up from the bottom.
    if( fabs(uz.dot(nray))>max(fabs(ux.dot(nray)),fabs(uy.dot(nray))) )
    {
   
      double bearing_about_z = atan2(to[1],to[0]);
      if(uz.dot(nray)<0)
        bearing_about_z = M_PI-bearing_about_z;      

      KDL::Frame rotatetocenter=KDL::Frame::Identity();
      if(nray[0]<0){
        bearing_about_z += M_PI;
      }
      rotatetocenter.M =  KDL::Rotation::RPY(bearing_about_z,0,0);
      T_objectgeometry_hand = T_objectgeometry_hand*rotatetocenter;
    }//end if
      
     //depending on left or right hand, sign of rotation will change.
     KDL::Frame T_rotatedhand_lhand = KDL::Frame::Identity();
     KDL::Frame T_rotatedhand_rhand = KDL::Frame::Identity(); //dependent on hand model and object dimensions
     if(is_sandia)
     {
       T_rotatedhand_lhand.p[0]=0;
       T_rotatedhand_lhand.p[1]=0;
       T_rotatedhand_lhand.p[2]=0.125;
        
       T_rotatedhand_rhand.p=T_rotatedhand_lhand.p;
       if((min_dimension_tag=="XY")&&(fabs(uz.dot(nray))<=max(fabs(ux.dot(nray)),fabs(uy.dot(nray))))){     
        T_rotatedhand_lhand.M =  KDL::Rotation::RPY(-3*(M_PI/8),0,0);
        T_rotatedhand_rhand.M =  KDL::Rotation::RPY(3*(M_PI/8),0,0);
       }
       else if(min_dimension_tag=="Z"){ 
        T_rotatedhand_lhand.M =  KDL::Rotation::RPY(-M_PI/8,0,0);
        T_rotatedhand_rhand.M =  KDL::Rotation::RPY(-M_PI/8,0,0);
       }
      }
      else{
        if((min_dimension_tag=="XY")&&(fabs(uz.dot(nray))<=max(fabs(ux.dot(nray)),fabs(uy.dot(nray))))){     
          T_rotatedhand_lhand.M =  KDL::Rotation::RPY((M_PI/2),0,0);
          T_rotatedhand_rhand.M =  KDL::Rotation::RPY((M_PI/2),0,0);
         }
         else if(min_dimension_tag=="Z"){ 
          T_rotatedhand_lhand.M =  KDL::Rotation::RPY(0,0,0);
          T_rotatedhand_rhand.M =  KDL::Rotation::RPY(0,0,0);
        } 
      }

    T_objectgeometry_lhand = T_objectgeometry_hand *(T_rotatedhand_lhand.Inverse());//T_objectgeometry_rotatedhand 
    T_objectgeometry_rhand = T_objectgeometry_hand *(T_rotatedhand_rhand.Inverse());//T_objectgeometry_rotatedhand 

  }// end get_hand_approach
  

  //---------------------------------------------------------------------------------------------------------------
  bool get_stickyhand_init_positions(string& object_name, string& geometry_name,OtdfInstanceStruc &obj,
                         Eigen::Vector3f &ray_start,Eigen::Vector3f &ray_hit,Eigen::Vector3f &ray_hit_drag, bool dragging,
                         KDL::Frame &T_graspgeometry_lhandinitpos, KDL::Frame &T_graspgeometry_rhandinitpos, bool is_sandia)
  {
  
      KDL::Frame T_world_graspgeometry = KDL::Frame::Identity();
      if(!obj._gl_object->get_link_geometry_frame(geometry_name,T_world_graspgeometry))
      {
        cerr << " failed to retrieve " << geometry_name<<" in object " << object_name <<endl;
        return false;
      }
      else {
        KDL::Frame T_graspgeometry_world = T_world_graspgeometry.Inverse();
        Eigen::Vector3d from_geomframe,hit_pt_geomframe;
        rotate_eigen_vector_given_kdl_frame(ray_start,T_graspgeometry_world,from_geomframe);
        rotate_eigen_vector_given_kdl_frame(ray_hit,T_graspgeometry_world,hit_pt_geomframe);


        boost::shared_ptr<otdf::Geometry> link_geom;
        obj._gl_object->get_link_geometry(geometry_name,link_geom); 
        
        // when not dragging set handpose deterministically (assumes z is up). 
        // Other wise you the user specified orientation as the direction of the fingers.        
        Eigen::Vector3f diff = ray_hit_drag - ray_hit;
        double length =diff.norm();
        if ((dragging)&&(length>1e-3))
        {
           Eigen::Vector3f diff = ray_hit_drag - ray_hit; // finger direction in world frame
           Eigen::Vector3d fingerdir_geomframe;//(temp.data);// finger direction in geometry frame
           diff.normalize();
           KDL::Frame temp_frame = T_graspgeometry_world;
           temp_frame.p = 0*temp_frame.p;// ignore translation while dealing with direction vectors
           rotate_eigen_vector_given_kdl_frame(diff,temp_frame,fingerdir_geomframe);//something wrong here.
           fingerdir_geomframe.normalize();
           get_user_specified_hand_approach(fingerdir_geomframe,from_geomframe,hit_pt_geomframe,link_geom,T_graspgeometry_lhandinitpos,T_graspgeometry_rhandinitpos,is_sandia);
        }
        else{
           get_hand_approach(from_geomframe,hit_pt_geomframe,link_geom,T_graspgeometry_lhandinitpos,T_graspgeometry_rhandinitpos,is_sandia);
        }//end else
              
        return true;
      }//end else
      
      return false;  
  }// end get_stickyhand_init_positions
  
   //--------------------------------------------------------------------------------------------------------------- 
  void set_joint_position(StickyHandStruc &hand_struc,std::string &name, double val)
  {
    std::vector<std::string>::const_iterator found;
    found = std::find (hand_struc.joint_name.begin(), hand_struc.joint_name.end(), name); 

     if (found != hand_struc.joint_name.end()) {
        unsigned int index = found - hand_struc.joint_name.begin();
        hand_struc.joint_position[index]=val;
        hand_struc.optimized_joint_position[index]=val;
     }
  }
  
  
  //---------------------------------------------------------------------------------------------------------------   
  // Eigen Grasps - Hard Coded
  // --------------
  void get_eigen_grasp_types(int hand_type,std::vector<std::string> &_names)
  {
    _names.clear();
    if((hand_type==drc::desired_grasp_state_t::SANDIA_LEFT)||(hand_type==drc::desired_grasp_state_t::SANDIA_RIGHT))
    {
      _names.push_back("Cylindrical");
      _names.push_back("Hook-2fgrs");
      _names.push_back("Hook-3fgrs");
      _names.push_back("ZeroThumb");
      _names.push_back("FlipPinky");
      _names.push_back("ZeroThumbBase");
      _names.push_back("Cyl-NoThumb");
    }
    else if((hand_type==drc::desired_grasp_state_t::IROBOT_LEFT)||(hand_type==drc::desired_grasp_state_t::IROBOT_RIGHT))
    {
      _names.push_back("Cylindrical");
    }
    else if((hand_type==drc::desired_grasp_state_t::ROBOTIQ_LEFT)||(hand_type==drc::desired_grasp_state_t::ROBOTIQ_RIGHT))
    {
      _names.push_back("Nominal");
      _names.push_back("Pinch");
      _names.push_back("Wide");
      _names.push_back("Zero");
    }
  }
  
   //--------------------------------------------------------------------------------------------------------------- 
  void set_eigen_grasp(StickyHandStruc &hand_struc,int eig_grasp_type)
  {
      int hand_type = hand_struc.hand_type; //SANDIA_LEFT=0, SANDIA_RIGHT=1, SANDIA_BOTH=2, IROBOT_LEFT=3, IROBOT_RIGHT=4, IROBOT_BOTH=5;
      bool is_sandia = ((hand_struc.hand_type == StickyHandStruc::SANDIA_RIGHT)||(hand_struc.hand_type == StickyHandStruc::SANDIA_LEFT));
      bool is_irobot = ((hand_struc.hand_type == StickyHandStruc::IROBOT_RIGHT)||(hand_struc.hand_type == StickyHandStruc::IROBOT_LEFT));
      bool is_robotiq = ((hand_struc.hand_type == StickyHandStruc::ROBOTIQ_RIGHT)||(hand_struc.hand_type == StickyHandStruc::ROBOTIQ_LEFT));
    
      bool is_left = true;
      if((hand_struc.hand_type == StickyHandStruc::SANDIA_RIGHT)||(hand_struc.hand_type == StickyHandStruc::IROBOT_RIGHT)||(hand_struc.hand_type == StickyHandStruc::ROBOTIQ_RIGHT))
      is_left = false;      
 
      string side;
      if(is_left)
       side ="left_";
      else
       side ="right_";
      std::string name;
      
      if(is_sandia)
      {
      /* _names.push_back("Cylindrical");
      _names.push_back("Hook-2fgrs");
      _names.push_back("Hook-3fgrs");
      _names.push_back("ZeroThumb");
      _names.push_back("FlipPinky");
      _names.push_back("ZeroThumbBase");
      _names.push_back("Cyl-NoThumb");*/
        if(eig_grasp_type == 0)
        {
            name = side+"f0_j0"; set_joint_position(hand_struc,name, 0);
            name = side+"f0_j1"; set_joint_position(hand_struc,name, 1.5);   
            name = side+"f0_j2"; set_joint_position(hand_struc,name, 1.7);
            name = side+"f1_j0"; set_joint_position(hand_struc,name, 0);
            name = side+"f1_j1"; set_joint_position(hand_struc,name, 1.5);   
            name = side+"f1_j2"; set_joint_position(hand_struc,name, 1.7); 
            name = side+"f2_j0"; set_joint_position(hand_struc,name, 0);
            name = side+"f2_j1"; set_joint_position(hand_struc,name, 1.5);   
            name = side+"f2_j2"; set_joint_position(hand_struc,name, 1.7);   
            name = side+"f3_j0"; set_joint_position(hand_struc,name, 0.2);
            name = side+"f3_j1"; set_joint_position(hand_struc,name, 1.3);   
            name = side+"f3_j2"; set_joint_position(hand_struc,name, 0.5);
        }
        else if(eig_grasp_type == 1)
        {
            name = side+"f0_j0"; set_joint_position(hand_struc,name, 0);
            name = side+"f0_j1"; set_joint_position(hand_struc,name, 1.35);   
            name = side+"f0_j2"; set_joint_position(hand_struc,name, 0.0);
            name = side+"f1_j0"; set_joint_position(hand_struc,name, 0);
            name = side+"f1_j1"; set_joint_position(hand_struc,name, 1.35);   
            name = side+"f1_j2"; set_joint_position(hand_struc,name, 0.00); 
            name = side+"f2_j0"; set_joint_position(hand_struc,name, 0);
            name = side+"f2_j1"; set_joint_position(hand_struc,name, -1.5);   
            name = side+"f2_j2"; set_joint_position(hand_struc,name, -1.0);   
            name = side+"f3_j0"; set_joint_position(hand_struc,name, 0.0);
            name = side+"f3_j1"; set_joint_position(hand_struc,name, 0.0);   
            name = side+"f3_j2"; set_joint_position(hand_struc,name, 0.0);     
        }
        else if(eig_grasp_type == 2)
        {
            name = side+"f0_j0"; set_joint_position(hand_struc,name, 0);
            name = side+"f0_j1"; set_joint_position(hand_struc,name, 1.35);   
            name = side+"f0_j2"; set_joint_position(hand_struc,name, 0.0);
            name = side+"f1_j0"; set_joint_position(hand_struc,name, 0);
            name = side+"f1_j1"; set_joint_position(hand_struc,name, 1.35);   
            name = side+"f1_j2"; set_joint_position(hand_struc,name, 0.00); 
            name = side+"f2_j0"; set_joint_position(hand_struc,name, 0);
            name = side+"f2_j1"; set_joint_position(hand_struc,name, 1.35);   
            name = side+"f2_j2"; set_joint_position(hand_struc,name, 0.00);  
            name = side+"f3_j0"; set_joint_position(hand_struc,name, 0.0);
            name = side+"f3_j1"; set_joint_position(hand_struc,name, 0.0);   
            name = side+"f3_j2"; set_joint_position(hand_struc,name, 0.0);       
        }
        else if(eig_grasp_type == 3)
        {  
            name = side+"f3_j0"; set_joint_position(hand_struc,name, 0.0);
            name = side+"f3_j1"; set_joint_position(hand_struc,name, 0.0);   
            name = side+"f3_j2"; set_joint_position(hand_struc,name, 0.0);    
        }
        else if(eig_grasp_type == 4)
        {  
            name = side+"f2_j0"; set_joint_position(hand_struc,name, 0);
            name = side+"f2_j1"; set_joint_position(hand_struc,name, -1.5);   
            name = side+"f2_j2"; set_joint_position(hand_struc,name, -1.0);   
        }  
        else if(eig_grasp_type == 5)
        {    
         name = side+"f3_j0"; set_joint_position(hand_struc,name, 0); 
        }
        else if(eig_grasp_type == 6)
        {  
            name = side+"f0_j0"; set_joint_position(hand_struc,name, 0);
            name = side+"f0_j1"; set_joint_position(hand_struc,name, 1.5);   
            name = side+"f0_j2"; set_joint_position(hand_struc,name, 1.7);
            name = side+"f1_j0"; set_joint_position(hand_struc,name, 0);
            name = side+"f1_j1"; set_joint_position(hand_struc,name, 1.5);   
            name = side+"f1_j2"; set_joint_position(hand_struc,name, 1.7); 
            name = side+"f2_j0"; set_joint_position(hand_struc,name, 0);
            name = side+"f2_j1"; set_joint_position(hand_struc,name, 1.5);   
            name = side+"f2_j2"; set_joint_position(hand_struc,name, 1.7);   
        }              
      }
      else if(is_irobot)
      {
         if(eig_grasp_type == 0)
         {
          name = side+"finger[0]/joint_base_rotation"; set_joint_position(hand_struc,name, 0);
          name = side+"finger[0]/joint_base"; set_joint_position(hand_struc,name, 2.0);   
          name = side+"finger[0]/joint_flex"; set_joint_position(hand_struc,name, 0.5); 
          name = side+"finger[1]/joint_base_rotation"; set_joint_position(hand_struc,name, 0);
          name = side+"finger[1]/joint_base"; set_joint_position(hand_struc,name, 2.0);   
          name = side+"finger[1]/joint_flex"; set_joint_position(hand_struc,name, 0.5); 
          name = side+"finger[2]/joint_base_rotation"; set_joint_position(hand_struc,name, 0);
          name = side+"finger[2]/joint_base"; set_joint_position(hand_struc,name, 2.0);   
          name = side+"finger[2]/joint_flex"; set_joint_position(hand_struc,name, 0.5);    
         }
      
      }
      else if(is_robotiq)
      {
     /* _names.push_back("Nominal");
      _names.push_back("Pinch");
      _names.push_back("Wide");
      _names.push_back("Zero");*/
         if(eig_grasp_type == 0)
         {
          name = side+"finger_1_joint_1"; set_joint_position(hand_struc,name, 1.0);
          name = side+"finger_1_joint_2"; set_joint_position(hand_struc,name, 1.0);   
          name = side+"finger_1_joint_3"; set_joint_position(hand_struc,name, 0); 
          name = side+"finger_2_joint_1"; set_joint_position(hand_struc,name, 1.0);
          name = side+"finger_2_joint_2"; set_joint_position(hand_struc,name, 1.0);   
          name = side+"finger_2_joint_3"; set_joint_position(hand_struc,name, 0); 
          name = side+"finger_middle_joint_1"; set_joint_position(hand_struc,name, 1.0);
          name = side+"finger_middle_joint_2"; set_joint_position(hand_struc,name, 1.0);   
          name = side+"finger_middle_joint_3"; set_joint_position(hand_struc,name, 0);   
          name = side+"palm_finger_1_joint"; set_joint_position(hand_struc,name, 0);   
          name = side+"palm_finger_2_joint"; set_joint_position(hand_struc,name, 0);    
         }
         else if(eig_grasp_type == 1)
         {
          name = side+"finger_1_joint_1"; set_joint_position(hand_struc,name, 0.8);
          name = side+"finger_1_joint_2"; set_joint_position(hand_struc,name, 0);   
          name = side+"finger_1_joint_3"; set_joint_position(hand_struc,name, 0); 
          name = side+"finger_2_joint_1"; set_joint_position(hand_struc,name, 0.8);
          name = side+"finger_2_joint_2"; set_joint_position(hand_struc,name, 0);   
          name = side+"finger_2_joint_3"; set_joint_position(hand_struc,name, 0); 
          name = side+"finger_middle_joint_1"; set_joint_position(hand_struc,name, 0.8);
          name = side+"finger_middle_joint_2"; set_joint_position(hand_struc,name, 0);   
          name = side+"finger_middle_joint_3"; set_joint_position(hand_struc,name, 0);   
          //lower = "-0.16" upper="0.25" 
          name = side+"palm_finger_1_joint"; set_joint_position(hand_struc,name, -0.16);  
          //lower = "-0.25" upper="0.16"  
          name = side+"palm_finger_2_joint"; set_joint_position(hand_struc,name, 0.16);    
         }
         else if(eig_grasp_type == 2)
         {
          name = side+"finger_1_joint_1"; set_joint_position(hand_struc,name, 1.0);
          name = side+"finger_1_joint_2"; set_joint_position(hand_struc,name, 1.0);   
          name = side+"finger_1_joint_3"; set_joint_position(hand_struc,name, 0); 
          name = side+"finger_2_joint_1"; set_joint_position(hand_struc,name, 1.0);
          name = side+"finger_2_joint_2"; set_joint_position(hand_struc,name, 1.0);   
          name = side+"finger_2_joint_3"; set_joint_position(hand_struc,name, 0); 
          name = side+"finger_middle_joint_1"; set_joint_position(hand_struc,name, 1.0);
          name = side+"finger_middle_joint_2"; set_joint_position(hand_struc,name, 1.0);   
          name = side+"finger_middle_joint_3"; set_joint_position(hand_struc,name, 0); 
           //lower = "-0.16" upper="0.25"    
          name = side+"palm_finger_1_joint"; set_joint_position(hand_struc,name, 0.25);
          //lower = "-0.25" upper="0.16"    
          name = side+"palm_finger_2_joint"; set_joint_position(hand_struc,name, -0.25);    
         } 
         else if(eig_grasp_type == 3)
         {
          name = side+"finger_1_joint_1"; set_joint_position(hand_struc,name, 0.0);
          name = side+"finger_1_joint_2"; set_joint_position(hand_struc,name, 0.0);   
          name = side+"finger_1_joint_3"; set_joint_position(hand_struc,name, 0.0); 
          name = side+"finger_2_joint_1"; set_joint_position(hand_struc,name, 0.0);
          name = side+"finger_2_joint_2"; set_joint_position(hand_struc,name, 0.0);   
          name = side+"finger_2_joint_3"; set_joint_position(hand_struc,name, 0.0); 
          name = side+"finger_middle_joint_1"; set_joint_position(hand_struc,name, 0.0);
          name = side+"finger_middle_joint_2"; set_joint_position(hand_struc,name, 0.0);   
          name = side+"finger_middle_joint_3"; set_joint_position(hand_struc,name, 0.0);   
          name = side+"palm_finger_1_joint"; set_joint_position(hand_struc,name, 0.0);   
          name = side+"palm_finger_2_joint"; set_joint_position(hand_struc,name, 0.0);    
         }    
      }
     
     drc::joint_angles_t posture_msg;
     posture_msg.num_joints= hand_struc.joint_name.size();
     posture_msg.joint_name = hand_struc.joint_name;
     posture_msg.joint_position = hand_struc.joint_position;  
     hand_struc._gl_hand->set_state(hand_struc._gl_hand->_T_world_body,posture_msg);

  }

//--------------------------------------------------------------------------------------------------------  

}//end_namespace

  
