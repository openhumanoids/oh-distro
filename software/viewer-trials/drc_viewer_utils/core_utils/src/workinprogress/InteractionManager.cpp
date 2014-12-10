#include "InteractionManager.hpp"

using namespace std;
using namespace boost;
using namespace collision;
using namespace visualization_utils;

double InteractionManager::get_shortest_distance(Eigen::Vector3f &ray_start,Eigen::Vector3f &ray_dir)
   
{
  autoclean();
  update_params(ray_start,ray_dir);
  
  collision::Collision_Object * intersected_object = NULL;
  Eigen::Vector3f hit_pt;
  double shortest_distance = -1; 
  
  std::vector<ItemWeakPtr>::iterator iter = _items.begin();
  while(iter != _items.end())
  {
     if (!(*iter).expired())
     {
        //dosomething();

          if(robotStateListener->_gl_robot) // to make sure that _gl_robot is initialized 
          {
           //robotStateListener->_gl_robot->_collision_detector->num_collisions();
             if((*iter)->is_jointdof_adjustment_enabled())
                (*iter)->_collision_detector_jointdof_markers->ray_test( ray_start, ray_end, intersected_object,hit_pt);
             else if((*iter)->is_bodypose_adjustment_enabled())
                (*iter)->_collision_detector_bodypose_markers->ray_test( ray_start, ray_end, intersected_object,hit_pt);
             else
                (*iter)->_collision_detector->ray_test(ray_start, ray_end, intersected_object,hit_pt);
          }
        
          if( intersected_object != NULL ){
            Eigen::Vector3f diff = (ray_start-hit_pt);
              double distance = diff.norm();
             // std::cout  << "RobotStateRenderer distance " << distance << std::endl;
           
             if(shortest_distance>0) {
                shortest_distance = distance;
                _ray_hit = hit_pt;
                _ray_hit_drag = hit_pt; // reset on pick query.
                _ray_hit_t = (hit_pt - _ray_start).norm();
                _selectionManager.add(std::string(intersected_object->id().c_str()));
                /*
                if(((*iter)->is_jointdof_adjustment_enabled())||((*iter)->is_bodypose_adjustment_enabled()))
                 //(*self->marker_selection)  = string(intersected_object->id().c_str());
                else
                 //(*self->selection)  = std::string(intersected_object->id().c_str()); 
                */      
             }
             else {
                shortest_distance = distance;
                _ray_hit = hit_pt;
                _ray_hit_drag = hit_pt;
                _ray_hit_t = (hit_pt - _ray_start).norm();
                _selectionManager.add(std::string(intersected_object->id().c_str()));
                /*if(((*iter)->is_jointdof_adjustment_enabled())||((*iter)->is_bodypose_adjustment_enabled()))
                 //(*self->marker_selection)  = string(intersected_object->id().c_str());
                else
                 //(*self->selection)  = std::string(intersected_object->id().c_str());
                 */
             }
          }
          else {
            // (*self->selection)  = " ";
             //(*self->marker_selection)  = " ";
             _selectionManager.clear();
             string no_selection = " ";
             (*iter)->highlight_link(no_selection); 
             (*iter)->highlight_marker(no_selection);
          }    
          
        ++iter;
     }
  }  

  return shortest_distance;

}

// calls set_state() of a selected+marker enabled InteractableGlKinematicBody
void InteractionManager::adjust_state_on_marker_motion()
{


}

// calls set_future_state() of a selected+marker enabled InteractableGlKinematicBody
void InteractionManager::adjust_future_state_on_marker_motion(Eigen::Vector3f &start,Eigen::Vector3f &dir)
{
  autoclean();
  update_params(start,dir);
  
  double gain = 1;
  std::vector<ItemWeakPtr>::iterator iter = _items.begin();
  while(iter != _items.end())
  {
     if (!(*iter).expired())
     {
     
        if(!(*iter)->is_future_state_changing()) {
        (*iter)->set_future_state_changing(true);  
        }//end if(!(*iter)->is_future_state_changing())    

        // set desired state
        KDL::Frame T_world_body_future = (*iter)->_T_world_body_future;
        double currentAngle, angleTo,dtheta;       
        KDL::Frame DragRotation=KDL::Frame::Identity();       
        if((*iter)->is_jointdof_adjustment_enabled())
        {
          //===========================================================================
          // set joint dof

          string link_name = (*self->selection); 
          string marker_name = (*self->marker_selection); 
          string token  = "markers::";
          size_t found = marker_name.find(token);  
          if (found==std::string::npos)
              return;
          string joint_name =marker_name.substr(found+token.size());
          
          //std::cout <<"markername: "<< marker_name<< " mouse on joint marker: " << joint_name << std::endl;


        // Get joint marker draw frame

          
          visualization_utils::JointFrameStruct jointInfo;
          (*iter)->get_joint_info(joint_name,jointInfo);
          KDL::Frame T_world_body = (*iter)->_T_world_body;
          KDL::Frame T_world_body_future = (*iter)->_T_world_body_future;
          
          Eigen::Vector3f joint_axis;
          if((*iter)->is_future_display_active())        
            joint_axis << jointInfo.future_axis[0],jointInfo.future_axis[1],jointInfo.future_axis[2];
          else
            joint_axis << jointInfo.axis[0],jointInfo.axis[1],jointInfo.axis[2]; // in world frame
          joint_axis.normalize();
         
          
          Eigen::Vector3f u_body_to_joint;
          KDL::Frame T_world_joint;
          T_world_joint = jointInfo.future_frame;
          u_body_to_joint[0] = T_world_body_future.p[0]-T_world_joint.p[0];
          u_body_to_joint[1] = T_world_body_future.p[1]-T_world_joint.p[1];
          u_body_to_joint[2] = T_world_body_future.p[2]-T_world_joint.p[2];
          u_body_to_joint.normalize();

          
          double normal = acos(u_body_to_joint.dot(joint_axis));
          double flipped = acos(u_body_to_joint.dot(-joint_axis));
          
          KDL::Frame T_world_jointaxis;// Axis
          T_world_jointaxis.p = jointInfo.future_frame.p;

          double theta;
          Eigen::Vector3f axis;      
          Eigen::Vector3f uz; 
          uz << 0 , 0 , 1; 
          axis = uz.cross(joint_axis);
          axis.normalize();
          theta = acos(uz.dot(joint_axis));
          KDL::Vector axis_temp;
          axis_temp[0]=axis[0];axis_temp[1]=axis[1];axis_temp[2]=axis[2];
          T_world_jointaxis.M = KDL::Rotation::Rot(axis_temp,theta); //T_axis_world
          
          KDL::Frame T_world_marker = KDL::Frame::Identity();
          KDL::Frame T_jointaxis_marker = KDL::Frame::Identity();
          double arrow_length =0.2;
          if(flipped>normal+1e-1) {
            T_jointaxis_marker.p[2] =-2*arrow_length/3;
           }
          else{
            T_jointaxis_marker.p[2] = 2*arrow_length/3;
          }
          T_world_marker = T_world_jointaxis*T_jointaxis_marker; // T_axismarker_world = T_axismarker_axis*T_axis_world
          
         // proper hit_drag point via marker plane ray intersection.
          Eigen::Vector3f plane_normal,plane_point;
            
          plane_normal = joint_axis;
          plane_point[0]=T_world_marker.p[0];
          plane_point[1]=T_world_marker.p[1];
          plane_point[2]=T_world_marker.p[2];       
          double lambda1 = dir.dot(plane_normal);
          double lambda2 = (plane_point - start).dot(plane_normal);
          double t;

         // check for degenerate case where ray is (more or less) parallel to plane
         if (fabs (lambda1) >= 1e-9) {
           t = lambda2 / lambda1;
            _ray_hit_drag << start[0]+t*dir[0], start[1]+t*dir[1], start[2]+t*dir[2];  
           }
          // else  no solution        
          
          
          Eigen::Vector3f diff = _prev_ray_hit_drag - _ray_hit_drag; 
          if(diff.norm() > 0.05){
            _prev_ray_hit_drag = _ray_hit_drag; 
          }
          Eigen::Vector3f hit_markerframe,hitdrag_markerframe;
          //convert to joint dof marker frame .
          rotate_eigen_vector_given_kdl_frame(_prev_ray_hit_drag,T_world_marker.Inverse(),hit_markerframe); 
          rotate_eigen_vector_given_kdl_frame(_ray_hit_drag,T_world_marker.Inverse(),hitdrag_markerframe); 

          int type = jointInfo.type;   
          if((type==otdf::Joint::REVOLUTE)||(type==otdf::Joint::CONTINUOUS))
          {          
            double currentAngle, angleTo, dtheta;         
            currentAngle = atan2(hit_markerframe[1],hit_markerframe[0]);
            angleTo = atan2(hitdrag_markerframe[1],hitdrag_markerframe[0]);
           /* cout << "currentAngle :"<< currentAngle*(180/M_PI)
                 << " angleTo :"<< angleTo*(180/M_PI) <<endl;
            cout << "radius" << sqrt(pow(hitdrag_markerframe[0],2)+pow(hitdrag_markerframe[1],2)) << endl; */
            dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
            std::map<std::string, double> jointpos_in;
            jointpos_in = (*iter)->_future_jointpos;
            jointpos_in.find(joint_name)->second = (jointpos_in.find(joint_name)->second +dtheta); 
            (*iter)->set_future_state(T_world_body_future,jointpos_in);
           }// end revolute joints
           else if(type==otdf::Joint::PRISMATIC)
           {
            double s=1;
            if(diff.dot(joint_axis)<0)
              s = -1;
            double distance = s*diff.norm();
             std::map<std::string, double> jointpos_in;
             jointpos_in = (*iter)->_future_jointpos;
             jointpos_in.find(joint_name)->second -= distance;
             (*iter)->set_future_state(T_world_body_future,jointpos_in);       
           }
        }

        _prev_ray_hit_drag = _ray_hit_drag;      

        ++iter;
     }
  }// end while   
     

}

//------------------------------------------------------------------------------------------
void InteractionManager::update_params(Eigen::Vector3f &ray_start,Eigen::Vector3f &ray_dir)
{

  Eigen::Vector3f to;
  Eigen::Vector3f plane_normal,plane_pt;
  plane_normal << 0,0,1;
  if(ray_start[2]<0)
      plane_pt << 0,0,_plane_height;
  else
      plane_pt << 0,0,-_plane_height;
  double lambda1 = ray_dir[0] * plane_normal[0]+
                   ray_dir[1] * plane_normal[1] +
                   ray_dir[2] * plane_normal[2];
   // check for degenerate case where ray is (more or less) parallel to plane
    if (fabs (lambda1) < 1e-9) return -1.0;

   double lambda2 = (plane_pt[0] - ray_start[0]) * plane_normal[0] +
       (plane_pt[1] - ray_start[1]) * plane_normal[1] +
       (plane_pt[2] - ray_start[2]) * plane_normal[2];
   double t = lambda2 / lambda1;// =1;
  
  to << ray_start[0]+t*ray_dir[0], ray_start[1]+t*ray_dir[1], ray_start[2]+t*ray_dir[2];
 
  _ray_start = ray_start;
  _ray_end = to;
  _ray_hit_t = t;
  _ray_hit_drag = to;
  _ray_hit = to; 
}

void InteractionManager::set_predrag_offsets_on_mouse_press()
{
    /*self->dragging = 1;
     KDL::Frame T_world_body = self->robotStateListener->_gl_robot->_T_world_body;
     self->marker_offset_on_press << self->ray_hit[0]-T_world_body.p[0],self->ray_hit[1]-T_world_body.p[1],self->ray_hit[2]-T_world_body.p[2]; 
     */

}

