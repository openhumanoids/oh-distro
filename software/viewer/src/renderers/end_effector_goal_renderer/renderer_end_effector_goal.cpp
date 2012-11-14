// Renderer for a end-effector-goal-click-and-drag message publisher
// used to send a message to relocalize a robot's end effector
// this was orginally part of envoy/renderers
// ppetrova aug2012
#include <stdio.h>
#include <stdlib.h>

#include <math.h>
#include <string>
#include <boost/algorithm/string.hpp>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>

#include <bot_vis/bot_vis.h>
#include <bot_core/rotations.h>
#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc_lcmtypes.h>
#include <lcmtypes/bot_core.h>

#include "RobotStateListener.hpp"
#include "renderer_end_effector_goal.hpp"

#define RENDERER_NAME "EndEffectorGoal"

#define PARAM_ADJUST_XY "Adjust in XY"
#define PARAM_ADJUST_Z "Adjust in Z"
#define PARAM_ADJUST_X "Adjust in X"
#define PARAM_ADJUST_Y "Adjust in Y"
#define PARAM_CLEAR "Clear EEGoal"
#define PARAM_EE_GOAL "EE Goal"
#define PARAM_L "Left Arm"
#define PARAM_R "Right Arm"
#define VARIANCE_THETA (30.0 * 180.0 / M_PI);
#define SPHERE_RADIUS 0.15
//Not sure why abe has this high a variance for the angle //((2*M_PI)*(2*M_PI))

////////////////// THE FOLLOWING CODE WAS COPIED IN HERE TO AVOID
////////////////// DEPENDENCY WITH THE COMMON_UTILS/GEOM_UTILS POD [MFALLON]
#define GEOM_EPSILON 1e-9

// ===== 2 dimensional structure =====
#ifndef _point2d_t_h
typedef struct _point2d {
	double x;
	double y;
} point2d_t;
#endif

// ===== 3 dimensional strucutres =====
// double 
typedef struct _point3d {
	double x;
	double y;
	double z;
} point3d_t;

#define point3d_as_array(p) ((double*)p)

/* The magic below allows you to use the POINT3D() macro to convert a
 * double[3] to a point3d_t.  gcc is smart -- if you try to cast anything
 * other than a point3d_t or a double[] with this macro, gcc will emit
 * a warning. */
union _point3d_any_t {
	point3d_t point;
	double array[3];
};

typedef point3d_t vec3d_t;

#define POINT3D(p) (&(((union _point3d_any_t *)(p))->point))


// int geom_ray_plane_intersect_3d_1 (const point3d_t *ray_point, const vec3d_t *ray_dir,
//     const point3d_t *plane_point, const vec3d_t *plane_normal,
//     point3d_t *result, double *u)
// {
//   double lambda1 = ray_dir->x * plane_normal->x +
//       ray_dir->y * plane_normal->y +
//       ray_dir->z * plane_normal->z;
// 
//   // check for degenerate case where ray is (more or less) parallel to plane
//   if (fabs (lambda1) < GEOM_EPSILON) return 0;
// 
//   double lambda2 = (plane_point->x - ray_point->x) * plane_normal->x +
//       (plane_point->y - ray_point->y) * plane_normal->y +
//       (plane_point->z - ray_point->z) * plane_normal->z;
//   double v = lambda2 / lambda1;
//   result->x = ray_point->x + v * ray_dir->x;
//   result->y = ray_point->y + v * ray_dir->y;
//   result->z = ray_point->z + v * ray_dir->z;
//   if (u) *u = v;
//   return 1;
// }
// 
// 
// int geom_ray_z_plane_intersect_3d(const point3d_t *ray_point,
// 		const point3d_t *ray_dir, double plane_z, point3d_t *result)
// {
// 	point3d_t plane_pt = { 0, 0, plane_z};
// 	point3d_t plane_normal = { 0, 0, 1};
// 	point3d_t plane_isect_point;
// 	double plane_point_dist;
// 	if (!geom_ray_plane_intersect_3d_1 (ray_point, ray_dir, &plane_pt,
// 			&plane_normal, &plane_isect_point, &plane_point_dist) ||
// 			plane_point_dist <= 0) {
// 		return -1;
// 	}
// 	result->x = plane_isect_point.x;
// 	result->y = plane_isect_point.y;
// 	result->z = 0;
// 	return 0;
// }
// 
// int geom_ray_x_plane_intersect_3d(const point3d_t *ray_point,
// 		const point3d_t *ray_dir, double plane_x, point3d_t *result)
// {
// 	point3d_t plane_pt = {plane_x, 0, 0};
// 	point3d_t plane_normal = { 1, 0, 0};
// 	point3d_t plane_isect_point;
// 	double plane_point_dist;
// 	if (!geom_ray_plane_intersect_3d_1 (ray_point, ray_dir, &plane_pt,
// 			&plane_normal, &plane_isect_point, &plane_point_dist) ||
// 			plane_point_dist <= 0) {
// 		return -1;
// 	}
// 	result->x = 0;
// 	result->y = plane_isect_point.y;
// 	result->z = plane_isect_point.z;
// 	return 0;
// }
// 
// int geom_ray_y_plane_intersect_3d(const point3d_t *ray_point,
// 		const point3d_t *ray_dir, double plane_y, point3d_t *result)
// {
// 	point3d_t plane_pt = { 0, plane_y, 0};
// 	point3d_t plane_normal = { 0, 1, 0};
// 	point3d_t plane_isect_point;
// 	double plane_point_dist;
// 	if (!geom_ray_plane_intersect_3d_1 (ray_point, ray_dir, &plane_pt,
// 			&plane_normal, &plane_isect_point, &plane_point_dist) ||
// 			plane_point_dist <= 0) {
// 		return -1;
// 	}
// 	result->x = plane_isect_point.x;
// 	result->y = 0;
// 	result->z = plane_isect_point.z;
// 	return 0;
// }

typedef struct _RendererEndEffectorGoal {
	BotRenderer renderer;
	BotEventHandler ehandler;
	BotViewer *viewer;
	lcm_t *lcm;
	boost::shared_ptr<lcm::LCM> _lcm;
	//std::string channel;	
  boost::shared_ptr<ee_goal_renderer::RobotStateListener> robotStateListener;
	BotGtkParamWidget *pw;

	int active; // 0 = don't display, 1 = display
	int translate; //0 = init, 1 = x, 2 = y, 3 = z
	int moving; // 0 = no, 1 = yes
	int arm; // 0- left, 1- right
	point3d_t drag_start_local;
	point3d_t drag_finish_local;

	point3d_t dir_start;
	point3d_t dir_finish;

	point2d_t particle_mean;
	double theta;
	double particle_std;
	point3d_t center;

}RendererEndEffectorGoal;

#define X .525731112119133606 
#define Z .850650808352039932

static GLfloat vdata[12][3] = {    
		{-X, 0.0, Z}, {X, 0.0, Z}, {-X, 0.0, -Z}, {X, 0.0, -Z},
		{0.0, Z, X}, {0.0, Z, -X}, {0.0, -Z, X}, {0.0, -Z, -X},
		{Z, X, 0.0}, {-Z, X, 0.0}, {Z, -X, 0.0}, {-Z, -X, 0.0}
};
static GLuint tindices[20][3] = { 
		{0,4,1}, {0,9,4}, {9,5,4}, {4,5,8}, {4,8,1},
		{8,10,1}, {8,3,10}, {5,3,8}, {5,2,3}, {2,7,3},
		{7,10,3}, {7,6,10}, {7,11,6}, {11,0,6}, {0,1,6},
		{6,1,10}, {9,0,11}, {9,11,2}, {9,2,5}, {7,2,11} };

void normalize(GLfloat *a) {
	GLfloat d=sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
	a[0]/=d; a[1]/=d; a[2]/=d;
}

void drawtri(GLfloat *a, GLfloat *b, GLfloat *c, int div, float r) {
	if (div<=0) {
		glNormal3fv(a); glVertex3f(a[0]*r, a[1]*r, a[2]*r);
		glNormal3fv(b); glVertex3f(b[0]*r, b[1]*r, b[2]*r);
		glNormal3fv(c); glVertex3f(c[0]*r, c[1]*r, c[2]*r);
	} else {
		GLfloat ab[3], ac[3], bc[3];
		for (int i=0;i<3;i++) {
			ab[i]=(a[i]+b[i])/2;
			ac[i]=(a[i]+c[i])/2;
			bc[i]=(b[i]+c[i])/2;
		}
		normalize(ab); normalize(ac); normalize(bc);
		drawtri(a, ab, ac, div-1, r);
		drawtri(b, bc, ab, div-1, r);
		drawtri(c, ac, bc, div-1, r);
		drawtri(ab, bc, ac, div-1, r);
	}
}

/*
 * Draws a sphere using subdivisions. TODO: add static sphere vertices
 */
void drawSphere(int ndiv, float radius) {
	glBegin(GL_TRIANGLES);
	for (int i=0;i<20;i++)
		drawtri(vdata[tindices[i][0]], vdata[tindices[i][1]], vdata[tindices[i][2]], ndiv, radius);
	glEnd();
}
 
// TODO: use dot product from bot-vis
static double dot_product(const point3d_t* a, const point3d_t * b) {
	return a->x*b->x + a->y*b->y + a->z*b->z;
}

/*
 * Return true(1) is the ray intersect a sphere
 */
int geom_ray_intersect_sphere(const point3d_t *ray_point, const point3d_t *ray_dir, point3d_t &intersection_pt, RendererEndEffectorGoal *self, double radius) {

	double a = dot_product(ray_dir, ray_dir);
	const point3d_t x = {2.0*ray_dir->x, 2.0*ray_dir->y, 2.0*ray_dir->z};
	const point3d_t diff = {ray_point->x - self->center.x, ray_point->y - self->center.y, ray_point->z - self->center.z};
	double b = dot_product(&x, &diff);	
	double c = dot_product(&diff, &diff) - radius*radius;
  double disc = b*b - 4*a*c;
  double discSqrt = sqrt(disc);
  double q;
    if (b < 0)
        q = (-b - discSqrt)/2.0;
    else
        q = (-b + discSqrt)/2.0;
        
    // compute t0 and t1
    double t0 = q / a;
    double t1 = c / q;  
  // make sure t0 is smaller than t1
    if (t0 > t1)
    {
        // if t0 is bigger than t1 swap them around
        double temp = t0;
        t0 = t1;
        t1 = temp;
    }

  double t;
	if (disc >= 0) {
	  // if t0 is less than zero, the intersection point is at t1
    if (t0 < 0)
    {
        t = t1;
    }
    // else the intersection point is at t0
    else
    {
        t = t0;
    }  
    intersection_pt = {ray_point->x + t*ray_dir->x, ray_point->y + t*ray_dir->y, ray_point->z + t*ray_dir->z};
		return 1;
	} else {
	  intersection_pt = {0,0,0}; 
		return -1;
	}

}

static void _draw (BotViewer *viewer, BotRenderer *renderer)
{
	RendererEndEffectorGoal *self = (RendererEndEffectorGoal*) renderer;

	// Are we displaying the goal at the moment?
	if (self->active == 0) {
		return;
	}
  glEnable(GL_LINE_SMOOTH); 
	glPushMatrix();

	glTranslatef(self->center.x, self->center.y, self->center.z);

	glLineWidth (3.0);

	float ORG[3] = {0.0, 0.0, 0.0};
	float XP[3] = {0.5, 0.0, 0.0};
	float YP[3] = {0.0, 0.5, 0.0};
	float ZP[3] = {0.0, 0.0, 0.5};

	glBegin (GL_LINES);
	
	glColor3f (1,0,0); // X axis is red.
	glVertex3fv (ORG);
	glVertex3fv (XP );
	glColor3f (0,1,0); // Y axis is green.
	glVertex3fv (ORG);
	glVertex3fv (YP );
	glColor3f (0,0,1); // z axis is blue.
	glVertex3fv (ORG);
	glVertex3fv (ZP );
	glEnd();


  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);
  glEnable(GL_BLEND);
  glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
 	glShadeModel(GL_SMOOTH);
 double alpha = 0.6;
	glColor4f(1.0, 0.647059, 0.00,alpha);
	drawSphere(6, SPHERE_RADIUS);
	glPopMatrix();
}

//static void recompute_particle_distribution(RendererEndEffectorGoal *self)
//{
//	double dx = self->drag_finish_local.x - self->drag_start_local.x;
//	double dy = self->drag_finish_local.y - self->drag_start_local.y;
//	double dz = self->drag_finish_local.z - self->drag_start_local.z;

//	//printf("dx , dy , dz: %f, %f, %f\n", dx, dy, dz);

//	if (self->translate == 0) {
//		self->center.x = self->center.x + dx;
//		self->center.y = self->center.y + dy;
//		self->center.z = self->center.z + dz;
//	} else if (self->translate == 1) {
//		// X translation
//		self->center.x += dx;
//	} else if (self->translate == 2) {
//		// Y translation
//		self->center.y += dy;
//	} else if (self->translate == 3) {
//		// Z translation
//		self->center.z += dz;
//	}
//}

static int mouse_press (BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], const double ray_dir[3], const GdkEventButton *event)
{
	RendererEndEffectorGoal *self = (RendererEndEffectorGoal*) ehandler->user;

	point3d_t ray_s = {ray_start[0], ray_start[1], ray_start[2]};
	point3d_t ray_d = {ray_dir[0], ray_dir[1], ray_dir[2]};
  point3d_t ray_sphere_intersect;
	if (geom_ray_intersect_sphere(&ray_s, &ray_d,ray_sphere_intersect, self,SPHERE_RADIUS) < 0) {
		//printf("NO sphere\n");
		self->moving = 0;
		return 0;
	}
	if (self->translate == 0) {
		//printf("Intersected sphere - NOT moving\n");
		self->moving = 0;
	} else {
		self->moving = 1;
		//printf("Intersected sphere - moving\n");
	}

	if(ehandler->picking==0){
		//fprintf(stderr, "Ehandler Not active\n");
		return 0;
	}

	if(event->button != 1){
		fprintf(stderr,"Wrong Button\n");
		return 0;
	}

//	point3d_t drag_pt_xy;
//	if (0 != geom_ray_z_plane_intersect_3d(POINT3D(ray_start), POINT3D(ray_dir), 0, &drag_pt_xy)) {
//		return 0;
//	} else {
//		self->dir_start = {drag_pt_xy.x, drag_pt_xy.y, 0.0};
//	}


	bot_viewer_request_redraw(self->viewer);
	return 1;
}

static double getTime_now()
{
	struct timeval tv;
	gettimeofday (&tv,NULL);
	return (int64_t) tv.tv_sec*1000000+tv.tv_usec;
};

static int mouse_release(BotViewer *viewer, BotEventHandler *ehandler,
		const double ray_start[3], const double ray_dir[3],
		const GdkEventButton *event)
{
  RendererEndEffectorGoal *self = (RendererEndEffectorGoal*) ehandler->user;

  if (self->active != 0) {

		// Build the message
	drc::ee_goal_t goalmsg;

    	if(self->robotStateListener->_robot_name == "mit_drc_robot")
	{
	    goalmsg.robot_name =self->robotStateListener->_robot_name;
	    goalmsg.root_name = "utorso";
	    if(self->arm==0)
	    	goalmsg.ee_name = "l_hand";
	    else
		goalmsg.ee_name = "r_hand";

	}
    	else if(self->robotStateListener->_robot_name == "wheeled_atlas")
	{
	    goalmsg.robot_name =self->robotStateListener->_robot_name;
	    goalmsg.root_name = "base";
	    if(self->arm==0)
	    	goalmsg.ee_name = "LWristRoll_link";
	    else
		goalmsg.ee_name = "RWristRoll_link";  
		
	}
	else
	{
            // unknown robot
          std::cout << "Unknown robot in end effector goal renderer" << std::endl;
	}

	goalmsg.utime = getTime_now();


	// TODO: add a check to see if this goal is reachable and only publish if it is within (dx and dy) of the robot
	KDL::Frame  T_world_ee, T_body_world, T_body_ee, T_body_root, T_root_ee;

	T_world_ee.p[0]= self->center.x;
	T_world_ee.p[1]= self->center.y;
	T_world_ee.p[2]= self->center.z;		    
	T_world_ee.M =  KDL::Rotation::RPY(0*(M_PI/180),0*(M_PI/180),0.0);//  KDL::Rotation::Quaternion(0,0,0,1);
	T_body_world= self->robotStateListener->T_body_world;
	T_body_ee = T_body_world*T_world_ee;

        if(self->robotStateListener->getLinkTf(goalmsg.root_name, T_body_root))
	{
          T_root_ee  = T_body_root.Inverse()*T_body_ee;
	}
	else {
          std::cout << "root link : " << goalmsg.root_name <<" not found in FK" << std::endl;
          T_root_ee =T_body_ee;
	}

	double x,y,z,w;
	T_root_ee.M.GetQuaternion(x,y,z,w);

	goalmsg.ee_goal_pos.translation.x = T_root_ee.p[0];
	goalmsg.ee_goal_pos.translation.y = T_root_ee.p[1];
	goalmsg.ee_goal_pos.translation.z = T_root_ee.p[2];

	goalmsg.ee_goal_pos.rotation.x = x;
	goalmsg.ee_goal_pos.rotation.y = y;
	goalmsg.ee_goal_pos.rotation.z = z;
	goalmsg.ee_goal_pos.rotation.w = w;

	goalmsg.ee_goal_twist.linear_velocity.x = 0.0;
	goalmsg.ee_goal_twist.linear_velocity.y = 0.0;
	goalmsg.ee_goal_twist.linear_velocity.z = 0.0;
	goalmsg.ee_goal_twist.angular_velocity.x = 0.0;
	goalmsg.ee_goal_twist.angular_velocity.y = 0.0;
	goalmsg.ee_goal_twist.angular_velocity.z = 0.0;
	goalmsg.num_chain_joints  = 6;

	// No specified posture bias
	goalmsg.use_posture_bias  = false;
	goalmsg.joint_posture_bias.resize(goalmsg.num_chain_joints);
	goalmsg.chain_joint_names.resize(goalmsg.num_chain_joints);
	for(int i = 0; i < goalmsg.num_chain_joints; i++){
		goalmsg.joint_posture_bias[i]=0;
		goalmsg.chain_joint_names[i]= "dummy_joint_names";
			// goalmsg.joint_posture_bias.push_back(0);
			// std::string dummy = "dummy_joint_names";
			//goalmsg.chain_joint_names.push_back(dummy);
	}

	// Publish the message
	goalmsg.halt_ee_controller = false;
	//self->_goal = goalmsg; // seg faults;

        //drc_ee_goal_t_publish(self->lcm, "LWRISTROLL_LINK_GOAL", &goalmsg);
        std::string channel = boost::to_upper_copy(goalmsg.ee_name)+"_GOAL";
	self->_lcm->publish(channel, &goalmsg);
    
	ehandler->picking = 0;
	bot_viewer_request_redraw(self->viewer);
	self->moving = 0;

	return 1;
  }
  else {
	ehandler->picking = 0;
	bot_viewer_request_redraw(self->viewer);
  }
	return 0;
}

static int mouse_motion (BotViewer *viewer, BotEventHandler *ehandler,
		const double ray_start[3], const double ray_dir[3],
		const GdkEventMotion *event)
{
	RendererEndEffectorGoal *self = (RendererEndEffectorGoal*) ehandler->user;

	if(self->moving == 0) {
		bot_viewer_request_redraw(self->viewer);
		return 0;
	}

//	point3d_t drag_pt_xy; // intersection with xy plane
//	if (0 != geom_ray_z_plane_intersect_3d(POINT3D(ray_start),
//			POINT3D(ray_dir), 0, &drag_pt_xy)) {
//		return 0;
//	}
//	
//	point3d_t drag_pt_z;  // intersection with yz or xz planes
//	if (0 != geom_ray_x_plane_intersect_3d(POINT3D(ray_start),POINT3D(ray_dir),0 , &drag_pt_z)) {
//				if (0 != geom_ray_y_plane_intersect_3d(POINT3D(ray_start),POINT3D(ray_dir),0 , &drag_pt_z)) {
//		      return 0;
//		    }
//	}

	point3d_t dP=  {ray_start[0]-self->center.x, ray_start[1]-self->center.y, ray_start[2]-self->center.z};
	double zoom = 0.5*sqrt(dot_product(&dP,&dP));
	point3d_t ray_s = {ray_start[0], ray_start[1], ray_start[2]};
	point3d_t ray_d = {ray_dir[0], ray_dir[1], ray_dir[2]};
	point3d_t drag_pt_xyz;
	int intersects = geom_ray_intersect_sphere(&ray_s, &ray_d,drag_pt_xyz,self,zoom);
	drag_pt_xyz =  {drag_pt_xyz.x+zoom*ray_dir[0], drag_pt_xyz.y+zoom*ray_dir[1], drag_pt_xyz.z+zoom*ray_dir[2]};
  
	if (intersects < 0) {
	  drag_pt_xyz = {self->center.x, self->center.y, self->center.z};
	}


	if (self->translate == 1) {

		double dx =  drag_pt_xyz.x - self->center.x;
		self->center.x = self->center.x +  dx;

	} else if (self->translate == 2) {

		double dy = drag_pt_xyz.y - self->center.y;
		self->center.y = self->center.y +  dy;

	} else if (self->translate == 3) {

		double dz = drag_pt_xyz.z - self->center.z;
		self->center.z = self->center.z +  dz;

	} else if (self->translate == 4) {
		double dx = drag_pt_xyz.x - self->center.x;
   	double dy = drag_pt_xyz.y - self->center.y;
   	self->center.x = self->center.x +  dx;
		self->center.y = self->center.y +  dy;
	}

	bot_viewer_request_redraw(self->viewer);
	return 1;
}

void activate(RendererEndEffectorGoal *self, int type)
{
	self->active = type;
}

static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
	RendererEndEffectorGoal *self = (RendererEndEffectorGoal*) user;
	if(!strcmp(name, PARAM_EE_GOAL)) {
		fprintf(stderr,"Clicked L EE goal, activate\n");
	
   	KDL::Frame  T_world_ee,T_body_world,T_root_ee,T_body_ee,T_body_root;
	drc::ee_goal_t goalmsg;  
    	if(self->robotStateListener->_robot_name == "mit_drc_robot")
	{
		goalmsg.robot_name =self->robotStateListener->_robot_name;
		goalmsg.root_name = "utorso";
		if(self->arm==0)
			goalmsg.ee_name = "l_hand";
		else
			goalmsg.ee_name = "r_hand";

	}
	else if(self->robotStateListener->_robot_name == "wheeled_atlas")
	{
		goalmsg.robot_name =self->robotStateListener->_robot_name;
		goalmsg.root_name = "base";
		if(self->arm==0)
			goalmsg.ee_name = "LWristRoll_link";
		else
			goalmsg.ee_name = "RWristRoll_link";  
		
	}
	else
	{
		 // unknown robot
	   std::cout << "Unknown robot in end effector goal renderer" << std::endl;
	}

// TODO:: adjust initial position relative to root link	

	   if(!self->robotStateListener->getLinkTf(goalmsg.root_name, T_body_root))
	   {
             std::cout << "root link : " << goalmsg.root_name <<" not found in FK" << std::endl;
	     T_body_root = KDL::Frame::Identity();
	   }

	 T_root_ee.p[0]= 0.35;
	 T_root_ee.p[1]= 0.3;
	 T_root_ee.p[2]= 0.5;
// 	 T_body_ee.p[0]= 0.0;
// 	 T_body_ee.p[1]= 0.241059; 
// 	 T_body_ee.p[2] = 0.312513;

	 T_body_ee = T_body_root*T_root_ee;
	 if (self->arm ==1)
	    T_body_ee.p[1]=  -T_body_ee.p[1];
	 
	 T_body_ee.M =  KDL::Rotation::Quaternion(0,0,0,1); // TODO: Need an intelligent way to set orientation
	 // T_body_ee.M =  KDL::Rotation::RPY(0.0,80*(M_PI/180),0.0);
	 KDL::Vector v(T_body_ee.p[0],T_body_ee.p[1],T_body_ee.p[2]); //vector from body origin to goal
	 v.Normalize();
	
	  T_body_world= self->robotStateListener->T_body_world;
	  T_world_ee = (T_body_world.Inverse())*T_body_ee;
	  
		self->center = {T_world_ee.p[0], T_world_ee.p[1],T_world_ee.p[2]};
		//self->center = {0.0, 0.0, 0.0};
		//self->center = {0.47, 0.15, 1.0};
		bot_viewer_request_pick (self->viewer, &(self->ehandler));
		activate(self, 2);
	}
        else if (!strcmp(name, PARAM_CLEAR)) {
		fprintf(stderr,"Clicked clear goal\n");
		
   	 	drc::ee_goal_t goalmsg;   
		goalmsg.utime = getTime_now();
		
	    	if(self->robotStateListener->_robot_name == "mit_drc_robot")
		{
			goalmsg.robot_name =self->robotStateListener->_robot_name;
			goalmsg.root_name = "utorso";
			if(self->arm==0)
				goalmsg.ee_name = "l_hand";
			else
				goalmsg.ee_name = "r_hand";

		}
		else if(self->robotStateListener->_robot_name == "wheeled_atlas")
		{
			goalmsg.robot_name =self->robotStateListener->_robot_name;
			goalmsg.root_name = "base";
			if(self->arm==0)
				goalmsg.ee_name = "LWristRoll_link";
			else
				goalmsg.ee_name = "RWristRoll_link";  
		
		}
		else
		{
			 // unknown robot
		   std::cout << "Unknown robot in end effector goal renderer" << std::endl;
		}
		goalmsg.ee_goal_pos.translation.x = 0;
		goalmsg.ee_goal_pos.translation.y = 0;
		goalmsg.ee_goal_pos.translation.z = 0;
		goalmsg.ee_goal_pos.rotation.x = 0;
		goalmsg.ee_goal_pos.rotation.y = 0;
		goalmsg.ee_goal_pos.rotation.z = 0;
		goalmsg.ee_goal_pos.rotation.w = 1;
		goalmsg.ee_goal_twist.linear_velocity.x = 0.0;
		goalmsg.ee_goal_twist.linear_velocity.y = 0.0;
		goalmsg.ee_goal_twist.linear_velocity.z = 0.0;
		goalmsg.ee_goal_twist.angular_velocity.x = 0.0;
		goalmsg.ee_goal_twist.angular_velocity.y = 0.0;
		goalmsg.ee_goal_twist.angular_velocity.z = 0.0;
		goalmsg.num_chain_joints  = 6;
		goalmsg.use_posture_bias  = false;
		goalmsg.joint_posture_bias.resize(goalmsg.num_chain_joints);
		goalmsg.chain_joint_names.resize(goalmsg.num_chain_joints);
		for(int i = 0; i < goalmsg.num_chain_joints; i++){
			goalmsg.joint_posture_bias[i]=0;
			goalmsg.chain_joint_names[i]= "dummy_joint_names";
		}

		// Publish the message
		goalmsg.halt_ee_controller = true;
        	std::string channel = boost::to_upper_copy(goalmsg.ee_name)+"_GOAL";
		self->_lcm->publish(channel, &goalmsg);
		
		bot_viewer_request_pick (self->viewer, &(self->ehandler));
		activate(self, 0);
	}
	else if (! strcmp(name, PARAM_ADJUST_X)) {
		bot_viewer_request_pick (self->viewer, &(self->ehandler));
		self->translate = 1;
		activate(self, 1);
	}
	else if (! strcmp(name, PARAM_ADJUST_Y)) {
		bot_viewer_request_pick (self->viewer, &(self->ehandler));
		activate(self, 2);
		self->translate = 2;
	}
	else if (! strcmp(name, PARAM_ADJUST_Z)) {
		bot_viewer_request_pick (self->viewer, &(self->ehandler));
		activate(self, 3);
		self->translate = 3;
	}
	else if (! strcmp(name, PARAM_ADJUST_XY)) {
		bot_viewer_request_pick (self->viewer, &(self->ehandler));
		activate(self, 4);
		self->translate = 4;
	}
	else if (! strcmp(name, PARAM_L)) {
		if (bot_gtk_param_widget_get_bool(pw, PARAM_L) > -1) {
			//fprintf(stderr, "Clicked translate X. \n");
			bot_viewer_request_pick(self->viewer, &(self->ehandler));
			bot_gtk_param_widget_set_bool(pw, PARAM_R, 0);
			self->arm = 0;
			
		} 
		else{
		  self->arm = 1;
		}
	}
	else if (! strcmp(name, PARAM_R)) {
		if (bot_gtk_param_widget_get_bool(pw, PARAM_R) > -1) {
			//fprintf(stderr, "Clicked translate X. \n");
			bot_viewer_request_pick(self->viewer, &(self->ehandler));
			bot_gtk_param_widget_set_bool(pw, PARAM_L, 0);
			self->arm = 1;
		}
		else{
		  self->arm = 0;
		}
	}
// 	else if (! strcmp(name, PARAM_TRANSLATE_X)) {
// 		if (bot_gtk_param_widget_get_bool(pw, PARAM_TRANSLATE_X) > -1) {
// 			//fprintf(stderr, "Clicked translate X. \n");
// 			bot_viewer_request_pick(self->viewer, &(self->ehandler));
// 			bot_gtk_param_widget_set_bool(pw, PARAM_TRANSLATE_Y, 0);
// 			bot_gtk_param_widget_set_bool(pw, PARAM_TRANSLATE_Z, 0);
// 			self->translate = 1;
// 			activate(self, 1);
// 		} else {
// 			self->translate = 0;
// 		}
// 	} else if (! strcmp(name, PARAM_TRANSLATE_Y)) {
// 		if (bot_gtk_param_widget_get_bool(pw, PARAM_TRANSLATE_Y) > -1) {
// 			//fprintf(stderr, "Clicked translate Y. \n");
// 			bot_viewer_request_pick(self->viewer, &(self->ehandler));
// 			bot_gtk_param_widget_set_bool(pw, PARAM_TRANSLATE_X, 0);
// 			bot_gtk_param_widget_set_bool(pw, PARAM_TRANSLATE_Z, 0);
// 			self->translate = 2;
// 			activate(self, 1);
// 		} else {
// 			self->translate = 0;
// 		}
// 	} else if (! strcmp(name, PARAM_TRANSLATE_Z)) {
// 		if (bot_gtk_param_widget_get_bool(pw, PARAM_TRANSLATE_Z) > -1) {
// 			//fprintf(stderr, "Clicked translate Z.\n");
// 			bot_viewer_request_pick(self->viewer, &(self->ehandler));
// 			bot_gtk_param_widget_set_bool(pw, PARAM_TRANSLATE_X, 0);
// 			bot_gtk_param_widget_set_bool(pw, PARAM_TRANSLATE_Y, 0);
// 			self->translate = 3;
// 			activate(self, 1);
// 		} else {
// 			self->translate = 0;
// 		}
// 	}
	
}

static void _free (BotRenderer *renderer)
{
	free (renderer);
}

BotRenderer *renderer_end_effector_goal_new (BotViewer *viewer, int render_priority, lcm_t *lcm)
{
	RendererEndEffectorGoal *self = (RendererEndEffectorGoal*) calloc (1, sizeof (RendererEndEffectorGoal));
	self->viewer = viewer;
	self->renderer.draw = _draw;
	self->renderer.destroy = _free;
	self->renderer.name = (char*)RENDERER_NAME;
	self->renderer.user = self;
	self->renderer.enabled = 1;
	self->center = {0.0, 0.0, 0.0};
	//self->center = {0.47, 0.15, 1.0};
	self->translate = 0;
	self->active = 0;
	
	  //boost::shared_ptr<lcm::LCM> _lcm(new lcm::LCM(self->lcm));
  self->_lcm = boost::shared_ptr<lcm::LCM>(new lcm::LCM(lcm));
	self->robotStateListener = boost::shared_ptr<ee_goal_renderer::RobotStateListener>(new ee_goal_renderer::RobotStateListener(self->_lcm,viewer));

	BotEventHandler *ehandler = &self->ehandler;
	ehandler->name = (char*) RENDERER_NAME;
	ehandler->enabled = 1;
	ehandler->pick_query = NULL;
	ehandler->hover_query = NULL;
	ehandler->mouse_press = mouse_press;
	ehandler->mouse_release = mouse_release;
	ehandler->mouse_motion = mouse_motion;
	ehandler->user = self;

	bot_viewer_add_event_handler(viewer, &self->ehandler, render_priority);

	//self->lcm = lcm; //globals_get_lcm_full(NULL,1);
	//self->channel = "LWRISTROLL_LINK_GOAL";

	self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
	bot_gtk_param_widget_add_buttons(self->pw, PARAM_EE_GOAL, NULL);
	bot_gtk_param_widget_add_buttons(self->pw, PARAM_CLEAR, NULL);
	bot_gtk_param_widget_add_buttons(self->pw, PARAM_ADJUST_XY, NULL);
	bot_gtk_param_widget_add_buttons(self->pw, PARAM_ADJUST_Z, NULL);
	bot_gtk_param_widget_add_buttons(self->pw, PARAM_ADJUST_X, NULL);
	bot_gtk_param_widget_add_buttons(self->pw, PARAM_ADJUST_Y, NULL);
 	bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_L, 0, NULL);
  	bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_R, 0, NULL);
// 	bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_TRANSLATE_Z, 0, NULL);
	bot_gtk_param_widget_set_bool(self->pw, PARAM_L, 1); // seg faults
	self->arm=0;
	g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
	self->renderer.widget = GTK_WIDGET(self->pw);

	return &self->renderer;
}

void setup_renderer_end_effector_goal(BotViewer *viewer, int render_priority, lcm_t *lcm)
{
	bot_viewer_add_renderer(viewer, renderer_end_effector_goal_new(viewer, render_priority, lcm), render_priority);
}
