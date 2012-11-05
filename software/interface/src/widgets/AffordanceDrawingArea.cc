/*
 * AffordanceDrawingArea.cpp
 *
 *Created on: Oct 5, 2012
 *Author: ppetrova
 */

#include <iostream>
#include "AffordanceDrawingArea.h"

using namespace std;

namespace gui {

AffordanceDrawingArea::AffordanceDrawingArea(boost::shared_ptr<otdf::ModelInterface> otdfObject, boost::shared_ptr<lcm::LCM> &lcm) :
	GlDrawingArea(lcm), _otdfObject(otdfObject)
{

	//lcm ok?
	if(!_lcm->good())
	{
		cerr << "\nLCM Not Good: AffordanceDrawingArea" << endl;
		return;
	} else {
		cout << "LCM is good" << endl;
	}

	_lcm->subscribe("AFF_PARAM", &gui::AffordanceDrawingArea::handleAffordanceParameterMsg, this);
	
	if (!_otdfObject){
		std::cerr << "ERROR: null affordance object" << std::endl;
	}

	// TODO: create a KDL tree parser from OTDF instance, without having to convert to urdf.
	// otdf can contain some elements that are not part of urdf. e.g. TORUS
	std::string _urdf_xml_string = otdf::convertObjectInstanceToCompliantURDFstring(_otdfObject);

	// Parse KDL tree
	KDL::Tree tree;
	if (!kdl_parser::treeFromString(_urdf_xml_string,tree))
	{
		std::cerr << "ERROR: Failed to extract kdl tree from object "<< std::endl; 
	}

	_fksolver = boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));
	run_fk_and_gen_link_shapes_and_tfs();
}

//AffordanceDrawingArea::AffordanceDrawingArea(const AffordanceDrawingArea& other) {
// TODO Auto-generated destructor stub
//}

AffordanceDrawingArea::~AffordanceDrawingArea() {
	// TODO Auto-generated destructor stub
}

// This should not be called, left it here for reference for now
void AffordanceDrawingArea::handleAffordanceParameterMsg(const lcm::ReceiveBuffer* rbuf,
		const string& chan,
		const drc::affordance_parameter_t* msg)
{
	/*
	double newValue = _otdfObject->getParam(msg->parameter_name);
	const char * param_name = msg->parameter_name.c_str();

	if (strcmp(param_name, "x") == 0 || strcmp(param_name, "y") == 0 || strcmp(param_name, "z") == 0) {
		double change = msg->value * 50.0;
		newValue += change;
	} else if (strcmp(param_name, "roll") == 0 || strcmp(param_name, "pitch") == 0 || strcmp(param_name, "yaw") == 0) {
		newValue += msg->value * 3.141592654 * 2;
	} else {
		newValue *= (1 + msg->value);
	}
	_otdfObject->setParam(msg->parameter_name, newValue);
	update_OtdfInstance();
	//redraw
	GlDrawingArea::doRefresh();
	*/
}

void AffordanceDrawingArea::on_param_change(std::string parameter_name, double delta) {
	double newValue = _otdfObject->getParam(parameter_name);
	const char * param_name = parameter_name.c_str();

	if (strcmp(param_name, "x") == 0 || strcmp(param_name, "y") == 0 || strcmp(param_name, "z") == 0) {
		double change = delta * 50.0;
		newValue += change;
	} else if (strcmp(param_name, "roll") == 0 || strcmp(param_name, "pitch") == 0 || strcmp(param_name, "yaw") == 0) {
		newValue += delta * 3.141592654 * 2;
	} else {
		newValue *= (1 + delta);
	}
	_otdfObject->setParam(parameter_name, newValue);
	update_OtdfInstance();

	//redraw
	GlDrawingArea::doRefresh();
}


void AffordanceDrawingArea::update_OtdfInstance (){
     _otdfObject->update();
      std::string _urdf_xml_string = otdf::convertObjectInstanceToCompliantURDFstring(_otdfObject);
      // Parse KDL tree
      // TODO: create a KDL tree parser from OTDF instance, without having to convert to urdf.
      // otdf can contain some elements that are not part of urdf. e.g. TORUS
      KDL::Tree tree;
      if (!kdl_parser::treeFromString(_urdf_xml_string,tree))
      {
        std::cerr << "ERROR: Failed to extract kdl tree from "  << _otdfObject->getName() << " xml object description "<< std::endl; 
      }
      
      _fksolver = boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));
      run_fk_and_gen_link_shapes_and_tfs();
 }

//-----------------------------------
static void link_draw(boost::shared_ptr<otdf::Geometry> link, const drc::link_transform_t &nextTf)
{

	//--get rotation in angle/axis form
	double theta;
	double axis[3];
	double quat[4] = {nextTf.tf.rotation.w,
			nextTf.tf.rotation.x,
			nextTf.tf.rotation.y,
			nextTf.tf.rotation.z};
	bot_quat_to_angle_axis(quat, &theta, axis);


	 GLUquadricObj* quadric = gluNewQuadric();
	 gluQuadricDrawStyle(quadric, GLU_FILL);
	 gluQuadricNormals(quadric, GLU_SMOOTH);
	 gluQuadricOrientation(quadric, GLU_OUTSIDE);
	 

	int type = link->type ;
	enum {SPHERE, BOX, CYLINDER, MESH, TORUS}; 

	if (type == SPHERE)
	{

		glPushMatrix();
		boost::shared_ptr<otdf::Sphere> sphere(boost::shared_dynamic_cast<otdf::Sphere>(link));	
		double radius = sphere->radius;
		glPointSize(radius);
		//glColor3ub(0,1,0);
		glBegin(GL_POINTS);
		glVertex3f(radius, radius, radius);
		glEnd();
		glPopMatrix();
	}
	else if(type == BOX) {
		boost::shared_ptr<otdf::Box> box(boost::shared_dynamic_cast<otdf::Box>(link));
		double xDim = box->dim.x;
		double yDim = box->dim.y;
		double zDim = box->dim.z;
		//todo
		glPushMatrix();
		//size cuboid

		// move base up so that bottom face is at origin
		 glTranslatef(nextTf.tf.translation.x,
		 	 	nextTf.tf.translation.y,
				nextTf.tf.translation.z);
		 glRotatef(theta * 180/3.141592654, 
		 	 axis[0], axis[1], axis[2]); 
		 glScalef(xDim,yDim,zDim);
		 bot_gl_draw_cube();
		//cube();
		glPopMatrix();
	}
	else if(type == CYLINDER)
	{
		boost::shared_ptr<otdf::Cylinder> cyl(boost::shared_dynamic_cast<otdf::Cylinder>(link));

		 glPushMatrix();
		 double v[] = {0,0, -cyl->length/2.0};
		 double result[3];
		 bot_quat_rotate_to(quat,v,result);

		 // Translate tf origin to cylinder centre
		 glTranslatef(result[0],result[1],result[2]);

		 glTranslatef(nextTf.tf.translation.x,
				 nextTf.tf.translation.y,
				 nextTf.tf.translation.z);

		 glRotatef(theta * 180/3.141592654, 
		 		axis[0], axis[1], axis[2]); 

		 gluCylinder(quadric,
				cyl->radius,
				cyl->radius,
				(double) cyl->length,
				36,
				1);

		//gluDeleteQuadric(quadric);
		glPopMatrix();

		// drawing two disks to make a SOLID cylinder
		glPushMatrix();

		v[2] = -(cyl->length/2.0);
		bot_quat_rotate_to(quat,v,result);

		 // Translate tf origin to cylinder centre
		 glTranslatef(result[0],result[1],result[2]); 
		 glTranslatef(nextTf.tf.translation.x,
				 nextTf.tf.translation.y,
				 nextTf.tf.translation.z);
		 glRotatef(theta * 180/3.141592654, 
		 		axis[0], axis[1], axis[2]); 
		 gluDisk(quadric,
				0,
				cyl->radius,
				36,
				1);
		glPopMatrix();
		glPushMatrix(); 

		 v[2] = (cyl->length/2.0);
		bot_quat_rotate_to(quat,v,result);

		 // Translate tf origin to cylinder centre
		 glTranslatef(result[0],result[1],result[2]); 
		 glTranslatef(nextTf.tf.translation.x,
				 nextTf.tf.translation.y,
				 nextTf.tf.translation.z);
		 glRotatef(theta * 180/3.141592654, 
		 		axis[0], axis[1], axis[2]); 
		 gluDisk(quadric,
				0,
				cyl->radius,
				36,
				1);
		glPopMatrix();
	}
	else if(type == MESH)
	{
		//boost::shared_ptr<otdf::Mesh> mesh(boost::shared_dynamic_cast<otdf::Mesh>(it->second->visual->geometry));
		//renderMesh(mesh->filename)
	}
	else if(type == TORUS)
	{
		boost::shared_ptr<otdf::Torus> torus(boost::shared_dynamic_cast<otdf::Torus>(link));
		double innerRadius = torus->tube_radius;
		double outerRadius = torus->radius;
		 
		//todo
		glPushMatrix();
		//size cuboid

		// move base up so that bottom face is at origin
		 glTranslatef(nextTf.tf.translation.x,
		 	 	nextTf.tf.translation.y,
				nextTf.tf.translation.z);

		 glRotatef(theta * 180/3.141592654, 
		 	 axis[0], axis[1], axis[2]); 
		glutSolidTorus(innerRadius,outerRadius,36,36); 
		// glutWireTorus(innerRadius,outerRadius,8,8); 
		 
		glPopMatrix();
	}
	else {
		//cout << "UNKNOWN"<< endl;
	}

	gluDeleteQuadric(quadric);
}


bool AffordanceDrawingArea::on_expose_event(GdkEventExpose* event)
{

	GlDrawingArea::on_expose_event(event);

	// Get GL::Window.
	Glib::RefPtr<Gdk::GL::Window> glwindow = get_gl_window();
	if (!glwindow) return false;

	// GL calls.

	// *** OpenGL BEGIN ***
	if (!glwindow->gl_begin(get_gl_context()))
		return false;

	glEnable(GL_DEPTH_TEST);

	//glEnable(GL_LIGHTING);
	//glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_BLEND);
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
	 
	double c[3] = {0.5,0.0,1.0};
	double alpha = 0.8;
	glColor4f(c[0],c[1],c[2],alpha);

	 for(uint i = 0; i < _link_tfs.size(); i++)
	{
		drc::link_transform_t nextTf = _link_tfs[i];
		boost::shared_ptr<otdf::Geometry> nextLink = _link_shapes[i];
		link_draw(nextLink, nextTf);
	}

	/*static GLfloat light_diffuse[] = {1.0, 0.0, 0.0, 1.0};
	static GLfloat light_position[] = {1.0, 1.0, 1.0, 0.0};
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
*/
	//glClearColor(1.0, 1.0, 1.0, 1.0);
//	glClearDepth(1.0);

	glViewport(0, 0, get_width(), get_height());

	glCallList(1);

	// Swap buffers.
	if (glwindow->is_double_buffered())
		glwindow->swap_buffers();
	else
		glFlush();

	glwindow->gl_end();
	
	return true;
}

void AffordanceDrawingArea::run_fk_and_gen_link_shapes_and_tfs ()
{
	//clear stored data
	_link_tfs.clear();
	_link_shapes.clear();

	std::map<std::string, double> jointpos_in;
	enum{UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED};
	 
	typedef std::map<std::string,boost::shared_ptr<otdf::Joint> > joints_mapType;
	for (joints_mapType::iterator joint = _otdfObject->joints_.begin();joint != _otdfObject->joints_.end(); joint++) {
		if(joint->second->type!=(int) FIXED) { // All joints that not of the type FIXED.
			double dof_current_pos = 0; // TODO: need object's initial dof state from fitting
			jointpos_in.insert(make_pair(joint->first, dof_current_pos)); 
		}
	}
	 
	//Have to handle joint_patterns separately 
	// DoF of all joints in joint patterns.
	typedef std::map<std::string,boost::shared_ptr<otdf::Joint_pattern> > jp_mapType;
	for (jp_mapType::iterator jp_it = _otdfObject->joint_patterns_.begin();jp_it != _otdfObject->joint_patterns_.end(); jp_it++) {
		// for all joints in joint pattern.
		for (unsigned int i=0; i < jp_it->second->joint_set.size(); i++) {
			if(jp_it->second->joint_set[i]->type!=(int) FIXED) { // All joints that not of the type FIXED.
				double dof_current_pos = 0; //TODO: need object's initial dof state from fitting
				jointpos_in.insert(make_pair(jp_it->second->joint_set[i]->name, dof_current_pos)); 
			} // end if
		} // end for all joints in jp
	}// for all joint patterns

	std::map<std::string, drc::transform_t > cartpos_out;

	// Calculate forward position kinematics
	bool kinematics_status;
	bool flatten_tree=true; // determines the absolute transforms with respect to robot origin. 
	//Otherwise returns relative transforms between joints. 

	kinematics_status = _fksolver->JntToCart(jointpos_in,cartpos_out,flatten_tree);
	if(kinematics_status>=0){
	}
	else{
		std::cerr << "Error: could not calculate forward kinematics!" <<std::endl;
		return;
	}

	 std::map<std::string, boost::shared_ptr<otdf::Link> > _links_map =_otdfObject->links_;// static links

	//Have to handle link_patterns separately
	typedef std::map<std::string,boost::shared_ptr<otdf::Link_pattern> > lp_mapType;
	for (lp_mapType::iterator lp_it = _otdfObject->link_patterns_.begin();lp_it != _otdfObject->link_patterns_.end(); lp_it++) {
		// for all joints in joint pattern.
		for (unsigned int i=0; i < lp_it->second->link_set.size(); i++) {
			_links_map.insert(std::make_pair(lp_it->second->link_set[i]->name,lp_it->second->link_set[i])); 
		} // end for all links in lp
	}// for all link patterns


	 typedef std::map<std::string, boost::shared_ptr<otdf::Link> > links_mapType;
	 for( links_mapType::const_iterator it =_links_map.begin(); it!= _links_map.end(); it++) { 
		if(it->second->visual) {

			otdf::Pose visual_origin = it->second->visual->origin;
			KDL::Frame T_parentjoint_visual, T_body_parentjoint, T_body_visual, T_world_body, T_world_visual;

			T_world_body.p[0]= _otdfObject->getParam("x");
			T_world_body.p[1]= _otdfObject->getParam("y");
			T_world_body.p[2]= _otdfObject->getParam("z");
			T_world_body.M =KDL::Rotation::RPY(_otdfObject->getParam("roll"),
			 _otdfObject->getParam("pitch"),
			 _otdfObject->getParam("yaw"));

			std::map<std::string, drc::transform_t>::const_iterator transform_it;
			transform_it=cartpos_out.find(it->first);	
			 
			 if(transform_it!=cartpos_out.end())// fk cart pos exists
			 {
				T_body_parentjoint.p[0]= transform_it->second.translation.x;
				T_body_parentjoint.p[1]= transform_it->second.translation.y;
				T_body_parentjoint.p[2]= transform_it->second.translation.z;	
			 
				T_body_parentjoint.M =KDL::Rotation::Quaternion(transform_it->second.rotation.x, transform_it->second.rotation.y, transform_it->second.rotation.z, transform_it->second.rotation.w);


				T_parentjoint_visual.p[0]=visual_origin.position.x;
				T_parentjoint_visual.p[1]=visual_origin.position.y;
				T_parentjoint_visual.p[2]=visual_origin.position.z;
				T_parentjoint_visual.M =KDL::Rotation::Quaternion(visual_origin.rotation.x, visual_origin.rotation.y, visual_origin.rotation.z, visual_origin.rotation.w);

				T_body_visual= T_body_parentjoint*T_parentjoint_visual;
				 
				T_world_visual = T_world_body*T_body_visual;
			 //T_world_visual= T_world_camera*T_camera_body*T_body_visual;

				drc::link_transform_t state;	

				state.link_name = transform_it->first;

				state.tf.translation.x = T_world_visual.p[0];
				state.tf.translation.y = T_world_visual.p[1];
				state.tf.translation.z = T_world_visual.p[2];
				T_world_visual.M.GetQuaternion(state.tf.rotation.x,state.tf.rotation.y,state.tf.rotation.z,state.tf.rotation.w);
	

				boost::shared_ptr<otdf::Geometry> geom =it->second->visual->geometry;

				//---store
				_link_shapes.push_back(geom);
				_link_tfs.push_back(state);
			 }// end if(transform_it!=cartpos_out.end())
		} // end if(it->second->visual)
	 } // end for links in_links_map
} // end function run_fk_and_gen_link_shapes_and_tfs
}
