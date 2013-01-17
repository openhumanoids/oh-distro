#ifndef SELECTABLEOPENGLWIDGET_H
#define SELECTABLEOPENGLWIDGET_H

#include <opengl/opengl_object_gfe.h>
#include <qt4/qt4_widget_opengl.h>

#include <iostream>
#include <Eigen/Dense>
#include <collision/collision.h>
#include <collision/collision_detector.h>
#include <collision/collision_object_box.h>



namespace robot_opengl
{

class SelectableOpenGLWidget : public qt4::Qt4_Widget_OpenGL
{
	//------fields
protected:
	const boost::shared_ptr<collision::Collision_Detector> _collisionDetector;


	//------constructor
public:
	SelectableOpenGLWidget();


	//--------methods
	void raycast( const KDL::Vector eyePosition, const KDL::Vector clickPosition );
   	void add_object_with_collision(const boost::shared_ptr<collision::Collision_Object> collisionObject);
	
    };
}

#endif /* SELECTABLEOPENGLWIDGET_H */
