#ifndef SELECTABLEOPENGLWIDGET_H
#define SELECTABLEOPENGLWIDGET_H

#include <opengl/opengl_object_gfe.h>
#include <qt4/qt4_widget_opengl.h>

#include <iostream>
#include <Eigen/Dense>
#include <collision/collision.h>
#include <collision/collision_detector.h>
#include <collision/collision_object_box.h>

using namespace std;
using namespace Eigen;
using namespace collision;

using namespace std;
using namespace KDL;
using namespace opengl;
using namespace qt4;

namespace robot_opengl {
    class SelectableOpenGLWidget : public qt4::Qt4_Widget_OpenGL {
	void raycast( const Vector eyePosition, const Vector clickPosition );

    public:
	void add_object_with_collision(Collision_Object* collisionObject);

    protected:
	Collision_Detector* _collisionDetector;
	
    };
}

#endif /* SELECTABLEOPENGLWIDGET_H */
