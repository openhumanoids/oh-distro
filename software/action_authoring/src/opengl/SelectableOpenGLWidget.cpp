#include "SelectableOpenGLWidget.h"


using namespace collision;
using namespace Eigen;
using namespace KDL;
using namespace opengl;
using namespace qt4;
using namespace robot_opengl;
using namespace boost;
using namespace std;


SelectableOpenGLWidget::SelectableOpenGLWidget()
	: _collisionDetector(shared_ptr<Collision_Detector>(new Collision_Detector()))
{
}

void
SelectableOpenGLWidget::
raycast(const Vector eyePosition,
	const Vector clickPosition) {
    cout << "ine calling raycast (eyePosition:" << eyePosition(0) << "," 
	 << eyePosition(1) << "," << eyePosition(2) << ",clickPosition:" 
	 << clickPosition(0) << "," << clickPosition(1) << "," << clickPosition(2) << ")" << endl;

    Collision_Object * intersected_object = NULL;
    return;

    _collisionDetector->ray_test( Vector3f( eyePosition(0), eyePosition(1), eyePosition(2)), 
				 Vector3f( clickPosition(0), clickPosition(1), clickPosition(2)),
				 intersected_object);
//    _collisionDetector->ray_test( Vector3f( 10.0, 0.0, 0.0 ), Vector3f( 0.0, 0.0, 0.0 ), intersected_object );
    if( intersected_object != NULL ){
	cout << "intersected " << intersected_object->id().c_str() << " ";
    } else {
	cout << "did not intersect with any objects ";
    }
    cout << endl;

    return;
}

void SelectableOpenGLWidget::add_object_with_collision(const shared_ptr<Collision_Object> collisionObject)
{

    _collisionDetector->add_collision_object(collisionObject.get());
    
    //  Collision_Object_Box* collision_object_2 = new Collision_Object_Box("box2", Vector3f( 0.5, 0.5, 0.5 ), Vector3f( 2.0, 0.0, 0.0 ), Vector4f( 0.0, 0.0, 0.0, 1.0)); 
    //_collisionDetector->add_collision_object(collision_object_2);
/*
    Collision_Object * intersected_object = NULL;
    _collisionDetector->ray_test( Vector3f( 10.0, 0.0, 0.0 ), Vector3f( 0.0, 0.0, 0.0 ), intersected_object );
    if( intersected_object != NULL ){
	cout << "!!! intersected " << intersected_object->id().c_str() << " ";
    } else {
	cout << "!!! did not intersect with any objects ";
    }
    cout << "(should intersect with box2)" << endl;
*/
}
