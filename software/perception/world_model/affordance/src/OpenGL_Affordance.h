/*
 * OpenGLAffordance.h
 *
 *  Created on: Jan 14, 2013
 *      Author: mfleder
 */

#ifndef OPENGLAFFORDANCE_H_
#define OPENGLAFFORDANCE_H_

#include "opengl/opengl_object.h"
#include "affordance/AffordanceState.h"
#include "opengl/opengl_object_cylinder.h"
#include "opengl/opengl_object_sphere.h"
#include "opengl/opengl_object_box.h"

namespace affordance
{

class OpenGL_Affordance: public opengl::OpenGL_Object
{
	//------------------fields
private:
	/**underlying affordance state*/

	bool _isHighlighted;
	const Eigen::Vector3f _highlightColor;
	//---we're drawing 1 of these
	opengl::OpenGL_Object_Box _box;
	opengl::OpenGL_Object_Cylinder _cylinder;
	opengl::OpenGL_Object_Sphere _sphere;

	//-----------------constructors
public:
	affordance::AffordanceState _affordance; // TODO Mike fixme
	OpenGL_Affordance(const affordance::AffordanceState &affordance, 
			  bool isHighlighted = false, 
			  Eigen::Vector3f highlightedColor = Eigen::Vector3f(1.0, 0.0, 0.0));
	virtual ~OpenGL_Affordance();

	//-------drawing
	virtual void draw();
	
	//--------mutators
	void setState(const affordance::AffordanceState &state);
	void setHighlighted(bool);
};

} /* namespace affordance */
#endif /* OPENGLAFFORDANCE_H_ */
