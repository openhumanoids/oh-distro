/*
 * OpenGLAffordance.cpp
 *
 *  Created on: Jan 14, 2013
 *      Author: mfleder
 */

#include "OpenGL_Affordance.h"

using namespace affordance;
//using namespace opengl;
using namespace std;
using namespace boost;
using namespace Eigen;
//--------------constructor/destructor
OpenGL_Affordance::OpenGL_Affordance(const AffordanceState &affordance)
	: _affordance(affordance)
{
}

OpenGL_Affordance::~OpenGL_Affordance()
{
}

/**sets the state of the object we want to draw and then draws*/
void OpenGL_Affordance::draw()
{
	//frame will be used by everything -- only compute once
	KDL::Frame frame;
	_affordance.getFrame(frame);

	switch(_affordance._otdf_id)
	{
		case AffordanceState::CYLINDER:
			_cylinder.set(frame, Vector2f(_affordance.radius(),
										  _affordance.length()));
			_cylinder.set_color(Vector3f(1.0,1.0,1.0)); //todo
			_cylinder.draw();
			break;

		case AffordanceState::LEVER:
			throw runtime_error("not handling lever right now");
		break;

		case AffordanceState::BOX:
			throw runtime_error("not handling box right now");
			break;

		case AffordanceState::SPHERE:
			_sphere.set(frame, _affordance.radius());
			_sphere.set_color(Vector3f(10.0, 0.0, 1.0));
			_sphere.draw();
			break;

		default:
			throw runtime_error("unhandled affordance state");
	}
}


//-------drawing

//-----mutators
/**set the affordance state and update the _drawable object*/
void OpenGL_Affordance::setState(const affordance::AffordanceState &state)
{
	_affordance = state;
}

