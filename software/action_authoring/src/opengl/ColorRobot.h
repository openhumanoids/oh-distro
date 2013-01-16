#ifndef ROBOT_LINK_H
#define ROBOT_LINK_H

#include <opengl/opengl_object_gfe.h>
#include <urdf/model.h>

namespace robot_opengl {
    class ColorRobot : public opengl::OpenGL_Object_GFE {
    protected:
	std::string _selected_link_name;
    public:
	virtual void draw( void );
	void setSelectedJoint(std::string jointName);
    };
}

#endif /* ROBOT_LINK_H */
