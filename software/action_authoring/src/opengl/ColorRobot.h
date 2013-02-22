#ifndef ROBOT_LINK_H
#define ROBOT_LINK_H

#include <opengl/opengl_object_gfe.h>
#include <kinematics_model/kinematics_model_gfe.h>
#include <urdf/model.h>

namespace robot_opengl
{
typedef boost::shared_ptr<std::vector<boost::shared_ptr<urdf::Collision > > > CollisionGroupPtr;
class ColorRobot : public opengl::OpenGL_Object_GFE
{
protected:
    std::string _selected_link_name;
public:
    ColorRobot() : OpenGL_Object_GFE() { }
    ColorRobot(std::string urdfFilename) : OpenGL_Object_GFE(urdfFilename) { }
    kinematics_model::Kinematics_Model_GFE getKinematicsModel()
    {
        return _kinematics_model;
    }

    virtual void draw(void);
    void setSelectedLink(std::string link_name);
    CollisionGroupPtr getCollisionGroupsForLink(std::string link_name);
    boost::shared_ptr<const urdf::Link> getLinkFromJointName(std::string joint_name);
    opengl::OpenGL_Object* getOpenGLObjectForLink(std::string link_name);

};
}

#endif /* ROBOT_LINK_H */
