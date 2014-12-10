#include "ColorRobot.h"

using namespace std;
using namespace boost;
using namespace urdf;
using namespace KDL;
using namespace Eigen;
using namespace drc;
using namespace state;
using namespace kinematics;
using namespace opengl;
using namespace robot_opengl;

/*
void
ColorRobot::
draw(void)
{
    if (visible())
    {
        for (unsigned int i = 0; i < _opengl_objects.size(); i++)
        {
            if (_opengl_objects[ i ] != NULL)
            {
                _opengl_objects[ i ]->set_transform(_kinematics_model.link(_opengl_objects[ i ]->id()));

                if (_selected_link_name.compare(_opengl_objects[ i ]->id()) == 0)
                {
                    Eigen::Vector3f color(1.0, 0.0, 0.0);
                    _opengl_objects[ i ]->draw(color);
                }
                else
                {
                    _opengl_objects[ i ]->draw();
                }
            }
        }
    }
}
*/

void
ColorRobot::
setSelectedLink(std::string link_name)
{
    _selected_link_name = link_name;
}

CollisionGroupPtr
ColorRobot::
getCollisionGroupForLink(const string &link_name,
                          const string &collisionGroupName)
{
    const urdf::Model &m = _kinematics_model.model();
    shared_ptr<const urdf::Link> t = m.getLink(link_name);
    shared_ptr<vector<shared_ptr<Collision > > > cols = t->getCollisions(collisionGroupName);
    return cols;
}

/*
shared_ptr<const urdf::Link>
ColorRobot::
getLinkFromJointName(string joint_name)
{

  //cout << "(1) top of getLinkFromJointName" << endl;
  const urdf::Model &m = _kinematics_model.model();

  //cout << "(2) getting the joint w/ name = " << joint_name << endl;

  shared_ptr<const urdf::Joint> j = m.getJoint(joint_name);

  //cout << "(3) joint is null == " << (j == shared_ptr<const urdf::Joint>()) << endl;

  if (j == shared_ptr<const urdf::Joint>()) //joint doesn't exist?
    {
      cout << "\n joint " << joint_name << " not found. please update the urdf" << endl;
      return shared_ptr<const urdf::Link>(); //return "null" 
    }
  //cout << "(4) asking " << joint_name << " for child_link_name" << endl;

  string name = j->child_link_name;

  //cout << "(5) ------get linking w/ name = " << name << endl;

  shared_ptr<const urdf::Link> result = m.getLink(name);

  //cout << "\n--returning link--" << endl;
  
return result;
}
*/

OpenGL_Object*
ColorRobot::
getOpenGLObjectForLink(string link_name) 
{
    for (unsigned int i = 0; i < _opengl_objects.size(); i++)
    {
        if (_opengl_objects[i] != NULL)
        {
            if (link_name.compare(_opengl_objects[i]->id()) == 0)
            {
                return _opengl_objects[i];
            }
        }
    }
    return NULL;
}

void
ColorRobot::
getLinks(vector < shared_ptr< Link > >& links) {
    _kinematics_model.model().getLinks(links);
}
