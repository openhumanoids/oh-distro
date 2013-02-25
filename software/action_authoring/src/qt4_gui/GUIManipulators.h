#ifndef GUIMANIPULATORS_H
#define GUIMANIPULATORS_H

#include "MainWindow.h"

using namespace std;
using namespace opengl;
using namespace state;
using namespace collision;
using namespace action_authoring;
using namespace affordance;

namespace action_authoring 
{

struct WorldStateView;

typedef std::map<string, boost::shared_ptr<vector<boost::shared_ptr<urdf::Collision> > > > CollisionMapType;

class GUIManipulators
{

public:
    // create the manipulators from the robot's joints
    // TODO : pull out worldState and robot
    static void createManipulators(WorldStateView &worldState, robot_opengl::SelectableOpenGLWidget &targetWidget);

}; // end class
} // end namespace

#endif // GUIMANIPULATORS_H
