#include "GUIManipulators.h"

using namespace std;
using namespace opengl;
using namespace state;
using namespace collision;
using namespace action_authoring;
using namespace affordance;

void
GUIManipulators::
createManipulators(WorldStateView &worldState, robot_opengl::SelectableOpenGLWidget &targetWidget)
{
    // read the joints from the robot state
    std::map< std::string, State_GFE_Joint > joints = worldState.state_gfe.joints();

    for (std::map< std::string, State_GFE_Joint >::const_iterator it = joints.begin(); it != joints.end(); it++)
    {
        const State_GFE_Joint &state_gfe_joint = it->second;
        std::string id = state_gfe_joint.id();
        shared_ptr<const urdf::Link> link = worldState.colorRobot.getLinkFromJointName(id);
        
        // TODO mfleder
        // std::map<string, boost::shared_ptr<vector<boost::shared_ptr<Collision> > > > 
        // link->collision_groups
        // for (collision_group_name in link->collision_groups.keys()) {
        for(CollisionMapType::const_iterator iter = link->collision_groups.begin();
            iter != link->collision_groups.end();
            ++iter)
        {
            ManipulatorStateConstPtr manipulator(new ManipulatorState(link, iter->first,
                                                                      worldState.colorRobot.getKinematicsModel().link(link->name),
                                                                      GlobalUID(rand(), rand()))); //todo guid
            
            worldState.manipulators.push_back(manipulator);
            
            OpenGL_Manipulator *asGlMan = new OpenGL_Manipulator(manipulator);
            targetWidget.opengl_scene().add_object(*asGlMan);
            worldState.glObjects.push_back(asGlMan);
            
                        
            if (Collision_Object_Manipulator::isSupported(manipulator))
            {
                Collision_Object_Manipulator *cObjManip = new Collision_Object_Manipulator(manipulator);
                targetWidget.add_collision_object(cObjManip);
                worldState.collisionObjs.push_back(cObjManip);
            }
        }
    }
}
