#include "GUIManipulators.h"

using namespace std;
using namespace opengl;
using namespace state;
using namespace collision;
using namespace action_authoring;
using namespace affordance;
using namespace boost;

void
GUIManipulators::
createManipulators(WorldStateView &worldState, robot_opengl::SelectableOpenGLWidget &targetWidget)
{
    // read the joints from the robot state
    //std::map< std::string, State_GFE_Joint > joints = worldState.state_gfe.joints();

    //cout << "making mNAIPS for join  !!!" << joints.size() << endl;
    
    vector < shared_ptr< urdf::Link > > allLinks;
    worldState.colorRobot.getLinks(allLinks);
    
    for (uint i = 0; i < allLinks.size(); i++)
    {
        shared_ptr<const urdf::Link> link = allLinks[i];
        if (link == shared_ptr<const urdf::Link>())
          continue; //need to fix the urdf

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
