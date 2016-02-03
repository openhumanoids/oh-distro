#include "finalPosePlanner.hpp"

using namespace std;
using namespace Eigen;

FinalPosePlanner::FinalPosePlanner()
{

}

int FinalPosePlanner::findFinalPose(RigidBodyTree robot, unsigned int end_effector_id, VectorXd start_configuration, VectorXd endeffector_final_pose,
		vector<RigidBodyConstraint> additional_constraints, VectorXd nominal_configuration, CapabilityMap capability_map, IKoptions ik_options,
		string reaching_hand, double min_distance, Vector3d endeffector_point)
{
	robot.num_positions;
	return 0;
}
