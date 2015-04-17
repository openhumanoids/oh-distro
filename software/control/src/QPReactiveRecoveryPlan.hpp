#include "drake/RigidBodyManipulator.h"

class QPReactiveRecoveryPlan {

	public:
		static VectorXd closestPointInConvexHull(VectorXd &x, MatrixXd &V);
};