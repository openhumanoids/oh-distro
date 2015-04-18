#include "drake/RigidBodyManipulator.h"

class QPReactiveRecoveryPlan {

	public:
		static VectorXd closestPointInConvexHull(const Ref<const VectorXd> &x, const Ref<const MatrixXd> &V);
    static VectorXd closestPointInConvexHullCVXGEN(const Ref<const VectorXd> &x, const Ref<const MatrixXd> &V);
};