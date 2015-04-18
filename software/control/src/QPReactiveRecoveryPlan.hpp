#include "drake/RigidBodyManipulator.h"
#include "drake/Polynomial.h"

class QPReactiveRecoveryPlan {

	public:
		static VectorXd closestPointInConvexHull(const Ref<const VectorXd> &x, const Ref<const MatrixXd> &V);

    static Polynomial expTaylor(double a, double b, double c, int degree);
};