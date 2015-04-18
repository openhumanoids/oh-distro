#include "drake/RigidBodyManipulator.h"
#include "drake/Polynomial.h"
#include "control/ExponentialForm.hpp"

class QPReactiveRecoveryPlan {

	public:
		static VectorXd closestPointInConvexHull(const Ref<const VectorXd> &x, const Ref<const MatrixXd> &V);

    static std::set<double> expIntercept(const ExponentialForm &expform, double l0, double ld0, double u, int degree);
};

