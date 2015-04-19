#include "drake/RigidBodyManipulator.h"
#include "drake/Polynomial.h"
#include "control/ExponentialForm.hpp"

struct BangBangIntercept {
  double tf;
  double tswitch;
  double u;
};

class QPReactiveRecoveryPlan {

	public:
		static VectorXd closestPointInConvexHull(const Ref<const VectorXd> &x, const Ref<const MatrixXd> &V);

    static std::vector<double> expIntercept(const ExponentialForm &expform, double l0, double ld0, double u, int degree);

    // a polynomial representing position as a function of time subject to initial position x0, initial velocity xd0, and final velocity of 0, and acceleration of +u followed by acceleration of -u. 
    static Polynomial bangBangPolynomial(double x0, double xd0, double u);

    // bang-bang policy intercepts from initial state [x0, xd0] to final state [xf, 0] at max acceleration u_max
    static std::vector<BangBangIntercept> bangBangIntercept(double x0, double xd0, double xf, double u_max);
};

