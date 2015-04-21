#include "drake/RigidBodyManipulator.h"
#include "drake/Polynomial.h"
#include "control/ExponentialForm.hpp"

#define QP_REACTIVE_RECOVERY_VERTICES_PER_FOOT 4

struct BangBangIntercept {
  double tf;
  double tswitch;
  double u;
};

typedef Matrix<double, 7, 1> XYZQuat;

struct FootState {
  Isometry3d pose;
  XYZQuat velocity;
  bool contact;
  double terrain_height;
};

class QPReactiveRecoveryPlan {

	public:
    double capture_max_flyfoot_height = 0.025;
    double capture_shrink_factor = 0.8;

		static VectorXd closestPointInConvexHull(const Ref<const VectorXd> &x, const Ref<const MatrixXd> &V);

    static std::vector<double> expIntercept(const ExponentialForm &expform, double l0, double ld0, double u, int degree);

    // a polynomial representing position as a function of time subject to initial position x0, initial velocity xd0, and final velocity of 0, and acceleration of +u followed by acceleration of -u. 
    static Polynomial bangBangPolynomial(double x0, double xd0, double u);

    // bang-bang policy intercepts from initial state [x0, xd0] to final state [xf, 0] at max acceleration u_max
    static std::vector<BangBangIntercept> bangBangIntercept(double x0, double xd0, double xf, double u_max);

    bool isICPCaptured(Vector2d r_ic, std::map<std::string, FootState> foot_states, std::map<std::string, Matrix<double, 2, QP_REACTIVE_RECOVERY_VERTICES_PER_FOOT>> foot_vertices);
};

