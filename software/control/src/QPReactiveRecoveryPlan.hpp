#include "drake/RigidBodyManipulator.h"
#include "drake/Polynomial.h"
#include "control/ExponentialForm.hpp"

#define QP_REACTIVE_RECOVERY_VERTICES_PER_FOOT 4

enum FootID {RIGHT, LEFT};

std::map<FootID, std::string> footIDToName = {std::make_pair(RIGHT, "right"),
                                              std::make_pair(LEFT, "left")};

std::map<std::string, FootID> footNameToID = {std::make_pair("right", RIGHT),
                                              std::make_pair("left", LEFT)};

std::map<FootID, FootID> otherFoot = {std::make_pair(RIGHT, LEFT),
                                      std::make_pair(LEFT, RIGHT)};

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

struct InterceptPlan {
  double tf;
  double tswitch;
  Isometry3d pose_next;
  Isometry3d icp_plus_offset_next;
  Isometry3d cop;
  FootID swing_foot;
  FootID stance_foot;
  double error;
};


typedef std::map<FootID, FootState> FootStateMap;
typedef std::map<FootID, Matrix<double, 2, 4>> VertMap;

struct BipedDescription {
  std::map<FootID, Matrix<double, 2, 4>> reachable_vertices;
  std::map<FootID, Matrix<double, 2, QP_REACTIVE_RECOVERY_VERTICES_PER_FOOT>> foot_vertices;
  double u_max; // foot acceleration bounds used for computing ICP intercepts
  double omega; // characteristic frequency of the linear inverted pendulum system. Defined as sqrt(g / height)
};

BipedDescription getAtlasDefaults() {
  BipedDescription biped;
  biped.reachable_vertices[RIGHT] << -0.4, 0.4, 0.4, -0.4,
                                    -0.2, -0.2, -0.45, -0.45;
  biped.reachable_vertices[LEFT] << -0.4, 0.4, 0.4, -0.4,
                                    0.2, 0.2, 0.45, 0.45;
  biped.foot_vertices[RIGHT] << -0.05, 0.05, 0.05, -0.05,
                               -0.02, -0.02, 0.02, 0.02;
  biped.foot_vertices[LEFT] << -0.05, 0.05, 0.05, -0.05,
                              -0.02, -0.02, 0.02, 0.02;
  biped.omega = sqrt(9.81 / 1.098);
  biped.u_max = 5;
  return biped;
}
  
class QPReactiveRecoveryPlan {

	public:
    double capture_max_flyfoot_height = 0.025;
    double capture_shrink_factor = 0.8;
    double desired_icp_offset = 0.1;
    double min_step_duration = 0.4;

		static VectorXd closestPointInConvexHull(const Ref<const VectorXd> &x, const Ref<const MatrixXd> &V);

    static Isometry3d closestPoseInConvexHull(const Isometry3d &pose, const Ref<const Matrix<double, 2, Dynamic>> &V);

    static std::vector<double> expIntercept(const ExponentialForm &expform, double l0, double ld0, double u, int degree);

    // a polynomial representing position as a function of time subject to initial position x0, initial velocity xd0, final velocity of 0, and acceleration of +u followed by acceleration of -u. 
    static Polynomial bangBangPolynomial(double x0, double xd0, double u);

    // bang-bang policy intercepts from initial state [x0, xd0] to final state [xf, 0] at max acceleration u_max
    static std::vector<BangBangIntercept> bangBangIntercept(double x0, double xd0, double xf, double u_max);

    static Isometry3d getTWorldToLocal(const Isometry3d &icp, const Isometry3d &cop);

    static double getMinTimeToXprimeAxis(const FootState foot_state, const BipedDescription &biped, Isometry3d &T_world_to_local);

    static ExponentialForm icpTrajectory(double x_ic, double x_cop, double omega);

    bool isICPCaptured(Vector2d r_ic, FootStateMap foot_states, VertMap foot_vertices);

    std::vector<InterceptPlan> getInterceptsWithCoP(const FootID &swing_foot, std::map<FootID, FootState> &foot_states, const BipedDescription &biped, const Isometry3d &icp, const Isometry3d &cop);

};

