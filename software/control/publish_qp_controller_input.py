import drake
import time
import numpy as np

def timestamp_now (): 
    return int (time.time () * 1000000)

msg = drake.lcmt_qp_controller_input()
msg.be_silent = False
msg.timestamp = timestamp_now()

zmp_data = drake.lcmt_zmp_data()
zmp_data.timestamp = timestamp_now()
zmp_data.A = np.array([
    [0, 0, 1, 0],
    [0, 0, 0, 1],
    [0, 0, 0, 0],
    [0, 0, 0, 0]
])
assert zmp_data.A[0][2] == 1
assert zmp_data.A[1][3] == 1
zmp_data.B = np.array([
    [0, 0],
    [0, 0],
    [1, 0],
    [0, 1]
])
assert zmp_data.B[2][0] == 1
assert zmp_data.B[3][1] == 1
zmp_data.C = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0]
])
zmp_data.D = np.array([
    [-0.101937, 0],
    [0, -0.101937]
])
zmp_data.x0 = np.array([[0], [0], [0], [0]]) # TODO: don't assume 0
zmp_data.y0 = np.array([[0], [0]]) # TODO: don't assume 0
zmp_data.u0 = np.array([[0], [0]])

zmp_data.R = np.array([
    [0, 0],
    [0, 0]
])

zmp_data.Qy = np.array([
    [1, 0],
    [0, 1]
])

zmp_data.S = np.array([
    [0.6386, 0, 0.2039, 0],
    [0, 0.6386, 0, 0.2039],
    [0.2039, 0, 0.0651, 0],
    [0, 0.2039, 0, 0.0651]
])

zmp_data.s1 = np.zeros((4, 1))
zmp_data.s1dot = np.zeros((4, 1))
zmp_data.s2 = 0
zmp_data.s2dot = 0
msg.zmp_data = zmp_data

msg.num_support_data = 2
r_foot_support = drake.lcmt_support_data()
r_foot_support.timestamp = timestamp_now()
r_foot_support.body_name = "r_foot"
r_foot_support.num_contact_pts = 4
r_foot_support.contact_pts = np.array([
    [0.1728, 0.1728, -0.0876, -0.0876],
    [0.0626, -0.0626, 0.0626, -0.0626],
    [-0.07645, -0.07645, -0.07645, -0.07645]
])
r_foot_support.total_normal_force_upper_bound = 6000
r_foot_support.total_normal_force_lower_bound = 5.0
r_foot_support.support_logic_map = [True, True, True, True]
r_foot_support.use_support_surface = True
r_foot_support.support_surface = [0, 0, 1, 0]
r_foot_support.mu = 0.7

l_foot_support = drake.lcmt_support_data()
l_foot_support.timestamp = timestamp_now()
l_foot_support.body_name = "l_foot"
l_foot_support.num_contact_pts = 4
l_foot_support.contact_pts = np.array([
    [0.1728, 0.1728, -0.0876, -0.0876],
    [0.0626, -0.0626, 0.0626, -0.0626],
    [-0.07645, -0.07645, -0.07645, -0.07645]
])
l_foot_support.total_normal_force_upper_bound = 6000
l_foot_support.total_normal_force_lower_bound = 5.0
l_foot_support.support_logic_map = [True, True, True, True]
l_foot_support.use_support_surface = True
l_foot_support.support_surface = [0, 0, 1, 0]
l_foot_support.mu = 0.7
msg.support_data = [l_foot_support, r_foot_support]

pelvis_tracking = drake.lcmt_body_motion_data()
pelvis_tracking.timestamp = timestamp_now()
pelvis_tracking.body_or_frame_name = "pelvis"
spline = drake.lcmt_piecewise_polynomial()
spline.num_breaks = 2
spline.breaks = [0, 1]
def lcmt_polynomial(coeffs):
    msg = drake.lcmt_polynomial()
    msg.num_coefficients = len(coeffs)
    msg.coefficients = coeffs
    return msg

def constant_polynomial(x):
    return lcmt_polynomial([x, 0, 0, 0])

def polynomial_matrix(polys):
    polys = np.asarray(polys)
    msg = drake.lcmt_polynomial_matrix()
    msg.rows = polys.shape[0]
    msg.cols = polys.shape[1]
    msg.polynomials = polys
    return msg

spline.num_segments = 1
spline.polynomial_matrices = [
    polynomial_matrix(np.array([[constant_polynomial(x)] for x in 0, 0, 0.4, 0, 0, 0]))
]

pelvis_tracking.spline = spline
pelvis_tracking.quat_task_to_world = [1, 0, 0, 0]
pelvis_tracking.translation_task_to_world = [0, 0, 0]
pelvis_tracking.xyz_kp_multiplier = [1, 1, 1]
pelvis_tracking.xyz_damping_ratio_multiplier = [1, 1, 1]
pelvis_tracking.expmap_kp_multiplier = 1
pelvis_tracking.expmap_damping_ratio_multiplier = 1
pelvis_tracking.weight_multiplier = [1, 1, 1, 0, 0, 1]
pelvis_tracking.in_floating_base_nullspace = False
pelvis_tracking.control_pose_when_in_contact = False

msg.num_tracked_bodies = 1
msg.body_motion_data = [pelvis_tracking]

msg.num_external_wrenches = 0

whole_body = drake.lcmt_whole_body_data()

fname = "/home/atlas/oh-distro/software/control/matlab/data/atlas_fp.mat"
import scipy.io
xstar = scipy.io.loadmat(fname)["xstar"]
qstar = xstar[:36]

whole_body.num_positions = 36
whole_body.q_des = qstar

spline = drake.lcmt_piecewise_polynomial()
spline.num_breaks = 2
spline.breaks = [0, 1]
spline.num_segments = 1
spline.polynomial_matrices = [
    polynomial_matrix(np.array([[constant_polynomial(x)] for x in qstar]))
]
whole_body.spline = spline

whole_body.num_constrained_dofs = 17
whole_body.constrained_dofs = [10, 17, 18, 19, 20, 21, 22, 23, 30, 31, 32, 33, 34, 35, 36, 8, 7]

msg.whole_body_data = whole_body

msg.num_joint_pd_overrides = 0

msg.param_set_name = "standing"

msg.torque_alpha_filter = 0

import lcm
lc = lcm.LCM()
lc.publish("QP_CONTROLLER_INPUT", msg.encode())
