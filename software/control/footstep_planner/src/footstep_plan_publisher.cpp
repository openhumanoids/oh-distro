#include <stdio.h>
#include <math.h>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

inline void
bot_quat_mult (double c[4], const double a[4], const double b[4])
{
    c[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    c[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    c[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    c[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

void
bot_quat_rotate_rev (const double rot[4], double v[3])
{
    double a[4], b[4], c[4];

    b[0] = 0;
    memcpy (b+1, v, 3 * sizeof (double));

    bot_quat_mult (a, b, rot);
    b[0] = rot[0];
    b[1] = -rot[1];
    b[2] = -rot[2];
    b[3] = -rot[3];
    bot_quat_mult (c, b, a);

    memcpy (v, c+1, 3 * sizeof (double));
}

static inline int
feq (double a, double b) {
    return fabs (a - b) < 1e-9;
}

void
bot_quat_to_angle_axis (const double q[4], double *theta, double axis[3])
{
    double halftheta = acos (q[0]);
    *theta = halftheta * 2;
    double sinhalftheta = sin (halftheta);
    if (feq (halftheta, 0)) {
        axis[0] = 0;
        axis[1] = 0;
        axis[2] = 1;
        *theta = 0;
    } else {
        axis[0] = q[1] / sinhalftheta;
        axis[1] = q[2] / sinhalftheta;
        axis[2] = q[3] / sinhalftheta;
    }
}


class Handler
{
	drc::vector_3d_t translation;
	drc::quaternion_t rotation;
	public:
		lcm::LCM lcm;
		~Handler() {}

 		void handleStateMessage(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drc::robot_state_t* msg) {
			// printf("received message on channel \"%s\":\n", channel.c_str());
			// printf("timestamp = %11d\n", (long long)msg->utime);
			translation = msg->origin_position.translation;
			rotation = msg->origin_position.rotation;

		};

		void handleGoalMessage(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drc::nav_goal_timed_t* msg) {
			printf("start = %f, %f, %f\n", translation.x, translation.y, translation.z);
			printf("goal = %f, %f, %f\n", msg->goal_pos.translation.x, msg->goal_pos.translation.y, msg->goal_pos.translation.z);
			double translation_to_goal[3] = {msg->goal_pos.translation.x - translation.x, msg->goal_pos.translation.y - translation.y, msg->goal_pos.translation.z - translation.z};
			// translation_to_goal[0] = msg->goal_pos.translation.x - translation.x;
			// translation_to_goal[1] = msg->goal_pos.translation.y - translation.y;
			// translation_to_goal[2] = msg->goal_pos.translation.z - translation.z};

			// printf("x vec: %f - %f = %f, %f\n", msg->goal_pos.translation.x, translation.x,msg->goal_pos.translation.x - translation.x, translation_to_goal[0]);
			printf("vec to goal: %f, %f, %f\n", translation_to_goal[0], translation_to_goal[1], translation_to_goal[2]);

			// double robot_rotation[4] = {rotation.x, rotation.y, rotation.z, rotation.w};
			// printf("quaternion: %f, %f, %f, %f\n", rotation.x, rotation.y, rotation.z, rotation.w);
			// double rot_axis[3];
			// double rot_theta;
			// bot_quat_to_angle_axis(robot_rotation, &rot_theta, rot_axis);
			// printf("axis: %f, %f, %f\n", rot_axis[0], rot_axis[1], rot_axis[2]);
			// printf("angle: %f\n", rot_theta);
			// bot_quat_rotate_rev(robot_rotation, translation_to_goal);
			printf("vec to goal in robot frame: %f, %f, %f\n", translation_to_goal[0], translation_to_goal[1], translation_to_goal[2]);
			// double x_dist = msg->goal_pos.translation.x - translation.x;
			// double y_dist = msg->goal_pos.translation.y - translation.y;
			double total_dist = sqrt(pow(translation_to_goal[0], 2) + pow(translation_to_goal[1], 2));
			// TODO: Handle case when y_dist == total_dist
			double theta = 2 * acos(translation_to_goal[0] / total_dist);
			double R = translation_to_goal[0] / sin(theta);
			printf("plan: R = %f, theta = %f\n", R, theta);

			if (translation_to_goal[1] < 0) {
				R = -R;
				theta = -theta;
			} 
			double path_center[2];
			path_center[0] = translation.x;
			path_center[1] = translation.y + R;
			printf("path center = %f, %f\n", path_center[0], path_center[1]);

			double step_size_m = 0.5;
			double step_width_m = 0.30;
			double step_dt_s = 1;

			double arc_dist_to_goal_m = theta * R;
			int num_steps = floor(arc_dist_to_goal_m / step_size_m);
			double step_arc_size_rad = theta / num_steps;

			drc::ee_goal_sequence_t right_foot_goals;
			right_foot_goals.utime = msg->utime;
			right_foot_goals.robot_name = msg->robot_name;
			right_foot_goals.num_goals = num_steps;

			drc::ee_goal_sequence_t left_foot_goals;
			left_foot_goals.utime = msg->utime;
			left_foot_goals.robot_name = msg->robot_name;
			left_foot_goals.num_goals = num_steps;


			for (int i=0; i < num_steps; i++) {
				double right_foot_angle = step_arc_size_rad * (i + 0.5);
				double left_foot_angle = step_arc_size_rad * (i + 1);
				double left_foot_radius;
				double right_foot_radius;
				if (R > 0) {
					right_foot_radius = R - step_width_m / 2;
					left_foot_radius = R + step_width_m / 2;
				} else {
					right_foot_radius = R + step_width_m / 2;
					left_foot_radius = R - step_width_m / 2;
				}
				drc::ee_goal_t ee_goal;
				ee_goal.utime = msg->utime + i * step_dt_s;
				ee_goal.robot_name = msg->robot_name;
				ee_goal.ee_name = "RbackWheel";
				ee_goal.num_chain_joints = 0;
				drc::position_3d_t step_pos;
				step_pos.rotation = rotation;
				step_pos.translation.x = translation.x + right_foot_radius * sin(right_foot_angle);
				step_pos.translation.y = translation.y + right_foot_radius * (1 - cos(right_foot_angle));
				step_pos.translation.z = 0;
				ee_goal.ee_goal_pos = step_pos;
				right_foot_goals.goals.push_back(ee_goal);

				step_pos.rotation = rotation;
				step_pos.translation.x = translation.x + left_foot_radius * sin(left_foot_angle);
				step_pos.translation.y = translation.y + left_foot_radius * (1 - cos(left_foot_angle));
				step_pos.translation.z = 0;
				ee_goal.ee_goal_pos = step_pos;
				ee_goal.ee_name = "LbackWheel";
				left_foot_goals.goals.push_back(ee_goal);
			} 

			lcm.publish("RIGHT_FOOT_STEPS", &right_foot_goals);
			lcm.publish("LEFT_FOOT_STEPS", &left_foot_goals);
		};
};
	

int main(int argc, char **argv)
{
	lcm::LCM lcm;
	if (!lcm.good())
		return 1;

	Handler handlerObject;
	handlerObject.lcm = lcm;
	lcm.subscribe("EST_ROBOT_STATE", &Handler::handleStateMessage, &handlerObject);
	lcm.subscribe("NAV_GOAL_TIMED", &Handler::handleGoalMessage, &handlerObject);
	while (0 == lcm.handle());

	return 0;
}
