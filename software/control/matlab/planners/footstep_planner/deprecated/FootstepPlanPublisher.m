classdef FootstepPlanPublisher
	properties
		lc
		channel
	end

	methods
		function obj = FootstepPlanPublisher(channel)
			obj.channel = channel;
			obj.lc = lcm.lcm.LCM.getSingleton();
		end

		function publish(obj, X, utime)
			if nargin < 3
				utime = get_timestamp_now()*1e-6;
			end
			obj.lc.publish(obj.channel, FootstepPlanPublisher.encodeFootstepPlan(X, utime));
		end

	end

	methods(Static)
		function msg = encodeFootstepGoal(X, t)
			if nargin < 2
				t = get_timestamp_now()*1e-6;
			end

			msg = drc.footstep_goal_t();
			msg.utime = t * 1000000;
			msg.robot_name = 'atlas';
			msg.pos = drc.position_3d_t();
			trans = drc.vector_3d_t();
			trans.x = X.pos(1);
			trans.y = X.pos(2);
			trans.z = X.pos(3);
			rot = drc.quaternion_t();
			q = rpy2quat([X.pos(4), X.pos(5), X.pos(6)]);
% 			q = angle2quat(X.pos(4), X.pos(5), X.pos(6), 'XYZ');
			rot.w = q(1);
			rot.x = q(2);
			rot.y = q(3);
			rot.z = q(4);

			msg.pos.translation = trans;
			msg.pos.rotation = rot;
			msg.step_speed = X.step_speed;
			msg.step_height = X.step_height;
			msg.id = int32(X.id);
			msg.fixed_x = X.pos_fixed(1);
			msg.fixed_y = X.pos_fixed(2);
			msg.fixed_z = X.pos_fixed(3);
			msg.fixed_roll = X.pos_fixed(4);
			msg.fixed_pitch = X.pos_fixed(5);
			msg.fixed_yaw = X.pos_fixed(6);
			msg.is_right_foot = X.is_right_foot;
			msg.is_in_contact = X.is_in_contact;
			msg.bdi_step_duration = X.bdi_step_duration;
			msg.bdi_sway_duration = X.bdi_sway_duration;
			msg.bdi_lift_height = X.bdi_lift_height;
			msg.bdi_toe_off = X.bdi_toe_off;
			msg.bdi_knee_nominal = X.bdi_knee_nominal;
            msg.bdi_max_body_accel = X.bdi_max_body_accel;
            msg.bdi_max_foot_vel = X.bdi_max_foot_vel;
            msg.bdi_sway_end_dist = X.bdi_sway_end_dist;
            msg.bdi_step_end_dist = X.bdi_step_end_dist;

			if ~isfield(X, 'terrain_pts') || isempty(X.terrain_pts)
				msg.num_terrain_pts = 0;
			else
				msg.num_terrain_pts = size(X.terrain_pts, 2);
				msg.terrain_path_dist = X.terrain_pts(1,:);
				msg.terrain_height = X.terrain_pts(2,:);
			end
    end

    function msg = encodeFootstepPlan(X, t, isnew, options)
			if nargin < 3
				isnew = true;
			end
			if nargin < 2
				t = get_timestamp_now()*1e-6;% get_timestamp already returns in usec, so converting to sec as we multiply by 1e6 later
			end
		    robot_name = 'atlas';
		    msg = drc.deprecated_footstep_plan_t();
		    msg.utime = t * 1000000;
		    msg.robot_name = robot_name;
		    msg.num_steps = length(X);
		    msg.is_new_plan = isnew;
  
  		    goals = javaArray('drc.footstep_goal_t', length(X));
  		    for j = 1:length(X)
  		    	new_goal = FootstepPlanPublisher.encodeFootstepGoal(X(j), t);
  		      goals(j) = new_goal;
  		    end
		    msg.footstep_goals = goals;
		    msg.footstep_opts = drc.footstep_opts_t();
		    msg.footstep_opts.ignore_terrain = options.ignore_terrain;
		    msg.footstep_opts.mu = options.mu;
		    msg.footstep_opts.behavior = options.behavior;
		    msg.footstep_opts.map_command = options.map_command;
		    msg.footstep_opts.velocity_based_steps = options.velocity_based_steps;
		end
	end
end
