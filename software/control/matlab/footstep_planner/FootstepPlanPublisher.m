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
				utime = now() * 24 * 60 * 60;
			end
			obj.lc.publish(obj.channel, FootstepPlanPublisher.encodeFootstepPlan(X, utime));
		end

	end

	methods(Static)
		function msg = encodeFootstepGoal(X, t)
			if nargin < 2
				t = now() * 24 * 60 * 60;
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
			% q = angle2quat(X.pos(4), X.pos(5), X.pos(6), 'XYZ');
			rot.w = q(1);
			rot.x = q(2);
			rot.y = q(3);
			rot.z = q(4);

			msg.pos.translation = trans;
			msg.pos.rotation = rot;
			msg.step_time = int64((X.time + t) * 1000000);
			msg.id = int32(X.id);
			msg.fixed_x = X.pos_fixed(1);
			msg.fixed_y = X.pos_fixed(2);
			msg.fixed_z = X.pos_fixed(3);
			msg.fixed_roll = X.pos_fixed(4);
			msg.fixed_pitch = X.pos_fixed(5);
			msg.fixed_yaw = X.pos_fixed(6);
			msg.is_right_foot = X.is_right_foot;
			msg.is_in_contact = X.is_in_contact;
    end

    function msg = encodeFootstepPlan(X, t, isnew)
			if nargin < 3
				isnew = true;
			end
			if nargin < 2
				t = now() * 24 * 60 * 60;
			end
		    robot_name = 'atlas';
		    msg = drc.footstep_plan_t();
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
		end
	end
end
