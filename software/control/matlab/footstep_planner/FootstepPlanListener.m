classdef FootstepPlanListener
	properties
		lc
		aggregator
	end

	methods
		function obj = FootstepPlanListener(channel)
			obj.lc = lcm.lcm.LCM.getSingleton();
			obj.aggregator = lcm.lcm.MessageAggregator();
			obj.lc.subscribe(channel, obj.aggregator);
		end

		function [X, options] = getNextMessage(obj, t_ms)
			plan_msg = obj.aggregator.getNextMessage(t_ms);
			if isempty(plan_msg)
				X = []; options = struct();
			else
				[X, options] = FootstepPlanListener.decodeFootstepPlan(drc.footstep_plan_t(plan_msg.data));
			end
		end

	end

	methods(Static)
		function [X, options] = decodeFootstepPlan(plan_msg)
		  for j = 1:length(plan_msg.footstep_goals)
		    X(j) = FootstepPlanListener.decodeFootstepGoal(plan_msg.footstep_goals(j));
		  end
		  options = struct('ignore_terrain', plan_msg.footstep_opts.ignore_terrain,...
		  	               'mu', plan_msg.footstep_opts.mu);
		end

		function X = decodeFootstepGoal(goal_msg)
		 rpy = quat2rpy([goal_msg.pos.rotation.w,...
		                        goal_msg.pos.rotation.x,...
		                        goal_msg.pos.rotation.y,...
		                        goal_msg.pos.rotation.z]);
% 		  [r, p, y] = quat2angle([goal_msg.pos.rotation.w,...
% 		                        goal_msg.pos.rotation.x,...
% 		                        goal_msg.pos.rotation.y,...
% 		                        goal_msg.pos.rotation.z], 'XYZ');
      % rpy = [r p y];
		  % disp('r p y:')
		  % [r p y]
		  X.pos = [goal_msg.pos.translation.x;...
		         goal_msg.pos.translation.y;...
		         goal_msg.pos.translation.z;...
		         rpy];

		  % Handle NaNs from network compression
		  if any(isnan(X.pos([1,2,6])))
		  	error('I don''t know how to handle NaN in x, y, or yaw');
		  else
		  	X.pos(isnan(X.pos)) = 0;
		  end

		  % X.time = goal_msg.step_time / 1000000;
		  X.step_speed = goal_msg.step_speed;
		  X.step_height = goal_msg.step_height;
		  X.id = goal_msg.id;
		  X.pos_fixed = [goal_msg.fixed_x;
					           goal_msg.fixed_y;
					           goal_msg.fixed_z;
					           goal_msg.fixed_roll;
					           goal_msg.fixed_pitch;
					           goal_msg.fixed_yaw];
		  X.is_right_foot = goal_msg.is_right_foot;
		  X.is_in_contact = goal_msg.is_in_contact;
		  %%%% HACK for DRC Qual 1 %%%%%
		  % X.pos(3) = X.pos(3) - 1;
		  %%%% end
		end
	end
end
