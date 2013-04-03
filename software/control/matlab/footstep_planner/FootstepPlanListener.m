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

		function X = getNextMessage(obj, t_ms)
			plan_msg = obj.aggregator.getNextMessage(t_ms);
			if isempty(plan_msg)
				X = [];
			else
				X = FootstepPlanListener.decodeFootstepPlan(drc.footstep_plan_t(plan_msg.data));
			end
		end

	end

	methods(Static)
		function X = decodeFootstepPlan(plan_msg)
		  for j = 1:length(plan_msg.footstep_goals)
		    X(j) = FootstepPlanListener.decodeFootstepGoal(plan_msg.footstep_goals(j));
		  end
		end

		function X = decodeFootstepGoal(goal_msg)
			% disp('got quat:')
			% [goal_msg.pos.rotation.x,...
		 %                        goal_msg.pos.rotation.y,...
		 %                        goal_msg.pos.rotation.z,...
		 %                        goal_msg.pos.rotation.w]
		 rpy = quat2rpy([goal_msg.pos.rotation.w,...
		                        goal_msg.pos.rotation.x,...
		                        goal_msg.pos.rotation.y,...
		                        goal_msg.pos.rotation.z]);
		  % [r p y] = quat2angle([goal_msg.pos.rotation.w,...
		  %                       goal_msg.pos.rotation.x,...
		  %                       goal_msg.pos.rotation.y,...
		  %                       goal_msg.pos.rotation.z], 'XYZ');
		  % disp('r p y:')
		  % [r p y]
		  X.pos = [goal_msg.pos.translation.x;...
		         goal_msg.pos.translation.y;...
		         goal_msg.pos.translation.z;...
		         rpy];
		  X.time = goal_msg.step_time / 1000000;
		  X.id = goal_msg.id;
		  X.pos_fixed = [goal_msg.fixed_x;
					           goal_msg.fixed_y;
					           goal_msg.fixed_z;
					           goal_msg.fixed_roll;
					           goal_msg.fixed_pitch;
					           goal_msg.fixed_yaw];
		  X.is_right_foot = goal_msg.is_right_foot;
		  %%%% HACK for DRC Qual 1 %%%%%
		  X.pos(3) = X.pos(3) - 1;
		  %%%% end
		end
	end
end
