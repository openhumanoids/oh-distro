classdef turnGoTraj
	properties
		poses
		lambdas
	end

	methods
		function obj = turnGoTraj(start_pos, goal_pos)
			angle_to_goal = atan2(goal_pos(2) - start_pos(2), goal_pos(1) - start_pos(1));
			obj.poses = [start_pos, [start_pos(1:5); angle_to_goal], [goal_pos(1:5); angle_to_goal], goal_pos];
			obj.lambdas = [0, 1/3, 2/3, 1];
		end

		function Xi = eval(obj, li)
			Xi = interp1(obj.lambdas, obj.poses', li)';
		end
	end
end
