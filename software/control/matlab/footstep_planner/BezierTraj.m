classdef BezierTraj
	properties
    sp
    df
    start_pos
    goal_pos
	end

	methods
		function obj = BezierTraj(poses)
      sizecheck(poses, [6,2]);
      obj.start_pos = poses(:,1);
      obj.goal_pos = poses(:,2);

      control_dist = min(1, sqrt(sum((obj.start_pos(1:2) - obj.goal_pos(1:2)).^2))/2);
			p1 = obj.start_pos(1:2);
			p4 = obj.goal_pos(1:2);
			initial_angle = obj.start_pos(6);
			R = [cos(initial_angle), -sin(initial_angle); ...
			  sin(initial_angle), cos(initial_angle)];
      p2 = p1 + R * [control_dist; 0];
      
      
			final_angle = obj.goal_pos(6);
      R = [cos(final_angle), -sin(final_angle); ...
			  sin(final_angle), cos(final_angle)];
      p3 = p4 - R * [control_dist; 0];
      obj.sp = spmak([0 0 0 0 1 1 1 1], [p1 p2 p3 p4]);
      obj.df = fndir(obj.sp, eye(2));
		end

		function Xi = eval(obj, li)
      d = ppval(obj.df, li);
      Xi = [fnval(obj.sp, li); repmat(obj.start_pos(3), 1, length(li)); zeros(2, length(li)); atan2(d(2, :), d(1, :))];
		end
	end
end