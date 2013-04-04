classdef BezierTraj
	properties
    sp
    df
	end

	methods
		function obj = BezierTraj(poses)
      sizecheck(poses, [6,2]);
      start_pos = poses(:,1);
      goal_pos = poses(:,2);
			p1 = start_pos(1:2);
			p4 = goal_pos(1:2);
			initial_angle = start_pos(6);
			R = [cos(initial_angle), -sin(initial_angle); ...
			  sin(initial_angle), cos(initial_angle)];
      p2 = p1 + R * [1; 0];
      
      
			final_angle = goal_pos(6);
      R = [cos(final_angle), -sin(final_angle); ...
			  sin(final_angle), cos(final_angle)];
      p3 = p4 - R * [1; 0];
      obj.sp = spmak([0 0 0 0 1 1 1 1], [p1 p2 p3 p4]);
      obj.df = fndir(obj.sp, eye(2));
		end

		function Xi = eval(obj, li)
      d = ppval(df, li);
      Xi = [fnval(obj.sp, li); zeros(3, length(li)); atan2(d(2, :), d(1, :))];
		end
	end
end