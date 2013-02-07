function [X] = footstepLocations(traj, lambda, foot_angle, step_width)
  X = traj.eval(lambda);
	yaw = X(6,:);
	foot_angle = yaw + foot_angle;
	step_offset = [cos(foot_angle); sin(foot_angle); zeros(4, length(X(1,:)))] .* (step_width / 2);
	X = X + step_offset;
end