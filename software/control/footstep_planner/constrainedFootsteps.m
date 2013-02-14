function [lambda, ndx_r, ndx_l] = constrainedFootsteps(traj, step_length, step_width, max_rot)


lambda = [0];
foot_angles = [-pi/2, pi/2];


ds = zeros(1,2);
rs = zeros(1,2);

while 1
	if lambda(end) == 1
		break
	end
	lambda_n = 1;
	while 1
		for current_foot = 1:2
      X0 = footstepLocations(traj, lambda(end), foot_angles(current_foot), step_width);
      Xf = footstepLocations(traj, lambda_n, foot_angles(current_foot), step_width);
			[ds(current_foot), rs(current_foot)] = stepDistance(X0, Xf);
    end
    d = max(ds);
    r = max(rs);
		if d <= (step_length / 2) && r <= (max_rot / 2)
			break
		end
		lambda_n = lambda(end) + (lambda_n - lambda(end)) * .9;
	end
	lambda(end+1) = lambda_n;
end

num_steps = length(lambda);
ndx_r = int32([1, 2, 4:2:(num_steps-1), num_steps]);
ndx_l = int32([1:2:(num_steps-1), num_steps]);
% foot_lambdas = {lambda(ndx_r), lambda(ndx_l)};
% 
% steps = {};
% for current_foot = 1:2
%   steps{current_foot} = footstepLocations(traj, foot_lambdas{current_foot}, foot_angles(current_foot), step_width);
% end
% 
% X = traj.eval(linspace(0,1));
% Xright = steps{1};
% Xleft = steps{2};
% lambda = foot_lambdas

end