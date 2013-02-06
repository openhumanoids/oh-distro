function [Xright, Xleft, X] = constrainedFootsteps(traj, step_length, step_width)

max_rot = pi/16;


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
			[ds(current_foot), rs(current_foot)] = distance(lambda(end), lambda_n, foot_angles(current_foot), step_width);
    end
    d = max(ds);
    r = max(rs);
		if d <= (step_length / 2) && r <= (max_rot / 2)
			break
		end
		lambda_n = lambda(end) + (lambda_n - lambda(end)) / 2;
	end
	lambda(end+1) = lambda_n;
end

num_steps = length(lambda);
ndx_r = int32([1, 2, 4:2:(num_steps-1), num_steps]);
ndx_l = int32([1:2:(num_steps-1), num_steps]);
foot_lambdas = {lambda(ndx_r), lambda(ndx_l)};

steps = {};
for current_foot = 1:2
	X = traj.eval(foot_lambdas{current_foot});
	yaw = X(6,:);
	foot_angle = yaw + foot_angles(current_foot);
	step_offset = [cos(foot_angle); sin(foot_angle); zeros(4, length(X(1,:)))] .* (step_width / 2);
	steps{current_foot} = X + step_offset;
end

X = traj.eval(linspace(0,1));
Xright = steps{1};
Xleft = steps{2};

function [d, r] = distance(lambda0, lambdaf, foot_angle, step_width)
	X0 = traj.eval(lambda0);
	yaw0 = X0(6,:);
	foot_angle0 = yaw0 + foot_angle;
	step_offset_0 = [cos(foot_angle0); sin(foot_angle0); zeros(4, length(X0(1,:)))] .* (step_width / 2);
	X0 = X0 + step_offset_0;
  
	Xf = traj.eval(lambdaf);
	yawf = Xf(6,:);
	foot_anglef = yawf + foot_angle;
	step_offset_f = [cos(foot_anglef); sin(foot_anglef); zeros(4, length(Xf(1,:)))] .* (step_width / 2);
	Xf = Xf + step_offset_f;
  
	d = sqrt(sum((X0(1:2) - Xf(1:2)) .^ 2));
	r = abs(Xf(6) - X0(6));
end
end