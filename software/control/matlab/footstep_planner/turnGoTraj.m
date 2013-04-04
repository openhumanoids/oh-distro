classdef turnGoTraj
	properties
		poses
		lambdas
	end

	methods
		function obj = turnGoTraj(poses)
      step_dist = 0.3;
      max_rot = pi/8;
      
      sizecheck(poses(:,1), [6]);
      start_pos = poses(:,1);
      
      angles_to_next = atan2(poses(2,2:end) - poses(2,1:(end-1)), poses(1,2:end) - poses(1,1:(end-1)))
      obj.poses = zeros(6, 3 * length(poses(1,:)) - 2);
      obj.poses(:,1) = start_pos;
      for i = 2:length(poses(1,:))
        obj.poses(:,i * 3 - 4) = [obj.poses(1:5,i * 3 - 5); angles_to_next(i-1)];
        obj.poses(:,i * 3 - 3) = [poses(1:5,i); angles_to_next(i-1)];
        obj.poses(:,i * 3 - 2) = poses(:,i);
      end
      % [d, r] = stepDistance(obj.poses(:,1:(end-1)), obj.poses(:, 2:end),0);
      % step_dists = d + r .* (step_dist / max_rot);
      % step_dists = [0, cumsum(step_dists)];
      % obj.lambdas = step_dists ./ step_dists(end);
      obj.lambdas = linspace(0, 1, length(obj.poses(1,:)));
      
      toDelete = [];
      for i = 2:length(obj.lambdas)
        if obj.lambdas(i) == obj.lambdas(i-1)
          toDelete(end+1) = i;
        end
      end
      obj.lambdas(toDelete) = [];
      obj.poses(:, toDelete) = [];
  
      
%       obj.lambdas = linspace(0, 1, length(obj.poses(1,:)));
      obj.poses
      obj.lambdas
		end

		function Xi = eval(obj, li)
      li = max(zeros(size(li)), li);
      li = min(ones(size(li)), li);
			Xi = interp1(obj.lambdas, obj.poses', li)';
      Xi(3:5, :) = 0;
		end
	end
end
