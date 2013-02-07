classdef turnGoTraj
	properties
		poses
		lambdas
	end

	methods
		function obj = turnGoTraj(poses)
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
      obj.lambdas = linspace(0, 1, length(obj.poses(1,:)));
      obj.poses
		end

		function Xi = eval(obj, li)
      li = max(zeros(size(li)), li);
      li = min(ones(size(li)), li);
			Xi = interp1(obj.lambdas, obj.poses', li)';
		end
	end
end
