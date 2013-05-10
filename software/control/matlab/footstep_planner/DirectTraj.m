classdef DirectTraj
  properties
    poses
    lambdas
  end

  methods
    function obj = DirectTraj(poses)
      
      sizecheck(poses(:,1), [6]);
      start_pos = poses(:,1);
      obj.lambdas = linspace(0, 1, length(poses(1,:)));
      obj.poses = poses;
    end

    function Xi = eval(obj, li)
      li = max(zeros(size(li)), li);
      li = min(ones(size(li)), li);
      Xi = interp1(obj.lambdas, obj.poses', li)';
      Xi(4:5, :) = 0;
    end
  end
end
