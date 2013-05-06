classdef SampleTerrain < RigidBodyTerrain

  methods
    function [z, normal] = getHeight(obj, xy)
      last_pos = [0;0;0;0;0;0];
      next_pos = [.5;0;0;0;0;0];
      lambda = sqrt(sum(bsxfun(@minus, xy, last_pos(1:2)).^2));
      terrain = PPTrajectory(foh([0, .2,.201, .299,.3, .5], [0, 0, .1, .1, 0, 0]));

      z = terrain.eval(lambda);
      normal = nan;
    end
  end
end

