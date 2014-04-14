classdef KinematicTerrainMap < RigidBodyTerrain
  properties
    normal
    offset
  end

  methods
    function obj = KinematicTerrainMap(biped, q, always_level)
      if nargin < 3
        always_level = false;
      end
      feet_position = biped.feetPosition(q);
      r_center = biped.footOrig2Contact(feet_position.right, 'center', true);
      if always_level
        obj.normal = [0;0;1];
      else
        obj.normal = rpy2rotmat(r_center(4:6)) * [0;0;1];
      end
      obj.offset = obj.normal' * r_center(1:3);
    end

    function [height, normal] = getHeight(obj,xy)
      height = (obj.offset - obj.normal(1) * xy(1,:) - obj.normal(2) * xy(2,:)) / obj.normal(3);
      normal = repmat(obj.normal,1,size(xy,2));
    end
  end
end
