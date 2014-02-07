classdef KinematicTerrainMap < RigidBodyTerrain
  properties
    normal
    offset
  end

  methods
    function obj = KinematicTerrainMap(biped, q)
      feet_position = biped.feetPosition(q);
      r_center = biped.footOrig2Contact(feet_position.right, 'center', true);
      obj.normal = rpy2rotmat(r_center(4:6)) * [0;0;1];
      obj.offset = obj.normal' * r_center(1:3);
    end

    function [height, normal] = getHeight(obj,xy)
      height = (obj.offset - obj.normal(1) * xy(1,:) - obj.normal(2) * xy(2,:)) / obj.normal(3);
      normal = obj.normal;
    end
  end
end
