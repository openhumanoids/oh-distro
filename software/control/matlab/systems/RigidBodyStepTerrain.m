classdef RigidBodyStepTerrain < RigidBodyTerrain
%  This class provides an implementation of RigidBodyTerrain with z=0
%  everywhere, except at small box that raises up 5cm.
  
  methods 
    function obj = RigidBodyStepTerrain()
      obj.center_x = 1.0;
      obj.center_y = 0.0;
      obj.box_x = 1;
      obj.box_y = 1;
      obj.box_z = 0.05;
      obj.geom = constructRigidBodyGeometry(obj);
    end
    
    function [z,normal] = getHeight(obj,xy)
      [~, n] = size(xy);
      x_in_box = xy(1, :) >= obj.center_x - obj.box_x/2 & xy(1, :) <=  obj.center_x+ obj.box_x/2;
      y_in_box = xy(2, :) >= obj.center_y - obj.box_y/2 & xy(2, :) <= obj.center_y+obj.box_y/2;
      z = zeros(1,n);
      z(x_in_box&y_in_box) = obj.box_z;
      normal = repmat([0;0;1],1,n);
      
    end

    function geom = getRigidBodyContactGeometry(obj)
      geom = obj.geom;
    end

    function geom = getRigidBodyShapeGeometry(obj)
      geom = obj.geom;
    end
    
    function geom = constructRigidBodyGeometry(obj)
      ground_width = 1000;
      ground_depth = 10;
      geom_ground = RigidBodyBox([ground_width;ground_width;ground_depth]);
      geom_ground.T(3,4) = -ground_depth/2;
      geom_ground.c = hex2dec({'ee','cb','ad'})'/256;  % something a little brighter (peach puff 2 from http://www.tayloredmktg.com/rgb/)
      geom_ground.name = 'terrain';
%      geom.c = hex2dec({'cd','af','95'})'/256;

      geom_box = RigidBodyBox([obj.box_x; obj.box_y; obj.box_z]);
      geom_box.T(1:3, 4) = [obj.center_x; obj.center_y; obj.box_z/2];
      geom_box.c = hex2dec({'ee','cb','ad'})'/256;
      geom_box.name = 'terrain';
      
      geom = {geom_ground geom_box};
      
    end
  end
  
  properties
    geom;
    center_x;
    center_y;
    box_x;
    box_y;
    box_z;
  end
end
