classdef DRCTerrainMap < RigidBodyTerrain

  methods
    function obj = DRCTerrainMap()
      obj.map_ptr = SharedDataHandle(mapAPIwrapper());% ,@mapAPIwrapper);
      
      % temporary hack because the robot is initialized without knowing the ground under it's feet
%      ptcloud = mapAPIwrapper(obj.map_ptr.getData(),[]);
%      obj.minval = min(ptcloud(3,:));
      % end hack
    end
    
    function [z,normal] = getHeight(obj,xy)
      [p,normal] = mapAPIwrapper(obj.map_ptr.getData(),[xy;0*xy(1,:)]);
      z=p(3,:);
      if 0 %any(z==nan)  % temporary hack because the robot is initialized without knowing the ground under it's feet
        p(:,z==nan) = repmat([0;0;1],1,sum(z==nan));
        z(z==nan)=obj.minval;
      end      
    end
    
    function writeWRL(obj,fptr)
      error('not implemented yet, but could be done using the getAsMesh() interface'); 
    end
  end
  
  properties
    map_ptr = 0;
    minval = 0;  % only used for temporary hack
  end
end
