classdef DRCTerrainMap < RigidBodyTerrain

  methods
    function obj = DRCTerrainMap()
      obj.map_ptr = SharedDataHandle(mapAPIwrapper());% ,@mapAPIwrapper);
    end
    
    function [z,normal] = getHeight(obj,xy)
      [p,normal] = mapAPIWrapper(obj.map_ptr,[xy;0*xy(1,:)]);
      z=p(1,:);
    end
    
    function writeWRL(obj,fptr)
      error('not implemented yet, but could be done using the getAsMesh() interface'); 
    end
  end
  
  properties
    map_ptr = 0;
  end
end