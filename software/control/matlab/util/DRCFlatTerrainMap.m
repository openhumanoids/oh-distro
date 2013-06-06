classdef DRCFlatTerrainMap < RigidBodyTerrain

  methods
    function obj = DRCFlatTerrainMap()
      obj.monitor = PoseGroundMonitor();
    end
    
    function [z,normal] = getHeight(obj,xy)
      [m n] = size(xy);
      z = repmat(obj.monitor.getHeight(),1,n);
      normal = repmat([0;0;1],1,n);
    end
  end
  
  properties
    monitor;
  end
end
