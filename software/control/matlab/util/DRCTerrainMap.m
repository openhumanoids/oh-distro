classdef DRCTerrainMap < RigidBodyTerrain

  methods
    function obj = DRCTerrainMap(mapWrapperFunc)
      if (~exist('mapWrapperFunc','var'))
          mapWrapperFunc = @mapAPIwrapper;
      end
      
      obj.map_handle = MapHandle(mapWrapperFunc);

      % wait for at least one map message to arrive before continuing
      
      map_mon = drake.util.MessageMonitor(drc.map_image_t,'utime');
      lc = lcm.lcm.LCM.getSingleton();
      lc.subscribe('MAP_DEPTH',map_mon);
      
      msg = ['????????? : Waiting for a non-empty terrain map message... [DRCTerrainMap.m]'];
      send_status(3, 0, 0, msg );
      fprintf(1,msg);
      obj.minval=[];
      while isempty(obj.minval)
        d=[];
        while isempty(d)
          d = getNextMessage(map_mon,100);  
          if ~isempty(d)
            msg = drc.map_image_t(d);
            if (msg.view_id ~= 6), d=[]; end  % check specifically for HEIGHT_MAP_SCENE
          end
          drawnow;  % allow matlab gui interaction
        end

        % temporary hack because the robot is initialized without knowing the ground under it's feet
        ptcloud = obj.map_handle.getPointCloud();
        obj.minval = min(ptcloud(3,:));
        % end hack
      end
      fprintf(1,'received!\n');
    end
    
    function [z,normal] = getHeight(obj,xy)
      z = ones(1,size(xy,2));  normal=repmat([0;0;1],1,size(xy,2));
      [p,normal] = obj.map_handle.getClosest([xy;0*xy(1,:)]);
      z=p(3,:);
      if any(isnan(z))  % temporary hack because the robot is initialized without knowing the ground under it's feet
        nn=sum(isnan(z)); 
        normal(:,isnan(z)) = [zeros(2,nn);ones(1,nn)];
        z(isnan(z))=obj.minval;
      end      
    end
    
    function writeWRL(obj,fptr)
      error('not implemented yet, but could be done using the getAsMesh() interface'); 
    end
  end
  
  properties
    map_handle = [];
    minval = 0;  % only used for temporary hack
  end
end
