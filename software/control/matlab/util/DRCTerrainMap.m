classdef DRCTerrainMap < RigidBodyTerrain

  methods
    function obj = DRCTerrainMap()
      obj.map_ptr = SharedDataHandle(mapAPIwrapper());% ,@mapAPIwrapper);

      % wait for at least one map message to arrive before continuing
      
      map_mon = drake.util.MessageMonitor(drc.map_image_t,'utime');
      lc = lcm.lcm.LCM.getSingleton();
      lc.subscribe('MAP_DEPTH',map_mon);
      
      fprintf(1,'waiting for a non-empty terrain map message...');
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
        ptcloud = mapAPIwrapper(obj.map_ptr.getData(),[]);
        obj.minval = median(ptcloud(3,:));
        % end hack
      end
      fprintf(1,'received!\n');
    end
    
    function [z,normal] = getHeight(obj,xy)
      [p,normal] = mapAPIwrapper(obj.map_ptr.getData(),[xy;0*xy(1,:)]);
      z=p(3,:);
      if any(isnan(z))  % temporary hack because the robot is initialized without knowing the ground under it's feet
        normal(:,isnan(z)) = repmat([0;0;1],1,sum(isnan(z)));
        z(isnan(z))=obj.minval;
      end      
%       % TMP HACK, having some issues with normals in walking controller
%       normal(1:2,:) = 0;
%       normal(3,:) = 1;
%       % end hack
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
