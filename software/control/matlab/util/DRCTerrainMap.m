classdef DRCTerrainMap < RigidBodyTerrain

  methods
    function obj = DRCTerrainMap(is_robot,options)
     
      if nargin < 1
        is_robot = false;
      end
      
      if is_robot
        private_channel = true;
      else
        private_channel = false;
      end
      
      if nargin < 2
        options = struct();
      else
        typecheck(options,'struct');
      end

      if isfield(options,'name');
        typecheck(options.name,'char');
      else
        options.name = '';
      end
      
      if isfield(options,'raw');
        typecheck(options.raw,'logical');
      else
        options.raw = false;
      end
      obj.raw = options.raw;
      
      if isfield(options,'fill')
          typecheck(options.fill,'logical');
      else
          options.fill= false;
      end

      if (private_channel)
          obj.map_handle = HeightMapHandle(@HeightMapWrapper,'true');
      else
          obj.map_handle = HeightMapHandle(@HeightMapWrapper,'false');
      end
      
      obj.map_handle.setFillMissing(options.fill);

      % wait for at least one map message to arrive before continuing
      msg = [options.name,' : Waiting for a non-empty terrain map message... [DRCTerrainMap.m]\n'];
      send_status(3, 0, 0, msg );
      fprintf(1,msg);
      obj.minval=[];
      while isempty(obj.minval)
        ptcloud=[];
        while true
            % temporary hack because the robot is initialized without knowing the ground under it's feet
            ptcloud = obj.map_handle.getPointCloud();
            if (~isempty(ptcloud))
                break;
            end
            % end hack
            pause(1.0);
        end
        obj.minval = min(ptcloud(3,:));
      end
      fprintf(1,'Received terrain map!\n');
    end
    
    function [z,normal] = getHeight(obj,xy)
      z = ones(1,size(xy,2));  normal=repmat([0;0;1],1,size(xy,2));
      [p,normal] = obj.map_handle.getClosest([xy;0*xy(1,:)]);
      z=p(3,:);
      if ~obj.raw
        if any(isnan(z))  % temporary hack because the robot is initialized without knowing the ground under it's feet
          nn=sum(isnan(z)); 
          normal(:,isnan(z)) = [zeros(2,nn);ones(1,nn)];
          z(isnan(z))=obj.minval;
        end     
      end
    end
    
    function writeWRL(obj,fptr)
      error('not implemented yet, but could be done using the getAsMesh() interface'); 
    end
  end
  
  properties
    map_handle = [];
    minval = 0;  % only used for temporary hack
    raw = false; % hackish for footstep planner---probably going away
  end
end
