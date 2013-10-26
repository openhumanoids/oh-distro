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

      if isfield(options,'status_code')
        typecheck(options.status_code, 'numeric');
      else
        options.status_code = 3;
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

      if isfield(options,'normal_radius')
        % Radius (in pixels) around each point to use for smoothing normals
        typecheck(options.normal_radius, 'numeric');
      else
        options.normal_radius = 1;
      end

      if isfield(options, 'auto_request')
        typecheck(options.auto_request, 'logical');
      else
        options.auto_request = false;
      end

      if (private_channel)
          obj.map_handle = HeightMapHandle(@HeightMapWrapper,'true');
      else
          obj.map_handle = HeightMapHandle(@HeightMapWrapper,'false');
      end
      
      obj.map_handle.setFillMissing(options.fill);
      obj.map_handle.setNormalRadius(options.normal_radius);

      % wait for at least one map message to arrive before continuing
      msg = [options.name,' : Waiting for terrain map...'];
      send_status(options.status_code, 0, 0, msg );
      fprintf(1,msg);
      obj.minval=[];
      lc = lcm.lcm.LCM.getSingleton();
      req_msg = drc.data_request_list_t();
      req_msg.num_requests = 1;
      req_msg.requests = javaArray('drc.data_request_t', 1);
      req = drc.data_request_t();
      req.type = drc.data_request_t.HEIGHT_MAP_SCENE;
      req.period = 0;
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
            if options.auto_request
              req_msg.utime = etime(clock,[1970 1 1 0 0 0])*1000000;
              req_msg.requests(1) = req;
              lc.publish('DATA_REQUEST', req_msg);
            end
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

    function feas_check = getStepFeasibilityChecker(obj, foot_radius, options)
      % Return a function which can be called on a list of xy points and which returns a vector of length size(xy, 2) whose entries are 1 for each xy point which is OK for stepping and 0 for each point which is unsafe.
      % Assumes a circular foot of radius foot_radius.

      if nargin < 3; options = struct(); end
      if ~isfield(options, 'resample'); options.resample = 2; end
      if ~isfield(options, 'debug'); options.debug = false; end

      %% Get the full heightmap from the map wrapper and a transform to world coordinate and then interpolate the heightmap and update the transform
      [heights, px2world] = obj.map_handle.getRawHeights();
      mag = 2^(options.resample-1);
      heights = interp2(heights, (options.resample-1));
      px2world = px2world * [1/mag 0 0 (1-1/mag); 0 1/mag 0 (1-1/mag ); 0 0 1 0; 0 0 0 1];
      world2px = inv(px2world);
      world2px_2x3 = world2px(1:2,[1,2,4]);

      %% Run a pair of simple edge detectors across the heightmap
      Q = (abs(imfilter(heights, [1, -1])) - 0.03) > 0;
      Q = Q + ((abs(imfilter(heights, [1; -1])) - 0.03) > 0);
      Q(isnan(heights)) = 1;
      Q(Q > 0) = 1;

      %% Construct a circular domain corresponding to foot_radius
      dworld = px2world * [0; 1; 0; 1] - px2world * [0; 0; 0; 1];
      dworld = norm(dworld(1:2));
      domain = zeros(ceil(2 * foot_radius / dworld), ceil(2 * foot_radius / dworld));
      xy = [0;0];
      for j = 1:size(domain, 1)
        xy(2) = (j - (size(domain, 1) / 2 + 0.5)) * dworld;
        for k = 1:size(domain, 2)
          xy(1) = (k - (size(domain, 2) / 2 + 0.5)) * dworld;
          dist_from_foot_center = sqrt(sum(xy.^2));
          if  dist_from_foot_center < foot_radius
      %       domain(j, k) = foot_radius - dist_from_foot_center;
            domain(j, k) = 1;
          end
        end
      end

      %% A point is infeasible if any point within the domain centered on that point is infeasible (this is just configuration space planning)
      Infeas = double(imfilter(Q, domain) > 0);

      [px_X, px_Y] = meshgrid(1:size(heights, 2), 1:size(heights, 1));
      function feas = feas_check_fcn(xy)
        px = world2px_2x3 * [xy(1:2,:); ones(1,size(xy,2))];
        % feas = griddata(px_X, px_Y, F2, px(1,:), px(2,:), 'nearest') < 0.5;
        feas = interp2(px_X, px_Y, Infeas, px(1,:), px(2,:), 'nearest') < 0.5;
        % valuecheck(feas, feas2);
      end


      feas_check = @feas_check_fcn;

      if options.debug
        px2world_2x3 = px2world(1:2, [1,2,4]);
        world_xy = px2world_2x3 * [reshape(px_X, 1, []); reshape(px_Y, 1, []); ones(1, size(px_X, 1) * size(px_X, 2))];
        
        colors = zeros(length(reshape(F2,[],1)), 3);
        colors(reshape(F2, [], 1) == 1, :) = repmat([1 0 0], length(find(reshape(F2, [], 1) == 1)), 1);
        colors(reshape(Q2, [], 1) == 0, :) = repmat([1 1 0], length(find(reshape(Q2, [], 1) == 0)), 1);
        colors(reshape(F2, [], 1) == 0, :) = repmat([0 1 0], length(find(reshape(F2, [], 1) == 0)), 1);
        plot_lcm_points([world_xy', reshape(heights, [], 1)], colors, 71, 'Terrain Feasibility', 1, 1);
      end
    end


  end
  
  properties
    map_handle = [];
    minval = 0;  % only used for temporary hack
    raw = false; % hackish for footstep planner---probably going away
  end
end
