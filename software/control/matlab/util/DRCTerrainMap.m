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

    function obj = setMapMode(obj,mode)
      fprintf(1, 'setting mode: %d\n', mode);
      obj.map_handle.setMapMode(mode);
    end
    
    function writeWRL(obj,fptr)
      error('not implemented yet, but could be done using the getAsMesh() interface'); 
    end

    function feas_check = getStepFeasibilityChecker(obj, contact_pts, options)
      % Return a function which can be called on a list of [x;y;yaw] points and which returns a vector of length size(xyyaw, 2) whose entries are 1 for each point which is OK for stepping and 0 for each point which is unsafe.
      if nargin < 3; options = struct(); end
      if ~isfield(options, 'resample'); options.resample = 2; end
      if ~isfield(options, 'debug'); options.debug = false; end
      if ~isfield(options, 'n_bins'); options.n_bins = 16; end

      %% Get the full heightmap from the map wrapper and a transform to world coordinate and then interpolate the heightmap and update the transform
      [heights, px2world] = obj.map_handle.getRawHeights();
      px2world(1,end) = px2world(1,end) - sum(px2world(1,1:3)); % stupid matlab 1-indexing...
      px2world(2,end) = px2world(2,end) - sum(px2world(2,1:3));
      mag = 2^(options.resample-1);
      heights = interp2(heights, (options.resample-1));
      px2world = px2world * [1/mag 0 0 (1-1/mag); 0 1/mag 0 (1-1/mag ); 0 0 1 0; 0 0 0 1];

      world2px = inv(px2world);
      world2px_2x3 = world2px(1:2,[1,2,4]);      

      %% Run simple edge detectors across the heightmap
      Q = imfilter(heights, [1, -1]) - 0.03 > 0;
      Q = Q | imfilter(heights, [-1, 1]) - 0.03 > 0;
      Q = Q | imfilter(heights, [1; -1]) - 0.03 > 0;
      Q = Q | imfilter(heights, [-1; 1]) - 0.03 > 0;
      Q(isnan(heights)) = 1;

      contact_pts_px = world2px_2x3 * [contact_pts(1:2,:); ones(1,size(contact_pts,2))];
      contact_pts_px = bsxfun(@minus, contact_pts_px, mean(contact_pts_px, 2));
      contact_pts_px = contact_pts_px - 1 * sign(contact_pts_px); % shrink by 1px to adjust for edge effects

      n_bins = options.n_bins;
      expansions = repmat({Q}, n_bins, 1);
      dtheta = pi / n_bins;
      function bin = findBin(theta)
        bin = mod(round(theta/(2*dtheta)), n_bins) + 1;
      end
      function theta = findTheta(bin)
        theta = (bin - 1) * 2 * dtheta;
      end

      for j = 1:n_bins
        expansions{j} = filter2(obj.makeDomain(contact_pts_px, findTheta(j), dtheta), Q) > 0;
        d = obj.makeDomain(contact_pts_px, findTheta(j), dtheta);
        i1 = filter2(d, Q);
        i2 = conv2(double(Q), double(d(end:-1:1,end:-1:1)), 'same');
        valuecheck(i1, i2);
      end

      [px_X, px_Y] = meshgrid(1:size(heights, 2), 1:size(heights, 1));
      function feas = feas_check_fcn(xyy)
        bins = findBin(xyy(3,:));
        px = world2px_2x3 * [xyy(1:2,:); ones(1,size(xyy,2))];
        feas = zeros(1,size(xyy,2));
        for j = 1:n_bins
          mask = bins == j;
          near_px_x = min(max(1, round(px(1,mask))), size(heights, 2));
          near_px_y = min(max(1, round(px(2,mask))), size(heights, 1));
          ndx = (near_px_x-1) * size(heights,1) + near_px_y;
          feas(mask) = ~expansions{j}(ndx);
        end
      end
      feas_check = @feas_check_fcn;

      if options.debug
        px2world_2x3 = px2world(1:2, [1,2,4]);
        world_xy = px2world_2x3 * [reshape(px_X, 1, []); reshape(px_Y, 1, []); ones(1, size(px_X, 1) * size(px_X, 2))];
        Infeas = expansions{findBin(-pi/2)};
        colors = zeros(length(reshape(Infeas,[],1)), 3);
        colors(reshape(Infeas, [], 1) == 1, :) = repmat([1 0 0], length(find(reshape(Infeas, [], 1) == 1)), 1);
        colors(reshape(Q, [], 1) == 0, :) = repmat([1 1 0], length(find(reshape(Q, [], 1) == 0)), 1);
        colors(reshape(Infeas, [], 1) == 0, :) = repmat([0 1 0], length(find(reshape(Infeas, [], 1) == 0)), 1);
        plot_lcm_points([world_xy', reshape(heights, [], 1)], colors, 71, 'Terrain Feasibility', 1, 1);
      end
    end

  end

  methods (Static=true)
    function domain = makeDomain(contact_pts_px, theta, dtheta)
      n_pts = size(contact_pts_px, 2);
      n_steps = 4;
      expanded_contacts_px = zeros(2,n_steps+size(contact_pts_px,2));
      expanded_contacts_px(:,1:n_pts) = rotmat(theta) * contact_pts_px;
      thetas = linspace(-dtheta, dtheta, n_steps);
      for j = 1:n_steps
        R = rotmat(thetas(j) + theta);
        expanded_contacts_px(:,n_pts*(j)+1:n_pts*(j+1)) = R * contact_pts_px;
      end
      [A, b] = poly2lincon(expanded_contacts_px(1,:), expanded_contacts_px(2,:));
      ma = max(expanded_contacts_px, [], 2);
      mi = min(expanded_contacts_px, [], 2);
      domain = false(ceil(ma(2) - mi(2)), ceil(ma(1) - mi(1)));
      x = (1:size(domain, 2)) - (size(domain, 2)/2 + 0.5);
      y = ((1:size(domain, 1)) - (size(domain, 1)/2 + 0.5));
      [X, Y] = meshgrid(x,y);
      inpoly = all(bsxfun(@minus, A * [reshape(X,1,[]); reshape(Y,1,[])], b) <= 0);
      domain(inpoly) = 1;
    end

    function [Expanded, dx, dy] = expandInfeasibility(Infeas, foot_radius_px)
      % Configuration-space expansion of infeasible region
      % @retval Expanded a matrix of the same size as Infeas. A point for which Expanded(j,k) > 0 is unsafe for stepping.
      % @retval dx gradient of infeasibility in x direction. Moving downhill means moving towards a safe step

      Infeas(Infeas > 0) = 1;

      %% Construct a circular domain corresponding to foot_radius
      domain = zeros(ceil(2 * foot_radius_px), ceil(2 * foot_radius_px));
      x = (1:size(domain, 2)) - (size(domain, 2)/2 + 0.5);
      y = (1:size(domain, 1)) - (size(domain, 1)/2 + 0.5);
      [X, Y] = meshgrid(x,y);
      d = sqrt(X.^2 + Y.^2);
      domain(d < foot_radius_px) = 1 - d(d < foot_radius_px) / foot_radius_px;

      %% A point is infeasible if any point within the domain centered on that point is infeasible (this is just configuration space planning)
      Expanded = imfilter(Infeas, domain);
    end
  end
  
  properties
    map_handle = [];
    minval = 0;  % only used for temporary hack
    raw = false; % hackish for footstep planner---probably going away
  end
end
