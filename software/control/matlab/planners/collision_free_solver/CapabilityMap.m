classdef CapabilityMap
  
  properties
    rbm
    map
    reachability_index
    vox_centres
    vox_edge
    n_samples
    ang_tolerance
    pos_tolerance
    urdf
    n_voxels
    n_directions_per_voxel
    map_left_centre
    end_effector_link
    end_effector_axis
    base_link
    active_voxels
    n_active_voxels
    nominal_configuration
    map_ub
    map_lb
    EE_pose
    occupancy_map
    occupancy_map_resolution
    occupancy_map_dimensions
    occupancy_map_n_voxels
    occupancy_map_lb
    occupancy_map_ub
    occupancy_map_orient_steps
    occupancy_map_active_orient
    occupancy_map_n_orient
  end
  
  methods
    
    function obj = CapabilityMap(file_path, kinematic_chain_left, ...
        end_effector_right, end_effector_axis, map_left_centre, nominal_configuration)
      if nargin == 1
        obj = obj.loadFromFile(file_path);
      elseif nargin > 1
      
        original_urdf = xmlread(file_path);
        obj.base_link = kinematic_chain_left{1};
        obj.end_effector_link.left = kinematic_chain_left{end};
        obj.end_effector_link.right = end_effector_right;
        obj.end_effector_axis = end_effector_axis;
      
        doc = com.mathworks.xml.XMLUtils.createDocument('robot');
        robotNode = doc.getDocumentElement;
        robot_name = original_urdf.getDocumentElement().getAttribute('name');
        if ~isempty(robot_name)
          robotNode.setAttribute('name',robot_name);
        end

        links = original_urdf.getDocumentElement().getElementsByTagName('link');
        for l = 0:links.getLength()-1
          if any(strcmp(links.item(l).getAttribute('name'), kinematic_chain_left))
            linkNode = doc.importNode(links.item(l), true);
            robotNode.appendChild(linkNode);
          end
        end

        joints = original_urdf.getDocumentElement().getElementsByTagName('joint');
        for j = 0:joints.getLength()-1
          parent = joints.item(j).getElementsByTagName('parent').item(0);
          child = joints.item(j).getElementsByTagName('child').item(0);
          if ~isempty(parent) && ~isempty(child) && ...
              any(strcmp(parent.getAttribute('link'), kinematic_chain_left)) && ...
              any(strcmp(child.getAttribute('link'), kinematic_chain_left))
            jointNode = doc.importNode(joints.item(j), true);
            robotNode.appendChild(jointNode);
          end
        end

        urdf_string = xmlwrite(doc);
        obj.urdf = doc;
        obj.rbm = RigidBodyManipulator();
        obj.rbm = obj.rbm.addRobotFromURDFString(urdf_string);
        
        if nargin < 6
          obj.nominal_configuration = zeros(obj.rbm.num_positions, 1);
        else
          obj.nominal_configuration = nominal_configuration;
        end
        kinsol = obj.rbm.doKinematics(obj.nominal_configuration, []);
        
        if isnumeric(map_left_centre)
          obj.map_left_centre = map_left_centre;
        else
          obj.map_left_centre = obj.rbm.forwardKin(kinsol, obj.rbm.findLinkId(map_left_centre), [0;0;0]);
        end
      end
    end
    
    function obj = loadFromFile(obj, file)
      vars = load(file);
      if ~all(isfield(vars, {'urdf', 'map_left_centre', 'end_effector_link', 'end_effector_axis', ...
        'base_link', 'nominal_configuration'}))
        error('Some data is missing')
      end
      obj.urdf = vars.urdf;
      obj.map_left_centre = vars.map_left_centre;
      obj.end_effector_link = vars.end_effector_link;
      obj.end_effector_axis = vars.end_effector_axis;
      obj.base_link = vars.base_link;
      obj.nominal_configuration = vars.nominal_configuration;
      obj.rbm = RigidBodyManipulator();
      obj.rbm = obj.rbm.addRobotFromURDFString(xmlwrite(obj.urdf));
      if isfield(vars, 'map')
        if ~all(isfield(vars,  {'map', 'reachability_index', ...
        'vox_centres', 'vox_edge', 'n_samples', 'ang_tolerance', 'pos_tolerance', ...
        'n_voxels', 'n_directions_per_voxel', 'map_lb', 'map_ub'}))
          error('Some data is missing')
        end
        obj.map = vars.map;
        obj.reachability_index = vars.reachability_index;
        obj.vox_centres = vars.vox_centres;
        obj.vox_edge = vars.vox_edge;
        obj.n_samples = vars.n_samples;
        obj.ang_tolerance = vars.ang_tolerance;
        obj.pos_tolerance = vars.pos_tolerance;
        obj.n_voxels = size(obj.map, 1);
        obj.n_directions_per_voxel = size(obj.map, 2);
        obj.map_ub = vars.map_ub;
        obj.map_lb = vars.map_lb;
        obj = obj.resetActiveVoxels();
      else
        warning('No map data found')
      end
      if isfield(vars, 'occupancy_map')
        if ~all(isfield(vars,  {'occupancy_map', 'occupancy_map_resolution',...
            'occupancy_map_dimensions', 'occupancy_map_n_voxels', ...
            'occupancy_map_lb', 'occupancy_map_ub', ...
            'occupancy_map_orient_steps', 'occupancy_map_n_orient'}))
          error('Some data is missing')
        end
        obj.occupancy_map = vars.occupancy_map;
        obj.occupancy_map_resolution = vars.occupancy_map_resolution;
        obj.occupancy_map_dimensions = vars.occupancy_map_dimensions;
        obj.occupancy_map_n_voxels = vars.occupancy_map_n_voxels;
        obj.occupancy_map_lb = vars.occupancy_map_lb;
        obj.occupancy_map_ub = vars.occupancy_map_ub;
        obj.occupancy_map_orient_steps = vars.occupancy_map_orient_steps;
        obj.occupancy_map_n_orient = vars.occupancy_map_n_orient;
      else
        warning('No occupancy map data found')
      end
    end
    
    function saveToFile(obj, file)
      urdf = obj.urdf;
      map_left_centre = obj.map_left_centre;
      end_effector_link = obj.end_effector_link;
      end_effector_axis = obj.end_effector_axis;
      base_link = obj.base_link;
      nominal_configuration = obj.nominal_configuration;
      saved_on = datestr(now);
      vars = {'urdf', 'map_left_centre', 'end_effector_link', 'end_effector_axis', ...
        'base_link', 'nominal_configuration', 'saved_on'};
      if ~isempty(obj.map)
        map = obj.map;
        reachability_index = obj.reachability_index;
        vox_centres = obj.vox_centres;
        vox_edge = obj.vox_edge;
        n_samples = obj.n_samples;
        ang_tolerance = obj.ang_tolerance;
        pos_tolerance = obj.pos_tolerance;
        n_voxels = obj.n_voxels;
        n_directions_per_voxel = obj.n_directions_per_voxel;
        map_lb = obj.map_lb;
        map_ub = obj.map_ub;
        vars = [vars, {'map', 'reachability_index', ...
        'vox_centres', 'vox_edge', 'n_samples', 'ang_tolerance', 'pos_tolerance', ...
        'n_voxels', 'n_directions_per_voxel', 'map_lb', 'map_ub'}];
      end
      if ~isempty(obj.occupancy_map)
        occupancy_map = obj.occupancy_map;
        occupancy_map_resolution = obj.occupancy_map_resolution;
        occupancy_map_dimensions = obj.occupancy_map_dimensions;
        occupancy_map_n_voxels = obj.occupancy_map_n_voxels;
        occupancy_map_lb = obj.occupancy_map_lb;
        occupancy_map_ub = obj.occupancy_map_ub;
        occupancy_map_orient_steps = obj.occupancy_map_orient_steps;
        occupancy_map_n_orient = obj.occupancy_map_n_orient;
        vars = [vars, {'occupancy_map', 'occupancy_map_resolution', ...
          'occupancy_map_dimensions', 'occupancy_map_n_voxels', ...
          'occupancy_map_lb', 'occupancy_map_ub',...
          'occupancy_map_orient_steps', 'occupancy_map_n_orient'}];
      end
      save(file, vars{:}, '-v7.3');
    end
    
    function obj = activateVoxels(obj, idx)
      obj.active_voxels(idx) = true;
      obj.n_active_voxels = nnz(obj.active_voxels);
    end
    
    function obj = deactivateVoxels(obj, idx)
      obj.active_voxels(idx) = false;
      obj.n_active_voxels = nnz(obj.active_voxels);
    end
    
    function obj = activateOrient(obj, voxels, active_orients)
      if length(voxels) > 1
        if ~iscell(active_orients)
          error('active_orients must be a cell with the orientations idx to activate for each voxel')
        end
      else
        if iscell(active_orients)
          active_orients = active_orients{1};
        end
      end
      for vox = 1:length(voxels)
        obj.occupancy_map_active_orient(vox,active_orients) = true;
      end
    end
    
    function obj = deactivateOrient(obj, voxels, inactive_orients)
      if length(voxels) > 1
        if ~iscell(inactive_orients)
          error('inactive_orients must be a cell with the orientations idx to deactivate for each voxel')
        end
      else
        if iscell(inactive_orients)
          inactive_orients = inactive_orients{1};
        end
      end
      for vox = 1:length(voxels)
        obj.occupancy_map_active_orient(vox,inactive_orients) = false;
      end
    end
    
    function obj = setEEPose(obj, EE_pose)
      obj.EE_pose = EE_pose(1:3);
    end
    
    function centres = getCentresRelativeToWorld(obj)
      centres = bsxfun(@plus, obj.EE_pose, obj.vox_centres);
    end
    
    function centres = getActiveCentresRelativeToOrigin(obj)
      centres = bsxfun(@plus, obj.getActiveVoxelCentres(), obj.EE_pose);
    end
    
    function points = findPointsFromDirection(obj, direction, threshold)
      [~, frames] = obj.distributePointsOnSphere(obj.n_directions_per_voxel);
      points = false(obj.n_directions_per_voxel, 1);
%       voxel();
%       hold on
%       plot3(P(1,:), P(2,:), P(3,:), 'r.')
      for p = 1:obj.n_directions_per_voxel
        if acos(frames(p,:,3)*direction)/norm(direction) <= threshold
          points(p) = true;
%           plot3([P(1,p), P(1,p) - frames(p, 1, 3)'], [P(2,p), P(2,p) - frames(p, 2, 3)'], [P(3,p), P(3,p) - frames(p, 3, 3)'], 'b')
        end
      end
    end
    
    function idx = findVoxelsFromDirection(obj, direction, threshold, in_active_set)
      
      if in_active_set
        active_idx = find(obj.active_voxels);
      else
        active_idx = 1:obj.n_voxels;
      end
      idx = [];
      points = obj.findPointsFromDirection(direction, threshold);
      for s = active_idx'
        if any(obj.map(s, points))
          idx(end+1) = s;
        end
      end  
      
    end

    function obj = reduceActiveSet(obj, direction, des_vox_num, reset_active,...
        point_cloud, sagittal_angle, transverse_angle, sagittal_weight, transverse_weight)
      
      collidingTimer = tic;
      obj = obj.deactivateCollidingVoxels(point_cloud, reset_active);
      fprintf('Colliding Time: %.2f s\n', toc(collidingTimer))
      
      if obj.n_active_voxels > des_vox_num
        max_threshold = pi;
        min_threshold = 0;
        threshold_range = pi/50;
        while max_threshold - min_threshold > threshold_range
          mid_threshold = (max_threshold + min_threshold)/2;
          idx_mid = obj.findVoxelsFromDirection(direction, mid_threshold, true);
          if length(idx_mid) > des_vox_num
            max_threshold = mid_threshold;
          else
            min_threshold = mid_threshold;
          end
        end
        idx_min = obj.findVoxelsFromDirection(direction, min_threshold, true);
        idx_max = obj.findVoxelsFromDirection(direction, max_threshold, true);
        idx = min(abs([numel(idx_min), numel(idx_max)] - des_vox_num));
        obj = obj.activateVoxels(idx);
      end
      
      reachability_weight = 0;
      while obj.n_active_voxels > des_vox_num
        reachability_weight = reachability_weight + 0.5;
        obj = obj.prune(sagittal_angle, transverse_angle, sagittal_weight, transverse_weight, reachability_weight, false);
      end
    end
    
    function drawCapabilityMap(obj, text, draw_cubes)
      if nargin < 2 || isempty(text), text = 'Capability Map'; end
      if nargin < 3 || isempty(draw_cubes), draw_cubes = true; end
      obj.drawMap(true(1, obj.n_voxels), text, [], draw_cubes);
    end
    
    function drawCapabilityMapCentredOnPoint(obj, point, text, draw_cubes)
      if nargin < 3 || isempty(text), text = 'Capability Map'; end
      if nargin < 3 || isempty(draw_cubes), draw_cubes = true; end
      obj.drawMap(true(1, obj.n_voxels), text, point, draw_cubes);
    end
    
    function drawActiveMap(obj, text, draw_cubes)
      if nargin < 2 || isempty(text), text = 'Active Capability Map'; end
      if nargin < 3 || isempty(draw_cubes), draw_cubes = true; end
      obj.drawMap(obj.active_voxels, text, [], draw_cubes);
    end
    
    function drawActiveMapCentredOnPoint(obj, point, text, draw_cubes)
      if nargin < 3 || isempty(text), text = 'Active Capability Map'; end
      if nargin < 3 || isempty(draw_cubes), draw_cubes = true; end
      obj.drawMap(obj.active_voxels, text, point, draw_cubes);
    end
    
    function drawMap(obj, voxels, text, offset, draw_cubes)
      lcmClient = LCMGLClient(text);
      if nargin < 4 || isempty(offset), offset = [0;0;0]; end
      if nargin < 5, draw_cubes = true; end
      if draw_cubes
        lcmClient = obj.drawMapCubes(lcmClient, obj.map_lb + offset, obj.map_ub + offset, obj.vox_edge, offset);
        start_idx = 0;
      else
        start_idx = 1;
      end
      for i = start_idx:obj.n_directions_per_voxel
        h = 1-(i/obj.n_directions_per_voxel*2/3);
        rgb = hsv2rgb(h, 1, 1);
        lcmClient.glColor3f(rgb(1), rgb(2), rgb(3));
        if i == 0
          lcmClient.glPointSize(1);
        else
          lcmClient.glPointSize(10);
        end
        coords = obj.vox_centres(:, (obj.reachability_index == i/obj.n_directions_per_voxel) & voxels);
        if nargin > 3
          coords = bsxfun(@plus, offset(1:3), coords);
        end
        if ~isempty(coords)
          lcmClient.points(coords(1,:), coords(2,:), coords(3,:));
        end
      end
      lcmClient.switchBuffers();
    end
    
    function drawOccupancyMap(obj, voxel, r, p, y, offset, text, draw_cubes, draw_base)
      if nargin < 7 || isempty(text), text = 'Occupancy Map'; end
      if nargin < 8 || isempty(draw_cubes), draw_cubes = true; end
      if nargin < 8, draw_base = false; end
      lcmClient = LCMGLClient(text);
      coords = obj.getOccupancyMapCentres();
      
      if draw_base
        doc = com.mathworks.xml.XMLUtils.createDocument('robot');
        robotNode = doc.getDocumentElement;
        robot_name = obj.urdf.getDocumentElement().getAttribute('name');
        if ~isempty(robot_name)
          robotNode.setAttribute('name',robot_name);
        end

        links = obj.urdf.getDocumentElement().getElementsByTagName('link');
        for l = 0:links.getLength()-1
          if strcmp(links.item(l).getAttribute('name'), obj.base_link)
            linkNode = doc.importNode(links.item(l), true);
            robotNode.appendChild(linkNode);
            break
          end
        end
        urdf_string = xmlwrite(doc);
        base = RigidBodyManipulator();
        base = base.addRobotFromURDFString(urdf_string, [], [], struct('floating', true));
        v = base.constructVisualizer();
        rpy = [obj.occupancy_map_orient_steps.roll(r); obj.occupancy_map_orient_steps.pitch(p); obj.occupancy_map_orient_steps.yaw(y)];
        pos = obj.vox_centres(:,voxel)-rpy2rotmat(rpy)*obj.map_left_centre;
        v.draw(0, [pos;rpy])
      end
      
      if nargin < 6 || isempty(offset)
        offset = [0;0;0];
      end
      if any(offset ~= 0)
        coords = bsxfun(@plus, offset(1:3), coords);
      end
      
      if draw_cubes
        lcmClient = obj.drawMapCubes(lcmClient, obj.occupancy_map_lb, obj.occupancy_map_ub, obj.occupancy_map_resolution, offset);
      end
      orient_size = [length(obj.occupancy_map_orient_steps.roll),...
        length(obj.occupancy_map_orient_steps.pitch),...
        length(obj.occupancy_map_orient_steps.yaw)];
      colliding_points = coords(:,obj.occupancy_map{sub2ind(orient_size,r,p,y)}(:, voxel));
      free_points = coords(:,~obj.occupancy_map{sub2ind(orient_size,r,p,y)}(:, voxel));
      if ~isempty(colliding_points)
        lcmClient.glPointSize(10);
        lcmClient.glColor3f(1, 0, 0);
        lcmClient.points(colliding_points(1,:), colliding_points(2,:), colliding_points(3,:))
      end
      if ~isempty(free_points) && ~draw_cubes
        lcmClient.glPointSize(1);
        lcmClient.glColor3f(0, 1, 0);
        lcmClient.points(free_points(1,:), free_points(2,:), free_points(3,:))
      end
      lcmClient.switchBuffers();
    end
    
    function obj = deactivateCollidingVoxels(obj, point_cloud, reset_active)
      
      if reset_active
        obj = obj.resetActiveVoxels();
        obj = obj.resetActiveOrient();
      end
      
      if isempty(obj.EE_pose)
        obj.EE_pose = [0;0;0];
      end
      
%       cm_ub = max(obj.getActiveCentresRelativeToOrigin(), [], 2);
%       cm_lb = min(obj.getActiveCentresRelativeToOrigin(), [], 2);
%       n_vox_per_edge = nthroot(obj.n_voxels, 3);
      
      for pt = 1:size(point_cloud, 2)
        if all(point_cloud(:,pt) < obj.occupancy_map_ub) && all(point_cloud(:,pt) > obj.occupancy_map_lb)
          sub = ceil((point_cloud(:,pt) - obj.EE_pose(1:3))/obj.occupancy_map_resolution) + n_vox_per_edge/2 * ones(3,1);
          voxInd = sub2ind(n_vox_per_edge * ones(1,3), sub(1), sub(2), sub(3));
%           obj = obj.deactivateOrient(obj.occupancy_map(
          obj = obj.deactivateVoxels(obj.occupancy_map(:,voxInd));
        end
      end
    end
    
    function obj = prune(obj, sagittal_angle,...
        transverse_angle, sagittal_weight, transverse_weight, reachability_weight, reset_active)
      
      if reset_active
        obj = obj.resetActiveVoxels();
      end
      
      Dmax = max(obj.reachability_index);
      
      for vox = 1:obj.n_voxels
        if obj.active_voxels(vox)
          sa = atan2(obj.vox_centres(3,vox), obj.vox_centres(1,vox));
          sa = sa - sign(sa) * pi;
          ta = atan2(obj.vox_centres(2,vox), obj.vox_centres(1,vox));
          ta = ta - sign(ta) * pi;
          sagittal_cost = sagittal_weight * abs(sa - sagittal_angle);
          transverse_cost = transverse_weight * abs(ta - transverse_angle);
          reachability_cost = reachability_weight * (Dmax - obj.reachability_index(vox));
          if sqrt(sagittal_cost^2 + transverse_cost^2) + reachability_cost >= 2
            obj = obj.deactivateVoxels(vox);
          end
        end
      end
    end
    
    function obj = resetActiveVoxels(obj, include_zero_reachability)
      if nargin < 2
        include_zero_reachability = false;
      end
      obj = obj.activateVoxels(1:obj.n_voxels);
      if ~include_zero_reachability
        obj = obj.deactivateVoxels(obj.reachability_index == 0);
      end
    end
    
    function obj = resetActiveOrient(obj)
      obj = obj.deactivateOrient(1:obj.n_voxels, false(obj.n_voxels, obj.occupancy_map_n_orient));
    end
    
    function centres = getActiveVoxelCentres(obj)
      centres = obj.vox_centres(:, obj.active_voxels);
    end
    
    function obj = generateCapabilityMap(obj, options)
  
      if nargin < 2 || isempty(options), options = struct(); end
      if isfield(options,'vox_edge'), obj.vox_edge = options.vox_edge; else obj.vox_edge = 0.05; end;
      if isfield(options,'n_samples'), obj.n_samples = options.n_samples; else obj.n_samples = 1e6; end;
      if isfield(options,'n_directions_per_voxel'), obj.n_directions_per_voxel = options.n_directions_per_voxel; else obj.n_directions_per_voxel = 50; end;
      if isfield(options,'pos_tolerance'), obj.pos_tolerance = options.pos_tolerance; else obj.pos_tolerance = 0.01; end;
      if isfield(options,'ang_tolerance'), obj.ang_tolerance = options.ang_tolerance; else obj.ang_tolerance = pi/180; end;
      if ~isfield(options,'use_parallel_toolbox'), options.use_parallel_toolbox = true; end;
  
      %Compute arm length
      kinsol = obj.rbm.doKinematics(obj.nominal_configuration, []);
      end_effector = obj.rbm.findLinkId(obj.end_effector_link.left);
      end_effector_position = obj.rbm.forwardKin(kinsol, end_effector, [0;0;0]);
      distance = norm(obj.map_left_centre-end_effector_position);
      
      % Workspace discretization
      n_vox_per_edge = 2*ceil(distance/obj.vox_edge);
      workspace_edge = n_vox_per_edge * obj.vox_edge;
      obj.n_voxels = n_vox_per_edge^3;
      sphX = linspace(-(workspace_edge-obj.vox_edge)/2, (workspace_edge-obj.vox_edge)/2, n_vox_per_edge);
      sphY = linspace(-(workspace_edge-obj.vox_edge)/2, (workspace_edge-obj.vox_edge)/2, n_vox_per_edge);
      sphZ = linspace(-(workspace_edge-obj.vox_edge)/2, (workspace_edge-obj.vox_edge)/2, n_vox_per_edge);
      [vecY, vecX, vecZ] = meshgrid(sphY, sphX, sphZ);
      vecX = reshape(vecX, numel(vecX), 1);
      vecY = reshape(vecY, numel(vecY), 1);
      vecZ = reshape(vecZ, numel(vecZ), 1);
      obj.vox_centres = [vecX vecY vecZ]';
      obj.map_lb = obj.vox_centres(:,1) - ones(3,1) * obj.vox_edge/2;
      obj.map_ub = obj.vox_centres(:,end) + ones(3,1) * obj.vox_edge/2;

      directions = obj.distributePointsOnSphere(obj.n_directions_per_voxel);
      obj.map = false(obj.n_voxels, obj.n_directions_per_voxel);
      obj.reachability_index = zeros(1, obj.n_voxels);
      obj.active_voxels = false(1, obj.n_voxels);
      
      %Compute map
      nv = obj.n_voxels;
      ndpv = obj.n_directions_per_voxel;
      eea = obj.end_effector_axis;
      ve = obj.vox_edge;
      vc = obj.vox_centres;
      pt = obj.pos_tolerance;
      at = obj.ang_tolerance;
      mc = obj.map_left_centre;
      v = ver;
      urdf_string = xmlwrite(obj.urdf);
      if options.use_parallel_toolbox && any(strcmp({v.Name}, 'Parallel Computing Toolbox'))
        pp = gcp;
        n_samples_per_worker = ceil(obj.n_samples/pp.NumWorkers);
        parfor w = 1:pp.NumWorkers
          worker_map{w} = CapabilityMap.computeMap(urdf_string, nv, ndpv, ...
            n_vox_per_edge, n_samples_per_worker, eea, ve, vc, ...
            directions, pt, at, end_effector, mc, w);
        end
        for w = 1:numel(worker_map)
          obj.map = obj.map | worker_map{w};
        end
      else
        disp('No parallel toolbox installed, computation might take very long!')
        obj.map = CapabilityMap.computeMap(urdf_string, nv, ndpv, ...
          n_vox_per_edge, obj.n_samples, eea, ve, vc, ...
          directions, pt, at, end_effector, mc, 1);
      end
      for v = 1:obj.n_voxels
        obj.reachability_index(v) = nnz(obj.map(v,:))/obj.n_directions_per_voxel;
      end
      obj = obj.resetActiveVoxels();
    end
    
    function obj = generateOccupancyMap(obj, resolution, roll_steps, ...
        pitch_steps, yaw_steps, use_parallel_toolbox)
      
      if nargin < 6, use_parallel_toolbox = true; end
      
%       Generate rigid body manipulator from urdf
      if isempty(obj.map)
        error('A capability map is needed to generate an occupancy map.')
      end
      doc = com.mathworks.xml.XMLUtils.createDocument('robot');
      robotNode = doc.getDocumentElement;
      robot_name = obj.urdf.getDocumentElement().getAttribute('name');
      if ~isempty(robot_name)
        robotNode.setAttribute('name',robot_name);
      end
      
      links = obj.urdf.getDocumentElement().getElementsByTagName('link');
      for l = 0:links.getLength()-1
        if strcmp(links.item(l).getAttribute('name'), obj.base_link)
          linkNode = doc.importNode(links.item(l), true);
          robotNode.appendChild(linkNode);
          break
        end
      end
      urdf_string = xmlwrite(doc);
      base = RigidBodyManipulator();
      base = base.addRobotFromURDFString(urdf_string, [], [], struct('floating', true));
      
      obj.occupancy_map_orient_steps = struct('roll', roll_steps, 'pitch', pitch_steps, 'yaw', yaw_steps);
      obj.occupancy_map_resolution = resolution;
      obj.occupancy_map_n_orient = length(roll_steps)*length(pitch_steps)*length(yaw_steps);
      [BB_lb, BB_ub] = obj.getBoundingBoxBoundsRelativeToMapCentre(base.body(2));
      obj.occupancy_map_lb = obj.vox_centres(:,1) + BB_lb;
      obj.occupancy_map_ub = obj.vox_centres(:,end) + BB_ub;
      obj.occupancy_map_dimensions = ceil((obj.occupancy_map_ub - obj.occupancy_map_lb)/obj.occupancy_map_resolution);
      obj.occupancy_map_ub = obj.occupancy_map_lb + obj.occupancy_map_dimensions * obj.occupancy_map_resolution;
      obj.occupancy_map_n_voxels = prod(obj.occupancy_map_dimensions);
      obj.occupancy_map_active_orient = false(obj.n_voxels, obj.occupancy_map_n_orient);
      obj.occupancy_map = cell(obj.occupancy_map_n_orient);
      
      
      %Compute map
      vis = base.constructVisualizer();
      voxels_to_check = find(obj.reachability_index~=0);
      omnv = obj.occupancy_map_n_voxels;
      vc = obj.vox_centres;
      mc = obj.map_left_centre;
      omc = obj.getOccupancyMapCentres();
      v = ver;
      [p, r, y] = meshgrid(pitch_steps, roll_steps, yaw_steps);
      for idx = 1:obj.occupancy_map_n_orient
        fprintf('Computing map %d of %d\n', idx, obj.occupancy_map_n_orient);
        if use_parallel_toolbox && any(strcmp({v.Name}, 'Parallel Computing Toolbox'))
          spmd
            om = false(omnv, length(voxels_to_check), 'codistributed');
            base = RigidBodyManipulator();
            base = base.addRobotFromURDFString(urdf_string, [], [], struct('floating', true));
            for i = drange(1:length(voxels_to_check))
              q = [vc(:, voxels_to_check(i))- rpy2rotmat([r(idx); p(idx); y(idx)])*mc; r(idx); p(idx); y(idx)];
              kinsol = base.doKinematics(q);
              colliding_points = base.collidingPoints(kinsol, omc, resolution/2);
              if ~isempty(colliding_points)
                om(colliding_points, i) = ~om(colliding_points, i);
              end
            end
          end
          om = gather(om);
        else
          om = false(omnv, length(voxels_to_check));
          for i = 1:length(voxels_to_check)
            q = [vc(:, voxels_to_check(i))- rpy2rotmat([r(idx); p(idx); y(idx)])*mc; r(idx); p(idx); y(idx)];
            kinsol = base.doKinematics(q);
            colliding_points = base.collidingPoints(kinsol, omc, resolution/2);
            if ~isempty(colliding_points)
              vis.draw(0, q)
              drawTreePoints(omc(:, colliding_points));
              drawTreePoints(vc(:, voxels_to_check(i)), 'colour', [1 0 1], 'text', 'joint')
              om(colliding_points, i) = ~om(colliding_points, i);
            end
          end
        end
%         obj.occupancy_map(:, :, idx) = false(omnv, obj.n_voxels);
        obj.occupancy_map{idx} = sparse(om);
      end
    end
    
    function [lb, ub] = getBoundingBoxBoundsRelativeToMapCentre(obj, body)
      [r,p,y] = meshgrid(obj.occupancy_map_orient_steps.roll,...
                         obj.occupancy_map_orient_steps.pitch,...
                         obj.occupancy_map_orient_steps.yaw);
       body_BB = bsxfun(@minus, body.collision_geometry{1}.getBoundingBoxPoints(), obj.map_left_centre);
       ub = zeros(3,1);
       lb = zeros(3,1);
       for o = 1:obj.occupancy_map_n_orient
         ub = max([max(rpy2rotmat([r(o); p(o); y(o)])*body_BB, [], 2), ub], [], 2);
         lb = min([min(rpy2rotmat([r(o); p(o); y(o)])*body_BB, [], 2), lb], [], 2);
       end
    end
    
    function centres = getOccupancyMapCentres(obj)
      [y,x,z] = meshgrid(obj.occupancy_map_lb(2) + obj.occupancy_map_resolution/2:obj.occupancy_map_resolution:obj.occupancy_map_ub(2), ...
                         obj.occupancy_map_lb(1) + obj.occupancy_map_resolution/2:obj.occupancy_map_resolution:obj.occupancy_map_ub(1), ...
                         obj.occupancy_map_lb(3) + obj.occupancy_map_resolution/2:obj.occupancy_map_resolution:obj.occupancy_map_ub(3));
      centres = [reshape(x, 1, obj.occupancy_map_n_voxels); ...
                 reshape(y, 1, obj.occupancy_map_n_voxels); ...
                 reshape(z, 1, obj.occupancy_map_n_voxels)];
    end
    
  end
    
  methods (Static)
    
    function [P, frames] = distributePointsOnSphere(N)
      k = 1:N;
      h = -1 + 2*(k-1)/(N-1);
      theta = acos(h);
      phi = zeros(1,N);
      for i = 2:N-1
        phi(i) = mod(phi(i-1) + 3.6/(sqrt(N) * sqrt(1 - h(i)^2)), 2*pi);
      end
      x = cos(phi).*sin(theta);
      y = sin(phi).*sin(theta);
      z = cos(theta);
      P = [x; y; z];
      frames = zeros(N, 3, 3);
      for p = 1:N
        frame = zeros(3);
        frame(1:3,3) = -P(:,p);
        if abs(frame(1,3)) <= 1e-10 && abs(frame(2,3)) <= 1e-10
          frame(:,1) = [sign(frame(3,3)); 0; 0];
        else
          frame(2,1) = sqrt(frame(1,3)^2/(frame(2,3)^2 + frame(1,3)^2));
          frame(1,1) = -sign(frame(1,3)*frame(2,3))*sqrt(1-frame(2,1)^2);
        end
        frame(:,2) = cross(frame(:,3), frame(:,1));
        frames(p, :, :) = frame;
      end
    end
    
    function worker_map = computeMap(urdf_string, n_voxels, n_directions_per_voxel, n_vox_per_edge, ...
        n_samples, end_effector_axis, vox_edge, centres, ...
        directions, pos_tolerance, ang_tolerance, end_effector, map_centre, worker)
      
      manipulator = RigidBodyManipulator();
      manipulator = manipulator.addRobotFromURDFString(urdf_string, [], [], struct('floating', true));
      torso_constraint = PostureConstraint(manipulator);
      torso_constraint = torso_constraint.setJointLimits((1:6)', [-map_centre; 0; 0; 0], [-map_centre; 0; 0; 0]);
      
      %IK Options
      Q = diag(manipulator.num_positions:-1:1);
      ikoptions = IKoptions(manipulator);
      ikoptions = ikoptions.setMajorIterationsLimit(100);
      ikoptions = ikoptions.setQ(Q);
      ikoptions = ikoptions.setMajorOptimalityTolerance(1e-3);
      
      worker_map = false(n_voxels, n_directions_per_voxel);
      status = 0;
      pos_constraints = WorldPositionConstraint.empty(0,size(centres,2));
      gaze_constraints = WorldGazeDirConstraint.empty(0, n_directions_per_voxel);
      for c = 1:size(centres,2)
        pos_constraints(c) = WorldPositionConstraint(manipulator, end_effector, [0;0;0], centres(:,c) - pos_tolerance/2, centres(:,c) + pos_tolerance/2);
      end
      for p = 1:n_directions_per_voxel
        gaze_constraints(p) = WorldGazeDirConstraint(manipulator, end_effector, end_effector_axis, directions(:, p), ang_tolerance/2);
      end
        
%       counter_times = zeros(n_samples, 1);
%       setup_times = zeros(n_samples, 1);
%       constraint_times = zeros(n_samples*n_directions_per_voxel,1);
%       ik_times = zeros(n_samples*n_directions_per_voxel, 1);
%       v = rbm.constructVisualizer();
      for sample = 1:n_samples
%         counter_timer = tic;
        if floor(sample/n_samples*100) > status
          status = floor(sample/n_samples*100);
          fprintf('Worker %d: %d%% complete\n', worker, status)
        end
%         counter_times(sample) = toc(counter_timer);
%         setup_timer = tic;
        active_joints = 7:manipulator.num_positions;
        q = [-map_centre; 0; 0; 0; manipulator.joint_limit_min(active_joints) + (manipulator.joint_limit_max(active_joints)- ...
          manipulator.joint_limit_min(active_joints)).*rand(manipulator.num_positions-6,1)];
%         v.draw(0, q)
        kinsol = manipulator.doKinematics(q);
        pos = manipulator.forwardKin(kinsol, end_effector, [0;0;0]);
        sub = ceil(pos/vox_edge) + (n_vox_per_edge/2) * ones(3,1);
        vox_ind = sub2ind(n_vox_per_edge * ones(1,3), sub(1), sub(2), sub(3));
%         posConstraint = WorldPositionConstraint(rbm, end_effector, end_effector_point, centres(:,vox_ind) - pos_tolerance/2, centres(:,vox_ind) + pos_tolerance/2);
%         setup_times(sample) = toc(setup_timer);
        for point = 1:n_directions_per_voxel
%           constraint_timer = tic;
          if worker_map(vox_ind, point)
            continue
          end
%           drawTreePoints(bsxfun(@plus, centres(:,vox_ind), [[0;0;0] directions(:, point)]), 'lines', true, 'text', 'dir')
%           GazeConstraint = WorldGazeDirConstraint(rbm, end_effector, end_effector_axis, directions(:, point), ang_tolerance/2);
%           constraint_times(sample*(n_directions_per_voxel-1)+point) = toc(constraint_timer);
%           ik_timer = tic;
          [q_new, info] = manipulator.inverseKin(q, q, pos_constraints(vox_ind), gaze_constraints(point), torso_constraint, ikoptions);
%           ik_times(sample*(n_directions_per_voxel-1)+point) = toc(ik_timer);
          if info < 10
%             v.draw(0, q_new)
            worker_map(vox_ind, point) = true;
          end
        end
      end
%       sprintf(['counter: %.1d\n' ...
%               'setup: %.1d\n' ...
%               'constraints: %.1d\n'...
%               'ik: %.1d\n'...
%               't per sample: %.1d\n'...
%               'tot time: %.1d h\n'], mean(counter_times), mean(setup_times), mean(constraint_times), mean(ik_times),...
%               sum([mean(counter_times), mean(setup_times), 50*mean(constraint_times), 50*mean(ik_times)]),...
%               sum([mean(counter_times), mean(setup_times), 50*mean(constraint_times), 50*mean(ik_times)])*1e6/3600)
    end
    
    function lcmClient = drawMapCubes(lcmClient, lb, ub, resolution, centre)
      dimensions = (ub - lb)/resolution;
      lcmClient.glColor3f(0.3,0.3,0.3);
      lcmClient.glLineWidth(.1);
      for i = 1:dimensions(2) + 1
        for j = 1:dimensions(3) + 1
          lcmClient.line3(lb(1),...
                          lb(2) + resolution * (i-1),...
                          lb(3) + resolution * (j-1),...
                          ub(1),...
                          lb(2) + resolution * (i-1),...
                          lb(3) + resolution * (j-1))
        end
      end
      for i = 1:dimensions(3) + 1
        for j = 1:dimensions(1) + 1
          lcmClient.line3(lb(1) + resolution * (j-1),...
                          lb(2),...
                          lb(3) + resolution * (i-1),...
                          lb(1) + resolution * (j-1),...
                          ub(2),...
                          lb(3) + resolution * (i-1))
        end
      end
      for i = 1:dimensions(1) + 1
        for j = 1:dimensions(2) + 1
          lcmClient.line3(lb(1) + resolution * (i-1),...
                          lb(2) + resolution * (j-1),...
                          lb(3),...
                          lb(1) + resolution * (i-1),...
                          lb(2) + resolution * (j-1),...
                          ub(3))
        end
      end
      lcmClient.glLineWidth(4);
      lcmClient.glColor3f(1,0,0);
      lcmClient.line3(centre(1), centre(2), centre(3),...
                      centre(1) + 0.2, centre(2), centre(3))
      lcmClient.glColor3f(0,1,0);
      lcmClient.line3(centre(1), centre(2), centre(3),...
                      centre(1), centre(2) + 0.2, centre(3))
      lcmClient.glColor3f(0,0,1);
      lcmClient.line3(centre(1), centre(2), centre(3),...
                      centre(1), centre(2), centre(3) + 0.2)
    end
    
  end
  
end