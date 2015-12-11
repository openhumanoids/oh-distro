classdef CapabilityMap
  
  properties
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
    end_effector_point
    end_effector_axis
    base_link
    active_voxels
    n_active_voxels
    EE_pose
    occupancy_map
    occupancy_map_resolution
    occupancy_map_dimensions
    occupancy_map_n_voxels
    occupancy_map_lb
    occupancy_map_ub
  end
  
  methods
    
    function obj = CapabilityMap(mat_file, EE_pose)
      if nargin > 0
        obj = obj.generateFromFile(mat_file);
      end
      if nargin > 1
        obj = obj.setEEPose(EE_pose);
      end
    end
    
    function obj = generateFromFile(obj, matFile)
      vars = load(matFile);
      obj.map = vars.map;
      obj.reachability_index = vars.reachability_index;
      obj.vox_centres = vars.vox_centres;
      obj.vox_edge = vars.options.vox_edge;
      obj.n_samples = vars.options.n_samples;
      obj.ang_tolerance = vars.options.ang_tolerance;
      obj.pos_tolerance = vars.options.pos_tolerance;
      obj.urdf = vars.options.urdf;
      obj.n_voxels = size(obj.map, 1);
      obj.n_directions_per_voxel = size(obj.map, 2);
      obj.map_left_centre = vars.options.map_left_centre;
      obj.end_effector_link = vars.options.end_effector_link;
      obj.end_effector_point = vars.options.end_effector_point;
      obj.base_link = vars.options.base_link;
      obj.active_voxels = true(obj.n_voxels, 1);
      obj.n_active_voxels = obj.n_voxels;
      obj.map_left_centre = vars.options.map_left_centre;
      obj.occupancy_map = vars.occupancy_map;
      
      obj = obj.resetActiveVoxels();
    end
    
    function obj = activateVoxels(obj, idx)
      obj.active_voxels(idx) = true;
      obj.n_active_voxels = nnz(obj.active_voxels);
    end
    
    function obj = deactivateVoxels(obj, idx)
      obj.active_voxels(idx) = false;
      obj.n_active_voxels = nnz(obj.active_voxels);
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
    
    function drawMap(obj, text)
      if nargin < 2, text = 'Capability Map'; end
      obj.drawVoxelCentres(true(obj.n_voxels, 1), text);
    end
    
    function drawMapCentredOnPoint(obj, point, text)
      if nargin < 3, text = 'Capability Map'; end
      obj.drawVoxelCentres(true(obj.n_voxels, 1), text, point);
    end
    
    function drawActiveMap(obj, text)
      if nargin < 2, text = 'Active Capability Map'; end
      obj.drawVoxelCentres(obj.active_voxels, text);
    end
    
    function drawActiveMapCentredOnPoint(obj, point, text)
      if nargin < 3, text = 'Active Capability Map'; end
      obj.drawVoxelCentres(obj.active_voxels, text, point);
    end
    
    function drawVoxelCentres(obj, voxels, text, offset)
      lcmClient = LCMGLClient(text);
      for i = 0:obj.n_directions_per_voxel
        h = 1-(i/obj.n_directions_per_voxel*2/3);
        rgb = hsv2rgb(h, 1, 1);
        lcmClient.glColor3f(rgb(1), rgb(2), rgb(3));
        if i == 0
          lcmClient.glPointSize(1);
        else
          lcmClient.glPointSize(5);
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
    
    function drawOccupancyMap(obj, voxel, offset, text)
      if nargin < 3, text = 'Occupancy Map'; end
      lcmClient = LCMGLClient(text);
      coords = obj.getOccupancyMapCentres();
      if nargin > 2
        coords = bsxfun(@plus, offset(1:3), coords);
      end
      colliding_points = coords(:,obj.occupancy_map(:, voxel));
      free_points = coords(:,~obj.occupancy_map(:, voxel));
      if ~isempty(colliding_points)
        lcmClient.glPointSize(5);
        lcmClient.glColor3f(1, 0, 0);
        lcmClient.points(colliding_points(1,:), colliding_points(2,:), colliding_points(3,:))
      end
      if ~isempty(free_points)
        lcmClient.glPointSize(1);
        lcmClient.glColor3f(0, 1, 0);
        lcmClient.points(free_points(1,:), free_points(2,:), free_points(3,:))
      end
      lcmClient.switchBuffers();
    end
    
    function obj = deactivateCollidingVoxels(obj, point_cloud, reset_active)
      
      if reset_active
        obj = obj.resetActiveVoxels();
      end
      
      cm_ub = max(obj.getActiveCentresRelativeToOrigin(), [], 2);
      cm_lb = min(obj.getActiveCentresRelativeToOrigin(), [], 2);
      n_vox_per_edge = nthroot(obj.n_voxels, 3);
      
      for pt = 1:size(point_cloud, 2)
        if all(point_cloud(:,pt) < cm_ub) && all(point_cloud(:,pt) > cm_lb)
          sub = ceil((point_cloud(:,pt) - obj.EE_pose(1:3))/obj.vox_edge) + n_vox_per_edge/2 * ones(3,1);
          voxInd = sub2ind(n_vox_per_edge * ones(1,3), sub(1), sub(2), sub(3));
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
    
    function centres = getActiveVoxelCentres(obj)
      centres = obj.vox_centres(:, obj.active_voxels);
    end
    
    function obj = generateCapabilityMap(obj, urdf_file, kinematic_chain_left, ...
        end_effector_right, end_effector_axis, map_left_centre, options)
      
      obj = CapabilityMap();
  
      if nargin < 7 || isempty(options), options = struct(); end
      if isfield(options,'vox_edge'), obj.vox_edge = options.vox_edge; else obj.vox_edge = 0.05; end;
      if isfield(options,'n_samples'), obj.n_samples = options.n_samples; else obj.n_samples = 1e6; end;
      if isfield(options,'n_directions_per_voxel'), obj.n_directions_per_voxel = options.n_directions_per_voxel; else obj.n_directions_per_voxel = 50; end;
      if isfield(options,'pos_tolerance'), obj.pos_tolerance = options.pos_tolerance; else obj.pos_tolerance = 0.01; end;
      if isfield(options,'ang_tolerance'), obj.ang_tolerance = options.ang_tolerance; else obj.ang_tolerance = pi/180; end;
      if isfield(options,'end_effector_point'), obj.end_effector_point = options.end_effector_point; else obj.end_effector_point = [0;0;0]; end;
      if ~isfield(options,'use_parallel_toolbox'), options.use_parallel_toolbox = true; end;
      
      
      obj.urdf = xmlread(urdf_file);
      obj.base_link = kinematic_chain_left{1};
      obj.end_effector_link.left = kinematic_chain_left{end};
      obj.end_effector_link.right = end_effector_right;
      obj.end_effector_axis = end_effector_axis;
      
%       Generate rigid body manipulator from urdf
      doc = com.mathworks.xml.XMLUtils.createDocument('robot');
      robotNode = doc.getDocumentElement;
      robot_name = obj.urdf.getDocumentElement().getAttribute('name');
      if ~isempty(robot_name)
        robotNode.setAttribute('name',robot_name);
      end
      
      links = obj.urdf.getDocumentElement().getElementsByTagName('link');
      for l = 0:links.getLength()-1
        if any(strcmp(links.item(l).getAttribute('name'), kinematic_chain_left))
          linkNode = doc.importNode(links.item(l), true);
          robotNode.appendChild(linkNode);
        end
      end
      
      joints = obj.urdf.getDocumentElement().getElementsByTagName('joint');
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
      rbm = RigidBodyManipulator();
      rbm = rbm.addRobotFromURDFString(urdf_string);
  
      %Compute arm length
      q = zeros(rbm.num_positions, 1);
      kinsol = rbm.doKinematics(q, []);
      if isnumeric(map_left_centre)
        obj.map_left_centre = map_left_centre;
      else
        obj.map_left_centre = rbm.forwardKin(kinsol, rbm.findLinkId(map_left_centre), [0;0;0]);
      end
      end_effector = rbm.findLinkId(obj.end_effector_link.left);
      end_effector_position = rbm.forwardKin(kinsol, end_effector, [0;0;0]);
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

      directions = obj.distributePointsOnSphere(obj.n_directions_per_voxel);
      obj.map = false(obj.n_voxels, obj.n_directions_per_voxel);
      
      %Compute map
      nv = obj.n_voxels;
      ndpv = obj.n_directions_per_voxel;
      eep = obj.end_effector_point;
      eea = obj.end_effector_axis;
      ve = obj.vox_edge;
      vc = obj.vox_centres;
      pt = obj.pos_tolerance;
      at = obj.ang_tolerance;
      mc = obj.map_left_centre;
      v = ver;
      if options.use_parallel_toolbox && any(strcmp({v.Name}, 'Parallel Computing Toolbox'))
        pp = gcp;
        n_samples_per_worker = ceil(obj.n_samples/pp.NumWorkers);
        parfor w = 1:pp.NumWorkers
          worker_map{w} = CapabilityMap.computeMap(urdf_string, nv, ndpv, ...
            n_vox_per_edge, n_samples_per_worker, eep, eea, ve, vc, ...
            directions, pt, at, end_effector, mc, w);
        end
        for w = 1:numel(worker_map)
          obj.map = obj.map | worker_map{w};
        end
      else
        disp('No parallel toolbox installed, computation might take very long!')
        obj.map = CapabilityMap.computeMap(urdf_string, nv, ndpv, ...
          n_vox_per_edge, obj.n_samples, eep, eea, ve, vc, ...
          directions, pt, at, end_effector, mc, 1);
      end
      for v = 1:obj.n_voxels
        obj.reachability_index(v) = nnz(obj.map(v,:))/obj.n_directions_per_voxel;
      end
      obj = obj.resetActiveVoxels();
      delete('capabilityMapManipulator.urdf')
    end
    
    function obj = generateOccupancyMap(obj, resolution, use_parallel_toolbox)
      
      if nargin < 3, use_parallel_toolbox = true; end
      
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
      
      base_BB = base.body(2).collision_geometry{1}.getBoundingBoxPoints();
      obj.occupancy_map_resolution = resolution;
      obj.occupancy_map_lb = min(bsxfun(@plus, obj.vox_centres(:,1) - obj.map_left_centre, base_BB), [], 2);
      obj.occupancy_map_ub = max(bsxfun(@plus, obj.vox_centres(:,end) - obj.map_left_centre, base_BB), [], 2);
      obj.occupancy_map_dimensions = ceil((obj.occupancy_map_ub - obj.occupancy_map_lb)/obj.occupancy_map_resolution);
      obj.occupancy_map_ub = obj.occupancy_map_lb + obj.occupancy_map_dimensions * obj.occupancy_map_resolution;
      obj.occupancy_map_n_voxels = prod(obj.occupancy_map_dimensions);
      
      %Compute map
      omnv = prod(obj.occupancy_map_dimensions);
      vc = obj.vox_centres;
      mc = obj.map_left_centre;
      omc = obj.getOccupancyMapCentres();
      voxels_to_check = find(obj.reachability_index~=0);
      v = ver;
      if use_parallel_toolbox && any(strcmp({v.Name}, 'Parallel Computing Toolbox'))
        spmd
          om = false(omnv, length(voxels_to_check), 'codistributed');
          base = RigidBodyManipulator();
          base = base.addRobotFromURDFString(urdf_string, [], [], struct('floating', true));
          for i = drange(1:length(voxels_to_check))
            q = [vc(:, voxels_to_check(i))- mc; 0; 0; 0];
            kinsol = base.doKinematics(q);
            colliding_points = base.collidingPoints(kinsol, omc, resolution/2);
            if ~isempty(colliding_points)
              om(colliding_points, i) = ~om(colliding_points, i);
            end
          end
        end
      end
      
      obj.occupancy_map = true(omnv, obj.n_voxels);
      om = gather(om);
      obj.occupancy_map(:, voxels_to_check) = om;
    end
    
    function centres = getOccupancyMapCentres(obj)
      [x,y,z] = meshgrid(obj.occupancy_map_lb(1) + obj.occupancy_map_resolution/2:obj.occupancy_map_resolution:obj.occupancy_map_ub(1), ...
                         obj.occupancy_map_lb(2) + obj.occupancy_map_resolution/2:obj.occupancy_map_resolution:obj.occupancy_map_ub(2), ...
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
        n_samples, end_effector_point, end_effector_axis, vox_edge, centres, ...
        directions, pos_tolerance, ang_tolerance, end_effector, map_centre, worker)
      
      rbm = RigidBodyManipulator();
      rbm = rbm.addRobotFromURDFString(urdf_string, [], [], struct('floating', true));
      torso_constraint = PostureConstraint(rbm);
      torso_constraint = torso_constraint.setJointLimits((1:6)', [-map_centre; 0; 0; 0], [-map_centre; 0; 0; 0]);
      
      %IK Options
      Q = diag(rbm.num_positions:-1:1);
      ikoptions = IKoptions(rbm);
      ikoptions = ikoptions.setMajorIterationsLimit(100);
      ikoptions = ikoptions.setQ(Q);
      ikoptions = ikoptions.setMajorOptimalityTolerance(1e-3);
      
      worker_map = false(n_voxels, n_directions_per_voxel);
      status = 0;
      pos_constraints = WorldPositionConstraint.empty(0,size(centres,2));
      gaze_constraints = WorldGazeDirConstraint.empty(0, n_directions_per_voxel);
      for c = 1:size(centres,2)
        pos_constraints(c) = WorldPositionConstraint(rbm, end_effector, end_effector_point, centres(:,c) - pos_tolerance/2, centres(:,c) + pos_tolerance/2);
      end
      for p = 1:n_directions_per_voxel
        gaze_constraints(p) = WorldGazeDirConstraint(rbm, end_effector, end_effector_axis, directions(:, p), ang_tolerance/2);
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
        active_joints = 7:rbm.num_positions;
        q = [-map_centre; 0; 0; 0; rbm.joint_limit_min(active_joints) + (rbm.joint_limit_max(active_joints)- ...
          rbm.joint_limit_min(active_joints)).*rand(rbm.num_positions-6,1)];
%         v.draw(0, q)
        kinsol = rbm.doKinematics(q);
        pos = rbm.forwardKin(kinsol, end_effector, [0;0;0]);
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
          [q_new, info] = rbm.inverseKin(q, q, pos_constraints(vox_ind), gaze_constraints(point), torso_constraint, ikoptions);
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
    
  end
  
end