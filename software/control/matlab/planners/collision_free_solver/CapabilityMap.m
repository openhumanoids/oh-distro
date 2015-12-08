classdef CapabilityMap
  
  properties
    map
    reachability_index
    vox_centers
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
    base_link
    active_voxels
    n_active_voxels
    EE_pose
    base2root
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
      obj.vox_centers = vars.vox_centers;
      obj.vox_edge = vars.options.vox_edge;
      obj.n_samples = vars.options.n_samples;
      obj.ang_tolerance = vars.options.ang_tolerance;
      obj.pos_tolerance = vars.options.pos_tolerance;
      obj.urdf = vars.options.urdf;
      obj.n_voxels = size(obj.map, 1);
      obj.n_directions_per_voxel = size(obj.map, 2);
      obj.root_link = vars.options.root_link;
      obj.root_point = vars.options.root_point;
      obj.end_effector_link = vars.options.end_effector_link;
      obj.end_effector_point = vars.options.end_effector_point;
      obj.base_link = vars.options.base_link;
      obj.active_voxels = true(obj.n_voxels, 1);
      obj.n_active_voxels = obj.n_voxels;
      obj.base2root = vars.options.base2root;
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
      centres = bsxfun(@plus, obj.EE_pose, obj.vox_centers);
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
    
    function drawMap(obj, colour, text)
      if nargin < 2, colour = [0 1 0]; end
      if nargin < 3, text = 'Capability Map'; end
      obj.drawVoxelCentres(obj.vox_centers, colour, text);
    end
    
    function drawMapCentredOnPoint(obj, point, colour, text)
      if nargin < 3, colour = [0 1 1]; end
      if nargin < 4, text = 'Capability Map'; end
      obj.drawVoxelCentres(bsxfun(@plus, point(1:3), obj.vox_centers), colour, text);
    end
    
    function drawActiveMap(obj, colour, text)
      if nargin < 2, colour = [0 1 1]; end
      if nargin < 3, text = 'Active Capability Map'; end
      obj.drawVoxelCentres(obj.getActiveVoxelCentres(), colour, text);
    end
    
    function drawActiveMapCentredOnPoint(obj, point, colour, text)
      if nargin < 3, colour = [0 1 1]; end
      if nargin < 4, text = 'Active Capability Map'; end
      obj.drawVoxelCentres(bsxfun(@plus, point(1:3), obj.getActiveVoxelCentres()), colour, text);
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
          sa = atan2(obj.vox_centers(3,vox), obj.vox_centers(1,vox));
          sa = sa - sign(sa) * pi;
          ta = atan2(obj.vox_centers(2,vox), obj.vox_centers(1,vox));
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
      centres = obj.vox_centers(:, obj.active_voxels);
    end
    
    function obj = generateCapabilityMap(obj, urdf_file, kinematic_chain_left, ...
        end_effector_right, options)
  
      if nargin < 5 || isempty(options), options = struct(); end
      if isfield(options,'vox_edge'), obj.vox_edge = options.vox_edge; else obj.vox_edge = 0.05; end;
      if isfield(options,'n_samples'), obj.n_samples = options.n_samples; else obj.n_samples = 10e3; end;
      if isfield(options,'n_directions_per_voxel'), obj.n_directions_per_voxel = options.n_directions_per_voxel; else obj.n_directions_per_voxel = 50; end;
      if isfield(options,'pos_tolerance'), obj.pos_tolerance = options.pos_tolerance; else obj.pos_tolerance = 0.01; end;
      if isfield(options,'ang_tolerance'), obj.ang_tolerance = options.ang_tolerance; else obj.ang_tolerance = pi/180; end;
      if isfield(options,'map_left_centre'), obj.map_left_centre = options.map_left_centre; else obj.map_left_centre = [0;0;0]; end;
      
      obj.urdf = xmlread(urdf_file);
      obj.base_link = kinematic_chain_left{1};
      obj.end_effector_link.left = kinematic_chain_left{end};
      obj.end_effector_link.right = end_effector_right;
      
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
      
      xmlwrite('capabilityMapManipulator.urdf', doc)
      rbm = RigidBodyManipulator('capabilityMapManipulator.urdf');
  
      %Compute arm length
      q = zeros(rbm.num_positions, 1);
      kinsol = rbm.doKinematics(q, []);
      if isnumeric(options.map_left_centre)
        obj.map_left_centre = options.map_left_centre;
      else
        obj.map_left_centre = rbm.forwardKin(kinsol, rbm.findLinkId(options.map_left_centre), [0;0;0]);
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
      obj.vox_centers = [vecX vecY vecZ]';

      directions = obj.distributePointsOnSphere(obj.n_directions_per_voxel);
      obj.map = false(obj.n_voxels, obj.n_directions_per_voxel);
      
      %Compute map
      pp = gcp;
      n_samples_per_worker = ceil(obj.n_samples/pp.NumWorkers);
      nv = obj.n_voxels;
      ndpv = obj.n_directions_per_voxel;
      ve = obj.vox_edge;
      vc = obj.vox_centers;
      pt = obj.pos_tolerance;
      at = obj.ang_tolerance;
      mc = obj.map_left_centre;
      parfor w = 1:pp.NumWorkers
        worker_map{w} = CapabilityMap.computeMap(nv, ndpv, ...
          n_vox_per_edge, n_samples_per_worker, ve, vc, ...
          directions, pt, at, end_effector, mc);
      end
      
      for w = 1:numel(worker_map)
        obj.map = obj.map | worker_map{w};
      end
      
      delete('capabilityMapManipulator.urdf')
    end
    
    function obj = generateOccupancyMap(obj, resolution)
%       Generate rigid body manipulator from urdf
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
      xmlwrite('base.urdf', doc)
      base = RigidBodyManipulator('base.urdf', struct('floating', true));
      
      base_BB = base.body(2).collision_geometry{1}.getBoundingBoxPoints();
      obj.occupancy_map_resolution = resolution;
      obj.occupancy_map_lb = min(bsxfun(@plus, obj.vox_centers(:,1) - obj.base2root, base_BB), [], 2);
      obj.occupancy_map_ub = max(bsxfun(@plus, obj.vox_centers(:,end) - obj.base2root, base_BB), [], 2);
      obj.occupancy_map_dimensions = ceil((obj.occupancy_map_ub - obj.occupancy_map_lb)/obj.occupancy_map_resolution);
      obj.occupancy_map_ub = obj.occupancy_map_lb + obj.occupancy_map_dimensions * obj.occupancy_map_resolution;
      om_n_voxels = prod(obj.occupancy_map_dimensions);
      
      centres = obj.vox_centers;
      b2r = obj.base2root;
      om_centres = obj.getOccupancyMapCentres();
      n_vox = obj.n_voxels;
      om = logical.empty(0, n_vox + 1);
      parfor vox = 1:n_vox
        vect = false(1, om_n_voxels);
        q = [centres(:, vox)- b2r; 0; 0; 0];
        kinsol = base.doKinematics(q);
        colliding_points = base.collidingPoints(kinsol, om_centres, resolution/2);
        if ~isempty(colliding_points)
          vect(colliding_points) = ~vect(colliding_points);
          om = [om; [vox vect]];
        end
      end
      obj.occupancy_map = [];
      for i = size(om,1)
        obj.occupancy_map(om(i,1), :) = om(i,2:end);
      end
      obj.occupancy_map_n_voxels = prod(obj.occupancy_map_dimensions);
      delete('base.urdf')
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
    
    function drawVoxelCentres(coords, colour, text)
      lcmClient = LCMGLClient(text);
      lcmClient.glColor3f(colour(1), colour(2), colour(3));
      lcmClient.points(coords(1,:), coords(2,:), coords(3,:));
      lcmClient.switchBuffers();
    end
    
    function worker_map = computeMap(n_voxels, n_directions_per_voxel, n_vox_per_edge, ...
        n_samples, vox_edge, centers, directions, pos_tolerance, ang_tolerance, ...
        end_effector, map_centre)
      
      rbm = RigidBodyManipulator('capabilityMapManipulator.urdf', struct('floating', true));
      
      %IK Options
      Q = diag(rbm.num_positions:-1:1);
      ikoptions = IKoptions(rbm);
      ikoptions = ikoptions.setMajorIterationsLimit(100);
      ikoptions = ikoptions.setQ(Q);
      ikoptions = ikoptions.setMajorOptimalityTolerance(1e-3);
      
      worker_map = false(n_voxels, n_directions_per_voxel);
      for sample = 1:n_samples
        fprintf('Sample %d of %d\n', sample, n_samples)
        active_joints = 7:rbm.num_positions;
        q = [-map_centre; 0; 0; 0; rbm.joint_limit_min(active_joints) + (rbm.joint_limit_max(active_joints)- ...
          rbm.joint_limit_min(active_joints)).*rand(rbm.num_positions-6,1)];
        kinsol = rbm.doKinematics(q);
        pos = rbm.forwardKin(kinsol, end_effector, [0;0;0]);
        sub = ceil(pos/vox_edge) + (n_vox_per_edge/2) * ones(3,1);
        vox_ind = sub2ind(n_vox_per_edge * ones(1,3), sub(1), sub(2), sub(3));
%           if options.visualize
%             drawTreePoints(sphCenters(:,sphInd), 'pointsize', diameter/2)
%             v.draw(0, q);
%           end
        for point = 1:n_directions_per_voxel
          pos = centers(:,vox_ind) + directions(:, point)*vox_edge/2;
%             quat = rotmat2quat(squeeze(frames(point,:,:)));
          posConstraint = WorldPositionConstraint(rbm, end_effector, [0;0;0], pos - pos_tolerance/2, pos + pos_tolerance/2);
          GazeConstraint = WorldGazeDirConstraint(rbm, end_effector, [-1; 0; 0], directions(:, point), ang_tolerance);
          [~, info] = rbm.inverseKin(q, q, posConstraint, GazeConstraint, ikoptions);
%             if options.visualize
%               drawTreePoints([pos; quat], 'frame', true, 'text', 'frame')
%               v.draw(0, qNew)
%               drawLinkFrame(r, palm, qNew, 'palm');
%             end
          if info < 10
            worker_map(vox_ind, point) = true;
          end
        end
      end
    end
    
  end
  
end