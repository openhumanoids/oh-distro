 classdef CapabilityMap
  
  properties
    rbm
    map
    reachability_index
    vox_centres
    vox_centres_prob
    vox_edge
    n_samples
    ang_tolerance
    pos_tolerance
    urdf
    n_voxels
    n_directions_per_voxel
    map_centre
    end_effector_link
    end_effector_axis
    base_link
    active_voxels
    n_active_voxels
    nominal_configuration
    map_ub
    map_lb
    EE_pose
    active_side
    occupancy_map
    occupancy_map_resolution
    occupancy_map_dimensions
    occupancy_map_n_voxels
    occupancy_map_lb
    occupancy_map_ub
    occupancy_map_orient_steps
    occupancy_map_active_orient
    occupancy_map_orient
    occupancy_map_n_orient
    occupancy_map_orient_prob
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
        obj.end_effector_axis = reshape(end_effector_axis, [3,1]);
      
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
          obj.map_centre.left = map_left_centre;
        else
          obj.map_centre.left = obj.rbm.forwardKin(kinsol, obj.rbm.findLinkId(map_left_centre), [0;0;0]);
        end
        obj.map_centre.right = obj.map_centre.left .* [1; -1; 1];
      end
    end
    
    function obj = loadFromFile(obj, file)
      vars = load(file);
      if ~all(isfield(vars, {'urdf', 'map_centre', 'end_effector_link', 'end_effector_axis', ...
        'base_link', 'nominal_configuration'}))
        error('Some data is missing')
      end
      obj.urdf = vars.urdf;
      obj.map_centre = vars.map_centre;
      obj.end_effector_link = vars.end_effector_link;
      obj.end_effector_axis = vars.end_effector_axis;
      obj.base_link = vars.base_link;
      obj.nominal_configuration = vars.nominal_configuration;
      obj.rbm = RigidBodyManipulator();
      obj.rbm = obj.rbm.addRobotFromURDFString(xmlwrite(obj.urdf));
      if isfield(vars, 'map')
        if ~all(isfield(vars,  {'map', 'reachability_index', ...
        'vox_edge', 'n_samples', 'ang_tolerance', 'pos_tolerance', ...
        'n_voxels', 'n_directions_per_voxel', 'map_lb', 'map_ub'}))
          error('Some data is missing')
        end
        obj.map = vars.map;
        obj.reachability_index = vars.reachability_index;
        obj.vox_edge = vars.vox_edge;
        obj.n_samples = vars.n_samples;
        obj.ang_tolerance = vars.ang_tolerance;
        obj.pos_tolerance = vars.pos_tolerance;
        obj.n_voxels = size(obj.map, 1);
        obj.n_directions_per_voxel = size(obj.map, 2);
        obj.map_ub = vars.map_ub;
        obj.map_lb = vars.map_lb;
        obj = obj.resetActiveVoxels();
        obj.active_side = 'left';
        obj = obj.computeVoxelsCentres();
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
        obj = obj.resetActiveOrient();
        obj = obj.computeOccupancyMapOrientations();
        obj = obj.computeOrientationProbabilityDistribution();
      else
        warning('No occupancy map data found')
      end
    end
    
    function saveToFile(obj, file)
      obj = obj.setActiveSide('left');
      urdf = obj.urdf;
      map_centre = obj.map_centre;
      end_effector_link = obj.end_effector_link;
      end_effector_axis = obj.end_effector_axis;
      base_link = obj.base_link;
      nominal_configuration = obj.nominal_configuration;
      saved_on = datestr(now);
      vars = {'urdf', 'map_centre', 'end_effector_link', 'end_effector_axis', ...
        'base_link', 'nominal_configuration', 'saved_on'};
      if ~isempty(obj.map)
        map = obj.map;
        reachability_index = obj.reachability_index;
        vox_edge = obj.vox_edge;
        n_samples = obj.n_samples;
        ang_tolerance = obj.ang_tolerance;
        pos_tolerance = obj.pos_tolerance;
        n_voxels = obj.n_voxels;
        n_directions_per_voxel = obj.n_directions_per_voxel;
        map_lb = obj.map_lb;
        map_ub = obj.map_ub;
        vars = [vars, {'map', 'reachability_index', ...
        'vox_edge', 'n_samples', 'ang_tolerance', 'pos_tolerance', ...
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
    
    function exportAsBinFile(obj, file)
      file_id = fopen(file, 'w');
%       fwrite(file_id, length(obj.urdf), 'uint32');
%       fwrite(file_id, obj.urdf, 'char*1');
      fwrite(file_id, obj.map_centre.left, 'double');
      fwrite(file_id, obj.map_centre.right, 'double');
      fwrite(file_id, length(obj.end_effector_link.left), 'uint32');
      fwrite(file_id, obj.end_effector_link.left, 'char*1');
      fwrite(file_id, length(obj.end_effector_link.right), 'uint32');
      fwrite(file_id, obj.end_effector_link.right, 'char*1');
      fwrite(file_id, obj.end_effector_axis, 'double');
      fwrite(file_id, length(obj.base_link), 'uint32');
      fwrite(file_id, obj.base_link, 'char*1');
      fwrite(file_id, length(obj.nominal_configuration), 'uint32');
      fwrite(file_id, obj.nominal_configuration, 'double');
      fwrite(file_id, ~isempty(obj.map), 'ubit8');
      if ~isempty(obj.map)
        fwrite(file_id, obj.n_voxels, 'uint32');
        fwrite(file_id, obj.n_directions_per_voxel, 'uint32');
        fwrite(file_id, nnz(obj.map), 'uint32');
        [r, c] = find(obj.map);
        fwrite(file_id, [r-1, c-1], 'uint32');
        fwrite(file_id, obj.reachability_index, 'double');
        fwrite(file_id, obj.vox_edge, 'double');
        fwrite(file_id, obj.ang_tolerance, 'double');
        fwrite(file_id, obj.pos_tolerance, 'double');
        fwrite(file_id, obj.map_lb, 'double');
        fwrite(file_id, obj.map_ub, 'double');
      end
      fwrite(file_id, ~isempty(obj.occupancy_map), 'ubit8');
      if ~isempty(obj.occupancy_map)
        fwrite(file_id, obj.occupancy_map_n_voxels, 'uint32');
        fwrite(file_id, obj.occupancy_map_n_orient, 'uint32');
        for h = {'left', 'right'}
          fullmap = [obj.occupancy_map.(h{1}){:}];
          for v = 1:obj.n_voxels
            for o = 1:obj.occupancy_map_n_orient
              colliding_voxels =  find(fullmap(:, v + obj.n_voxels * (o-1)));
              fwrite(file_id, numel(colliding_voxels), 'uint32');
              fwrite(file_id, colliding_voxels, 'uint32');
            end
          end
        end
%         for h = {'left', 'right'}
%           for v = 1:obj.occupancy_map_n_voxels
%             for i = 1:obj.occupancy_map_n_orient
%               coll = activeVoxelsIdx(obj.occupancy_map.(h{1}){i}(v,activeVoxels));
%               ncoll = length(coll);
%               collmap(v, i, 1:ncoll) = coll;
%             end
%           end
%           for i = 1:size(collmap, 1)
%             fwrite(file_id, nnz(sum(squeeze(collmap(i,:,:))~=0,2)), 'uint32');
%             for j = 1:size(collmap, 2)
%               ncoll = nnz(collmap(i,j,:));
%               if ncoll > 0
%                 fwrite(file_id, ncoll, 'uint32');
%                 fwrite(file_id, j, 'uint32');
%                 fwrite(file_id, collmap(i,j,1:ncoll), 'uint32');
%               end
%             end
%           end
%         end
        
%         for i = 1:numel(obj.occupancy_map.left)
%           fwrite(file_id, nnz(obj.occupancy_map.left{i}), 'uint32');
%           [r, c] = find(obj.occupancy_map.left{i});
%           fwrite(file_id, [r-1, c-1], 'uint32');
%         end
%         for i = 1:numel(obj.occupancy_map.right)
%           fwrite(file_id, nnz(obj.occupancy_map.right{i}), 'uint32');
%           [r, c] = find(obj.occupancy_map.right{i});
%           fwrite(file_id, [r-1, c-1], 'uint32');
%         end
        fwrite(file_id, obj.occupancy_map_resolution, 'double');
        fwrite(file_id, obj.occupancy_map_dimensions, 'uint32');
        fwrite(file_id, obj.occupancy_map_lb, 'double');
        fwrite(file_id, obj.occupancy_map_ub, 'double');
        fwrite(file_id, size(obj.occupancy_map_orient_steps.roll, 2), 'uint32');
        fwrite(file_id, obj.occupancy_map_orient_steps.roll, 'double');
        fwrite(file_id, size(obj.occupancy_map_orient_steps.pitch, 2), 'uint32');
        fwrite(file_id, obj.occupancy_map_orient_steps.pitch, 'double');
        fwrite(file_id, size(obj.occupancy_map_orient_steps.yaw, 2), 'uint32');
        fwrite(file_id, obj.occupancy_map_orient_steps.yaw, 'double');
      end
      fclose(file_id);
    end
    
    function obj = setActiveSide(obj, side)
      assert(any(strcmp(side, {'left', 'right'})))
      if ~strcmp(side, obj.active_side)
        obj.vox_centres = bsxfun(@times, obj.vox_centres, [1; -1; 1]);
      end
      obj.active_side = side;
      obj = obj.resetActiveVoxels();
      obj = obj.resetActiveOrient();
    end
    
    function obj = activateVoxels(obj, idx)
      if islogical(idx)
        obj.active_voxels = any([obj.active_voxels; reshape( idx, size(obj.active_voxels))]);
      else
        obj.active_voxels(idx) = true;
      end
      obj.n_active_voxels = nnz(obj.active_voxels);
    end
    
    function obj = deactivateVoxels(obj, idx)
      if islogical(idx)
        obj.active_voxels = all([obj.active_voxels; reshape( ~idx, size(obj.active_voxels))]);
      else
        obj.active_voxels(idx) = false;
      end
      obj.occupancy_map_active_orient(idx, :) = false(nnz(idx), obj.occupancy_map_n_orient);
      obj.n_active_voxels = nnz(obj.active_voxels);
    end
    
    function obj = activateOrient(obj, voxels, active_orients)
      if length(voxels) > 1
        if ~iscell(active_orients)
          error('active_orients must be a cell with the orientations idx to activate for each voxel')
        end
      else
        if ~iscell(active_orients)
          active_orients = {active_orients};
        end
      end
      for vox = 1:length(voxels)
        obj.occupancy_map_active_orient(voxels(vox),active_orients{vox}) = true;
      end
    end
    
    function obj = deactivateOrient(obj, voxels, inactive_orients)
      if length(voxels) > 1
        if ~iscell(inactive_orients)
          error('inactive_orients must be a cell with the orientations idx to deactivate for each voxel')
        end
      else
        if ~iscell(inactive_orients)
          inactive_orients = {inactive_orients};
        end
      end
      for vox = 1:length(voxels)
        obj.occupancy_map_active_orient(voxels(vox),inactive_orients{vox}) = false;
      end
    end
    
    function obj = setEEPose(obj, EE_pose)
      assert(length(EE_pose) == 7)
      obj.EE_pose = reshape(EE_pose, [7,1]);
    end
    
    function centres = getCentresRelativeToWorld(obj)
      centres = bsxfun(@plus, obj.EE_pose(1:3), obj.vox_centres);
    end
    
    function centres = getActiveCentresRelativeToOrigin(obj)
      centres = bsxfun(@plus, obj.getActiveVoxelCentres(), obj.EE_pose(1:3));
    end
    
    function vox_idx = findVoxelFromCoordinates(obj, coords)
      assert(~isempty(obj.EE_pose))
      assert(all(coords >= obj.map_lb + obj.EE_pose(1:3)) && all(coords <= obj.map_ub + obj.EE_pose(1:3)))
      n_voxels_per_edge = nthroot(obj.n_voxels, 3);
      sub = ceil(coords - obj.map_lb - obj.EE_pose(1:3) / n_voxels_per_edge);
      vox_idx = unique(sub2ind(n_voxels_per_edge * ones(1,3), sub(1,:), sub(2,:), sub(3,:)));
    end
    
    function points = findPointsFromDirection(obj, direction, threshold)
      directions = obj.distributePointsOnSphere(obj.n_directions_per_voxel);
      direction = reshape(direction/norm(direction), [3,1]);
      if strcmp(obj.active_side, 'right')
        direction = direction .* [1; -1; 1];
      end
      points = acos(directions'*direction) <= threshold;
%       clf
%       hold on
%       plot3(directions(1,~points), directions(2,~points), directions(3,~points), 'r.')
%       plot3(directions(1,points), directions(2,points), directions(3,points), 'g.')
%       plot3(direction(1,:), direction(2,:), direction(3,:), 'b.')
    end
    
    function idx = findVoxelsFromDirection(obj, direction, threshold, in_active_set)
      if nargin < 4, in_active_set = true; end
      points = obj.findPointsFromDirection(direction, threshold);
      if in_active_set
        idx = all([any(obj.map(:, points), 2)'; obj.active_voxels]);
      else
        idx = any(obj.map(:, points), 2)';
      end
    end
    
    function obj = deactivateVoxelsByDirection(obj, direction, threshold, reset_active)
      if nargin < 4, reset_active = false; end
      if reset_active
        obj = obj.resetActiveVoxels();
      end
      voxels_to_keep = false(size(obj.active_voxels));
      for o = 1:obj.occupancy_map_n_orient
        voxels_to_keep = any([voxels_to_keep; obj.findVoxelsFromDirection(rpy2rotmat(obj.occupancy_map_orient(:,o))' * direction, threshold, true)]);
      end
      obj = obj.deactivateVoxels(~voxels_to_keep);
    end

    function [obj, timing_vars] = reduceActiveSet(obj, reset_active,...
        point_cloud, sagittal_range, transverse_range, height_range, direction_threshold)
      if nargin < 5 || isempty(sagittal_range), sagittal_range = [-pi/3, pi/3]; end
      if nargin < 6 || isempty(transverse_range), transverse_range = [-pi/3, pi/3]; end
      if nargin < 7 || isempty(height_range), height_range = [0.6, 1.1]; end
      if nargin < 8, direction_threshold = pi/6; end
      
      cm_angle_timer = tic();
      obj = obj.deactivateVoxelsOutsideTransverseRange(transverse_range, reset_active);
      obj = obj.deactivateVoxelsOutsideSagittalRange(sagittal_range);
      timing_vars.cm_angle_time = toc(cm_angle_timer);
      cm_base_hight_timer = tic();
      obj = obj.deactivateVoxelsOutsideBaseHeightRange(height_range);
      timing_vars.cm_base_hight_time = toc(cm_base_hight_timer);
      cm_direction_timer = tic();
      direction = quat2rotmat(obj.EE_pose(4:7)) * obj.end_effector_axis;
      obj = obj.deactivateVoxelsByDirection(direction, direction_threshold);
      timing_vars.cm_direction_time = toc(cm_direction_timer);
      cm_collision_timer = tic();
      
%       collidingTimer = tic;
      obj = obj.deactivateCollidingVoxels(point_cloud);
      timing_vars.cm_collision_time = toc(cm_collision_timer);
%       fprintf('Colliding Time: %.2f s\n', toc(collidingTimer))
      
%       if obj.n_active_voxels > des_vox_num
%         max_threshold = pi;
%         min_threshold = 0;
%         threshold_range = pi/50;
%         while max_threshold - min_threshold > threshold_range
%           mid_threshold = (max_threshold + min_threshold)/2;
%           idx_mid = obj.findVoxelsFromDirection(direction, mid_threshold, true);
%           if length(idx_mid) > des_vox_num
%             max_threshold = mid_threshold;
%           else
%             min_threshold = mid_threshold;
%           end
%         end
%         idx_min = obj.findVoxelsFromDirection(direction, min_threshold, true);
%         idx_max = obj.findVoxelsFromDirection(direction, max_threshold, true);
%         idx = min(abs([numel(idx_min), numel(idx_max)] - des_vox_num));
%         obj = obj.activateVoxels(idx);
%       end
%       
%       reachability_weight = 0;
%       while obj.n_active_voxels > des_vox_num
%         reachability_weight = reachability_weight + 0.5;
%         obj = obj.prune(sagittal_angle, transverse_angle, sagittal_weight, transverse_weight, reachability_weight, false);
%       end
    end
    
    function drawCapabilityMap(obj, orient, text, draw_cubes)
      if nargin < 2, orient = []; end
      if nargin < 3 || isempty(text), text = 'Capability Map'; end
      if nargin < 4 || isempty(draw_cubes), draw_cubes = true; end
      obj.drawMap(true(1, obj.n_voxels), text, orient, [], draw_cubes);
    end
    
    function drawCapabilityMapCentredOnPoint(obj, offset, orient, text, draw_cubes)
      if nargin < 3, orient = []; end
      if nargin < 4 || isempty(text), text = 'Capability Map'; end
      if nargin < 5 || isempty(draw_cubes), draw_cubes = true; end
      obj.drawMap(true(1, obj.n_voxels), text, orient, offset, draw_cubes)
    end
    
    function drawActiveMap(obj, orient, text, draw_cubes)
      if nargin < 2, orient = []; end
      if nargin < 3 || isempty(text), text = 'Active Capability Map'; end
      if nargin < 4 || isempty(draw_cubes), draw_cubes = true; end
      if length(orient) > 1
        voxels = any(obj.occupancy_map_active_orient(:, orient)');
      else
        voxels = obj.occupancy_map_active_orient(:, orient)';
      end
      obj.drawMap(voxels, text, orient, [], draw_cubes);
    end
    
    function drawActiveMapCentredOnPoint(obj, offset, orient, text, draw_cubes)
      if nargin < 3, orient = []; end
      if nargin < 4 || isempty(text), text = 'Active Capability Map'; end
      if nargin < 5 || isempty(draw_cubes), draw_cubes = true; end
      if length(orient) > 1
        voxels = any(obj.occupancy_map_active_orient(:, orient)');
      else
        voxels = obj.occupancy_map_active_orient(:, orient)';
      end
      obj.drawMap(voxels, text, orient, offset, draw_cubes)
    end
    
    function drawMap(obj, voxels, text, orient, offset, draw_cubes)
      lcmClient = LCMGLClient(text);
      if nargin < 4 || isempty(orient), [~,orient] = ismember([0 0 0], obj.occupancy_map_orient', 'rows'); end
      if nargin < 5 || isempty(offset), offset = [0;0;0]; end
      if nargin < 6, draw_cubes = true; end
      if length(orient) > 1, draw_cubes = false; end
      if draw_cubes
        lcmClient = obj.drawMapCubes(lcmClient, obj.map_lb, obj.map_ub, obj.vox_edge, offset, rpy2rotmat(obj.occupancy_map_orient(:,orient)));
        start_idx = 0;
      else
        start_idx = 1;
      end
      if ischar(orient) && strcmp(orient, 'all')
        orient = 1:obj.occupancy_map_n_orient;
      end
      for o = orient
        for i = start_idx:obj.n_directions_per_voxel
          h = 1-(i/obj.n_directions_per_voxel*2/3);
          rgb = hsv2rgb(h, 1, 1);
          lcmClient.glColor3f(rgb(1), rgb(2), rgb(3));
          if i == 0
            lcmClient.glPointSize(1);
          else
            lcmClient.glPointSize(10);
          end
          coords = obj.vox_centres(:, all([(obj.reachability_index == i/obj.n_directions_per_voxel);...
                                            voxels; obj.occupancy_map_active_orient(:,o)']));
          coords = rpy2rotmat(obj.occupancy_map_orient(:,o)) * coords;
          if nargin > 3
            coords = bsxfun(@plus, offset(1:3), coords);
          end
          if ~isempty(coords)
            lcmClient.points(coords(1,:), coords(2,:), coords(3,:));
          end
        end
      end
      lcmClient.switchBuffers();
    end
    
    function drawOccupancyMap(obj, voxel, orient, offset, text, draw_cubes, draw_base)
      if nargin < 3 || isempty(orient), [~,orient] = ismember([0 0 0], obj.occupancy_map_orient', 'rows'); end
      if nargin < 4 || isempty(offset), offset = [0;0;0]; end
      if nargin < 5 || isempty(text), text = 'Occupancy Map'; end
      if nargin < 6 || isempty(draw_cubes), draw_cubes = true; end
      if nargin < 7, draw_base = false; end
      lcmClient = LCMGLClient(text);
      coords = obj.getOccupancyMapCentres();
      if any(offset ~= 0)
        coords = bsxfun(@plus, offset(1:3), coords);
      end
      
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
        pos = rpy2rotmat(obj.occupancy_map_orient(:,orient)) * (obj.vox_centres(:,voxel)-obj.map_centre.(obj.active_side));
        if any(offset ~= 0)
          pos = pos + offset(1:3);
        end
        v.draw(0, [pos;obj.occupancy_map_orient(:,orient)])
        lcmClient.glColor3f(0,1,0)
        lcmClient.glPointSize(10)
        centre = rpy2rotmat(obj.occupancy_map_orient(:,orient)) * obj.vox_centres(:,voxel);
        lcmClient.points(centre(1), centre(2), centre(3))
      end
      
      if draw_cubes
        lcmClient = obj.drawMapCubes(lcmClient, obj.occupancy_map_lb, obj.occupancy_map_ub, obj.occupancy_map_resolution, offset, eye(3));
      end
      colliding_points = coords(:,obj.occupancy_map.(obj.active_side){orient}(:, voxel));
      free_points = coords(:,~obj.occupancy_map.(obj.active_side){orient}(:, voxel));
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
      if nargin < 3, reset_active = false; end
      
      if isempty(obj.EE_pose)
        error('End effector pose must be set in order to detect colliding voxels')
      end
      
      if reset_active
        obj = obj.resetActiveVoxels();
        obj = obj.resetActiveOrient();
      end
      
      point_cloud = point_cloud(:,all([all(bsxfun(@gt, point_cloud,  obj.occupancy_map_lb + obj.EE_pose(1:3))); ...
                                       all(bsxfun(@lt, point_cloud,  obj.occupancy_map_ub + obj.EE_pose(1:3)))]));
      sub = ceil(bsxfun(@rdivide, bsxfun(@minus, point_cloud, obj.occupancy_map_lb + obj.EE_pose(1:3)), obj.occupancy_map_resolution));
      vox_idx = unique(sub2ind(obj.occupancy_map_dimensions, sub(1,:), sub(2,:), sub(3,:)));
      for orient = 1:obj.occupancy_map_n_orient
%           obj.drawActiveMapCentredOnPoint(obj.EE_pose(1:3), orient, [], 0)
          obj.occupancy_map_active_orient(:, orient) = all([~any(obj.occupancy_map.(obj.active_side){orient}(vox_idx,:)); ...
                                                            obj.occupancy_map_active_orient(:, orient)']);
%           obj.drawActiveMapCentredOnPoint(obj.EE_pose(1:3), orient, [], 0)
      end
      obj = obj.deactivateVoxels(all(~obj.occupancy_map_active_orient, 2));
    end
    
    function obj = deactivateVoxelsOutsideBaseHeightRange(obj, range, reset_active)
      if isempty(obj.EE_pose)
        error('End effector pose must be set in order to deactivate voxels')
      end
      if nargin < 3, reset_active = false; end
      if reset_active
        obj = obj.resetActiveVoxels();
      end
      for o = 1:obj.occupancy_map_n_orient
        h = rpy2rotmat(obj.occupancy_map_orient(:,o)) * bsxfun(@plus, obj.vox_centres, obj.EE_pose(1:3) - obj.map_centre.(obj.active_side));
        obj.occupancy_map_active_orient(:,o) =  all([h(3,:) > range(1); h(3, :) < range(2); obj.active_voxels]);
      end
      voxels = all(~obj.occupancy_map_active_orient, 2);
      obj = obj.deactivateVoxels(voxels);
      obj.occupancy_map_active_orient(voxels, :) = false(nnz(voxels), obj.occupancy_map_n_orient);
    end
    
    function obj = deactivateVoxelsOutsideSagittalRange(obj, range, reset_active)
      if nargin < 3, reset_active = false; end
      if reset_active
        obj = obj.resetActiveVoxels();
      end
      angle = atan2(obj.vox_centres(3,:), obj.vox_centres(1,:));
      angle = angle - sign(angle) * pi;
      voxels = any([angle < range(1) ; angle > range(2)]);
      obj = obj.deactivateVoxels(voxels);
      obj.occupancy_map_active_orient(voxels, :) = false(nnz(voxels), obj.occupancy_map_n_orient);
    end
    
    function obj = deactivateVoxelsOutsideTransverseRange(obj, range, reset_active)
      if nargin < 3, reset_active = false; end
      if reset_active
        obj = obj.resetActiveVoxels();
      end
      angle = atan2(obj.vox_centres(2,:), obj.vox_centres(1,:));
      angle = angle - sign(angle) * pi;
      voxels = any([angle < range(1) ; angle > range(2)]);
      obj = obj.deactivateVoxels(voxels);
      obj.occupancy_map_active_orient(voxels, :) = false(nnz(voxels), obj.occupancy_map_n_orient);
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
    
    function obj = resetActiveOrient(obj, include_zero_reachability)
      if nargin < 2
        include_zero_reachability = false;
      end
      obj.occupancy_map_active_orient = true(obj.n_voxels, obj.occupancy_map_n_orient);
      if ~include_zero_reachability
        obj.occupancy_map_active_orient(obj.reachability_index == 0, :) = false(nnz(obj.reachability_index == 0), obj.occupancy_map_n_orient);
      end
    end
    
    function centres = getActiveVoxelCentres(obj)
      centres = obj.vox_centres(:, obj.active_voxels);
    end
    
    function obj = computeVoxelsCentres(obj)
      [y,x,z] = meshgrid(obj.map_lb(2) + obj.vox_edge/2:obj.vox_edge:obj.map_ub(2), ...
                         obj.map_lb(1) + obj.vox_edge/2:obj.vox_edge:obj.map_ub(1), ...
                         obj.map_lb(3) + obj.vox_edge/2:obj.vox_edge:obj.map_ub(3));
      obj.vox_centres = [reshape(x, 1, obj.n_voxels); ...
                         reshape(y, 1, obj.n_voxels); ...
                         reshape(z, 1, obj.n_voxels)];
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
      distance = norm(obj.map_centre.left-end_effector_position);
      
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
      mc = obj.map_centre.left;
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
        disp('Not using parallel toolbox, computation might take very long!')
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
      
      obj.occupancy_map_n_orient = length(roll_steps)*length(pitch_steps)*length(yaw_steps);
      obj.occupancy_map_orient_steps = struct('roll', roll_steps, 'pitch', pitch_steps, 'yaw', yaw_steps);
      obj = obj.computeOccupancyMapOrientations();
      obj.occupancy_map_resolution = resolution;
      [obj.occupancy_map_lb, obj.occupancy_map_ub] = obj.getOccupancyMapBounds(base.body(2));
      obj.occupancy_map_dimensions = ceil((obj.occupancy_map_ub - obj.occupancy_map_lb)/obj.occupancy_map_resolution)';
      obj.occupancy_map_ub = obj.occupancy_map_lb + obj.occupancy_map_dimensions' * obj.occupancy_map_resolution;
      obj.occupancy_map_n_voxels = prod(obj.occupancy_map_dimensions);
      obj.occupancy_map_active_orient = false(obj.n_voxels, obj.occupancy_map_n_orient);
      
      
      %Compute map
      idx_to_check = find(obj.reachability_index~=0);
      omnv = obj.occupancy_map_n_voxels;
      nv = obj.n_voxels;
      mc.left = obj.map_centre.left;
      mc.right = obj.map_centre.right;
      omc = obj.getOccupancyMapCentres();
      v = ver;
      omno = obj.occupancy_map_n_orient;
      om = cell(1,obj.occupancy_map_n_orient);
      o = obj.occupancy_map_orient;
      for side = {'left', 'right'}
        obj = obj.setActiveSide(side{1});
        voxels_to_check = obj.vox_centres(:,idx_to_check);
        if use_parallel_toolbox && any(strcmp({v.Name}, 'Parallel Computing Toolbox'))
          parfor orient = 1:obj.occupancy_map_n_orient
            fprintf('Computing map %d of %d for %s side\n', orient, omno, side{1});
            single_om = sparse(false(omnv, nv));
            base = RigidBodyManipulator();
            base = base.addRobotFromURDFString(urdf_string, [], [], struct('floating', true));
            for i = 1:length(idx_to_check)
              q = [rpy2rotmat(o(:,orient)) * (voxels_to_check(:,i)- mc.(side{1})); o(:,orient)];
              kinsol = base.doKinematics(q);
              colliding_points = base.collidingPoints(kinsol, omc, resolution/2);
              if ~isempty(colliding_points)
                single_om(colliding_points, idx_to_check(i)) = ~single_om(colliding_points, idx_to_check(i));
              end
            end
            om{orient} = sparse(single_om);
            single_om = [];
          end
        else
          disp('Not using parallel toolbox, computation might take very long!')
          for orient = 1:obj.occupancy_map_n_orient
            fprintf('Computing map %d of %d for %s side\n', orient, omno, side{1});
            single_om = sparse(false(omnv, nv));
            base = RigidBodyManipulator();
            base = base.addRobotFromURDFString(urdf_string, [], [], struct('floating', true));
            for i = 1:length(idx_to_check)
              q = [rpy2rotmat(o(:,orient)) * (voxels_to_check(:,i)- mc.(side{1})); o(:,orient)];
              kinsol = base.doKinematics(q);
              colliding_points = base.collidingPoints(kinsol, omc, resolution/2);
              if ~isempty(colliding_points)
                single_om(colliding_points, idx_to_check(i)) = ~single_om(colliding_points, idx_to_check(i));
              end
            end
            om{orient} = sparse(single_om);
          end
        end
        obj.occupancy_map.(side{1}) = om;
      end
      obj = obj.resetActiveOrient();
      obj = obj.computeOrientationProbabilityDistribution();
    end
    
    function [lb, ub] = getOccupancyMapBounds(obj, body)
      norms = arrayfun(@(idx) norm(obj.vox_centres(:, idx)), 1:obj.n_voxels);
      [~, furthest_voxel] = max(norms.*obj.active_voxels);
      body_BB = bsxfun(@minus, body.collision_geometry{1}.getBoundingBoxPoints(), obj.map_centre.left - obj.vox_centres(:, furthest_voxel));
      max_dist = max(arrayfun(@(idx) norm(body_BB(:, idx)), 1:8));
      lb = -max_dist * ones(3,1);
      ub = max_dist * ones(3,1);
    end
    
    function centres = getOccupancyMapCentres(obj)
      [y,x,z] = meshgrid(obj.occupancy_map_lb(2) + obj.occupancy_map_resolution/2:obj.occupancy_map_resolution:obj.occupancy_map_ub(2), ...
                         obj.occupancy_map_lb(1) + obj.occupancy_map_resolution/2:obj.occupancy_map_resolution:obj.occupancy_map_ub(1), ...
                         obj.occupancy_map_lb(3) + obj.occupancy_map_resolution/2:obj.occupancy_map_resolution:obj.occupancy_map_ub(3));
      centres = [reshape(x, 1, obj.occupancy_map_n_voxels); ...
                 reshape(y, 1, obj.occupancy_map_n_voxels); ...
                 reshape(z, 1, obj.occupancy_map_n_voxels)];
    end
    
    function obj = computeOrientationProbabilityDistribution(obj, sigma, mu)
      if nargin < 3, mu = [0 0 0]; end
      if nargin < 2 || isempty(sigma), sigma = [5 0.5 10]; end
      f_id = fopen('/home/marco/oh-distro/software/planning/capabilityMapMatlab.log', 'w');
%       x = bsxfun(@rdivide, obj.occupancy_map_orient', [max(abs(obj.occupancy_map_orient_steps.roll)),...
%                                         max(abs(obj.occupancy_map_orient_steps.pitch)),...
%                                         max(abs(obj.occupancy_map_orient_steps.yaw))]);
      obj.occupancy_map_orient_prob = mvnpdf(obj.occupancy_map_orient', mu, sigma);
      obj.occupancy_map_orient_prob = obj.occupancy_map_orient_prob / sum(obj.occupancy_map_orient_prob);
      fprintf(f_id, '%.30g\n', obj.occupancy_map_orient_prob);
      fclose(f_id);
    end
    
    function obj = computePositionProbabilityDistribution(obj, sigma, mu)
      if nargin < 3, mu = [0 0 0]; end
      if nargin < 2 || isempty(sigma), sigma = [1e10 1e10 0.01]; end
      f_id = fopen('/home/marco/oh-distro/software/planning/capabilityMapMatlab.log', 'a');
      fprintf(f_id, '\n\n');
      x = bsxfun(@rdivide, obj.vox_centres, obj.map_ub);
      obj.vox_centres_prob = mvnpdf(x', mu, sigma);
      obj.vox_centres_prob = obj.vox_centres_prob / sum(obj.vox_centres_prob);
      fprintf(f_id, '%.30g\n', obj.vox_centres_prob);
      fclose(f_id);
    end
    
    function obj = computeOccupancyMapOrientations(obj)
      [r,p,y] = ind2sub([numel(obj.occupancy_map_orient_steps.roll),...
                           numel(obj.occupancy_map_orient_steps.pitch),...
                           numel(obj.occupancy_map_orient_steps.yaw)],...
                           1:obj.occupancy_map_n_orient);
      r = obj.occupancy_map_orient_steps.roll(r);
      p = obj.occupancy_map_orient_steps.pitch(p);
      y = obj.occupancy_map_orient_steps.yaw(y);
      obj.occupancy_map_orient = [r; p; y];
    end
        
    
  end
    
  methods (Static)
    
    function [P, frames] = distributePointsOnSphere(N, compute_frames)
      if nargin < 2, compute_frames = false; end
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
      if compute_frames
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
            worker_map(vox_ind, point) = true;
          end
        end
      end
      worker_map = flipud(worker_map);
%       sprintf(['counter: %.1d\n' ...
%               'setup: %.1d\n' ...
%               'constraints: %.1d\n'...
%               'ik: %.1d\n'...
%               't per sample: %.1d\n'...
%               'tot time: %.1d h\n'], mean(counter_times), mean(setup_times), mean(constraint_times), mean(ik_times),...
%               sum([mean(counter_times), mean(setup_times), 50*mean(constraint_times), 50*mean(ik_times)]),...
%               sum([mean(counter_times), mean(setup_times), 50*mean(constraint_times), 50*mean(ik_times)])*1e6/3600)
    end
    
    function lcmClient = drawMapCubes(lcmClient, lb, ub, resolution, centre, rotmat)
      centre = centre(1:3);
      dimensions = (ub - lb)/resolution;
      lcmClient.glColor3f(0.3,0.3,0.3);
      lcmClient.glLineWidth(.1);
      [p1y, p1z, p1x] = meshgrid(lb(2):resolution:ub(2), lb(3):resolution:ub(3), [lb(1), ub(1)]);
      [p2z, p2x, p2y] = meshgrid(lb(3):resolution:ub(3), lb(1):resolution:ub(1), [lb(2), ub(2)]);
      [p3x, p3y, p3z] = meshgrid(lb(1):resolution:ub(1), lb(2):resolution:ub(2), [lb(3), ub(3)]);
      p1x = reshape(p1x, [1, (dimensions(2)+1)*(dimensions(3)+1)*2]);
      p1y = reshape(p1y, [1, (dimensions(2)+1)*(dimensions(3)+1)*2]);
      p1z = reshape(p1z, [1, (dimensions(2)+1)*(dimensions(3)+1)*2]);
      p2x = reshape(p2x, [1, (dimensions(1)+1)*(dimensions(3)+1)*2]);
      p2y = reshape(p2y, [1, (dimensions(1)+1)*(dimensions(3)+1)*2]);
      p2z = reshape(p2z, [1, (dimensions(1)+1)*(dimensions(3)+1)*2]);
      p3x = reshape(p3x, [1, (dimensions(1)+1)*(dimensions(2)+1)*2]);
      p3y = reshape(p3y, [1, (dimensions(1)+1)*(dimensions(2)+1)*2]);
      p3z = reshape(p3z, [1, (dimensions(1)+1)*(dimensions(2)+1)*2]);
      p1 = rotmat * [p1x;p1y;p1z];
      p2 = rotmat * [p2x;p2y;p2z];
      p3 = rotmat * [p3x;p3y;p3z];
      for i = 1:length(p1)/2
        lcmClient.line3(p1(1, i) + centre(1),...
                        p1(2, i) + centre(2),...
                        p1(3, i) + centre(3),...
                        p1(1, i + length(p1)/2) + centre(1),...
                        p1(2, i + length(p1)/2) + centre(2),...
                        p1(3, i + length(p1)/2) + centre(3))
      end
      for i = 1:length(p2)/2
        lcmClient.line3(p2(1, i) + centre(1),...
                        p2(2, i) + centre(2),...
                        p2(3, i) + centre(3),...
                        p2(1, i + length(p2)/2) + centre(1),...
                        p2(2, i + length(p2)/2) + centre(2),...
                        p2(3, i + length(p2)/2) + centre(3))
      end
      for i = 1:length(p3)/2
        lcmClient.line3(p3(1, i) + centre(1),...
                        p3(2, i) + centre(2),...
                        p3(3, i) + centre(3),...
                        p3(1, i + length(p3)/2) + centre(1),...
                        p3(2, i + length(p3)/2) + centre(2),...
                        p3(3, i + length(p3)/2) + centre(3))
      end
      lcmClient.glLineWidth(4);
      lcmClient.glColor3f(1,0,0);
      centre2 = bsxfun(@plus, rotmat * eye(3) * 0.2, centre);
      lcmClient.line3(centre(1), centre(2), centre(3),...
                      centre2(1, 1), centre2(2, 1), centre2(3, 1))
      lcmClient.glColor3f(0,1,0);
      lcmClient.line3(centre(1), centre(2), centre(3),...
                      centre2(1, 2), centre2(2, 2), centre2(3, 2))
      lcmClient.glColor3f(0,0,1);
      lcmClient.line3(centre(1), centre(2), centre(3),...
                      centre2(1, 3), centre2(2, 3), centre2(3, 3))
    end
    
  end
  
end