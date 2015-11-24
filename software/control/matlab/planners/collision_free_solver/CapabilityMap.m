classdef CapabilityMap
  
  properties
    map
    reachability_index
    sph_centers
    sph_diameter
    n_samples
    ang_tolerance
    pos_tolerance
    urdf
    n_spheres
    n_points_per_sphere
    root_link
    root_point
    end_effector_link
    end_effector_point
    base_link
    active_spheres
    n_active_spheres
  end
  
  methods
    
    function obj = CapabilityMap(mat_file)
      if nargin > 0
        obj = obj.generateFromFile(mat_file);
      end
    end
    
    function obj = generateFromFile(obj, matFile)
      vars = load(matFile);
      obj.map = vars.map;
      obj.reachability_index = vars.reachability_index;
      obj.sph_centers = vars.sph_centers;
      obj.sph_diameter = vars.options.sph_diameter;
      obj.n_samples = vars.options.n_samples;
      obj.ang_tolerance = vars.options.ang_tolerance;
      obj.pos_tolerance = vars.options.pos_tolerance;
      obj.urdf = vars.options.urdf;
      obj.n_spheres = size(obj.map, 1);
      obj.n_points_per_sphere = size(obj.map, 2);
      obj.root_link = vars.options.root_link;
      obj.root_point = vars.options.root_point;
      obj.end_effector_link = vars.options.end_effector_link;
      obj.end_effector_point = vars.options.end_effector_point;
      obj.base_link = vars.options.base_link;
      obj.active_spheres = true(obj.n_spheres, 1);
      obj.n_active_spheres = obj.n_spheres;
      
      obj = obj.resetActiveSpheres();
    end
    
    function obj = activateSpheres(obj, idx)
      obj.active_spheres(idx) = true;
      obj.n_active_spheres = nnz(obj.active_spheres);
    end
    
    function obj = deactivateSpheres(obj, idx)
      obj.active_spheres(idx) = false;
      obj.n_active_spheres = nnz(obj.active_spheres);
    end
    
    function points = findPointsFromDirection(obj, direction, threshold)
      [~, frames] = obj.distributePointsOnSphere(obj.n_points_per_sphere);
      points = false(obj.n_points_per_sphere, 1);
%       sphere();
%       hold on
%       plot3(P(1,:), P(2,:), P(3,:), 'r.')
      for p = 1:obj.n_points_per_sphere
        if acos(frames(p,:,3)*direction)/norm(direction) <= threshold
          points(p) = true;
%           plot3([P(1,p), P(1,p) - frames(p, 1, 3)'], [P(2,p), P(2,p) - frames(p, 2, 3)'], [P(3,p), P(3,p) - frames(p, 3, 3)'], 'b')
        end
      end
    end
    
    function idx = findSpheresFromDirection(obj, direction, threshold, in_active_set)
      
      if in_active_set
        active_idx = find(obj.active_spheres);
      else
        active_idx = 1:obj.n_spheres;
      end
      idx = [];
      points = obj.findPointsFromDirection(direction, threshold);
      for s = active_idx'
        if any(obj.map(s, points))
          idx(end+1) = s;
        end
      end  
      
    end

    function obj = reduceActiveSet(obj, direction, des_sph_num, reset_active, rbm, EE_pose, sagittal_angle,...
        transverse_angle, sagittal_weight, transverse_weight)
      
      obj = obj.deactivateCollidingSpheres(rbm, EE_pose, reset_active);
      
%       if nargin > 7
%         obj = obj.prune(sagittal_angle, transverse_angle, sagittal_weight, transverse_weight, 0, false);
%       end
      
      if obj.n_active_spheres > des_sph_num
        max_threshold = pi;
        min_threshold = 0;
        threshold_range = pi/50;
        while max_threshold - min_threshold > threshold_range
          mid_threshold = (max_threshold + min_threshold)/2;
          idx_mid = obj.findSpheresFromDirection(direction, mid_threshold, true);
          if length(idx_mid) > des_sph_num
            max_threshold = mid_threshold;
          else
            min_threshold = mid_threshold;
          end
        end
        idx_min = obj.findSpheresFromDirection(direction, min_threshold, true);
        idx_max = obj.findSpheresFromDirection(direction, max_threshold, true);
        idx = min(abs([numel(idx_min), numel(idx_max)] - des_sph_num));
        obj = obj.activateSpheres(idx);
%         obj.drawActiveMapCentredOnEE(EE_pose)
      end
      
      reachability_weight = 0;
      while obj.n_active_spheres > des_sph_num
        reachability_weight = reachability_weight + 0.5;
        obj = obj.prune(sagittal_angle, transverse_angle, sagittal_weight, transverse_weight, reachability_weight, false);
      end
    end
    
    function drawActiveSpheres(obj)
      lcmClient = LCMGLClient('CapabilityMap');
      for sph = 1:obj.n_spheres
        if obj.active_spheres(sph)
          lcmClient.sphere(obj.sph_centers(:,sph), obj.sph_diameter/2, 20, 20);
        end
      end
      lcmClient.switchBuffers();
    end
    
    function drawActiveMapCentredOnEE(obj, EE_pose)
      lcmClient = LCMGLClient('CapabilityMap');
      for sph = 1:obj.n_spheres
        if obj.active_spheres(sph)
          lcmClient.sphere(EE_pose(1:3)-obj.sph_centers(:,sph), obj.sph_diameter/10, 20, 20);
        end
      end
      lcmClient.switchBuffers();
    end
    
    function obj = deactivateCollidingSpheres(obj, rbm, EE_pose, reset_active)
      
      if reset_active
        obj = obj.resetActiveSpheres();
      end
      
      points = bsxfun(@minus, EE_pose(1:3), obj.getActiveSphereCentres());
      sph_idx = find(obj.active_spheres);
      kinsol = rbm.doKinematics(zeros(rbm.num_positions, 1));
      colliding_points = rbm.collidingPoints(kinsol, points, obj.sph_diameter/2);
      obj = obj.deactivateSpheres(sph_idx(colliding_points));
    end
    
    function obj = prune(obj, sagittal_angle,...
        transverse_angle, sagittal_weight, transverse_weight, reachability_weight, reset_active)
      
      if reset_active
        obj = obj.resetActiveSpheres();
      end
      
      Dmax = max(obj.reachability_index);
      
      for sph = 1:obj.n_spheres
        if obj.active_spheres(sph)
          sa = atan2(obj.sph_centers(3,sph), obj.sph_centers(1,sph));
          ta = atan2(obj.sph_centers(2,sph), obj.sph_centers(1,sph));
          sagittal_cost = sagittal_weight * abs(sa - sagittal_angle);
          transverse_cost = transverse_weight * abs(ta - transverse_angle);
          reachability_cost = reachability_weight * (Dmax - obj.reachability_index(sph));
          if sqrt(sagittal_cost^2 + transverse_cost^2) + reachability_cost >= 2
            obj = obj.deactivateSpheres(sph);
          end
        end
      end
    end
    
    function obj = resetActiveSpheres(obj, include_zero_reachability)
      if nargin < 2
        include_zero_reachability = false;
      end
      obj = obj.activateSpheres(1:obj.n_spheres);
      if ~include_zero_reachability
        obj = obj.deactivateSpheres(obj.reachability_index == 0);
      end
    end
    
    function centres = getActiveSphereCentres(obj)
      centres = obj.sph_centers(:, obj.active_spheres);
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
    
  end
  
end