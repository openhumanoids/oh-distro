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
    
    function obj = activateSpheresFromDirection(obj, direction, threshold, reset_active)
      
      if reset_active
        obj = obj.resetActiveSpheres();
      end      
      
      points = obj.findPointsFromDirection(direction, threshold);
      for s = 1:obj.n_spheres
        if obj.active_spheres(s) && all(~obj.map(s, points))
          obj.active_spheres(s) = false;
          obj.n_active_spheres = nnz(obj.active_spheres);
        end
      end  
      
    end

    function obj = reduceActiveSet(obj, direction, min_sph, max_sph, reset_active, sagittal_angle,...
        transverse_angle, sagittal_weight, transverse_weight)
      
      if nargin > 5
        obj = obj.prune(sagittal_angle, transverse_angle, sagittal_weight, transverse_weight, 0, reset_active);
      end
      
      assert(min_sph < obj.n_active_spheres)
      active_sph = obj.active_spheres;
      threshold = 0;
      obj = activateSpheresFromDirection(obj, direction, threshold, false);
      while obj.n_active_spheres < min_sph
        obj.active_spheres = active_sph;
        threshold = threshold + pi/50;
        obj = activateSpheresFromDirection(obj, direction, threshold, false);
      end
      reachability_weight = 0;
      while obj.n_active_spheres > max_sph
        reachability_weight = reachability_weight + 0.5;
        obj = obj.prune(sagittal_angle, transverse_angle, sagittal_weight, transverse_weight, reachability_weight, false);
      end
    end
    
    function drawActiveSpheres(obj, direction, min_sph, max_sph)
      lcmClient = LCMGLClient('CapabilityMap');
%       obj = obj.reduceActiveSet(direction, min_sph, max_sph, true, 0, 0, 2, 1.5);
      for sph = 1:obj.n_spheres
        if obj.active_spheres(sph)
          lcmClient.sphere(obj.sph_centers(:,sph), obj.sph_diameter/2, 20, 20);
        end
      end
      disp(obj.n_active_spheres)
      lcmClient.switchBuffers();
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
            obj.active_spheres(sph) = false;
            obj.n_active_spheres = nnz(obj.active_spheres);
          end
        end
      end
    end
    
    function obj = resetActiveSpheres(obj)
      obj.active_spheres = true(obj.n_spheres, 1);
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