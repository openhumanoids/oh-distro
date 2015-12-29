classdef FinalPoseProblem
  
  properties
    robot
    end_effector_id
    q_start
    x_goal
    additional_constraints
    goal_constraints
    q_nom
    end_effector_point
    min_distance
    capability_map
    ikoptions
    grasping_hand
    active_collision_options
    debug
    verbose
  end
  
  properties (Constant)
    SUCCESS = 1
    FAIL_NO_FINAL_POSE = 12
  end  
  
  methods
    
    function obj = FinalPoseProblem(robot, end_effector_id,...
         q_start, x_goal, additional_constraints, q_nom, capabilty_map, ikoptions, varargin)
      
      opt.graspinghand =  'right';
      opt.mindistance = 0.005;
      opt.activecollisionoptions = struct();
      opt.endeffectorpoint = [0; 0; 0];
      opt.debug = false;
      opt.verbose = true;
      
      optNames = fieldnames(opt);
      nArgs = length(varargin);
      if round(nArgs/2)~=nArgs/2
        error('Needs propertyName/propertyValue pairs')
      end
      for pair = reshape(varargin,2,[])
        inpName = lower(pair{1});
        if any(strcmp(inpName,optNames))
          opt.(inpName) = pair{2};
        else
          error('%s is not a recognized parameter name',inpName)
        end
      end
      
      obj.robot = robot;
      obj.end_effector_id = end_effector_id;
      obj.q_start = q_start;
      obj.x_goal = x_goal;
      obj.additional_constraints = additional_constraints;
      obj.q_nom = q_nom;
      obj.capability_map = capabilty_map;
      obj.ikoptions = ikoptions;
      obj.end_effector_point = opt.endeffectorpoint;
      obj.min_distance = opt.mindistance;
      obj.grasping_hand = opt.graspinghand;
      obj.active_collision_options = opt.activecollisionoptions;
      obj.goal_constraints = obj.generateGoalConstraints();
      obj.debug = opt.debug;
      obj.verbose = opt.verbose;

    end

    function [x_goal, info, debug_vars] = findFinalPose(obj, point_cloud)
      
      info = obj.SUCCESS;
      if obj.debug
        debug_vars = obj.initDebugVars();
      else
        debug_vars = struct();
      end
      
      %compute final pose
      if obj.verbose || obj.debug
        FPP_timer = tic();
      end
      if length(obj.x_goal) == 7
        if obj.verbose, disp('Searching for a feasible final configuration...'); end
        [qGoal, debug_vars] = obj.searchFinalPose(point_cloud, debug_vars);
        if isempty(qGoal)
          info = obj.FAIL_NO_FINAL_POSE;
          x_goal = obj.x_goal;
          if obj.verbose, disp('Failed to find a feasible final configuration'); end
          return
        else
          kinsol = obj.robot.doKinematics(qGoal);
          obj.x_goal = obj.robot.forwardKin(kinsol, obj.end_effector_id, obj.end_effector_point, 2);
          obj.x_goal = [obj.x_goal; qGoal];
          if obj.verbose, disp('Final configuration found'); end
        end
      elseif length(obj.x_goal) == 7 + obj.robot.num_positions
        if obj.verbose, disp('Final configuration input found A'); end
      elseif length(obj.x_goal) == obj.robot.num_positions
        kinsol = obj.robot.doKinematics(obj.x_goal);
        obj.x_goal = [obj.robot.forwardKin(kinsol, obj.end_effector_id, obj.end_effector_point, 2); obj.x_goal];
        if obj.verbose, disp('Final configuration input found B'); end
      else
        if obj.verbose, error('Bad final configuration input'); end
      end
      
      x_goal = obj.x_goal;
      if obj.verbose || obj.debug
        computation_time = toc(FPP_timer);
        if obj.verbose, fprintf('Computation Time: %.2f s\n', computation_time); end
        if obj.debug
          debug_vars.info = info;
          debug_vars.computation_time = computation_time;
        end
      end
    end
    
    function [qOpt, debug_vars] = searchFinalPose(obj, point_cloud, debug_vars)
      if obj.verbose, setupTimer = tic; end
      
      options.rotation_type = 2;
      options.compute_gradients = true;
      options.rotation_type = 1;
      
      base = obj.robot.findLinkId(obj.capability_map.base_link);
      
      obj.capability_map = obj.capability_map.setEEPose(obj.x_goal);
      obj.capability_map = obj.capability_map.setActiveSide(obj.grasping_hand);
      if obj.verbose, reduceTimer = tic; end
      obj.capability_map = obj.capability_map.reduceActiveSet(true, point_cloud);
      if obj.verbose, fprintf('reduce Time: %.4f s\n', toc(reduceTimer)); end
      active_voxels = find(obj.capability_map.active_voxels);
      valid_samples = obj.capability_map.occupancy_map_active_orient(obj.capability_map.active_voxels, :);
      n_valid_samples = nnz(valid_samples);
      if obj.verbose, fprintf('n valid samples: %d\n', n_valid_samples); end
      orient_prob = obj.capability_map.occupancy_map_orient_prob;
      iter = 0;
      qOpt = [];
      cost = [];
      if obj.verbose
        fprintf('Setup time: %.2f s\n', toc(setupTimer));
        iterationTimer = tic;
      end
      for vox = 1:min([n_valid_samples, 100])
        orient_prob(~any(valid_samples)) = 0;
        cum_prob = cumsum(orient_prob) / sum(orient_prob);
        orient_idx = find(rand() < cum_prob, 1);
        rpy = obj.capability_map.occupancy_map_orient(:, orient_idx);
        voxel_idx = find(valid_samples(:, orient_idx));
        rand_voxel = randi(length(voxel_idx));
        voxel_idx = voxel_idx(rand_voxel);
        pos = rpy2rotmat(rpy) * obj.capability_map.vox_centres(:, active_voxels(voxel_idx));
        valid_samples(voxel_idx, orient_idx) = false;
        iter = iter + 1;
        if obj.verbose, fprintf('Iteration %d:', iter); end
%         dist = norm(vox_centers(:,vox));
%         axis = -vox_centers(:,vox)/dist;
%         drawTreePoints([[0;0;0] vox_centers(1:3,vox)], 'lines', true, 'text', 'cm_point')
%         drawTreePoints([obj.x_goal(1:3) - vox_centers(1:3,vox), obj.x_goal(1:3)], 'lines', true)
%         shDistance = Point2PointDistanceConstraint(obj.robot, base, obj.robot.findLinkId('world'), obj.capability_map.map_left_centre, obj.x_goal(1:3), dist, dist);
%         shGaze = WorldGazeTargetConstraint(obj.robot, base, axis, obj.x_goal(1:3), obj.capability_map.map_left_centre, 0);
%         shConstraint = WorldPositionConstraint(obj.robot, root, [0;0;0], obj.x_goal(1:3) - vox_centers(1:3,vox), obj.x_goal(1:3) - vox_centers(1:3,vox));
%         shOrient = WorldEulerConstraint(obj.robot, base, [-pi/50;-pi/20; -pi/20], [pi/50; pi/20; pi/20]);
        torsoPosConstraint = WorldPositionConstraint(obj.robot, base, obj.capability_map.map_centre.(obj.grasping_hand), pos + obj.x_goal(1:3),  pos + obj.x_goal(1:3));
        torsoEulerConstraint = WorldEulerConstraint(obj.robot, base, rpy, rpy);
        constraints = [{torsoPosConstraint, torsoEulerConstraint}, obj.goal_constraints, obj.additional_constraints];
        [q, info, infeasible_constraints] = inverseKin(obj.robot, obj.q_nom, obj.q_nom, constraints{:}, obj.ikoptions);
        valid = (info < 10);
        kinSol = obj.robot.doKinematics(q, ones(obj.robot.num_positions, 1), options);
        if valid
          valid = ~obj.robot.collidingPointsCheckOnly(kinSol, point_cloud, obj.min_distance);
          if valid
            phi = obj.robot.collisionDetect(q, false);
            if all(phi > obj.min_distance)
              qOpt = q;
              if obj.verbose, fprintf('Solution found\n'); end
              break
            else
              if obj.verbose, fprintf('Solution in collision\n'); end
            end
          else
            if obj.verbose, fprintf('Solution in collision\n'); end
%             cost = (obj.q_nom - q)'*obj.ikoptions.Q*(obj.q_nom - q);
%             validConfs(:,vox) = [cost; q];
%             if cost < 20
%               if obj.debug
%                 debug_vars.cost_below_threshold = true;
%               end
%               break
%             end
          end
        else
          if obj.verbose
            fprintf('IK solution not valid because of\n')
            for i = 1:numel(infeasible_constraints)
              fprintf('\t%s\n', infeasible_constraints{i})
            end
          end
        end
      end
%       if ~isempty(validConfs)
%         validConfs = validConfs(:, validConfs(1,:) > 0);
%         [cost, qOptIdx] =  min(validConfs(1,:));
%         qOpt = validConfs(2:end, qOptIdx);
%       end
      if obj.verbose, fprintf('iteration Time: %.4f\n', toc(iterationTimer)); end
      if obj.debug
        debug_vars.n_valid_samples = n_valid_samples;
        debug_vars.cost = (obj.q_nom - qOpt)'*obj.ikoptions.Q*(obj.q_nom - qOpt);
        debug_vars.n_valid_samples_used = iter;
        debug_vars.final_orient = orient_idx;
      end
      
    end
    
    function constraints = generateGoalConstraints(obj)
%       !!!!!!WARNING:This works for val2 only!!!!!!!!!
      end_effector = obj.robot.findLinkId(obj.capability_map.end_effector_link.(obj.grasping_hand));
      goalDistConstraint = Point2PointDistanceConstraint(obj.robot, end_effector, obj.robot.findLinkId('world'), obj.end_effector_point, obj.x_goal(1:3), -0.001, 0.001);
      goalQuatConstraint = WorldQuatConstraint(obj.robot, end_effector, obj.x_goal(4:7), 1/180*pi);
      constraints = {goalDistConstraint, goalQuatConstraint};
%       !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    end
    
  end
  
  methods (Static)    
    
    function debug_vars = initDebugVars()
      debug_vars = struct(...
        'final_orient', [], ...
        'n_valid_samples', [], ...
        'n_valid_samples_used', [], ...
        'cost', [], ...
        'info', [], ...
        'computation_time', []...
        );
    end
    
  end
  
end