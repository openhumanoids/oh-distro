classdef FinalPosePlanner
  
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
    seed
  end
  
  properties (Constant)
    SUCCESS = 1
    FAIL_NO_FINAL_POSE = 12
  end  
  
  methods
    
    function obj = FinalPosePlanner(robot, end_effector_id,...
         q_start, x_goal, additional_constraints, q_nom, capabilty_map, ikoptions, varargin)
      
      opt.graspinghand =  'right';
      opt.mindistance = 0.005;
      opt.activecollisionoptions = struct();
      opt.endeffectorpoint = [0; 0; 0];
      opt.debug = false;
      opt.verbose = false;
      opt.seed = 100;
      
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
      obj.debug = opt.debug;
      obj.verbose = opt.verbose;
      obj.seed = opt.seed;

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
          if obj.verbose, disp('Failed to find a feasible final configuration'); end
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
        fprintf('Computation Time: %.2f s\n', computation_time)
        if obj.debug
          debug_vars.info = info;
          debug_vars.computation_time = computation_time;
        end
      end
    end
    
    function [qOpt, debug_vars] = searchFinalPose(obj, point_cloud, debug_vars)
      
      est_pose_publisher = CandidateRobotPosePublisher('EST_ROBOT_STATE', true, obj.robot.getPositionFrame.getCoordinateNames);
      options.rotation_type = 2;
      options.compute_gradients = true;
      options.rotation_type = 1;
      
      if obj.debug, constraint_timer = tic(); end
      obj.goal_constraints = obj.generateGoalConstraints();
      if obj.debug, constraint_time = toc(constraint_timer); end
      
      if obj.verbose || obj.debug, CM_timer = tic; end
      obj.capability_map = obj.capability_map.setEEPose(obj.x_goal);
      obj.capability_map = obj.capability_map.setActiveSide(obj.grasping_hand);
      [obj.capability_map, timing_vars] = obj.capability_map.reduceActiveSet(true, point_cloud);
      valid_voxels = obj.capability_map.occupancy_map_active_orient(obj.capability_map.active_voxels, :);
      n_valid_samples = nnz(valid_voxels);
      if obj.verbose, fprintf('n valid samples: %d\n', n_valid_samples); end
      
      base = obj.robot.findLinkId(obj.capability_map.base_link);
      map_centre = obj.capability_map.map_centre.(obj.grasping_hand);
      obj.capability_map = obj.capability_map.computeProbabilityDistribution([map_centre' 0 0 0]);
%       f_id = fopen('/home/marco/oh-distro/software/planning/capabilityMapMatlab.log', 'w');
%       fclose(f_id);
      if obj.debug || obj.verbose
        CM_time = toc(CM_timer);
        if obj.verbose, fprintf('CM Time: %.4f s\n', CM_time); end
      end
      
      iter = 0;
      qOpt = [];
      cost = [];
      ik_time = 0;
      kin_time = 0;
      collision_time = 0;
      
      f_id = fopen('/home/marco/drc-testing-data/final_pose_planner/val_description/random_sequence');
      samples = fread(f_id, [1000, 100], 'double');
      fclose(f_id);
%       f_id = fopen('matlabCapabilityMap.log','a');
      samples = samples(:,obj.seed);
      for vox = 1:min([n_valid_samples, 1000])
        if obj.debug || obj.verbose
          sampling_timer = tic();
        end
        [v, o, obj.capability_map] = obj.capability_map.drawCapabilityMapSample(samples(vox));
        rpy = obj.capability_map.occupancy_map_orient(:,o);
        pos = rpy2rotmat(rpy) * obj.capability_map.vox_centres(:,v);
%         disp(v)
%         disp(o)
        
        iter = iter + 1;
        if obj.debug || obj.verbose
          sampling_time = toc(sampling_timer);
        end
        if obj.verbose, fprintf('Iteration %d:', iter); end
        
        if obj.debug, constraint_timer = tic(); end
        torsoPosConstraint = WorldPositionConstraint(obj.robot, base, obj.capability_map.map_centre.(obj.grasping_hand), pos + obj.x_goal(1:3),  pos + obj.x_goal(1:3));
        torsoEulerConstraint = WorldEulerConstraint(obj.robot, base, rpy, rpy);
        constraints = [{torsoPosConstraint, torsoEulerConstraint}, obj.goal_constraints, obj.additional_constraints];
        if obj.debug, constraint_time = constraint_time + toc(constraint_timer); end
        if obj.debug, ik_timer = tic(); end
        [q, info, infeasible_constraints] = inverseKin(obj.robot, obj.q_nom, obj.q_nom, constraints{:}, obj.ikoptions);
%         est_pose_publisher.publish([q; zeros(size(q))], get_timestamp_now());
%         pose_publisher.publish([q; zeros(size(q))], get_timestamp_now())
        valid = (info < 10);
        if obj.debug, ik_time = ik_time + toc(ik_timer); end
        if obj.debug, kin_timer = tic(); end
        kinSol = obj.robot.doKinematics(q);
        if obj.debug, kin_time = kin_time + toc(kin_timer); end
        if valid
          if obj.debug, collision_timer = tic(); end
          valid = ~obj.robot.collidingPointsCheckOnly(kinSol, point_cloud, obj.min_distance);
          if valid
            phi = obj.robot.collisionDetect(q, false);
            if obj.debug, collision_time = collision_time + toc(collision_timer); end
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
      if obj.debug
        debug_vars.n_valid_samples = n_valid_samples;
        if ~isempty(qOpt)
          debug_vars.cost = (obj.q_nom - qOpt)'*obj.ikoptions.Q*(obj.q_nom - qOpt);
        else
          debug_vars.cost = NaN;
        end
        debug_vars.n_valid_samples_used = iter;
        for f = fieldnames(timing_vars)'
          debug_vars.(f{1}) = timing_vars.(f{1});
        end
%         debug_vars.final_orient = orient_idx;
        debug_vars.IK_time = ik_time;
        debug_vars.capability_map_time = CM_time;
        debug_vars.collision_time = collision_time;
        debug_vars.constraints_time = constraint_time;
        debug_vars.kin_time = kin_time;
        debug_vars.sampling_time = sampling_time;
      end
      
%       fclose(f_id);
    end
    
    function constraints = generateGoalConstraints(obj)
%       !!!!!!WARNING:This works for val2 only!!!!!!!!!
      end_effector = obj.robot.findLinkId(obj.capability_map.end_effector_link.(obj.grasping_hand));
      goalPosConstraint = WorldPositionConstraint(obj.robot, end_effector, obj.end_effector_point, obj.x_goal(1:3)-0.001, obj.x_goal(1:3)+0.001);
      goalQuatConstraint = WorldQuatConstraint(obj.robot, end_effector, obj.x_goal(4:7), 1/180*pi);
      constraints = {goalPosConstraint, goalQuatConstraint};
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
        'computation_time', [],...
        'IK_time', [],...
        'capability_map_time', [],...
        'collision_time', [],...
        'constraints_time', [],...
        'kin_time', [],...
        'sampling_time', [],...
        'cm_angle_time', [],...
        'cm_base_hight_time', [],...
        'cm_direction_time', [],...
        'cm_collision_time', [],...
        'formats', struct('final_orient', '%d',...
                          'n_valid_samples', '%d',...
                          'n_valid_samples_used', '%d',...
                          'cost', '%g',...
                          'info', '%d',...
                          'computation_time', '%g',...
                          'capability_map_time', '%g',...
                          'collision_time', '%g',...
                          'IK_time', '%g',...
                          'constraints_time', '%g',...
                          'kin_time', '%g',...
                          'sampling_time', '%g',...
                          'cm_angle_time', '%g',...
                          'cm_base_hight_time', '%g',...
                          'cm_direction_time', '%g',...
                          'cm_collision_time', '%g'...
                        )...
        );
    end
    
  end
  
end