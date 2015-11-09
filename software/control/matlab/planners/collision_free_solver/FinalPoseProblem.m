classdef FinalPoseProblem
  
  properties
    robot
    end_effector_id
    x_start
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
    joint_space_tree
    debug
    visualizer
  end
  
  properties (Constant)
    SUCCESS = 1
    FAIL_NO_FINAL_POSE = 12
  end  
  
  methods
    
    function obj = FinalPoseProblem(robot, end_effector_id,...
         x_start, x_goal, additional_constraints, q_nom, capabilty_map, ikoptions, varargin)
      
      opt = struct('graspinghand', 'right', ...
                   'mindistance', 0.005, ...
                   'activecollisionoptions', struct(), ...
                   'endeffectorpoint', [0; 0; 0]', ...
                   'debug', false, ...
                   'visualizer', []);
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
      obj.x_start = x_start;
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
      obj.visualizer = opt.visualizer;

    end

    function [x_goal, info, debug_vars] = findFinalPose(obj)
      
      info = obj.SUCCESS;
      if obj.debug
        debug_vars = obj.initDebugVars();
      else
        debug_vars = struct();
      end
      
      %compute final pose
      tic
      if length(obj.x_goal) == 7
        disp('Searching for a feasible final configuration...')
        if obj.debug
          timer = tic();
        end
        [qGoal, debug_vars] = obj.searchFinalPose(obj.x_start, debug_vars);
        if isempty(qGoal)
          info = obj.FAIL_NO_FINAL_POSE;
          x_goal = obj.x_goal;
          disp('Failed to find a feasible final configuration')
          return
        else
          kinsol = obj.robot.doKinematics(qGoal);
          obj.x_goal = obj.robot.forwardKin(kinsol, obj.end_effector_id, obj.end_effector_point, 2);
          obj.x_goal = [obj.x_goal; qGoal];
          disp('Final configuration found')
        end
      elseif length(obj.x_goal) == 7 + obj.robot.num_positions
        disp('Final configuration input found A')
      elseif length(obj.x_goal) == obj.robot.num_positions
        kinsol = obj.robot.doKinematics(obj.x_goal);
        obj.x_goal = [obj.robot.forwardKin(kinsol, obj.end_effector_id, obj.end_effector_point, 2); obj.x_goal];
        disp('Final configuration input found B')
      else
        error('Bad final configuration input')
      end
      
      x_goal = obj.x_goal;
      if obj.debug
        debug_vars.info = info;
        debug_vars.computation_time = toc(timer);
      end
    end
    
    function [qOpt, debug_vars] = searchFinalPose(obj, x_start, debug_vars)
      
      filename = [getenv('DRC_BASE'), '/software/models/val_description/model/meshes/torso/torso.stl'];
      torso = RigidBodyManipulator();
      torso = torso.addGeometryToBody(1, RigidBodyMesh(filename));
      torso = torso.compile();
%       tv = torso.constructVisualizer();
      
      kinSol = obj.robot.doKinematics(x_start);
      options.rotation_type = 2;
      options.compute_gradients = true;
      options.rotation_type = 1;
      
      root = obj.capability_map.root_link.(obj.grasping_hand);
      endEffector = obj.capability_map.end_effector_link.(obj.grasping_hand);
      rootPoint = obj.capability_map.root_point.(obj.grasping_hand);
      EEPoint = obj.capability_map.end_effector_point.(obj.grasping_hand);
      base = obj.capability_map.base_link;
      
      rootPose = obj.robot.forwardKin(kinSol, root, rootPoint, 2);
      trPose = obj.robot.forwardKin(kinSol, base, [0;0;0], 2);
      tr2root = quat2rotmat(trPose(4:end))\(rootPose(1:3)-trPose(1:3));
%       !!!!!!WARNING:This works for val2 only!!!!!!!!!
      if strcmp(obj.grasping_hand, 'right')
        armJoints = 13:19;
      else
        armJoints = 20:26;
      end
%       !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      nArmJoints = size(armJoints, 2);
      np = obj.robot.num_positions;
%       !!!!!!WARNING:This works for val2 only!!!!!!!!!
      if isfield(obj.active_collision_options, 'body_idx')
        if strcmp(obj.grasping_hand, 'right')
          collisionLinksBody = setdiff(obj.active_collision_options.body_idx, 9:15);
        else
          collisionLinksBody = setdiff(obj.active_collision_options.body_idx, 16:22);
        end
      else
        collisionLinksBody = setdiff(1:numel(obj.robot.body), 16:22);
      end
%       !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      mapMirror.right = [1; 1; 1];
      mapMirror.left = [1; -1; 1];
      
      ee_rotmat = quat2rotmat(obj.x_goal(4:7));
      ee_direction = ee_rotmat(:,2);
      obj.capability_map = obj.capability_map.reduceActiveSet(ee_direction, 800, 1000, true, 0, 0, 2, 1.5);
      sphCenters = obj.capability_map.getActiveSphereCentres();
      nSph = obj.capability_map.n_active_spheres;
      iter = 0;
      qOpt = [];
      cost = [];
      c = 1/obj.min_distance;
      deltaQmax = 0.05;
      validConfs =  double.empty(np+1, 0);
      succ = zeros(nSph, 2);
      
      for sph = randperm(nSph)
        iter = iter + 1;
        point = (sphCenters(:,sph).*mapMirror.(obj.grasping_hand)) + tr2root;
        shConstraint = WorldPositionConstraint(obj.robot, base, point, obj.x_goal(1:3), obj.x_goal(1:3));
        shOrient = WorldEulerConstraint(obj.robot, base, [0;-pi/30; -pi/2], [0; pi/30; pi/2]);
        constraints = [{shConstraint, shOrient}, obj.goal_constraints];
        [q, info] = inverseKin(obj.robot, obj.q_nom, obj.q_nom, constraints{:}, obj.ikoptions);
        valid = (info < 10);
%         obj.visualizer.draw(0, q)
        kinSol = obj.robot.doKinematics(q, ones(obj.robot.num_positions, 1), options);
        palmPose = obj.robot.forwardKin(kinSol, endEffector, EEPoint, options);
        targetPos = palmPose;
        deltaX = zeros(6,1);
        if valid
          if obj.debug
            debug_vars.n_body_collision_iter = debug_vars.n_body_collision_iter + 1;
          end
          phiBody = obj.robot.collisionDetect(q, false, struct('body_idx', collisionLinksBody));
          if all(phiBody > obj.min_distance)
            if obj.debug
              debug_vars.n_arm_collision_iter = debug_vars.n_arm_collision_iter + 1;
            end
            eps  = Inf;
            nIter = 0;
            [phi,normal,~,~,idxA,idxB] = obj.robot.collisionDetect(q, false, obj.active_collision_options);
            nullspace = false;
            if any(phi < obj.min_distance)
              if obj.debug
                debug_vars.n_nullspace_iter = debug_vars.n_nullspace_iter + 1;
              end
              phi = phi - obj.min_distance;
%               while (eps > 1e-3 || any(phi < 0)) && nIter < 1
%                 if obj.debug
%                   nullspace = true;
%                 end
%                 qNdot = zeros(nArmJoints, 1);
%                 for joint = 1:nArmJoints
%                   dgamma_dq = zeros(size(phi));
%                   for coll = 1:size(phi,1)
%                     if phi(coll) < 0
%                       [~,JA] = obj.robot.forwardKin(kinSol, idxA(coll), [0;0;0], options);
%                       [~,JB] = obj.robot.forwardKin(kinSol, idxB(coll), [0;0;0], options);
%                       JA = JA(:,armJoints);
%                       JB = JB(:,armJoints);
%                       dD_dq = normal(:,coll)'*(JB(1:3,joint) - JA(1:3,joint));
%                       dgamma_dq(coll) = exp(1./(c*phi(coll))).*(1-c*phi(coll))./(c*phi(coll))*c.*dD_dq;
%                     else
%                       dgamma_dq(coll) = 0;
%                     end
%                   end
%                   qNdot(joint) = sum(dgamma_dq);
%                 end
%                 [~,J] = obj.robot.forwardKin(kinSol, endEffector, [0;0;0], options);
%                 J = J(:,armJoints);
%                 Jpsi = J'*inv(J*J');
%                 deltaQ = Jpsi*deltaX + (eye(nArmJoints) - Jpsi*J) * qNdot;
%                 if any(abs(deltaQ) > deltaQmax)
%                   alpha = deltaQmax/abs(deltaQ);
%                 elseif all(deltaQ < 1e-3)
%                   break
%                 else
%                   alpha = 1;
%                 end
%                 q(armJoints) = q(armJoints) + alpha*deltaQ;
%                 [phi,normal,~,~,idxA,idxB] = obj.robot.collisionDetect(q, false, obj.active_collision_options);
%                 kinSol = obj.robot.doKinematics(q, ones(obj.robot.num_positions, 1), options);
%                 palmPose = obj.robot.forwardKin(kinSol, endEffector, [0;0;0], options);
%                 deltaX = targetPos - palmPose;
%                 eps = norm(deltaX);
%                 nIter = nIter + 1;
%                 phi = phi - obj.min_distance;
%               end
              if obj.debug
                debug_vars.nullspace_iter.success(debug_vars.n_nullspace_iter) = eps <= 1e-3 && all(phi >= 0);
                debug_vars.nullspace_iter.nIter(debug_vars.n_nullspace_iter) = nIter;
              end
            else
              if obj.debug
                debug_vars.n_accepted_after_armcollision = debug_vars.n_accepted_after_armcollision + 1;
              end
              phi = phi - obj.min_distance;
              eps = 1e-3;
            end
            if eps <= 1e-3 && all(phi >= 0)
              if obj.debug && nullspace
                debug_vars.n_accepted_after_nullspace = debug_vars.n_accepted_after_nullspace + 1;
              end
              cost = (obj.q_nom - q)'*obj.ikoptions.Q*(obj.q_nom - q);
              validConfs(:,sph) = [cost; q];
              succ(sph, :) = [1, nIter];
              if cost < 20
                if obj.debug
                  debug_vars.cost_below_threshold = true;
                end
                break
              end
            else
              succ(sph, :) = [0, nIter];
            end
          end
        end
      end
      if ~isempty(validConfs)
        validConfs = validConfs(:, validConfs(1,:) > 0);
        [cost, qOptIdx] =  min(validConfs(1,:));
        qOpt = validConfs(2:end, qOptIdx);
      end
      
      debug_vars.n_capability_map_samples = iter;
      debug_vars.cost = cost;
      
    end
    
    function constraints = generateGoalConstraints(obj)
%       !!!!!!WARNING:This works for val2 only!!!!!!!!!
      end_effector = obj.capability_map.end_effector_link.(obj.grasping_hand);
      goalDistConstraint = Point2PointDistanceConstraint(obj.robot, end_effector, obj.robot.findLinkId('world'), obj.end_effector_point, obj.x_goal(1:3), -0.001, 0.001);
      goalQuatConstraint = WorldQuatConstraint(obj.robot, end_effector, obj.x_goal(4:7), 1/180*pi);
      constraints = {goalDistConstraint, goalQuatConstraint};
%       !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    end
    
    
    function obj = pruneCapabilityMap(obj, sagittalAngle,...
        transverseAngle, sagittalWeight, transverseWeight, reachabilityWeight)
      
      Dmax = max(obj.capability_map.reachabilityIndex);
      nSph = length(obj.capability_map.map);
      indices = [];
      
      for sph = 1:nSph
        sa = atan2(obj.capability_map.sphCenters(3,sph), obj.capability_map.sphCenters(1,sph));
        ta = atan2(obj.capability_map.sphCenters(2,sph), obj.capability_map.sphCenters(1,sph));
        sagittalCost = sagittalWeight * abs(sa - sagittalAngle);
        transverseCost = transverseWeight * abs(ta - transverseAngle);
        reachabilityCost = reachabilityWeight * (Dmax - obj.capability_map.reachabilityIndex(sph));
        if sqrt(sagittalCost^2 + transverseCost^2) + reachabilityCost < 2
          indices(end + 1) = sph;
        end
      end
      obj.capability_map.nSph = length(indices);
      obj.capability_map.reachabilityIndex = obj.capability_map.reachabilityIndex(indices);
      obj.capability_map.map = obj.capability_map.map(indices, :);
      obj.capability_map.sphCenters = obj.capability_map.sphCenters(:, indices);
    end    
    
  end
  
  methods (Static)    
    
    function debug_vars = initDebugVars()
      debug_vars = struct(...
        'n_capability_map_samples', [], ...
        'cost', [], ...
        'info', [], ...
        'cost_below_threshold', [], ...
        'n_body_collision_iter', 0, ...
        'n_arm_collision_iter', 0, ...
        'n_nullspace_iter', 0, ...
        'n_accepted_after_armcollision', 0, ...
        'n_accepted_after_nullspace', 0, ...
        'nullspace_iter', struct('success', [], 'nIter', []), ...
        'computation_time', []...
        );
    end
    
  end
  
end