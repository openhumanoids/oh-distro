% Class that will perform KinematicPoseTrajectory making sure we are stable
% at all the points that transition between support phases.


% The most important structures are data_struct and pose_struct
%       * data_struct is a struct with two fields, contacts which is a cell
%       array that lists all the active contact points. 'seed' is the pose
%       that we should seed the 'IKsolve' with  . . .
%
%
%
%       * pose_struct, is similar to data_struct but contains more
%       information. 'contacts' are the contact points that should be
%       touching the ground. 'qs_contacts' are the contact points that can
%       be used to satisfy the quasi-static constraint. 'no_movement' are
%       the contact points that cannot move from their position at knot
%       point idx-1. 'seed' is the seed that we should use in setting up
%       the solver
%
%
%       * c - a container.Map containing the actual contact points for each
%       contact group, i.e. c('l_toe')
%
%
%       * linkId - a container.Map containing the linkId of the contact
%       points in c, i.e. linkId('l_toe') = robot.findLinkId('l_foot')


classdef KinematicPoseTrajectory < KinematicTrajectoryOptimization & ...
    NonlinearProgram
  
  
  %
  
  
  properties
    data_struct;
    pose_struct;
    c;
    linkId;
    N; % number of knot points
    Xinds;
    shrink_factor = 0.8;
    nb;
    min_distance = 0.03;
  end
  
  
  methods
    
    
    function obj = KinematicPoseTrajectory(robot,data_struct,is_time_stepping_rigid_body_manipulator)
      
      % want to be able to handle empty data_struct object
      
      if (nargin < 2 || isempty(data_struct))
        data_struct = struct('contacts',{},'seed',{});
        data_struct(1).contacts = {};
        nq = robot.getNumPositions();
        data_struct(1).seed = zeros(nq,1);
      end
      
      if nargin < 3
        is_time_stepping_rigid_body_manipulator = 0;
      end
      
      % include something to handle an empty data_struct;
      
      % instantiate with no vars and then add vars as needed once we
      % can call genPoseStruct();
      obj = obj@NonlinearProgram(0);
      % the zero is so that we don't incorrectly call getXinds()
      % which has yet to be properly set
      obj = obj@KinematicTrajectoryOptimization(robot,0);
      
      
      if ~is_time_stepping_rigid_body_manipulator
        obj.robot.collision_filter_groups('ignores_ground') = CollisionFilterGroup();
        obj.robot.collision_filter_groups('ground') = CollisionFilterGroup();
        obj.robot = obj.robot.addLinksToCollisionFilterGroup('world','ground',0);
        obj.robot = obj.robot.addToIgnoredListOfCollisionFilterGroup('ignores_ground','ground');
        obj.robot = obj.robot.addToIgnoredListOfCollisionFilterGroup('ground','ignores_ground');
      end
      
      obj.robot = compile(obj.robot);
      
      
      
      
      % first need to determine the number of knot points, so before
      % anything else we need to generate pose_struct without using
      % the object. So we must use Static methods to do this
      obj.pose_struct = KinematicPoseTrajectory.genPoseStruct(data_struct);
      N = numel(obj.pose_struct);
      % create Xinds
      np = obj.robot.getNumPositions();
      Xinds = [1:N*np];
      obj.Xinds = reshape(Xinds,np,N);
      obj.nb = obj.robot.getNumBodies();
      
      % need to add the right number variables to the NonlinearProgram
      % object, must do this before calling constructorHack() on the
      % KinematicTrajectoryOptimization superclass
      var_name = cell(N*np,1);
      
      for j = 1:N
        idx_lb = 1 + (j-1)*np;
        idx_ub = idx_lb + np -1;
        str = strcat('q knot point',{' '},num2str(j));
        var_name(idx_lb:idx_ub) = repmat(str,np,1);
      end
      obj = obj.addDecisionVariable(np*N,var_name);
      
      
      % finish setting object properties
      obj.c = obj.genContactPts();
      obj.linkId = obj.genLinkId();
      obj.data_struct = data_struct;
      
      % add the collision geometries to the robot
      obj.robot = obj.addCollisionGeometryToRobot();
      
      % now we can call the initialize() for
      % KinematicTrajectoryOptimization to finish doing what the
      % constructor should have done the first time
      obj = obj.initialize();
      
    end
    
    
    % adds the constraint that the contacts bodies in contacts have
    % position with z = 0 with a WorldPositionConstraint. idx is the
    % knot point at which the constraint should be enforced
    function [obj,constraints] = addSingleContactConstraint(obj,contacts,time_index, add_to_obj)
      
      if nargin < 3
        warning('Added a contact constraint without specifying a time_index, setting it to 1 by default')
        time_index = 1;
      end
      
      % flag that determines whether or not these constraints are
      % added to the NonlinearProgram object through the superclass
      if nargin < 4
        add_to_obj = 1;
      end
      
      % cell array of the constraints to be added.
      constraints = {};
      
      bds = [nan;nan;0];
      for j = 1:numel(contacts)
        name = contacts{j};
        id = obj.linkId(name);
        pts = obj.c(name);
        s = size(pts);
        bds_resize = repmat(bds,1,s(2));
        wp = WorldPositionConstraint(obj.robot,id,pts,bds_resize,bds_resize);
        constraints{end+1} = wp;
        % adds the WorldPositionConstraint to the obj, which is
        % also of class NonlinearProgram
        if add_to_obj
          obj = obj.addRigidBodyConstraint(wp,time_index);
        end
      end
    end
    
    
    % use addKinematicConstraint from the superclass
    % KinematicTrajectoryOptimization
    function obj = addSingleMovementConstraint(obj,no_movement,time_index)
      time_index_both = {[time_index-1, time_index]};
      for j = 1:numel(no_movement)
        name = no_movement{j};
        pts = obj.c(name);
        id = obj.linkId(name);
        wfp = WorldFixedPositionConstraint(obj.robot,id,pts,[-inf,inf]);
        
        % addRigidBodyConstraint is a method in
        % KinematicTrajectoryOptimization which itself calls another
        % method addKinematicConstraint. The square brackets in the
        % function call are essential to ensure that this constraint
        % is applied to knot points time_index-1 and time_index
        % together
        obj = obj.addRigidBodyConstraint(wfp,{[time_index-1,time_index]});
      end
    end
    
    
    % need to add the collision avoidance constraints
    % this could be optimized to ignore some upper body collisions
    % maybe???
    function obj = addSingleCollisionConstraint(obj,time_index)
      not_collision_checked = [];
      for j = 1:numel(obj.pose_struct(time_index).contacts)
        not_collision_checked(end+1) = obj.linkId(obj.pose_struct(time_index).contacts{j});
      end
      
      robot = obj.robot.addLinksToCollisionFilterGroup(not_collision_checked,'ignores_ground',1);
      robot = compile(robot);
      
      % the bodies that are contacts don't need to be collision
      % checked for when we are including the world, i.e. the terrain
      
      mdc = MinDistanceConstraint(robot,obj.min_distance);
      obj = obj.addRigidBodyConstraint(mdc,time_index);
      %
      %
      %             % World vs. Robot min distance constraint
      %             active_collision_options.body_idx = setdiff(1:obj.nb,not_collision_checked);
      %             mdc = MinDistanceConstraint(obj.robot,obj.min_distance,active_collision_options);
      %
      %             % use addRigidBodyConstraint from superclass
      %             % KinematicTrajectoryOptimization
      %             obj = obj.addRigidBodyConstraint(mdc,time_index);
      %
      %
      %             % Robot self-collision constraint
      %             % exclude body 1 which is the world link
      %             aco_self_collision.body_idx = [2:obj.nb];
      %             mdc_self_collision = MinDistanceConstraint(obj.robot,obj.min_distance,aco_self_collision);
      %             obj = obj.addRigidBodyConstraint(mdc_self_collision,time_index);
    end
    
    
    % generates a single collision constraint and returns is
    %
    function constraints = genSingleCollisionConstraint(obj,contacts)
      not_collision_checked = [];
      for j = 1:numel(contacts)
        not_collision_checked(end+1) = obj.linkId(contacts{j});
      end
      
      robot = obj.robot.addLinksToCollisionFilterGroup(not_collision_checked,'ignores_ground',1);
      robot = compile(robot);
      
      % the bodies that are contacts don't need to be collision
      % checked for when we are including the world, i.e. the terrain
      
      mdc = MinDistanceConstraint(robot,obj.min_distance);
      constraints = {};
      constraints{end+1} = mdc;
    end
    
    
    
    % this can also be handled by addRigidBodyConstraint in the
    % KinematicTrajectoryOptimization class
    function [obj, constraints] = addSingleQuasiStaticConstraint(obj,qs_contacts,time_index,add_to_obj)
      
      % flag that controls whether or not constraint should be added
      % to the object or not, 1 means it IS added to the object
      if nargin < 4
        add_to_obj = 1;
      end
      
      if nargin < 3
        warning('Added a QS constraint without specifying a time_index, setting it to 1 by default')
        time_index = 1;
      end
      
      
      qs =QuasiStaticConstraint(obj.robot);
      qs = qs.setShrinkFactor(obj.shrink_factor);
      
      for j=1:numel(qs_contacts)
        name = qs_contacts{j};
        id = obj.linkId(name);
        pts = obj.c(name);
        qs = qs.addContact(id,pts);
      end
      
      qs = qs.setActive(true);
      
      % use inherited method from KinematicTrajectoryOptimization
      if add_to_obj
        obj = obj.addRigidBodyConstraint(qs,time_index);
      end
      
      constraints ={};
      constraints{end+1} = qs;
    end
    
    
    % adds a onetime quadratic cost (q - q_nom)^T*Q*(q-q_nom) to the
    % nonlinear program. If q_nom is not specified we will set q_nom to
    % be the seed that was passed in
    function obj = addSinglePoseCost(obj,time_index,q_nom)
      
      if nargin < 3
        q_nom = obj.pose_struct(time_index).seed;
      end
      
      Q = eye(obj.nq); % diagonal matrix
      % set the first 6 elements of this to be zero, since we don't
      % care about penalizing xyz or rpy???
      Q(1:6,1:6) = zeros(6,6);
      
      b = -Q*q_nom;
      quad_constraint = QuadraticConstraint(0,0,Q,b);
      
      % add this as a cost to the nonlinear program
      obj = obj.addCost(quad_constraint,obj.Xinds(:,time_index));
      
    end
    
    
    % adds the cost (q-q_0)'*Q*(q-q_0) to each knot point
    function obj = addRunningPoseCost(obj)
      for j = 1:obj.N
        obj = obj.addSinglePoseCost(j);
      end
    end
    
    
    function obj = addStartingPoseConstraint(obj,q_start)
      bbc = BoundingBoxConstraint(q_start,q_start);
      % calling method from NonlinearProgram superclass
      obj = obj.addBoundingBoxConstraint(bbc,obj.Xinds(:,1));
    end
    
    function obj = addPoseConstraint(obj,time_index,q_nom)
      bbc = BoundingBoxConstraint(q_nom,q_nom);
      % calling method from NonlinearProgram superclass
      obj = obj.addBoundingBoxConstraint(bbc,obj.Xinds(:,time_index));
    end
    
    function obj = addAllConstraints(obj)
      
      pose_struct = obj.pose_struct;
      
      % Need to take special care with j = 1, it doesn't have a movement constraint
      obj = obj.addSingleContactConstraint(pose_struct(1).contacts,1);
      obj = obj.addSingleQuasiStaticConstraint(pose_struct(1).qs_contacts,1);
      obj = obj.addSingleCollisionConstraint(1);
      
      for j = 2:numel(pose_struct)
        obj = obj.addSingleContactConstraint(pose_struct(j).contacts,j);
        obj = obj.addSingleMovementConstraint(pose_struct(j).no_movement,j);
        obj = obj.addSingleQuasiStaticConstraint(pose_struct(j).qs_contacts,j);
        obj = obj.addSingleCollisionConstraint(j);
      end
    end
    
    % Note: this should only be called once since we have been adding
    % variables to the nonlinear program object all along, have a
    % solved_flag and throw an error if you try to call solve again???
    function [q_sol,exitflag,infeasible_constraint_name, obj] = solve(obj)
      obj = obj.addAllConstraints();
      obj = obj.addRunningPoseCost();
      obj = obj.setSolverOptions('snopt','iterationslimit',1e6);
      x0 = obj.getSeed();
      
      % this is just going to throw the infeasible constraints, which
      % has actually been happening a lot, will this still be doing
      % that automatic constraint checking after the snopt output???
      [x,objval,exitflag,infeasible_constraint_name] = solve@NonlinearProgram(obj,x0);
      q_sol = obj.extractSolution(x);
    end
    
    % extract the solution
    function q_sol = extractSolution(obj,x)
      q_sol = zeros(obj.nq,obj.N);
      
      for j = 1:obj.N
        q_sol(:,j) = x(obj.Xinds(:,j));
      end
    end
    
    function qtraj = constructTrajectory(obj,q_sol)
      t_grid = linspace(0,2*obj.N,2*obj.N);
      
      q_grid = zeros(obj.nq,obj.N*2);
      
      for j = 1:obj.N
        idx = 2*(j-1) + 1;
        q_grid(:,idx) = q_sol(:,j);
        q_grid(:,idx+1) = q_sol(:,j);
      end
      
      qtraj = PPTrajectory(foh(t_grid,q_grid));
      
    end
    
    % constructs the correct constraints for calling
    function [q_ik, t_grid] = inverseKinPointwise(obj,q_sol,num_points)
      obj.robot = compile(obj.robot);
      constraints = obj.genIKConstraints(q_sol);
      
      % make this trajectory to get the seed for the inverseKin
      t_traj_grid = [1:obj.N];
      qtraj = PPTrajectory(foh(t_traj_grid,q_sol));
      
      t_grid = setdiff(linspace(1,obj.N,num_points),[1:obj.N]);
      q_seed = qtraj.eval(t_grid);
      q_nom = q_seed;
      [q,info,infeasible_constraint] = obj.robot.inverseKinPointwise(t_grid,...
        q_seed,q_nom,constraints{:});
      
      
      
      % make sure to skip over knot points in the time grid passed to
      % IK pointwise since there what has to be on the ground and not
      % gets tricky . . . .
      
      % don't worry about adding collision constraints for now, can
      % do that later . . .
      
      
      
    end
    
    
    
    % this constraint should be active during (time_index - 1,
    % time_index)
    function constraints = addSingleIKQuasiStaticConstraint(obj,time_index,constraints,q_sol)
      qs =QuasiStaticConstraint(obj.robot,[time_index - 1, time_index]);
      qs = qs.setShrinkFactor(obj.shrink_factor);
      
      % the contact points that didn't move from time_index -1 -->
      % time_index are the things that were in contact and are
      % eligible for the quasistatic constraint
      qs_contacts = obj.pose_struct(time_index).no_movement;
      for j=1:numel(qs_contacts)
        name = qs_contacts{j};
        id = obj.linkId(name);
        pts = obj.c(name);
        qs = qs.addContact(id,pts);
      end
      qs = qs.setActive(true);
      
      % add it to the constraints list
      constraints{end+1} = qs;
    end
    
    % this constraint should be active during (time_index - 1,
    % time_index)
    function constraints = addSingleIKMovementConstraint(obj,time_index,constraints,q_sol)
      no_movement = obj.pose_struct(time_index).no_movement;
      q_ref = q_sol(:,time_index);
      kinsol = obj.robot.doKinematics(q_ref);
      for j=1:numel(no_movement)
        name = no_movement{j};
        id = obj.linkId(name);
        pts = obj.c(name);
        % figure out where these points are in the q_sol solution
        % at the specified time_index
        bds = obj.robot.forwardKin(kinsol,id,pts);
        
        % create WorldPositionConstraint with the correct tspan to
        % be active
        wpc = WorldPositionConstraint(obj.robot,id,pts,bds,bds,[time_index - 1,time_index]);
        
        % add it to constraints;
        constraints{end+1} = wpc;
      end
    end
    
    % generate the appropriate constraints to pass to IK pointwise
    function constraints = genIKConstraints(obj,q_sol)
      constraints = {};
      for j = 2:obj.N
        constraints = obj.addSingleIKQuasiStaticConstraint(j,constraints,q_sol);
        constraints = obj.addSingleIKMovementConstraint(j,constraints,q_sol);
      end
    end
    
    % should only pass this time_index between 2 and obj.N
    function [q,t] = solveIKInterval(obj,q_sol,time_index,num_pts)
      obj.robot = compile(obj.robot);
      % t_grid not including the endpoints
      t = setdiff(linspace(time_index - 1, time_index,num_pts+2),[time_index - 1,time_index]);
      qtraj = PPTrajectory(foh([1:obj.N],q_sol));
      q_nom = qtraj.eval(t);
      q_seed = q_nom;
      
      % generate the correct constraints
      constraints = {};
      constraints = obj.addSingleIKQuasiStaticConstraint(time_index,constraints,q_sol);
      constraints = obj.addSingleIKMovementConstraint(time_index,constraints,q_sol);
      
      [q,info,infeasible_constraint] = obj.robot.inverseKinPointwise(t,...
        q_seed,q_nom,constraints{:});
      
      info
      % print out if there are infeasible_constraints
      info
      infeasible_constraint
      
    end
    
    function [q_ik,t_grid] = inverseKinPointwise_2(obj,q_sol,num_pts)
      
      q_ik = [];
      t_grid = [];
      
      for time_index=2:obj.N
        [q,t] = obj.solveIKInterval(q_sol,time_index,num_pts);
        q_ik = [q_ik,q];
        t_grid = [t_grid,t];
      end
    end
    
    
    function [q,info,infeasible_constraint] = genRandomPose(obj,contacts,q_nom)
      
      % num inputs
      ni = obj.robot.getNumInputs;
      
      eps = 0.8*(rand(obj.nq,1) - 0.5);
      eps(1:(obj.nq - ni)) = 0;
      q_nom = q_nom + eps;
      q_seed = q_nom;
      constraints = {};
      [~,contact_constraints] = obj.addSingleContactConstraint(contacts,1,0);
      collision_constraints = obj.genSingleCollisionConstraint(contacts);
      [~,qs_constraints] = obj.addSingleQuasiStaticConstraint(contacts,1,0);
      
      constraints = [contact_constraints,collision_constraints,...
        qs_constraints];
      
      
      
      % the solver complains if you add in the collision constraint,
      % it really doesn't like those . . .
      %            constraints = [qs_constraints, contact_constraints];
      
      %            [q,info,infeasible_constraint] = obj.robot.inverseKinPointwise(1,...
      %                 q_seed,q_nom,constraints{:});
      
      ik = InverseKinematics(obj.robot,q_nom,constraints{:});
      ik = ik.setSolverOptions('snopt','iterationslimit',1e6);
      ik = ik.setSolverOptions('snopt','majoriterationslimit',200);
      ik = ik.setSolverOptions('snopt','print','snopt.out');
      [q,F,info,infeasible_constraint] = ik.solve(q_seed);
      
      info
      infeasible_constraint
    end
    
    % constructs IK object for finding a quasistatic pose with
    % specified contact points and position costs on certain end
    % effector links
    function [q,F,info,infeasible_constraint] = findPose(obj,contacts,position_cost,q_nom)
      % @param position_cost  -- a structure array with fields 'name'
      % which is the name of the contact point, i.e. 'l_toe' and the
      % desired position which is a 3x1 pt in the field 'position'
      % generate the correct constraints and costs
      [~,contact_constraints] = obj.addSingleContactConstraint(contacts,1, 0);
      collision_constraints = obj.genSingleCollisionConstraint(contacts);
      [~,qs_constraints] = obj.addSingleQuasiStaticConstraint(contacts,1,0);
      
      
      constraints = [contact_constraints, collision_constraints,...
        qs_constraints];
      
      
      % test for quaternion stuff, don't include the quaternion constraints
      %       constraints = [contact_constraints,...
      %         qs_constraints];
      
      ik = InverseKinematics(obj.robot,q_nom,constraints{:});
      ik = ik.setSolverOptions('snopt','iterationslimit',1e6);
      ik = ik.setSolverOptions('snopt','majoriterationslimit',400);
      ik = ik.setSolverOptions('snopt','print','snopt.out');
      
      if ~isempty(position_cost)
        ik = obj.addContactPositionCost(ik,position_cost);
      end
      
      [q,F,info,infeasible_constraint] = ik.solve(q_nom);
    end
    
    
    % adds a cost on the position of a contact point or set of points, i.e.
    % 'l_hand'. position_cost is a structure array with fields
    function ik = addContactPositionCost(obj,ik,position_cost)
      
      for j = 1:numel(position_cost)
        
        if isfield(position_cost(j),'scale')
          scale = position_cost(j).scale;
        else
          scale = 1;
        end
        
        name = position_cost(j).name;
        pos = position_cost(j).position; % should be 3 x 1
        pts = obj.c(name);
        num_pts = size(pts,2);
        pts_desired = repmat(pos,1,num_pts); %
        c_pts_fun = drakeFunction.kinematic.WorldPosition(obj.robot,obj.linkId(name),pts);
        % note the scaling is embedded in this affine function
        affine = drakeFunction.Affine(c_pts_fun.output_frame,c_pts_fun.output_frame,scale*eye(3*num_pts),-pts_desired(:)*scale);
        norm_fun = drakeFunction.euclidean.NormSquared(affine.output_frame);
        distance_fun = norm_fun(affine(c_pts_fun));
        
        
        % create a drakeFunctionConstraint from this which can be
        % passed to NonlinearProgram.addCost()
        
        cost = DrakeFunctionConstraint(0,0,distance_fun);
        
        % need to figure out which are the variable inds
        % corresponding to q in the InverseKinematics object
        ik = ik.addCost(cost,ik.q_idx);
        
      end
    end
    
    
    
    
    function obj = setQuasiStaticShrinkFactor(obj,shrink_factor)
      obj.shrink_factor = shrink_factor;
    end
    
    function obj = setMinDistance(obj,min_distance)
      obj.min_distance = min_distance;
    end
    
    % computes the seed from the nominal poses in pose_struct
    function x0 = getSeed(obj)
      x0 =zeros(obj.num_vars,1);
      
      for j = 1:obj.N
        x0(obj.Xinds(:,j)) = obj.pose_struct(j).seed;
      end
    end
    
    % required method of KinematicTrajectoryOptimization
    function Xinds = getXinds(obj)
      Xinds = obj.Xinds;
    end
    
    % required method of KinematicTrajectoryOptimization
    function Hinds = getHinds(obj)
      Hinds = []; % empty because we aren't reasoning about time steps
    end
    
    % required abstract method of KinematicTrajectoryOptimization
    function N = getN(obj)
      N = numel(obj.pose_struct);
    end
    
    
    % function to generate the contact pts
    function c = genContactPts(obj)
      %% Copied over from test_poses
      l_face_cage = [0.313;0.154;0.622];
      r_face_cage = [0.313;-0.162;0.6228];
      face_cage_pts = [l_face_cage,r_face_cage];
      
      
      
      %% Initialize containers.Map
      
      c = containers.Map;
      
      c('l_toe') = obj.robot.getBody(obj.robot.findLinkId('l_foot')).getTerrainContactPoints('toe');
      c('l_heel') = obj.robot.getBody(obj.robot.findLinkId('l_foot')).getTerrainContactPoints('heel');
      c('r_toe') = obj.robot.getBody(obj.robot.findLinkId('r_foot')).getTerrainContactPoints('toe');
      c('r_heel') = obj.robot.getBody(obj.robot.findLinkId('r_foot')).getTerrainContactPoints('heel');
      
      c('l_knee') = [0.09485; 0.0; -0.0401];
      c('r_knee') = [0.09485; 0.0; -0.0401];
      c('r_hand') = [0.1, -0.1, 0, 0;
        -0.4, -0.4, -0.4, -0.4;
        0, 0, 0.1, -0.1];
      
      c('l_hand') = [0.1, -0.1, 0, 0;
        0.4, 0.4, 0.4, 0.4;
        0, 0, 0.1, -0.1];
      
      c('face_cage') = face_cage_pts;
      
      c('chest') = [0.258;0;0.1882];
      
      c('l_foot') = [c('l_toe'), c('l_heel')];
      c('r_foot') = [c('r_toe'), c('r_heel')];
      
      c('r_uback') = [-0.2879;-0.161;0.475];
      c('r_lback') = [-0.2879;-0.161;0.029];
      c('l_uback') = [-0.2879;0.161;0.475];
      c('l_lback') = [-0.2879; 0.161;0.029];
      c('back') = [c('r_uback'),c('r_lback'),c('l_uback'),c('l_lback')];
      
      c('r_fpelvis') = [-0.0927; -0.0616;-0.144];
      c('r_bpelvis') = [-0.128;-0.071;-0.144];
      c('l_fpelvis') = [-0.0927; 0.0623;-0.144];
      c('l_bpelvis') = [-0.128;0.0718;-0.144];
      c('pelvis') = [c('r_fpelvis'),c('r_bpelvis'),c('l_fpelvis'),c('l_bpelvis')];
      
    end
    
    
    % a container.Map that gives the link name corresponding to each
    % contact point group
    function lmap = genLinkId(obj)
      lmap = containers.Map;
      
      lmap('l_toe') = obj.robot.findLinkId('l_foot');
      lmap('r_toe') = obj.robot.findLinkId('r_foot');
      lmap('r_knee') =  obj.robot.findLinkId('r_lleg');
      lmap('l_knee') = obj.robot.findLinkId('l_lleg');
      lmap('r_hand') = 31;
      lmap('l_hand') = 18;
      lmap('face_cage')= obj.robot.findLinkId('utorso');
      lmap('chest') = obj.robot.findLinkId('utorso');
      lmap('l_foot') =  obj.robot.findLinkId('l_foot');
      lmap('r_foot')= obj.robot.findLinkId('r_foot');
      lmap('l_heel') = obj.robot.findLinkId('l_foot');
      lmap('r_heel') = obj.robot.findLinkId('r_foot');
      lmap('r_uback') = obj.robot.findLinkId('utorso');
      lmap('r_lback') = lmap('r_uback');
      lmap('l_uback') = lmap('r_uback');
      lmap('l_lback') = lmap('r_uback');
      lmap('back') = lmap('r_uback');
      
      lmap('r_fpelvis') = obj.robot.findLinkId('pelvis');
      lmap('r_bpelvis') = obj.robot.findLinkId('pelvis');
      lmap('l_fpelvis') = obj.robot.findLinkId('pelvis');
      lmap('l_bpelvis') = obj.robot.findLinkId('pelvis');
      lmap('pelvis') = obj.robot.findLinkId('pelvis');
    end
    
    
    % add spheres representing all the contact points on the robot
    function robot = addVisualContactPoints(obj,robot)
      if nargin < 2
        robot = obj.robot;
      end
      
      keySet = keys(obj.c);
      for j=1:length(keySet)
        name = keySet{j};
        link = obj.linkId(name);
        pts = obj.c(name);
        
        for k = 1:size(pts,2)
          sphere =  RigidBodySphere(0.01,pts(:,k),[0;0;0]);
          robot = robot.addVisualGeometryToBody(link,sphere);
        end
      end
      robot = compile(robot);
    end
    
    
    % helper method to add all the collision geometries to the robot
    function robot = addCollisionGeometryToRobot(obj,robot)
      if nargin < 2
        robot = obj.robot;
      end
      
      keySet = keys(obj.c);
      for j=1:length(keySet)
        name = keySet{j};
        link = obj.linkId(name);
        pts = obj.c(name);
        
        for k = 1:size(pts,2)
          sphere =  RigidBodySphere(0,pts(:,k),[0;0;0]);
          robot = robot.addCollisionGeometryToBody(link,sphere);
        end
      end
      robot = compile(robot);
    end
    
    % this is a helper method for use in the QPControllerContacts stuff
    function robot = addSpecifiedCollisionGeometryToRobot(obj,contacts)
      robot = obj.robot;
      for j = 1:numel(contacts)
        name = contacts{j};
        link = obj.linkId(name);
        pts = obj.c(name);
        
        for k = 1:size(pts,2)
          sphere = RigidBodySphere(0,pts(:,k),[0;0;0]);
          robot = robot.addCollisionGeometryToBody(link,sphere,name);
        end
        robot = compile(robot);
        obj.robot = robot;
      end
    end
    
    % helper function to generate the supports for a QPControllerData
    % structure
    function supports = genControllerDataSupport(obj,robot,q_sol)
      pose_struct = obj.pose_struct;
      supports = RigidBodySupportState.empty(0,0);;
      for j = 1:length(pose_struct)
        
        % construct the bodies array
        bodies = [];
        contact_groups = {};
        for k = 1:numel(pose_struct(j).qs_contacts)
          name = pose_struct(j).qs_contacts{k};
          bodies(k) = obj.linkId(name);
          
          % in my setup name is the same as the contact group
          contact_groups{k} = {name};
        end
        
        % construct a RBSS object and add it to the supports array
        supports(j) = RigidBodySupportState(robot,bodies,contact_groups);
      end
    end
    
    
    % generates a trajectory with the appropriate spacing
    function qtraj = genTrajectory(obj,q_sol,dt)
      if nargin < 3
        dt = 1;
      end
      t_span = [0:dt:dt*(obj.N-1)];
      qtraj = PPTrajectory(foh(t_span,q_sol));
    end
    
    
    
  end
  
  
  methods (Static)
    
    %         % The following three methods for constructing pose_struct need to
    %         % be static since we need pose_struct before we know how many
    %         % variables to have in Xinds/Nonlinear program but since
    %         % NonlinearProgram is also a super class we can't use the object
    %         % before we instantiate the object as part of that super class.
    function pose_struct = genPoseStruct(data_struct)
      
      pose_struct = struct('contacts',{},'qs_contacts',{},...
        'no_movement',{},'seed',{});
      
      
      % the case for idx = 1 is a bit special
      pose_struct(1).contacts = data_struct(1).contacts;
      pose_struct(1).no_movement = {};
      pose_struct(1).seed = data_struct(1).seed;
      pose_struct(1).qs_contacts = data_struct(1).contacts;
      
      
      M = numel(data_struct);
      if M < 2
        return;
      end
      
      for idx=2:M
        
        % contacts removed from idx - 1 to idx
        c_removed = setdiff(data_struct(idx-1).contacts,....
          data_struct(idx).contacts);
        % new contacts added at idx
        c_added = setdiff(data_struct(idx).contacts,...
          data_struct(idx-1).contacts);
        
        if ((~isempty(c_removed)) && (~isempty(c_added)))
          error('added and removed contacts at the same idx, cannot do both');
        end
        
        if (~isempty(c_removed))
          % you are removing a contact
          pose_struct = KinematicPoseTrajectory.removingSupport(pose_struct,data_struct,idx);
        else
          % if you get here you are adding a supports, or at
          % least not taking them away
          pose_struct = KinematicPoseTrajectory.addingSupport(pose_struct,data_struct,idx);
        end
      end
      
    end
    
    
    % updates pose_struct when data_struct(idx) is adding a support
    % relative to data_struct(idx - 1);
    function pose_struct = addingSupport(pose_struct,data_struct, idx)
      j = numel(pose_struct) + 1; % the current index in pose_struct
      
      pose_struct(j).contacts = data_struct(idx).contacts;
      
      % since we are adding contact pts use the contact pts of the
      % previous index for the quasi static constraint
      pose_struct(j).qs_contacts = data_struct(idx-1).contacts;
      
      % contacts that were active at the previous knot point will
      % still be active and thus can't have moved
      pose_struct(j).no_movement = data_struct(idx-1).contacts;
      
      pose_struct(j).seed = data_struct(idx).seed;
      
    end
    
    % updates pose_struct when data_struct(idx) is removing a support
    % relative to data_struct(idx-1).
    % Note we will have to add two elements to pose_struct in this case
    function pose_struct = removingSupport(pose_struct,data_struct,idx)
      
      j = numel(pose_struct)+1;
      
      % The first of the two knot points
      
      pose_struct(j).contacts = data_struct(idx-1).contacts;
      
      % For the quasi-static constraint only use the current contact
      % points
      pose_struct(j).qs_contacts = data_struct(idx).contacts;
      
      % For the no_movement constraint use the contact points of the
      % previous step
      pose_struct(j).no_movement = data_struct(idx-1).contacts;
      
      % **************************************
      % Use the seed of the previous pose
      pose_struct(j).seed = data_struct(idx-1).seed;
      
      
      
      % Add second of the two knot points
      j = numel(pose_struct)+1; % this should just be j+1 from before
      
      pose_struct(j).contacts = data_struct(idx).contacts;
      
      % for the quasi-static constraint use your actual contact
      % points
      pose_struct(j).qs_contacts = data_struct(idx).contacts;
      
      % Only the active contact points have no_movement constraints
      pose_struct(j).no_movement = data_struct(idx).contacts;
      
      pose_struct(j).seed = data_struct(idx).seed;
      
    end
    
  end
  
end
