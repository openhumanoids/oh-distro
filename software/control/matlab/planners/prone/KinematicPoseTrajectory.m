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
    c; % stores the contact points
    linkId; %stores linkId's of the contact points
    N; % number of knot points
    Xinds;
    shrink_factor = 0.8;
    nb;
    min_distance = 0.03;
    constraint_tol = 0;
    enforce_orientation_constraint = 0;
    allow_pelvis_collision = 0;
    pelvis_min_distance = 0.002;
    robotiq_weight_flag = false;
    contact_height; % (optional) a containers.Map specifying heights for specific contact points
  end
  
  
  methods


  function obj = KinematicPoseTrajectory(robot,data_struct)

      % want to be able to handle empty data_struct object
      
      if isa(robot,'Atlas')
        robot = robot.getManipulator();
      end
      
      if (nargin < 2 || isempty(data_struct))
        data_struct = struct('contacts',{},'seed',{});
        data_struct(1).contacts = {};
        nq = robot.getNumPositions();
        data_struct(1).seed = zeros(nq,1);
      end
      
      % instantiate with no vars and then add vars as needed once we
      % can call genPoseStruct();
      obj = obj@NonlinearProgram(0);
      % the zero is so that we don't incorrectly call getXinds()
      % which has yet to be properly set
      obj = obj@KinematicTrajectoryOptimization(robot,0);
      obj.contact_height = containers.Map();


      
      
      % these methods only applicable to RigidBodyManipulator, won't work
      % on a TimeSteppingRigidBodyManipulator
      if isa(robot,'RigidBodyManipulator')
        obj.robot.collision_filter_groups('ignores_ground') = CollisionFilterGroup();
        obj.robot.collision_filter_groups('ground') = CollisionFilterGroup();
        obj.robot = obj.robot.addLinksToCollisionFilterGroup('world','ground',0);
        obj.robot = obj.robot.addToIgnoredListOfCollisionFilterGroup('ignores_ground','ground');
        obj.robot = obj.robot.addToIgnoredListOfCollisionFilterGroup('ground','ignores_ground');
        if isempty(robot.terrain)
          warning('KinematicPoseTrajectory:Terrain','NO TERRAIN: your robot does not have a terrain, consider adding a RigidBodyFlatTerrain');
        end
      end
      obj.robot = compile(obj.robot);
      
      
      % remove duplicates from the data struct
      obj.data_struct = data_struct;
      obj = obj.removeDuplicatesFromDataStruct();
      
      
      % first need to determine the number of knot points, so before
      % anything else we need to generate pose_struct without using
      % the object. So we must use Static methods to do this
      obj.pose_struct = KinematicPoseTrajectory.genPoseStruct(obj.data_struct);
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
      % obj.data_struct = data_struct;
      
      % use the hand guards by default;
      obj = obj.useHandGuards();
      
      % add the collision geometries to the robot
      %       obj.robot = obj.addCollisionGeometryToRobot();
      
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
      
      for j = 1:numel(contacts)
        name = contacts{j};
        id = obj.linkId(name);
        pts = obj.c(name);
        s = size(pts);

        if isKey(obj.contact_height,name)
          height = obj.contact_height(name);
        else
          height = 0;
        end
        bds = [nan;nan;height];
        bds_resize = repmat(bds,1,s(2));
        lb = bds_resize - obj.constraint_tol*ones(size(bds_resize));
        ub = bds_resize + obj.constraint_tol*ones(size(bds_resize));
        wp = WorldPositionConstraint(obj.robot,id,pts,lb,ub);
        constraints{end+1} = wp;
        % adds the WorldPositionConstraint to the obj, which is
        % also of class NonlinearProgram
        if add_to_obj
          obj = obj.addRigidBodyConstraint(wp,time_index);
        end
      end
    end
    
    % A constraint that ensure that all the contact points are above the
    % plane z = 0, this can replace the collision with ground constraint,
    % may not be as good, but also may be faster and better conditioned???
    % Actually no, this doesn't work very well at all, the collision
    % constraints are much better
    function [obj, constraints] = addSingleAboveGroundConstraint(obj,contacts,time_index,add_to_obj)

      % by default add above ground constraint for all the contacts on the
      % robot that aren't suppose to be on the ground at this time step
      
      contact_linkId = [];
      
      for j =1:numel(obj.pose_struct(time_index).contacts)
        contact_linkId(end+1) = obj.linkId(obj.pose_struct(time_index).contacts{j});
      end
      
      
      % problem foot is still being checked against ground even though
      % 'l_foot' is down, need logic that consider bodies/links rather than
      % just contact names
      if isempty(contacts)
        contacts = {};
        Names = keys(obj.c);
        for j = 1:numel(Names)
          id = obj.linkId(Names{j});
          % check to see if it's linkId is the same as that of a body that
          % is going to be subject to a contact constraint
          if ~any(id == contact_linkId)
            contacts{end+1} = Names{j};
          end
        end
      end
      
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
      
      lb = [nan;nan;0];
      ub = [nan;nan;Inf];
      for j = 1:numel(contacts)
        name = contacts{j};
        id = obj.linkId(name);
        pts = obj.c(name);
        s = size(pts);
        lb_resize = repmat(lb,1,s(2));
        ub_resize = repmat(ub,1,s(2));
        wp = WorldPositionConstraint(obj.robot,id,pts,lb_resize,ub_resize);
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
        wfp = wfp.setTol(obj.constraint_tol);
        
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
      contacts = obj.pose_struct(time_index).contacts;
      constraints = obj.genSingleCollisionConstraint(contacts);
      
      for j = 1:numel(constraints)
        obj = obj.addRigidBodyConstraint(constraints{j},time_index);
      end
    end
    
    
    % generates a single collision constraint and returns is
    % allow for pelvis collision options
    function constraints = genSingleCollisionConstraint(obj,contacts)
      not_collision_checked = [];
      for j = 1:numel(contacts)
        not_collision_checked(end+1) = obj.linkId(contacts{j});
      end
      
      % This is the standard collision constraint, doesn't include pelvis - leg collisions
      robot = obj.robot.addLinksToCollisionFilterGroup(not_collision_checked,'ignores_ground',1);
      robot = compile(robot);
      
      % the bodies that are contacts don't need to be collision
      % checked for when we are including the world, i.e. the terrain
      mdc = MinDistanceConstraint(robot,obj.min_distance);
      constraints = {};
      constraints{end+1} = mdc;
      
      
      % Now do the collision between pelvis and uleg
      if obj.allow_pelvis_collision
        % need to create a new robot that allows collisions between l_uleg and pelvis
        robot = obj.allowPelvisCollisionRobot();
        active_collision_options.body_idx = ...
        [robot.findLinkId('l_uleg'),robot.findLinkId('r_uleg'),robot.findLinkId('pelvis')];
        mdc_pelvis = MinDistanceConstraint(robot,obj.pelvis_min_distance,active_collision_options);
        constraints{end+1} = mdc_pelvis;
      end
    end
    
    
    
    % this can also be handled by addRigidBodyConstraint in the
    % KinematicTrajectoryOptimization class
    function [obj, constraints] = addSingleQuasiStaticConstraint(obj,qs_contacts,time_index,add_to_obj,shrink_factor)

      % flag that controls whether or not constraint should be added
      % to the object or not, 1 means it IS added to the object
      if nargin < 4
        add_to_obj = 1;
      end
      
      if nargin < 3
        warning('Added a QS constraint without specifying a time_index, setting it to 1 by default')
        time_index = 1;
      end
      
      if nargin < 5
        shrink_factor = obj.shrink_factor;
      end
      
      
      qs =QuasiStaticConstraint(obj.robot);
      qs = qs.setShrinkFactor(shrink_factor);
      
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
    function obj = addSinglePoseCost(obj,time_index,q_nom,scale)

      % allow for scaling of the weighting matrix
      if nargin < 4
        scale = 1;
      end
      
      if nargin < 3
        q_nom = obj.pose_struct(time_index).seed;
      end
      
      Q = scale*eye(obj.nq); % diagonal matrix
      % set the first 6 elements of this to be zero, since we don't
      % care about penalizing xyz or rpy???
      Q(1:6,1:6) = zeros(6,6);
      
      b = -Q*q_nom;
      quad_constraint = QuadraticConstraint(0,0,Q,b);
      
      % add this as a cost to the nonlinear program
      obj = obj.addCost(quad_constraint,obj.Xinds(:,time_index));
      
    end
    
    % adds a cost on the velocity, basically penalizes the difference
    % between knot points
    function obj = addSingleVelocityCost(obj,time_index,scale)
      if nargin < 3
        scale = 1;
      end
      
      % T just serves to take the difference between q at the different
      % time steps
      T = [eye(obj.nq),-eye(obj.nq)];
      Q = scale*eye(obj.nq);
      b = zeros(2*obj.nq,1);
      
      Q_cost = T'*Q*T; % this produces a cost of the form (q_t - q_{t+1})'*Q*(q_t - q_{t+1})
      quad_constraint = QuadraticConstraint(0,0,Q_cost,b);
      obj = obj.addCost(quad_constraint,[obj.Xinds(:,time_index - 1), obj.Xinds(:,time_index)]);
    end
    
    % just a helper method to add all the costs from above
    function obj = addVelocityCosts(obj,scale)
      if nargin < 2
        scale = 1;
      end
      
      for j = 2:obj.N
        obj = obj.addSingleVelocityCost(j,scale);
      end
    end
    
    % CHANGED THIS TO A SYMMETRY CONSTRAINT INSTEAD OF SYMMETRY COST . . .
    function [obj,quad_constraint] = addSingleSymmetryCost(obj,time_index,Q,scale)
      if nargin < 4
        scale = 1;
      end
      b = zeros(obj.nq,1);
      quad_constraint = QuadraticConstraint(-1,1,Q,b);
      obj = obj.addConstraint(quad_constraint,obj.Xinds(:,time_index));
    end
    
    % helper method to add all symmetry costs
    function obj = addSymmetryCosts(obj,Q,scale)
      if nargin < 3;
        scale = 1;
      end
      for j = 1:obj.N
        obj = obj.addSingleSymmetryCost(j,Q,scale);
      end
    end
    
    % adds a symmetry constraint on the joints, need to pass in the correct
    % matrix A
    function [obj,symm_constraint] = addSingleSymmetryConstraint(obj,time_index,A,scale,tol)
      if nargin < 4
        scale = 1;
      end
      
      if nargin < 5
        tol = 0.1;
      end
      
      b = zeros(obj.nq,1);
      tol = 0.1;
      lb = -tol*ones(obj.nq,1);
      ub = tol*ones(obj.nq,1);
      symm_constraint = LinearConstraint(lb,ub,A);
      obj = obj.addConstraint(symm_constraint,obj.Xinds(:,time_index));
    end
    
    function obj = addSymmetryConstraints(obj,A,scale,tol)
      if nargin < 3;
        scale = 1;
      end
      
      if nargin < 4
        tol = 0.1;
      end
      
      for j = 1:obj.N
        obj = obj.addSingleSymmetryConstraint(j,A,scale,tol);
      end
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
    
    function obj = addPoseConstraint(obj,time_index,q_nom,idx)

      if nargin < 4
        idx = [1:obj.robot.getNumPositions];
      end
      lb = q_nom(idx) - obj.constraint_tol*ones(size(q_nom(idx)));
      ub = q_nom(idx) + obj.constraint_tol*ones(size(q_nom(idx)));
      bbc = BoundingBoxConstraint(lb,ub);
      % calling method from NonlinearProgram superclass
      obj = obj.addBoundingBoxConstraint(bbc,obj.Xinds(idx,time_index));
    end
    
    function obj = addAllConstraints(obj,options)

      if nargin < 2
        options = struct();
      end
      
      % Need to take special care with j = 1, it doesn't have a movement constraint
      obj = obj.addSingleContactConstraint(obj.pose_struct(1).contacts,1);
      obj = obj.addSingleQuasiStaticConstraint(obj.pose_struct(1).qs_contacts,1);
      obj = obj.addSingleCollisionConstraint(1);
      
      for j = 2:numel(obj.pose_struct)
        obj = obj.addSingleContactConstraint(obj.pose_struct(j).contacts,j);
        obj = obj.addSingleMovementConstraint(obj.pose_struct(j).no_movement,j);
        obj = obj.addSingleQuasiStaticConstraint(obj.pose_struct(j).qs_contacts,j);
        obj = obj.addSingleCollisionConstraint(j);
      end
      
      % if specified in the options add the above ground constraints
      if isfield(options,'above_ground')
        if options.above_ground
          for j = 1:numel(obj.pose_struct)
            % if we don't pass contacts explicityly it will add the above
            % ground constraint for all of them.
            obj = obj.addSingleAboveGroundConstraint({},j);
          end
          if ~isempty(obj.robot.terrain)
            warning('REDUNTANT CONSTRAINTS: added above ground constraint and also have terrain');
          end
        end
      end
    end
    
    % Note: this should only be called once since we have been adding
    % variables to the nonlinear program object all along, have a
    % solved_flag and throw an error if you try to call solve again???
    function [q_sol,exitflag,infeasible_constraint_name, obj, x] = solve(obj,options)
      if nargin < 2
        options = struct();
      end
      obj = obj.addAllConstraints(options);
      obj = obj.addRunningPoseCost();
      
      
      % option for allowing addition of a velocity cost
      if isfield(options,'velocity_cost')
        if isfield(options.velocity_cost,'scale')
          scale = options.velocity_cost.scale;
        else
          scale = 1;
        end
        obj = obj.addVelocityCosts(scale);
      end
      
      
      % option for allowing the addition of a symmetry cost
      if isfield(options,'symmetry_cost')
        if isfield(options.symmetry_cost,'scale')
          scale = options.symmetry_cost.scale;
        else
          scale = 1;
        end
        obj = obj.addSymmetryCosts(options.symmetry_cost.Q,scale);
      end
      
      % option for allowing the addition of a symmetry cost
      if isfield(options,'symmetry_constraint')
        if isfield(options.symmetry_constraint,'scale')
          scale = options.symmetry_constraint.scale;
        else
          scale = 1;
        end
        
        if isfield(options.symmetry_constraint,'tol')
          tol = options.symmetry_constraint.tol;
        else
          tol = 0.1;
        end
        obj = obj.addSymmetryConstraints(options.symmetry_constraint.A,scale,tol);
      end
      
      if isfield(options,'relax_constraints')
        if isfield(options.relax_constraints,'tol')
          tol = options.relax_constraint.tol;
        else
          tol =1e-4;
        end
      end
      
      
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
      t_grid = [1:size(q_sol,2)];
      qtraj = PPTrajectory(foh(t_grid,q_sol));
      
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
    
    % generate a single IK collision constraint
    function [constraints, mdc, robot] = addSingleIKCollisionConstraint(obj,time_index,constraints,collision_check_type)
      not_collision_checked = [];
      % the bodies in contact with the ground between time_index-1 and
      % time_index are pose_struct(time_index).no_movement, modify the
      % below from addSingleCollisionConstraint accordingly
      for j = 1:numel(obj.pose_struct(time_index).no_movement)
        not_collision_checked(end+1) = obj.linkId(obj.pose_struct(time_index).no_movement{j});
      end
      
      robot = obj.robot.addLinksToCollisionFilterGroup(not_collision_checked,'ignores_ground',1);
      robot = compile(robot);
      
      % update the collision geometries on robot depending on
      % collision_check_type
      switch collision_check_type
      case 0
        robot = robot.removeCollisionGroupsExcept({});
      case 1
        robot = obj.removeOtherCollisionGeometries(robot);

          % case 2 leaves everything as is, i.e. full collision checking
        end
        robot = compile(robot);
        mdc = MinDistanceConstraint(robot,obj.min_distance);
        constraints{end+1} = mdc;
      end

    % generate the appropriate constraints to pass to IK pointwise
    % DEPRECATED
    function constraints = genIKConstraints(obj,q_sol)
      constraints = {};
      for j = 2:obj.N
        constraints = obj.addSingleIKQuasiStaticConstraint(j,constraints,q_sol);
        constraints = obj.addSingleIKMovementConstraint(j,constraints,q_sol);
      end
    end
    
    % should only pass this time_index between 2 and obj.N
    function [q,t,infeasible_constraint,ifc_idx] = solveIKInterval(obj,q_sol,time_index,num_pts,collision_check_type)
      obj.robot = compile(obj.robot);
      % t_grid not including the endpoints
      t = setdiff(linspace(time_index - 1, time_index,num_pts+2),[time_index - 1,time_index]);
      qtraj = PPTrajectory(foh([1:obj.N],q_sol));
      % generate the correct constraints
      constraints = {};
      constraints = obj.addSingleIKQuasiStaticConstraint(time_index,constraints,q_sol);
      constraints = obj.addSingleIKMovementConstraint(time_index,constraints,q_sol);
      
      [~,mdc,robot] = obj.addSingleIKCollisionConstraint(time_index,constraints,collision_check_type);
      
      
      q = [];
      info = [];
      infeasible_constraint = [];
      
      % need to iterate through the loop for each time to see if we need to
      % add the min distance constraint or not.
      for j = 1:length(t)
        q_nom = qtraj.eval(t(j));
        q_seed = q_nom;
        q_seed(3) = q_seed(3); % move the robot "up" a bit
        constraints_temp = constraints;
        [q_temp,info_temp,infeasible_constraint_temp] = obj.robot.inverseKinPointwise(t(j),...
          q_seed,q_nom,constraints_temp{:});
        
        % if the min distacne constraint isn't satisfied then we need to resolve with it added in
        kinsol = robot.doKinematics(q_temp);
        if (mdc.eval(0,kinsol) ~= 0 && collision_check_type ~= 0)
          disp('min distance constraint was not satifsied, adding and re-solving')
          constraints_temp{end+1} = mdc;
          %           [q_temp,info_temp,infeasible_constraint_temp] = obj.robot.inverseKinPointwise(t(j),...
          %             q_seed,q_nom,constraints_temp{:});


          % if we need collision constraints switch to using
          % InverseKinematics, it is much more robust than
          % inverKinPointwise
          ik = InverseKinematics(obj.robot,q_nom,constraints_temp{:});
          ik = ik.setSolverOptions('snopt','iterationslimit',1e6);
          ik = ik.setSolverOptions('snopt','majoriterationslimit',200);
          ik = ik.setSolverOptions('snopt','print','snopt.out');
          [q_temp,~,info_temp,infeasible_constraint_temp] = ik.solve(q_seed);
          
          % manually check whether mdc satisfied
          if (mdc.eval(0,robot.doKinematics(q_temp)) < 1e-4)
            info_temp = -1;
          end
          
        end
        
        q = [q,q_temp];
        info = [info,info_temp];
        infeasible_constraint = [infeasible_constraint,infeasible_constraint_temp];
        
        mdc_val = mdc.eval(0,robot.doKinematics(q_temp));
        fprintf('evaluating the mdc after both solves gives %d \n',mdc_val)
      end
      
      % modify this to do it one at a time and redo it if the min distance
      % constraint was satisfied/
      
      % print out if there are infeasible_constraints
      info
      infeasible_constraint;
      ifc_idx = find(info > 10);
    end


    function [q,info,infeasible_constraint] = inverseKin(obj,q_seed,options)

      if nargin < 3
        options = struct();
      end

      if isfield(options,'constraints')
        constraints = options.constraints;
      else
        constraints = {};
      end

      if ~isfield(options,'shrink_factor'), options.shrink_factor = obj.shrink_factor; end;

      if isfield(options,'qs_contacts')
        [~, qs_constraints] = addSingleQuasiStaticConstraint(obj,options.qs_contacts,1,0,options.shrink_factor);
        constraints = [constraints, qs_constraints];
      end

      if isfield(options,'no_movement')
        q_nm = options.no_movement.q;
        kinsol = obj.robot.doKinematics(q_nm);
        nm_bodies = options.no_movement.bodies;
        pt = [1,-1,0;0,0,1;0,0,0];

        for j = 1:numel(nm_bodies)
          name = nm_bodies{j};
          pos = obj.robot.forwardKin(kinsol,obj.linkId(name),pt);
          constraints{end+1} = WorldPositionConstraint(obj.robot,obj.linkId(name),pt,pos,pos);
        end
      end

      if isfield(options,'height')
        for j = 1:numel(options.height.names)
          name = options.height.names{j};
          h = options.height.heights{j};
          bds = [nan;nan;h];
          bds = repmat(bds,1,size(obj.c(name),2));
          constraints{end+1} = WorldPositionConstraint(obj.robot,obj.linkId(name),obj.c(name),bds,bds);
        end
      end

      if ~isfield(options,'IKoptions'), options.IKoptions = IKoptions(obj.robot); end

      if isfield(options,'Q')
        options.IKoptions.setQ(options.Q);
      else
        options.Q = eye(obj.robot.getNumPositions);
      end

      if isfield(options,'use_mex') && options.use_mex
        disp('using mex')
        [q,info,infeasible_constraint] = inverseKin(obj.robot,q_seed,q_seed,constraints{:},options.IKoptions);
      else
        ik = InverseKinematics(obj.robot,q_seed,constraints{:});
        ik = ik.setQ(options.Q);
        [q,~,info,infeasible_constraint] = ik.solve(q_seed);
      end
    end
    
    function [q_ik,t_grid, infeasible_constraint, ifc_idx] = inverseKinPointwise_2(obj,q_sol,num_pts,collision_check_type)

      % this specificies the amount of collision checking to be done.
      % 0 - no collision checks
      % 1 - only collision checks on the "new" collision bodies
      % 2 - all collision checks
      if nargin < 4
        collision_check_type = 0;
      end
      
      q_ik = [];
      t_grid = [];
      
      idx = 0;
      infeasible_constraint = {};
      ifc_idx = [];
      for time_index=2:obj.N
        fprintf('currently computing for knot point %d',time_index);
        [q,t,ifc,idx_interval] = obj.solveIKInterval(q_sol,time_index,num_pts,collision_check_type);
        q_ik = [q_ik,q];
        t_grid = [t_grid,t];
        
        % this is for keeping track of indices where the constraints were
        % violated.
        idx_interval = idx_interval + idx;
        ifc_idx = [ifc_idx,idx_interval];
        infeasible_constraint = [infeasible_constraint,ifc];
        idx = idx + length(t);
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

function constraints = genStaticNoMovementConstraint(obj,contacts,q_nom)
  kinsol = obj.robot.doKinematics(q_nom);
  constraints = {};
  for j = 1:numel(contacts)
    name = contacts{j};
    id = obj.linkId(name);
    pts = obj.c(name);
    bds = obj.robot.forwardKin(kinsol,id,pts);
    lb = bds - obj.constraint_tol*ones(size(bds));
    ub = bds + obj.constraint_tol*ones(size(bds));
    wpc = WorldPositionConstraint(obj.robot,id,pts,bds,bds);
    constraints{end+1} = wpc;
  end
end

    % constructs IK object for finding a quasistatic pose with
    % specified contact points and position costs on certain end
    % effector links
    function [q,F,info,infeasible_constraint,ik] = findPose(obj,contacts,position_cost,q_nom,options)
      % @param position_cost  -- a structure array with fields 'name'
      % which is the name of the contact point, i.e. 'l_toe' and the
      % desired position which is a 3x1 pt in the field 'position'
      % generate the correct constraints and costs
      % @options an optinal argument that allows you to specify the qs
      % contacts if you wish.
      
      % add support for no movement constraints
      
      if nargin < 5
        options = struct();
      end

      % this should be a container map
      if isfield(options,'contact_height')
        obj.contact_height = contact_height;
      end

      if isfield(options,'shrink_factor')
        shrink_factor = options.shrink_factor;
      else
        shrink_factor = obj.shrink_factor;
      end
      
      if isfield(options,'qs_contacts')
        qs_contacts = options.qs_contacts;
      else
        qs_contacts = contacts;
      end
      
      if isfield(options,'enforce_collision')
        enforce_collision = options.enforce_collision;
      else
        enforce_collision = 1;
      end
      
      if isfield(options,'enforce_quasistatic')
        enforce_quasistatic = options.enforce_quasistatic;
      else
        enforce_quasistatic = 1;
      end
      
      if isfield(options,'enforce_contact')
        enforce_contact = options.enforce_contact;
      else
        enforce_contact = 1;
      end
      
      if enforce_contact
        [~,contact_constraints] = obj.addSingleContactConstraint(contacts,1, 0);
      else
        contact_constraints = {};
      end
      
      if enforce_collision
        collision_constraints = obj.genSingleCollisionConstraint(contacts);
      else
        collision_constraints = {};
      end
      
      if enforce_quasistatic
        [~,qs_constraints] = obj.addSingleQuasiStaticConstraint(qs_contacts,1,0,shrink_factor);
      else
        qs_constraints = {};
      end
      
      constraints = [contact_constraints, collision_constraints,...
      qs_constraints];
      
      % add any addition constraints that were passed in
      if isfield(options,'constraints')
        constraints = [constraints,options.constraints];
      end

      if isfield(options,'no_movement')
        no_movement_constraints = obj.genStaticNoMovementConstraint(...
          options.no_movement.contacts,options.no_movement.q);
        constraints = [constraints,no_movement_constraints];
      end
      
      
      % test for quaternion stuff, don't include the quaternion constraints
      %       constraints = [contact_constraints,...
      %         qs_constraints];
      
      ik = InverseKinematics(obj.robot,q_nom,constraints{:});
      ik = ik.setSolverOptions('snopt','iterationslimit',1e6);
      ik = ik.setSolverOptions('snopt','majoriterationslimit',200);
      ik = ik.setSolverOptions('snopt','print','snopt.out');

      if isfield(options,'Q')
        ik = ik.setQ(options.Q);
      end
      
      if ~isempty(position_cost)
        ik = obj.addContactPositionCost(ik,position_cost);
      end
      
      if isfield(options,'bbc')
        bbc = BoundingBoxConstraint(options.bbc.lb,options.bbc.ub);
        ik = ik.addBoundingBoxConstraint(bbc,ik.q_idx(options.bbc.idx));
      end
      
      if isfield(options,'symmetry_cost')
        if isfield(options.symmetry_cost,'scale')
          scale = options.symmetry_cost.scale;
        else
          scale = 1;
        end
        [~,symm_cost] = obj.addSingleSymmetryCost(1,options.symmetry_cost.Q,scale);
        symm_cost = symm_cost.setBounds(-0.5,0.5);
        ik = ik.addConstraint(symm_cost,ik.q_idx);
      end
      
      if isfield(options,'symmetry_constraint')
        if isfield(options.symmetry_constraint,'scale')
          scale = options.symmetry_constraint.scale;
        else
          scale = 1;
        end
        [~,symm_constraint] = obj.addSingleSymmetryConstraint(1,options.symmetry_constraint.A,scale);
        %         symm_constraint = symm_constraint.setBounds(-0.5,0.5);
        ik = ik.addConstraint(symm_constraint,ik.q_idx);
      end
      
      if isfield(options,'use_mex')
        use_mex = options.use_mex;
      else
        use_mex = 0;
      end
      
      % use the cpp implementation of inverse kinematics if specified, otherwise use regular
      % InverseKinematics
      if use_mex
        % need to include a hand above ground constraint . . .
        tol = 0.02;
        
        % loop thru position cost to add constraints on the hand positions . . .
        for j = 1:numel(position_cost)
          name = position_cost(j).name;
          pts = obj.c(name);
          pos = position_cost(j).position; % needs to have same number of points as on the body
          
          if ~sizecheck(position,size(pts))
            error('DRC:KINEMATICPOSETRAJECOTRY','position_cost.position and pts on body must have same dimension')
          end
          
          lb = pos - tol*ones(size(pos));
          ub = pos + tol*ones(size(pos));
          wpc = WorldPositionConstraint(robot,obj.linkId(name),pts,lb,ub);
          constraints{end+1} = wpc;
        end
        F = 0; ik = {};
        ik_options = IKoptions(obj.robot);
        ik_options = ik_options.setMajorFeasibilityTolerance(1e-4);
        disp('solving using inverseKin cpp method')
        [q,info,infeasible_constraint] = obj.robot.inverseKin(q_nom,q_nom,constraints{:},ik_options);
      elseif isfield(options,'dont_solve')
        q = 0; F = 0; info = 0; infeasible_constraint = 0;
      else
        [q,F,info,infeasible_constraint] = ik.solve(q_nom);
      end
      
    end
    
    function [q,qtraj,info,infeasible_constraint] = inverseKinTraj(obj,qtraj,tspan,qs_contacts,options)
      % Plot a trajectory to follow qtraj.
      % Enforce fixed initial and final position
      % Use 'contacts' as the quasi-static contacts
      % Maybe also enfore above ground constraints for all contact points????


      if isfield(options,'use_mex')
        use_mex = options.use_mex;
      else
        use_mex = 1;
      end

      if isfield(options,'shrink_factor')
        shrink_factor = options.shrink_factor;
      else
        shrink_factor = obj.shrink_factor;
      end

      t_grid = linspace(tspan(1),tspan(2),5);
      t_grid = t_grid(2:end);
      [~,qs_constraints] = obj.addSingleQuasiStaticConstraint(qs_contacts,1,0,shrink_factor)
      no_movement_constraints = obj.genStaticNoMovementConstraint(qs_contacts,qtraj.eval(tspan(1)));
      constraints = [qs_constraints, no_movement_constraints];
      ik_options = IKoptions(obj.robot);
      ik_options = ik_options.setMajorFeasibilityTolerance(1e-4);
      ik_options = ik_options.setSequentialSeedFlag(true);

      % add final pose constraint
      % nq = obj.robot.getNumPositions;
      % t = t_grid(end);
      % final_pose_constraint = PostureConstraint(obj.robot,[t,t]);
      % q = qtraj.eval(t);
      % lb = q - 2*obj.constraint_tol*ones(size(q));
      % ub = q + 2*obj.constraint_tol*ones(size(q));
      % final_pose_constraint = final_pose_constraint.setJointLimits([1:nq]',lb,ub);
      % constraints = [constraints,{final_pose_constraint}];

      q_seed = qtraj.eval(t_grid);
      q_nom = q_seed;
      [q,info,infeasible_constraint] = obj.robot.inverseKinPointwise(t_grid,q_seed,q_nom,constraints{:},ik_options);
      qtraj = PPTrajectory(pchip(t_grid,q));

      % q_init = qtraj.eval(tspan(1));
      % q_final = qtraj.eval(tspan(2));
      % t_grid = linspace(tspan(1),tspan(2),5);
      % constraints = {};
      % % generate no movement and quasistatic constraints
      % [~, qs_constraint] = obj.addSingleQuasiStaticConstraint(qs_contacts,1,0,shrink_factor);
      % movement_constraints = genStaticNoMovementConstraint(obj,qs_contacts,q_init);

      % nq = obj.robot.getNumPositions;
      % t = t_grid(1);
      % initial_pose_constraint = PostureConstraint(obj.robot,[t,t]);
      % q = qtraj.eval(t);
      % initial_pose_constraint = initial_pose_constraint.setJointLimits([1:nq]',q,q);

      % t = t_grid(end);
      % q = qtraj.eval(t);
      % final_pose_constraint = PostureConstraint(obj.robot,[t,t]);
      % final_pose_constraint = final_pose_constraint.setJointLimits([1:nq]',q,q);
      % pose_constraints = {initial_pose_constraint,final_pose_constraint};
      % constraints = [qs_constraint,movement_constraints,pose_constraints];

      % if use_mex == 1;
      %   ik_options = IKoptions(obj.robot);
      %   ik_options = ik_options.setMex(true);
      %   [xtraj,info,infeasible_constraint]= obj.robot.inverseKinTraj(t_grid,qtraj,qtraj,ik_options);

      % elseif use_mex == -2
      %   q_seed = qtraj.eval(t_grid);
      %   q_nom = q_seed;
      %   ik_options = IKoptions(obj.robot);
      %   ik_options = ik_options.setMex(true);
      %   % consider setting the sequential seed flag here . . . 
      %   [q,info,infeasible_constraint] = ...
      %   obj.robot.inverseKinPointwise(t_grid,q_seed,q_nom,constraints{:},ik_options);
      %   F = 0;
      %   qtraj = PPTrajectory(pchip(t_grid,q));
      %   xtraj = qtraj;
      % % try to have a version that uses InverseKinematicsTrajectory to compare performance
      % elseif use_mex == -1
      %   N = numel(t_grid);
      %   time_index_all = num2cell([1:N]);
      %   duration = 4;
      %   kin_dirtran = KinematicDritran(obj.robot,N,duration);
      %   for j = 1:numel(constraints)
      %     kin_dirtran = kin_dirtran.addConstraint(constraints{j},time_index_all);
      %   end
      %   [xtraj,z,F,info] = solveTraj(obj,duration,qtraj);
      %   infeasible_constraint = {};
      % else
      %   t_grid = linspace(tspan(1),tspan(2),5);
      %   N = numel(t_grid);
      %   time_index_all = num2cell([1:N]);
      %   x0 = [q_init;0*q_init];
      %   ik_traj = InverseKinematicsTrajectory(obj.robot,t_grid,qtraj,1,x0,constraints{:});
      %   % add all the constraints
      %   % solve
      %   [xtraj,F,info,infeasible_constraint] = ik_traj.solve(qtraj);
      % end
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
        pts = obj.c(name);
        pos = position_cost(j).position;
        num_pts = size(pts,2);
        
        % if they aren't the same size then resize pos to the appropriate dimensions
        % this allows the option of passing in pos which specifies the desired positions of
        % all the points
        if isequal(size(pos),size(pts))
          pts_desired = pos;
        else
          pts_desired = repmat(pos,1,num_pts); %
        end
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
    
    % adds knot points to smooth trajectory and make sure we make/break
    % contact precisely
    function [ik_struct,idx] = smoothContacts(obj,sol_struct,options)
      ik_struct = struct('contacts',{},'no_movement',{},'qs_contacts',{},'q',{});
      idx = {};
      if nargin < 3
        options = struct();
      end

      if ~isfield(options,'enforce_collision')
        options.enforce_collision = 1;
      end

      if ~isfield(options,'above_ground_constraint_flag')
        options.above_ground_constraint_flag = 0;
      end

      if ~isfield(options,'scale')
        options.scale = 1;
      end

      ik_struct(1).contacts = sol_struct(1).contacts;
      ik_struct(1).qs_contacts = sol_struct(1).qs_contacts;
      ik_struct(1).no_movement = sol_struct(1).no_movement;
      ik_struct(1).q = sol_struct(1).q;
      for j = 2:numel(sol_struct);
        
        % this means we are adding contacts
        if ~isempty(setdiff(sol_struct(j).contacts,...
          sol_struct(j-1).contacts))

          new_contact = setdiff(sol_struct(j).contacts,...
            sol_struct(j-1).contacts);
          contacts = sol_struct(j-1).contacts;
          q0 = sol_struct(j).q;
          k = numel(ik_struct)+1;
          fprintf('sol_struct idx is %d \n', j);
          fprintf('ik_struct idx is %d \n',k)

          [q,info,infeasible_constraint] = obj.contactAboveGround(q0,contacts,new_contact,options);

          ik_struct(k).contacts = contacts;
          ik_struct(k).no_movement = contacts;
          ik_struct(k).qs_contacts = contacts;
          ik_struct(k).q = q;
          % update the container map with the right indices
          idx{end+1} = struct('ik_idx',k','sol_idx',j,'adding_support',1);
        end
        
        % this means we are removing contacts
        if ~isempty(setdiff(sol_struct(j-1).contacts,...
          sol_struct(j).contacts))

          new_contact = setdiff(sol_struct(j-1).contacts,...
            sol_struct(j).contacts);
          contacts = sol_struct(j).contacts;
          q0 = sol_struct(j-1).q;
          k = numel(ik_struct)+1;

          fprintf('sol_struct idx is %d \n', j);
          fprintf('ik_struct idx is %d \n',k)
          [q,info,infeasible_constraint] = obj.contactAboveGround(q0,contacts,new_contact,options);
          
          ik_struct(k).contacts = contacts;
          ik_struct(k).no_movement = contacts;
          ik_struct(k).qs_contacts = contacts;
          ik_struct(k).q = q;
          % update the container map with the right indices
          idx{end+1} = struct('ik_idx',k','sol_idx',j,'adding_support',1);
        end
        
        % Always want to keep the originals as well
        k = numel(ik_struct)+1;
        ik_struct(k).q = sol_struct(j).q;
        ik_struct(k).contacts = sol_struct(j).contacts;
        ik_struct(k).qs_contacts = sol_struct(j).qs_contacts;
        ik_struct(k).no_movement = sol_struct(j).no_movement;
      end
      
    end

    function [q,info,infeasible_constraint] = contactAboveGround(obj,q0,contacts,new_contact,options)

      options.no_movement.contacts = contacts;
      options.no_movement.q = q0;

      if isfield(options,'above_ground_constraint_flag')
        above_ground_constraint_flag = options.above_ground_constraint_flag;
      else
        above_ground_constraint_flag = 0;
      end

      if isfield(options,'scale')
        scale = options.scale;
      else
        scale = 1;
      end

      if isfield(options,'constraints')
        constraints = options.constraints;
      else
        constraints = {};
      end

      if isfield(options,'position_cost_flag')
        position_cost_flag = options.position_cost_flag;
      else
        position_cost_flag = 1;
      end

      kinsol = obj.robot.doKinematics(q0);
      for i = 1:numel(new_contact)

        % add position cost if flag is set to true
        if position_cost_flag
          name = new_contact{i};
          position_cost(i).name = name;
          pos = obj.robot.forwardKin(kinsol,obj.linkId(name),obj.c(name));
          pos(3,:) = pos(3,:) + 0.1;
          position_cost(i).position = pos;
          position_cost(i).scale = scale;
        end

        % add an above ground constraint if flag is set to true
        if above_ground_constraint_flag
          lb = [nan;nan;0.02];
          lb = repmat(lb,1,size(pos,2));
          ub = [nan;nan;Inf];
          ub = repmat(ub,1,size(pos,2));
          wpc = WorldPositionConstraint(obj.robot,obj.linkId(name),obj.c(name),lb,ub);
          constraints{end+1} = wpc;
        end
      end

      [q,F,info,infeasible_constraint] = ...
      obj.findPose(contacts,position_cost,q0,options);

      info
      infeasible_constraint
    end

    % v_max_idx captures the joint indices at which the velocity hits its limits
    function [qtraj_smooth,new_times,v_max_idx] = smoothTrajectory(obj,qtraj,supp_times,pose_struct,joint_v_max,options)
      if nargin < 6
        options = struct();
      end

      if (nargin < 4) || isempty(pose_struct)
        pose_struct = obj.pose_struct;
      end

      if ~isfield(options,'velocity')
        options.velocity = 2/3;
      end
      
      if nargin < 5 || isempty(joint_v_max)
        joint_v_max = 15*pi/180*ones(obj.nq,1);
        joint_v_max(1:3) = 0.01*ones(3,1); % the floating base uses different units
        l_leg_aky = obj.robot.findPositionIndices('l_leg_aky');
        r_leg_aky = obj.robot.findPositionIndices('r_leg_aky');
        joint_v_max([l_leg_aky, r_leg_aky]) = joint_v_max([l_leg_aky, r_leg_aky]);
        r_leg_hpy = obj.robot.findPositionIndices('r_leg_hpy');
        l_leg_hpy = obj.robot.findPositionIndices('l_leg_hpy');
        joint_v_max([r_leg_hpy,l_leg_hpy]) = 2/3*joint_v_max([r_leg_hpy,l_leg_hpy]);
        back_idxs = obj.robot.findPositionIndices('back');
        joint_v_max(back_idxs) = 1/2 * joint_v_max(back_idxs);
      end
      joint_v_max = options.velocity * joint_v_max;
            
      new_times = [];
      v_max_idx = {};
      
      for j = 2:length(pose_struct)
        idx = [j-1,j];
        [traj_interval, max_idx] = obj.smoothTrajectoryInterval(qtraj,supp_times,pose_struct,idx,joint_v_max);
        v_max_idx{end+1} = max_idx;
        if j == 2
          qtraj_smooth = traj_interval;
          new_times = traj_interval.tspan;
        else
          % need to shift the time of traj_interval to make it match the end time of qtraj_smooth
          traj_interval = traj_interval.shiftTime(qtraj_smooth.tspan(2));
          qtraj_smooth = qtraj_smooth.append(traj_interval);
          new_times = [new_times, qtraj_smooth.tspan(2)];
        end
      end
    end
    
    function [traj_smooth, max_idx] = smoothTrajectoryInterval(obj,qtraj,supp_times,pose_struct,idx,joint_v_max)
      t_start = supp_times(idx(1));
      t_end = supp_times(idx(2));
      upsample = 60; % number of extra samples to take along trajectory
      t = linspace(t_start,t_end,upsample);
      q_path = eval(qtraj,t);
      
      % Determine max joint velocity at midpoint of  each segment
      t_mid = mean([t(2:end); t(1:end-1)],1);
      
      joint_v_max_resize = repmat(joint_v_max,1,length(t_mid));
      
      % find the maximum percentage that some joint is over their joint speed limit.
      [v_mid, max_idx] = max(abs(qtraj.fnder().eval(t_mid)./joint_v_max_resize), [], 1);
      
      % adjust durations to keep velocity below max
      t_scaled = [0, cumsum(diff(t).*v_mid)];
      tf = t_scaled(end);
      
      % Warp time to give gradual acceleration/deceleration
      t_scaled = t_scaled/tf;
      t_scaled = tf*arrayfun(@(x)obj.timeScale(x,3),t_scaled);
      
      
      % construct the new smoothed trajectory
      traj_smooth = PPTrajectory(pchip(t_scaled,q_path));
    end
    
    function [traj_smooth,new_times,q_ik,ik_struct,idx] = constructSmoothTrajectory(obj,q_sol,q_ik,idx)
      if nargin < 4;
        [q_ik,ik_struct,idx] = smoothContacts(obj,q_sol);
      end
      t_sol = [1:size(q_sol,2)];
      t_smooth = [];
      q_smooth = [];
      
      for j = 1:numel(idx)
        q_smooth = [q_smooth,q_ik(:,idx{j}.q_ik_idx)];
        if idx{j}.adding_support
          t_temp = idx{j}.q_sol_idx - 0.25;
        else
          t_temp = idx{j}.q_sol_idx + 0.25;
        end
        
        t_smooth =[t_smooth,t_temp];
      end
      
      t_all = [t_sol,t_smooth];
      q_all = [q_sol,q_smooth];
      % need to sort them before passing into phcip
      [t_all,I] = sort(t_all);
      q_all = q_all(:,I);
      
      % construct and smooth the trajectory
      traj = PPTrajectory(pchip(t_all,q_all));
      [traj_smooth, new_times] = obj.smoothTrajectory(traj,t_sol);
    end
    
    function [kp_data,sol_struct,ik_struct] = genKinematicPlanData(obj,q_sol,sol_struct,ik_struct,options)
      if nargin < 5 || isempty(options)
        options = {};
      end

      if ~isfield(options,'shrink_factor')
        options.shrink_factor = 0.6;
      end

      if ~isfield(options,'velocity')
        options.velocity = 2/3;
      end

      if nargin < 3 || isempty(sol_struct)
        sol_struct = obj.genSolutionStruct(q_sol,{},options)
      end
      if nargin < 4 || isempty(ik_struct)
        ik_struct = obj.smoothContacts(sol_struct,options);
      end

      qtraj = obj.constructTrajectory(horzcat(ik_struct.q));
      [min_struct,min_times] = obj.extractMinimalStruct(ik_struct);

      % options to be passed to smoothTrajectory
      clear options_smooth;
      options_smooth.velocity = options.velocity;
      [qtraj_smooth,support_times] = obj.smoothTrajectory(qtraj,min_times,min_struct,{},options);


      kp_data = struct();
      kp_data.qtraj = qtraj_smooth;
      kp_data.support_times = support_times;
      supports = cell(numel(min_times),1);
      for j = 2:numel(min_times)
        supports{j-1} = ik_struct(min_times(j)).no_movement;
      end
      supports{end} = ik_struct(min_times(end)).contacts;
      kp_data.supports = supports;
      kp_data.c_pts = obj.c;
      kp_data.linkName = obj.genLinkName;
    end

    function [min_struct,min_times] = extractMinimalStruct(obj,input_struct)
      min_times = [1];
      min_struct(1) = input_struct(1);
      N = numel(input_struct);
      for j = 2:numel(input_struct)
        % if we are making contact
        if ~isempty(setdiff(input_struct(j).contacts,input_struct(j-1).contacts)) 
          k = numel(min_struct)+1;
          min_struct(k) = input_struct(j);
          min_times = [min_times,j];
        end

        if ~isempty(setdiff(input_struct(j-1).contacts,input_struct(j).contacts))
          k = numel(min_struct)+1;
          min_struct(k) = input_struct(j-1);
          min_times = [min_times,j-1];
        end
      end
      k = numel(min_struct)+1;
      min_struct(k) = input_struct(N);
      min_times = [min_times,N]; 
    end

    
    % check quasi static constraint with the given contact points
    function [flag] = checkQuasiStaticConstraint(obj,q,contacts,shrink_factor)
      if nargin < 4
        shrink_factor = obj.shrink_factor;
      end
      [~,qs_constraint] = obj.addSingleQuasiStaticConstraint(contacts,1,0,shrink_factor);
      qs_constraint = qs_constraint{1}; % have to do this since qs_constraint is a cell
      kinsol = obj.robot.doKinematics(q);
      flag = qs_constraint.checkConstraint(kinsol);
      if flag
        disp('Quasi Static constraint satisfied')
      else
        disp('FAILED: Quasi Statit constraint NOT satisfied');
      end
    end
    
    % replaces 'l_foot' with {'l_toe','r_toe'} in the contacts field
    function obj = removeDuplicatesFromDataStruct(obj)
      for j = 1:numel(obj.data_struct)
        contacts = obj.data_struct(j).contacts;
        
        % check to see if it contains l_foot
        if ~isempty(find(strcmp(contacts,'l_foot')))
          % if so replace 'l_foot' with 'l_heel' and 'l_toe'
          contacts = setdiff(contacts,{'l_foot'});
          contacts = union(contacts,{'l_toe','l_heel'});
        end
        
        % replace 'r_foot' with 'r_toe','r_heel'
        if ~isempty(find(strcmp(contacts,'r_foot')))
          contacts = setdiff(contacts,{'r_foot'});
          contacts = union(contacts,{'r_toe','r_heel'});
        end
        
        obj.data_struct(j).contacts = contacts;
      end
    end


    function [t_bad,qtraj] = checkQuasiStaticConstraintAlongTrajectory(obj,sol_struct,shrink_factor)
      if nargin < 3;
        shrink_factor = obj.shrink_factor;
      end
      N = numel(sol_struct);
      t_grid = [1:N];
      qtraj = PPTrajectory(pchip(t_grid,horzcat(sol_struct.q)));
      t_sample = linspace(1,N,10*N);
      t_bad = [];

      for j = 1:numel(t_sample)
        t = t_sample(j);
        idx = max(ceil(t),2);
        contacts = sol_struct(idx).no_movement;
        flag = obj.checkQuasiStaticConstraint(qtraj.eval(t),contacts,shrink_factor);
        if ~flag
          t_bad = [t_bad,t];
        end
      end
    end


    function sol_struct = genSolutionStruct(obj,q_sol,pose_struct,options)

      if nargin < 3 || isempty(pose_struct)
        pose_struct = obj.pose_struct;
      end

      if nargin < 4
        options = struct();
      end

      shrink_factor_original = obj.shrink_factor;
      if isfield(options,'shrink_factor')
        obj = obj.setQuasiStaticShrinkFactor(options.shrink_factor);
      end      

      sol_struct = struct('q',{},'qs_contacts',{},'no_movement',{},'contacts',{});
      sol_counter = 0;
      for j = 1:numel(pose_struct)
        sol_counter = sol_counter + 1;
        sol_struct(sol_counter).contacts = pose_struct(j).contacts;
        sol_struct(sol_counter).qs_contacts = pose_struct(j).qs_contacts;
        sol_struct(sol_counter).no_movement = pose_struct(j).no_movement;
        sol_struct(sol_counter).q = q_sol(:,j);

        % check if we are adding contacts
        if j > 1
          if ~isempty(setdiff(pose_struct(j).contacts,pose_struct(j-1).contacts))
            % if we are adding contacts then add an intermediate solve that uses the full 
            % set of contacts for the quasi-static constraint
            sol_counter = sol_counter + 1;
            contacts = pose_struct(j).contacts;
            no_movement = pose_struct(j).no_movement;
            sol_struct(sol_counter).contacts = contacts;
            sol_struct(sol_counter).qs_contacts = contacts;
            sol_struct(sol_counter).no_movement = contacts;

            clear options;
            options.no_movement.contacts = contacts;
            options.no_movement.q = q_sol(:,j);
            options.enforce_contact = 0;
            [q,F,info,infeasible_constraint,~] = obj.findPose(contacts,{},q_sol(:,j),options);
            fprintf('j is %d \n',j);
            fprintf('sol_counter is %d \n',sol_counter);
            info
            infeasible_constraint
            sol_struct(sol_counter).q = q;
          end
        end
      end
      obj.shrink_factor = shrink_factor_original;
    end  

    % allows collisions between the l_uleg,r_uleg and the pelvis, not for this to make sense
    % you need to be using the updated pelvis mesh, maybe put in a check here
    function [r,obj] = allowPelvisCollisionRobot(obj,r)
      if nargin < 2
        r = obj.robot;
      end
      
      r = r.removeFromIgnoredListOfCollisionFilterGroup('core','l_uleg');
      r = r.removeFromIgnoredListOfCollisionFilterGroup('core','r_uleg');
      
      % this stops l_uleg and r_uleg from ignoring the 'core'
      r = r.addLinksToCollisionFilterGroup({'l_lglut','l_uglut'},'ignore_core',1);
      r = r.addLinksToCollisionFilterGroup({'r_lglut','r_uglut'},'ignore_core',1);
      
      r = compile(r);
      if nargin < 2
        obj.robot = r;
      end
    end
    
    function obj = useRobotiqHands(obj)
      if obj.robotiq_weight_flag == true
        warning('already using the robotiq_weight, ignoring this command');
        return;
      end

      obj.robotiq_weight_flag = true;
      options_hand.weld_to_link = obj.robot.findLinkId('r_hand');
      robotiq_urdf = [getenv('DRC_BASE'),'/software/control/matlab/systems/urdf/robotiq_box.urdf'];
      obj.robot = obj.robot.addRobotFromURDF(robotiq_urdf, [0; -0.195; -0.01], [0; -3.1415/2; 3.1415], options_hand);
      obj.robot = compile(obj.robot);
    end

    % helper method for setting pelvis collision option
    function obj = setPelvisCollision(obj,val)
      obj.allow_pelvis_collision = val;
    end
    
    % helper method for setting pelvis min distance
    function obj = setPelvisMinDistance(obj,val)
      obj.pelvis_min_distance = val;
    end
    
    
    % Helper methods for setting object properties
    function obj = setQuasiStaticShrinkFactor(obj,shrink_factor)
      obj.shrink_factor = shrink_factor;
    end
    
    function obj = setMinDistance(obj,min_distance)
      obj.min_distance = min_distance;
    end
    
    function obj = setConstraintTol(obj,constraint_tol)
      obj.constraint_tol = constraint_tol;
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
      c('m_pelvis') = [-0.049;-0.042;-0.128];
      c('r_thigh') = [0.0345;0.0;-0.332];
      c('l_thigh') = c('r_thigh');

      
    end
    
    
    % a container.Map that gives the link name corresponding to each
    % contact point group
    function lmap = genLinkId(obj)
      lmap = containers.Map;
      
      lmap('l_toe') = obj.robot.findLinkId('l_foot');
      lmap('r_toe') = obj.robot.findLinkId('r_foot');
      lmap('r_knee') =  obj.robot.findLinkId('r_lleg');
      lmap('l_knee') = obj.robot.findLinkId('l_lleg');
      lmap('r_hand') = obj.robot.findLinkId('r_hand');
      lmap('l_hand') = obj.robot.findLinkId('l_hand');
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
      lmap('r_thigh')= obj.robot.findLinkId('r_uleg');
      lmap('l_thigh') = obj.robot.findLinkId('l_uleg');
      lmap('m_pelvis') = obj.robot.findLinkId('pelvis');
    end
    
    function lmap = genLinkName(obj)
      lmap = containers.Map;
      
      lmap('l_toe') = 'l_foot';
      lmap('r_toe') = 'r_foot';
      lmap('r_knee') =  'r_lleg';
      lmap('l_knee') = 'l_lleg';
      lmap('r_hand') = 'r_hand';
      lmap('l_hand') = 'l_hand';
      lmap('face_cage')= 'utorso';
      lmap('chest') = 'utorso';
      lmap('l_foot') =  'l_foot';
      lmap('r_foot')= 'r_foot';
      lmap('l_heel') = 'l_foot';
      lmap('r_heel') = 'r_foot';
      lmap('r_uback') = 'utorso';
      lmap('r_lback') = lmap('r_uback');
      lmap('l_uback') = lmap('r_uback');
      lmap('l_lback') = lmap('r_uback');
      lmap('back') = lmap('r_uback');
      
      lmap('r_fpelvis') = obj.robot.findLinkId('pelvis');
      lmap('r_bpelvis') = obj.robot.findLinkId('pelvis');
      lmap('l_fpelvis') = obj.robot.findLinkId('pelvis');
      lmap('l_bpelvis') = obj.robot.findLinkId('pelvis');
      lmap('pelvis') = obj.robot.findLinkId('pelvis');
      lmap('r_thigh')= obj.robot.findLinkId('r_uleg');
      lmap('l_thigh') = obj.robot.findLinkId('l_uleg');
      lmap('m_pelvis') = pelvis;
    end
    
    
    % add spheres representing all the contact points on the robot
    function [robot,obj] = addVisualContactPoints(obj,robot)
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
      
      if nargin < 2
        obj.robot = robot;
      end
    end
    
    
    % helper method to add all the collision geometries to the robot
    function [robot,obj] = addCollisionGeometryToRobot(obj,robot,radius)
      if nargin < 2
        robot = obj.robot;
      end
      if nargin < 3
        radius = 0;
      end
      
      keySet = keys(obj.c);
      for j=1:length(keySet)
        name = keySet{j};
        link = obj.linkId(name);
        pts = obj.c(name);
        
        for k = 1:size(pts,2)
          sphere =  RigidBodySphere(radius,pts(:,k),[0;0;0]);
          robot = robot.addCollisionGeometryToBody(link,sphere,name);
        end
      end
      robot = compile(robot);
      
      if nargin < 2
        obj.robot = robot;
      end
    end
    
    % this is a helper method for use in the QPControllerContacts stuff
    function [robot,obj] = addSpecifiedCollisionGeometryToRobot(obj,contacts,robot)

      if nargin < 3
        robot = obj.robot;
      end
      
      for j = 1:numel(contacts)
        name = contacts{j};
        link = obj.linkId(name);
        pts = obj.c(name);
        
        for k = 1:size(pts,2)
          sphere = RigidBodySphere(0.01,pts(:,k),[0;0;0]);
          robot = robot.addCollisionGeometryToBody(link,sphere,name);
        end
      end
      
      robot = compile(robot);
      if nargin < 3
        obj.robot = robot;
      end
      
    end
    
    % removes all the collision geometries except 'terrain' and the ones in
    % obj.c from the robot
    function [robot,obj] = removeOtherCollisionGeometries(obj,robot)
      if nargin < 2
        robot = obj.robot;
      end
      keySet = keys(obj.c);
      except = keySet;
      except{end+1} = 'terrain';
      robot = robot.removeCollisionGroupsExcept(except);
      robot = compile(robot);
      
      if nargin < 2
        obj.robot = robot;
      end
    end
    
    
    
    % helper function to generate the supports for a QPControllerData
    % structure
    function supports = genControllerDataSupport(obj,robot,q_sol)
      pose_struct = obj.pose_struct;
      supports = RigidBodySupportState.empty(0,0);
      for j = 2:length(pose_struct)

        % construct the bodies array
        bodies = [];
        contact_groups = {};
        for k = 1:numel(pose_struct(j).no_movement)
          name = pose_struct(j).no_movement{k};
          bodies(k) = obj.linkId(name);
          
          % in my setup name is the same as the contact group
          contact_groups{k} = {name};
        end
        % construct a RBSS object and add it to the supports array
        supports(j-1) = RigidBodySupportState(robot,bodies,contact_groups);
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
    
    function obj = setRobot(obj,robot)
      robot = compile(robot);
      obj.robot = robot;
    end
    
    
    function obj = addTerrain(obj)
      obj.robot = obj.robot.setTerrain(RigidBodyFlatTerrain);
      obj.robot = compile(obj.robot);
    end
    
    % adjusts hand contact points to be the new ones in accordance with the
    % hand guards
    function obj = useHandGuards(obj)

      r_pts =   [0.0660   -0.0660    0.0450   -0.0450;
      -0.3030   -0.3030   -0.3038   -0.3038;
      -0.1890   -0.1890    0.1271    0.1271];
      
      l_pts =[0.0660   -0.0660    0.0450   -0.0450;
      -0.3030    -0.3030    -0.3038    -0.3038;
      0.1271    0.1271 -0.1890   -0.1890];
      
      obj.c('r_hand') = r_pts;
      obj.c('l_hand') = l_pts;
    end

    function [r,obj] = useNewPelvisMesh(obj,r)
      if nargin < 2
        r = obj.robot;
      end
      pelvis = r.findLinkId('pelvis');
      r = r.removeCollisionGroupsExcept({},1,pelvis);

      fpelvis_chull = RigidBodyMesh()

    end

    function obj = setContactHeight(obj,contact_height)
      obj.contact_height = contact_height;
    end

    function drawCOM(obj,robot,q)
      kinsol = robot.doKinematics(q);
      com_pos = robot.getCOM(kinsol);
      com_pos(3) = 0;
      
      lcmgl = LCMGLClient;
      lcmgl.glColor3f(1,0,0);
      lcmgl.sphere(com_pos,0.02,20,20);
      lcmgl.switchBuffers;
      v = robot.constructVisualizer;
      v.draw(0,q);
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
    
    % helper method for drawing a pose with COM projected onto the ground
    % in red

    
    function y = timeScale(x,param)
      alpha = 1/param;
      C = (alpha*(1/2)^(alpha - 1))^(-1);
      
      if x <= 1/2
        y  = C*x^alpha;
      else
        y = 2*C*(1/2)^alpha - C*(1-x)^alpha;
      end
    end
  end
  
end
