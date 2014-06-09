classdef Biped < TimeSteppingRigidBodyManipulator
  properties
    floating
    x0
    foot_contact_offsets
    foot_bodies_idx
    foot_soles_idx
    foot_sole_transforms
    lc
  end

  methods
    function obj = Biped(urdf,dt,options)
      if nargin < 3
        options = struct();
        options.floating = true;
      end
      if ~isfield(options, 'r_foot_name'); options.r_foot_name = 'r_foot'; end
      if ~isfield(options, 'l_foot_name'); options.l_foot_name = 'l_foot'; end
      if nargin < 2
        dt = 0.002;
      end
      obj = obj@TimeSteppingRigidBodyManipulator(urdf,dt,options);
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.foot_bodies_idx = struct('right', findLinkInd(obj, options.r_foot_name), 'left', findLinkInd(obj, options.l_foot_name));
      obj.foot_soles_idx = struct('right', obj.findFrameId([options.r_foot_name, '_sole']),...
                                  'left', obj.findFrameId([options.l_foot_name, '_sole']));
      obj.foot_contact_offsets = obj.findContactOffsets();
      obj.foot_sole_transforms = struct('right', obj.getFrame(obj.foot_soles_idx.right).T,...
                                        'left', obj.getFrame(obj.foot_soles_idx.left).T);
    end

    function foot_center = feetPosition(obj, q0)
      % Convenient way to find the poses of the centers of the feet given a
      % configuration vector q0

      typecheck(q0,'numeric');
      sizecheck(q0,[obj.getNumDOF,1]);

      kinsol = doKinematics(obj,q0);

      rfoot0 = forwardKin(obj,kinsol,obj.foot_soles_idx.right,[0;0;0],true);
      lfoot0 = forwardKin(obj,kinsol,obj.foot_soles_idx.left,[0;0;0],true);

      foot_center = struct('right', rfoot0, 'left', lfoot0);
    end

    function [A, b] = getReachabilityPolytope(obj, static_body_idx, swing_body_idx, params)
      % Compute an inner approximation of the reachable set for the swing_frame in the frame
      % of the (fixed) static_frame. Returns A, b such that Av <= b for feasible poses,
      % where v = [x y z r p y];
      %
      % Note: currently only defined for the right and left foot soles.

      bodies = [static_body_idx, swing_body_idx];
      if ~ (all(bodies == [obj.foot_bodies_idx.right, obj.foot_bodies_idx.left]) ||...
            all(bodies == [obj.foot_bodies_idx.left, obj.foot_bodies_idx.right]))
        error('DRC:Biped:BadBodyIdx', 'Feasibility polytope not defined for this pairing of body indices.');
      end
      params = struct(params);
      fields = {'max_forward_step',...
                'nom_forward_step',...
                'nom_step_width',...
                'max_step_width',...
                'min_step_width',...
                'max_outward_angle',...
                'max_inward_angle',...
                'max_upward_step',...
                'max_downward_step'};
      for f = fields
        field = f{1};
        if ~isfield(params, field)
          params.(field) = obj.default_footstep_params.(field);
        end
      end
      if params.max_step_width <= params.nom_step_width
        warning('DRC:Biped:BadNominalStepWidth', 'Nominal step width should be less than max step width');
        params.max_step_width = params.nom_step_width * 1.01;
      end
      if params.min_step_width >= params.nom_step_width
        warning('DRC:Biped:BadNominalStepWidth', 'Nominal step width should be greater than min step width');
        params.min_step_width = params.nom_step_width * 99;
      end

      [Axy, bxy] = poly2lincon([0, params.max_forward_step, 0, -params.max_forward_step],...
                               [params.min_step_width, params.nom_step_width, params.max_step_width, params.nom_step_width]);
      [Axz, bxz] = poly2lincon([0, params.nom_forward_step, params.max_forward_step, params.nom_forward_step, 0, -params.max_forward_step], ...
                               [params.max_upward_step, params.max_upward_step, 0, -params.max_downward_step, -params.max_downward_step, 0]);
      A = [Axy, zeros(size(Axy, 1), 4);
           Axz(:,1), zeros(size(Axz, 1), 1), Axz(:,2), zeros(size(Axz, 1), 3);
           0 0 0 0 0 -1;
           0 0 0 0 0 1;
           0, 1/(params.max_step_width-params.nom_step_width), 0, 0, 0, 1/params.max_outward_angle;
           0, 1/(params.min_step_width-params.nom_step_width), 0, 0, 0, 1/params.max_outward_angle];

      if bodies(1) == obj.foot_bodies_idx.left
        A(:,2) = -A(:,2);
        A(:,6) = -A(:,6);
      elseif bodies(1) == obj.foot_bodies_idx.right
      else
        error('invalid body idx');
      end
      b = [bxy;
           bxz;
           params.max_inward_angle;
           params.max_outward_angle;
           1 + params.nom_step_width/(params.max_step_width-params.nom_step_width);
           1 + params.nom_step_width/(params.min_step_width-params.nom_step_width)
           ];
    end

    function obj = setInitialState(obj,x0)
      if isa(x0,'Point')
        obj.x0 = double(x0); %.inFrame(obj.getStateFrame));
      else
        typecheck(x0,'double');
        sizecheck(x0,obj.getNumStates());
        obj.x0 = x0;
      end
    end

    function x0 = getInitialState(obj)
      x0 = obj.x0;
    end

    function xstar = loadFixedPoint(obj)
      error('DRC:Biped:NotImplemented', 'Biped subclasses should implement this.');
    end

    function [xstar,ustar,zstar] = getFixedPoint(obj,options)
      if nargin < 2 || ~isfield(options,'visualize')
        options.visualize = false;
      end

      x0 = Point(obj.getStateFrame());
      x0 = resolveConstraints(obj,x0);
      u0 = zeros(obj.getNumInputs(),1);

      nq = obj.getNumDOF();
      nu = obj.getNumInputs();
      nz = obj.getNumContacts()*3;
      z0 = zeros(nz,1);
      q0 = x0(1:nq);

      problem.x0 = [q0;u0;z0];
      problem.objective = @(quz) 0; % feasibility problem
      problem.nonlcon = @(quz) mycon(quz);
      problem.solver = 'fmincon';

      if options.visualize
        v = obj.constructVisualizer;
        %problem.options=optimset('DerivativeCheck','on','GradConstr','on','Algorithm','interior-point','Display','iter','OutputFcn',@drawme,'TolX',1e-14,'MaxFunEvals',5000);
        problem.options=optimset('GradConstr','on','Algorithm','interior-point','Display','iter','OutputFcn',@drawme,'TolX',1e-14,'MaxFunEvals',5000);
      else
        problem.options=optimset('GradConstr','on','Algorithm','interior-point','TolX',1e-14,'MaxFunEvals',5000);
      end

      lb_z = -1e6*ones(nz,1);
      lb_z(3:3:end) = 0; % normal forces must be >=0
      ub_z = 1e6*ones(nz,1);

      [jl_min,jl_max] = obj.getJointLimits();
      % force search to be close to starting position
      problem.lb = [max(q0-0.05,jl_min+0.01); obj.umin; lb_z];
      problem.ub = [min(q0+0.05,jl_max-0.01); obj.umax; ub_z];
      %problem.lb(2) = 0.0; % body z

      [quz_sol,~,exitflag] = fmincon(problem);
      success=(exitflag==1);
      xstar = [quz_sol(1:nq); zeros(nq,1)];
      ustar = quz_sol(nq+(1:nu));
      zstar = quz_sol(nq+nu+(1:nz));
      if (~success)
        error('failed to find fixed point');
      end

      function stop=drawme(quz,optimValues,state)
        stop=false;
        v.draw(0,[quz(1:nq); zeros(nq,1)]);
      end

      function [c,ceq,GC,GCeq] = mycon(quz)
        q=quz(1:nq);
        u=quz(nq+(1:nu));
        z=quz(nq+nu+(1:nz));

        [~,C,B,~,dC,~] = obj.manip.manipulatorDynamics(q,zeros(nq,1));
        [phiC,JC] = obj.contactConstraints(q);
        [~,J,dJ] = obj.contactPositions(q);

        % ignore friction constraints for now
        c = 0;
        GC = zeros(nq+nu+nz,1);

        dJz = zeros(nq,nq);
        for i=1:nq
            dJz(:,i) = dJ(:,(i-1)*nq+1:i*nq)'*z;
        end

        ceq = [C-B*u-J'*z; phiC];
        GCeq = [[dC(1:nq,1:nq)-dJz,-B,-J']',[JC'; zeros(nu+nz,length(phiC))]];
      end
    end

    function obj = configureDRCTerrain(obj, map_mode, q0)
      terrain = obj.getTerrain();
      if isa(terrain, 'DRCTerrainMap')
        if map_mode == drc.footstep_plan_params_t.HORIZONTAL_PLANE
          terrain = terrain.setFillPlaneFromConfiguration(obj, q0, true);
          terrain = terrain.overrideNormals(true);
          terrain = terrain.overrideHeights(true);
        elseif map_mode == drc.footstep_plan_params_t.FOOT_PLANE
          terrain = terrain.setFillPlaneFromConfiguration(obj, q0, false);
          terrain = terrain.overrideNormals(true);
          terrain = terrain.overrideHeights(true);
        elseif map_mode == drc.footstep_plan_params_t.TERRAIN_HEIGHTS_Z_NORMALS
          terrain = terrain.setFillPlaneFromConfiguration(obj, q0, true);
          terrain = terrain.overrideNormals(true);
          terrain = terrain.overrideHeights(false);
        elseif map_mode == drc.footstep_plan_params_t.TERRAIN_HEIGHTS_AND_NORMALS
          terrain = terrain.setFillPlaneFromConfiguration(obj, q0, false);
          terrain = terrain.overrideNormals(false);
          terrain = terrain.overrideHeights(false);
        end
      end
      obj = obj.setTerrain(terrain);
    end

  end

  methods (Static)
    function Xo = stepCenter2FootCenter(Xc, is_right_foot, nom_step_width)
      Xo = Biped.footCenter2StepCenter(Xc, is_right_foot, -nom_step_width);
    end

    function Xc = footCenter2StepCenter( Xo, is_right_foot, nom_step_width)
      % Convert a position of the center of one of the biped's feet to the
      % corresponding point half the step width toward the bot's center.
      % nom_step_width should be scalar or vector of size(1, size(Xo,2))
      if length(nom_step_width) == 1
        nom_step_width = repmat(nom_step_width, 1, size(Xo, 2));
      end
      if is_right_foot
        offs = [zeros(1,length(nom_step_width)); -nom_step_width/2; zeros(1,length(nom_step_width))];
      else
        offs = [zeros(1,length(nom_step_width)); nom_step_width/2; zeros(1,length(nom_step_width))];
      end
      for j = 1:length(Xo(1,:))
        M = rpy2rotmat(Xo(4:6,j));
        d = M * offs(:,j);
        Xc(:,j) = [Xo(1:3,j) - d(1:3); Xo(4:end,j)];
      end
    end
  end
end
