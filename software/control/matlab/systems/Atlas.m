classdef Atlas < Biped
  methods

    function obj=Atlas(urdf,options)

      if nargin < 1
        urdf = strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf');
      else
        typecheck(urdf,'char');
      end

      if nargin < 2
        options = struct();
      end
      if ~isfield(options,'dt')
        options.dt = 0.001;
      end
      if ~isfield(options,'floating')
        options.floating = true;
      end

      S = warning('off','Drake:RigidBodyManipulator:SingularH');
      warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');

%       obj = obj@TimeSteppingRigidBodyManipulator(urdf,options.dt,options);
      obj = obj@Biped(urdf,options.dt,options);

      obj.floating =options.floating;

      obj.stateToBDIInd = 6*obj.floating+[1 2 3 28 9 10 11 12 13 14 21 22 23 24 25 26 4 5 6 7 8 15 16 17 18 19 20 27]';
      obj.BDIToStateInd = 6*obj.floating+[1 2 3 17 18 19 20 21 5 6 7 8 9 10 22 23 24 25 26 27 11 12 13 14 15 16 28 4]';

      if options.floating
        % could also do fixed point search here
        obj = obj.setInitialState(double(obj.manip.resolveConstraints(zeros(obj.getNumStates(),1))));
      else
        % TEMP HACK to get by resolveConstraints
        %for i=1:length(obj.manip.body), obj.manip.body(i).contact_pts=[]; end
        %obj.manip = compile(obj.manip);
        %obj = obj.setInitialState(zeros(obj.getNumStates(),1));
      end
      warning(S);
    end

    function obj = compile(obj)
      S = warning('off','Drake:RigidBodyManipulator:SingularH');
      obj = compile@TimeSteppingRigidBodyManipulator(obj);
      warning(S);

      state_frame = AtlasState(obj);
      obj = obj.setStateFrame(state_frame);
      obj = obj.setOutputFrame(state_frame);

      input_frame = AtlasInput(obj);
      obj = obj.setInputFrame(input_frame);
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

    function [u,obj] = inverseDynamics(obj,q,qdot,qddot_des,active_supports)
      if isempty(obj.inverse_dyn_qp_controller)
        ctrl_data = SharedDataHandle(struct(...
          'A',zeros(4),...
          'B',zeros(4,2),...
          'C',zeros(2,4),...
          'D',zeros(2),...
          'Qy',zeros(2),...
          'R',zeros(2),...
          'is_time_varying',false,...
          'S',zeros(4),...
          's1',zeros(4,1),...
          's2',0,...
          'x0',zeros(4,1),...
          'u0',zeros(2,1),...
          'trans_drift',zeros(3,1),...
          'support_times',0,...
          'supports',active_supports,...
          'mu',1,...
          'ignore_terrain',true,...
          'y0',zeros(2,1)));

        % instantiate QP controller
        options.slack_limit = 30.0;
        options.w = 0.01;
        options.R = 1e-12*eye(getNumInputs(obj));
        options.lcm_foot_contacts = false;
        options.debug = false;
        options.use_mex = 1;

        obj.inverse_dyn_qp_controller = QPControlBlock(obj,ctrl_data,options);
      else
        ctrl_data = obj.inverse_dyn_qp_controller.controller_data;
        setField(ctrl_data,'supports',active_supports);
      end

      u = obj.inverse_dyn_qp_controller.mimoOutput(0,[],qddot_des,zeros(12,1),[q;qdot]);
    end

    function z = getPelvisHeightAboveFeet(obj,q)
      kinsol = doKinematics(obj,q);
      foot_z = getFootHeight(obj,q);
      pelvis = forwardKin(obj,kinsol,findLinkInd(obj,'pelvis'),[0;0;0]);
      z = pelvis(3) - foot_z;
    end

    function foot_z = getFootHeight(obj,q)
      kinsol = doKinematics(obj,q);
      rfoot_cpos = contactPositions(obj,kinsol,findLinkInd(obj,'r_foot'));
      lfoot_cpos = contactPositions(obj,kinsol,findLinkInd(obj,'l_foot'));
      foot_z = min(mean(rfoot_cpos(3,:)),mean(lfoot_cpos(3,:)));
    end

    function [zmin,zmax] = getPelvisHeightLimits(obj,q) % for BDI manip mode
      z_above_feet = getPelvisHeightAboveFeet(obj,q);
      zmin = q(3) - (z_above_feet-obj.pelvis_min_height);
      zmax = q(3) + (obj.pelvis_max_height-z_above_feet);
    end

    function [A, b] = getReachabilityPolytope(obj, static_body_idx, swing_body_idx, params)

      bodies = [static_body_idx, swing_body_idx];
      if ~ (all(bodies == [obj.foot_bodies_idx.right, obj.foot_bodies_idx.left]) ||...
            all(bodies == [obj.foot_bodies_idx.left, obj.foot_bodies_idx.right]))
        error('DRC:Atlas:BadBodyIdx', 'Feasibility polytope not defined for this pairing of body indices.');
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
      [Axy, bxy] = poly2lincon([0, params.max_forward_step, 0, -params.max_forward_step],...
                               [params.min_step_width, params.nom_step_width, params.max_step_width, params.nom_step_width]);
      [Axz, bxz] = poly2lincon([0, params.nom_forward_step, params.max_forward_step, params.nom_forward_step, 0, -params.max_forward_step], ...
                               [params.max_upward_step, params.max_upward_step, 0, -params.max_downward_step, -params.max_downward_step, 0]);
      A = [Axy, zeros(4, 4);
           Axz(:,1), zeros(size(Axz, 1), 1), Axz(:,2), zeros(size(Axz, 1), 3);
           0 0 0 0 0 -1;
           0 0 0 0 0 1;
           0, 1/(params.max_step_width-params.nom_step_width), 0, 0, 0, 1/params.max_outward_angle;
           0, 1/(params.min_step_width-params.nom_step_width), 0, 0, 0, 1/params.max_outward_angle];

      if bodies(1) == obj.foot_bodies_idx.left
        A(:,2) = -A(:,2);
        A(:,6) = -A(:,6);
      end
      b = [bxy;
           bxz;
           params.max_inward_angle;
           params.max_outward_angle;
           1 + params.nom_step_width/(params.max_step_width-params.nom_step_width);
           1 + params.nom_step_width/(params.min_step_width-params.nom_step_width)
           ];
    end


  end
  properties (SetAccess = protected, GetAccess = public)
    x0;
    inverse_dyn_qp_controller;
    pelvis_min_height = 0.65; % [m] above feet, for hardware
    pelvis_max_height = 0.92; % [m] above feet, for hardware
    floating = true;
    stateToBDIInd;
    BDIToStateInd;
    default_footstep_params = struct('nom_forward_step', 0.15,... %m
                                      'max_forward_step', 0.3,...%m
                                      'max_step_width', 0.40,...%m
                                      'min_step_width', 0.21,...%m
                                      'nom_step_width', 0.26,...%m (nominal step width)
                                      'max_outward_angle', pi/8,... % rad
                                      'max_inward_angle', 0.01,... % rad
                                      'max_upward_step', 0.2,...
                                      'max_downward_step', 0.2);
  end
end
