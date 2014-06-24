classdef QPControlBlock < MIMODrakeSystem

  methods
  function obj = QPControlBlock(r,body_accel_input_frames,controller_data,options)
    % @param r atlas instance
    % @param controller_data shared data handle containing linear system, zmp trajectories, Riccati solution, etc
    % @param options structure for specifying objective weight (w), slack
    % variable limits (slack_limit), and input cost (R)
    typecheck(r,'Biped');
    typecheck(controller_data,'SharedDataHandle');

    QPControlBlock.check_ctrl_data(controller_data)
    
    if nargin>3
      typecheck(options,'struct');
    else
      options = struct();
    end
    
    qddframe = AtlasCoordinates(r); % input frame for desired qddot, qdd constraints 
        
    input_frame = MultiCoordinateFrame({r.getStateFrame,qddframe,FootContactState,body_accel_input_frames{:}});
    
    if ~isfield(options,'output_qdd')
      options.output_qdd = false;
    else
      typecheck(options.output_qdd,'logical');
    end
    
    if options.output_qdd
      output_frame = MultiCoordinateFrame({r.getInputFrame(),qddframe});
    else
      output_frame = r.getInputFrame();
    end

    obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
    obj = setInputFrame(obj,input_frame);
    obj = setOutputFrame(obj,output_frame);

    obj.robot = r;
    obj.numq = getNumDOF(r);
    obj.controller_data = controller_data;
    obj.n_body_accel_inputs = length(body_accel_input_frames);
    
    if isfield(options,'dt')
      % controller update rate
      typecheck(options.dt,'double');
      sizecheck(options.dt,[1 1]);
      dt = options.dt;
    else
      dt = 0.001;
    end
    obj = setSampleTime(obj,[dt;0]); % sets controller update rate
   
    if ~isfield(obj.controller_data.data,'qp_active_set')
      obj.controller_data.setField('qp_active_set',[]);
    end
    
    if isfield(options,'use_bullet')
      obj.use_bullet = options.use_bullet;
    else
      obj.use_bullet = false;
    end
    
    % weight for desired qddot objective term
    if isfield(options,'w')
      typecheck(options.w,'double');
      sizecheck(options.w,1);
      obj.w = options.w;
    else
      obj.w = 0.1;
    end

    % weight for grf coefficients
    if isfield(options,'w_grf')
      typecheck(options.w_grf,'double');
      sizecheck(options.w_grf,1);
      obj.w_grf = options.w_grf;
    else
      obj.w_grf = 0.0;
    end    

    % weight for slack vars
    if isfield(options,'w_slack')
      typecheck(options.w_slack,'double');
      sizecheck(options.w_slack,1);
      obj.w_slack = options.w_slack;
    else
      obj.w_slack = 0.001;
    end    
    
    % hard bound on slack variable values
    if isfield(options,'slack_limit')
      typecheck(options.slack_limit,'double');
      sizecheck(options.slack_limit,1);
      obj.slack_limit = options.slack_limit;
    else
      obj.slack_limit = 10;
    end

    % array dictating whether body acceleration inputs should be
    % constraints (val<0) or cost terms with weight in [0,inf]
    if isfield(options,'body_accel_input_weights')
      typecheck(options.body_accel_input_weights,'double');
      sizecheck(options.body_accel_input_weights,obj.n_body_accel_inputs);
      obj.body_accel_input_weights = options.body_accel_input_weights;
    else
      obj.body_accel_input_weights = -1*ones(obj.n_body_accel_inputs,1);
    end
    
    if isfield(options,'debug')
      typecheck(options.debug,'logical');
      sizecheck(options.debug,1);
      obj.debug = options.debug;
    else
      obj.debug = false;
    end

    if obj.debug
      obj.debug_pub = ControllerDebugPublisher('CONTROLLER_DEBUG');
    end

    if isfield(options,'solver') 
      % 0: fastqp, fallback to gurobi barrier (default)
      % 1: gurobi primal simplex with active sets
      typecheck(options.solver,'double');
      sizecheck(options.solver,1);
      assert(options.solver==0 || options.solver==1);
    else
      options.solver = 0;
    end
    obj.solver = options.solver;
    
    if isfield(options,'use_mex')
      % 0 - no mex
      % 1 - use mex
      % 2 - run mex and non-mex and valuecheck the result
      sizecheck(options.use_mex,1);
      obj.use_mex = uint32(options.use_mex);
      rangecheck(obj.use_mex,0,2);
      if (obj.use_mex && exist('QPControllermex','file')~=3)
        error('can''t find QPControllermex.  did you build it?');
      end
      if (obj.use_mex==2 && obj.solver~=1)
        error('must use gurobi when using use_mex=2 (todo: generalize)');
      end
    else
      obj.use_mex = 1;
    end
    
    obj.lc = lcm.lcm.LCM.getSingleton();
    obj.rfoot_idx = findLinkInd(r,'r_foot');
    obj.lfoot_idx = findLinkInd(r,'l_foot');
    obj.rhand_idx = findLinkInd(r,'r_hand');
    obj.lhand_idx = findLinkInd(r,'l_hand');
    obj.pelvis_idx = findLinkInd(r,'pelvis');    
      
    obj.gurobi_options.outputflag = 0; % not verbose
    if options.solver==0
      obj.gurobi_options.method = 2; % -1=automatic, 0=primal simplex, 1=dual simplex, 2=barrier
    else
      obj.gurobi_options.method = 0; % -1=automatic, 0=primal simplex, 1=dual simplex, 2=barrier
    end
    obj.gurobi_options.presolve = 0;
    % obj.gurobi_options.prepasses = 1;

    if obj.gurobi_options.method == 2
      obj.gurobi_options.bariterlimit = 20; % iteration limit
      obj.gurobi_options.barhomogeneous = 0; % 0 off, 1 on
      obj.gurobi_options.barconvtol = 5e-4;
    end
    
    if (obj.use_mex>0)
      terrain = getTerrain(r);
      if isa(terrain,'DRCTerrainMap') 
        terrain_map_ptr = terrain.map_handle.getPointerForMex();
      else
        terrain_map_ptr = 0;
      end
      obj.mex_ptr = SharedDataHandle(QPControllermex(0,obj,obj.robot.getMexModelPtr.ptr,getB(obj.robot),r.umin,r.umax,terrain_map_ptr));
    end
    
    if isa(getTerrain(r),'DRCFlatTerrainMap')
      obj.using_flat_terrain = true;      
    else
      obj.using_flat_terrain = false;
    end
    
    [obj.jlmin, obj.jlmax] = getJointLimits(r);
        
    obj.output_qdd = options.output_qdd;
  end

  end
  
  methods (Static)
    function check_ctrl_data(ctrl_data)
      if ~isfield(ctrl_data.data,'D')
        % assumed  ZMP system
        hddot = 0; % could use estimated comddot here
        ctrl_data.setField('D',-0.89/(hddot+9.81)*eye(2)); % TMP hard coding height here. Could be replaced with htraj from planner
        % or current height above height map;
      end
      if ~isfield(ctrl_data.data,'qp_active_set')
        ctrl_data.setField('qp_active_set',[]);
      end
      
      ctrl_data = ctrl_data.data;
      
      % i've made the following assumptions to make things fast.  we can soften
      % them later as desired.  - Russ
      assert(isnumeric(ctrl_data.Qy));
%       sizecheck(ctrl_data.Qy,[2 2]); % commented out by sk--some of the
%       quasistatic systems pass 4x4 Qs
      assert(isnumeric(ctrl_data.R));
      sizecheck(ctrl_data.R,[2 2]);
      assert(isnumeric(ctrl_data.C));
      
      assert(isnumeric(ctrl_data.S));
      sizecheck(ctrl_data.S,[4 4]);
      assert(isnumeric(ctrl_data.x0));
      sizecheck(ctrl_data.x0,[4 1]);
      assert(isnumeric(ctrl_data.u0));
      if ctrl_data.is_time_varying
        assert(isa(ctrl_data.s1,'Trajectory'));
        assert(isa(ctrl_data.s2,'Trajectory'));
        assert(isa(ctrl_data.s1dot,'Trajectory'));
        assert(isa(ctrl_data.s2dot,'Trajectory'));
        assert(isa(ctrl_data.y0,'Trajectory'));
      else
        assert(isnumeric(ctrl_data.s1));
        assert(isnumeric(ctrl_data.s2));
        assert(isnumeric(ctrl_data.y0));
%        sizecheck(ctrl_data.supports,1);  % this gets initialized to zero
%        in constructors.. but doesn't get used.  would be better to
%        enforce it.
      end       
      sizecheck(ctrl_data.s1,[4 1]);
      sizecheck(ctrl_data.s2,1);
      assert(isnumeric(ctrl_data.mu));
      assert(islogical(ctrl_data.ignore_terrain));
    end
  end
  
  methods
    
  function varargout=mimoOutput(obj,t,~,varargin)
    persistent infocount
    if isempty(infocount)
      infocount = 0;
    end

    out_tic = tic;

    ctrl_data = obj.controller_data.data;
      
    x = varargin{1};
    qddot_des = varargin{2};
       
    r = obj.robot;
    nq = obj.numq; 
    q = x(1:nq); 
    qd = x(nq+(1:nq)); 
            
    %----------------------------------------------------------------------
    % Linear system stuff for zmp/com control -----------------------------
    A_ls = ctrl_data.A; % always TI
    B_ls = ctrl_data.B; % always TI
    Qy = ctrl_data.Qy;
    R_ls = ctrl_data.R;
    C_ls = ctrl_data.C;
    D_ls = ctrl_data.D;
    S = ctrl_data.S;
    Sdot = 0*S; % constant for ZMP/double integrator dynamics
    x0 = ctrl_data.x0 - [ctrl_data.trans_drift(1:2);0;0]; % for x-y plan adjustment
    u0 = ctrl_data.u0;
    if (ctrl_data.is_time_varying)
      s1 = fasteval(ctrl_data.s1,t);
%       s2 = fasteval(ctrl_data.s2,t);
      s1dot = fasteval(ctrl_data.s1dot,t);
      s2dot = fasteval(ctrl_data.s2dot,t);
      y0 = fasteval(ctrl_data.y0,t) - ctrl_data.trans_drift(1:2); % for x-y plan adjustment
    else
      s1 = ctrl_data.s1;
%       s2 = ctrl_data.s2;
      s1dot = 0*s1;
      s2dot = 0;
      y0 = ctrl_data.y0 - ctrl_data.trans_drift(1:2); % for x-y plan adjustment
    end
    mu = ctrl_data.mu;
    R_DQyD_ls = R_ls + D_ls'*Qy*D_ls;
    
    condof = ctrl_data.constrained_dofs; % dof indices for which q_ddd_des is a constraint
        
    fc = varargin{3};
    
    % TODO: generalize this again to arbitrary body contacts
    support_bodies = [];
    contact_pts = {};
    n_contact_pts = [];
    ind = 1;
    if fc(1)>0
      support_bodies(ind) = obj.lfoot_idx;
      contact_pts{ind} = 1:4;
      n_contact_pts(ind) = 4;
      ind=ind+1;
    end
    if fc(2)>0
      support_bodies(ind) = obj.rfoot_idx;
      contact_pts{ind} = 1:4;
      n_contact_pts(ind) = 4;
    end
    
%    supp = SupportState(r,support_bodies,contact_pts);
    supp.bodies = support_bodies;
    supp.contact_surfaces = 0*support_bodies;
    supp.contact_pts = contact_pts;
    supp.num_contact_pts = n_contact_pts;
    
    if (obj.use_mex==0 || obj.use_mex==2)
      kinsol = doKinematics(r,q,false,true,qd);

      active_supports = supp.bodies;
      active_contact_pts = supp.contact_pts;
      num_active_contacts = supp.num_contact_pts;      

      dim = 3; % 3D
      nd = 4; % for friction cone approx, hard coded for now
      float_idx = 1:6; % indices for floating base dofs
      act_idx = 7:nq; % indices for actuated dofs

      [H,C,B] = manipulatorDynamics(r,q,qd);

      H_float = H(float_idx,:);
      C_float = C(float_idx);

      H_act = H(act_idx,:);
      C_act = C(act_idx);
      B_act = B(act_idx,:);

      [xcom,J] = getCOM(r,kinsol);
      
      Jdot = forwardJacDot(r,kinsol,0);
      J = J(1:2,:); % only need COM x-y
      Jdot = Jdot(1:2,:);

      if ~isempty(active_supports)
        nc = sum(num_active_contacts);
        c_pre = 0;
        Dbar = [];
        for j=1:length(active_supports)
          [~,~,JB] = contactConstraintsBV(r,kinsol,false,struct('terrain_only',~obj.use_bullet,'body_idx',[1,active_supports(j)]));
          Dbar = [Dbar, vertcat(JB{:})'];
          c_pre = c_pre + length(active_contact_pts{j});
        end

        Dbar_float = Dbar(float_idx,:);
        Dbar_act = Dbar(act_idx,:);

        [~,Jp,Jpdot] = terrainContactPositions(r,kinsol,active_supports,true);
        Jp = sparse(Jp);
        Jpdot = sparse(Jpdot);

        xlimp = [xcom(1:2); J*qd]; % state of LIP model
        x_bar = xlimp - x0;      
      else
        nc = 0;
      end
      neps = nc*dim;


      %----------------------------------------------------------------------
      % Build handy index matrices ------------------------------------------

      nf = nc*nd; % number of contact force variables
      nparams = nq+nf+neps;
      Iqdd = zeros(nq,nparams); Iqdd(:,1:nq) = eye(nq);
      Ibeta = zeros(nf,nparams); Ibeta(:,nq+(1:nf)) = eye(nf);
      Ieps = zeros(neps,nparams);
      Ieps(:,nq+nf+(1:neps)) = eye(neps);


      %----------------------------------------------------------------------
      % Set up problem constraints ------------------------------------------

      lb = [-1e3*ones(1,nq) zeros(1,nf)   -obj.slack_limit*ones(1,neps)]'; % qddot/contact forces/slack vars
      ub = [ 1e3*ones(1,nq) 1e3*ones(1,nf) obj.slack_limit*ones(1,neps)]';

      Aeq_ = cell(1,length(varargin)+1);
      beq_ = cell(1,5);
      Ain_ = cell(1,2);
      bin_ = cell(1,2);

      % constrained dynamics
      if nc>0
        Aeq_{1} = H_float*Iqdd - Dbar_float*Ibeta;
      else
        Aeq_{1} = H_float*Iqdd;
      end
      beq_{1} = -C_float;

      % input saturation constraints
      % u=B_act'*(H_act*qdd + C_act - Jz_act'*z - Dbar_act*beta)

      if nc>0
        Ain_{1} = B_act'*(H_act*Iqdd - Dbar_act*Ibeta);
      else
        Ain_{1} = B_act'*H_act*Iqdd;
      end
      bin_{1} = -B_act'*C_act + r.umax;
      Ain_{2} = -Ain_{1};
      bin_{2} = B_act'*C_act - r.umin;

      if nc > 0
        % relative acceleration constraint
        Aeq_{2} = Jp*Iqdd + Ieps;
        beq_{2} = -Jpdot*qd - 1.0*Jp*qd;
      end

      eq_count=3;
      
      for ii=1:obj.n_body_accel_inputs
        if obj.body_accel_input_weights(ii) < 0
          body_input = varargin{ii+3};
          body_ind = body_input(1);
          body_vdot = body_input(2:7);
          if ~any(active_supports==body_ind)
            [~,J] = forwardKin(r,kinsol,body_ind,[0;0;0],1);
            Jdot = forwardJacDot(r,kinsol,body_ind,[0;0;0],1);
            cidx = ~isnan(body_vdot);
            Aeq_{eq_count} = J(cidx,:)*Iqdd;
            beq_{eq_count} = -Jdot(cidx,:)*qd + body_vdot(cidx);
            eq_count = eq_count+1;
          end
        end
      end

      if ~isempty(ctrl_data.constrained_dofs)
        % add joint acceleration constraints
        conmap = zeros(length(condof),nq);
        conmap(:,condof) = eye(length(condof));
        Aeq_{eq_count} = conmap*Iqdd;
        beq_{eq_count} = qddot_des(condof);
      end

      % linear equality constraints: Aeq*alpha = beq
      Aeq = sparse(vertcat(Aeq_{:}));
      beq = vertcat(beq_{:});

      % linear inequality constraints: Ain*alpha <= bin
      Ain = sparse(vertcat(Ain_{:}));
      bin = vertcat(bin_{:});

    
      %----------------------------------------------------------------------
      % QP cost function ----------------------------------------------------
      %
      %  min: quad(Jdot*qd + J*qdd,R_ls) + quad(C*x_bar+D*(Jdot*qd + J*qdd),Q) + (2*x_bar'*S + s1')*(A*x_bar + B*(Jdot*qd + J*qdd)) + w*quad(qddot_ref - qdd) + 0.001*quad(epsilon)
      if nc > 0
        Hqp = Iqdd'*J'*R_DQyD_ls*J*Iqdd;
        Hqp(1:nq,1:nq) = Hqp(1:nq,1:nq) + obj.w*eye(nq);
        
        fqp = xlimp'*C_ls'*Qy*D_ls*J*Iqdd;
        fqp = fqp + qd'*Jdot'*R_DQyD_ls*J*Iqdd;
        fqp = fqp + (x_bar'*S + 0.5*s1')*B_ls*J*Iqdd;
        fqp = fqp - u0'*R_ls*J*Iqdd;
        fqp = fqp - y0'*Qy*D_ls*J*Iqdd;
        fqp = fqp - obj.w*qddot_des'*Iqdd;
        
        Hqp(nq+(1:nf),nq+(1:nf)) = obj.w_grf*eye(nf); 
        Hqp(nparams-neps+1:end,nparams-neps+1:end) = obj.w_slack*eye(neps); 
      else
        Hqp = Iqdd'*Iqdd;
        fqp = -qddot_des'*Iqdd;
      end

      for ii=1:obj.n_body_accel_inputs
        w = obj.body_accel_input_weights(ii);
        if w>0
          body_input = varargin{ii+3};
          body_ind = body_input(1);
          body_vdot = body_input(2:7);
          if ~any(active_supports==body_ind)
            [~,J] = forwardKin(r,kinsol,body_ind,[0;0;0],1);
            Jdot = forwardJacDot(r,kinsol,body_ind,[0;0;0],1);
            cidx = ~isnan(body_vdot);
            Hqp(1:nq,1:nq) = Hqp(1:nq,1:nq) + w*J(cidx,:)'*J(cidx,:);
            fqp = fqp + w*(qd'*Jdot(cidx,:)'- body_vdot(cidx)')*J(cidx,:)*Iqdd;
          end
        end
      end
      
      %----------------------------------------------------------------------
      % Solve QP ------------------------------------------------------------

      REG = 1e-8;

      IR = eye(nparams);  
      lbind = lb>-999;  ubind = ub<999;  % 1e3 was used like inf above... right?
      Ain_fqp = full([Ain; -IR(lbind,:); IR(ubind,:)]);
      bin_fqp = [bin; -lb(lbind); ub(ubind)];

      if obj.use_mex ~= 2
      % call fastQPmex first
      QblkDiag = {Hqp(1:nq,1:nq) + REG*eye(nq), ...
                  obj.w_grf*ones(nf,1) + REG*ones(nf,1), ...
                  obj.w_slack*ones(neps,1) + REG*ones(neps,1)};
      Aeq_fqp = full(Aeq);

      % NOTE: model.obj is 2* f for fastQP!!!
      [alpha,info_fqp] = fastQPmex(QblkDiag,fqp,Ain_fqp,bin_fqp,Aeq_fqp,beq,ctrl_data.qp_active_set);
      else
        info_fqp = -1;
      end
      
      if info_fqp<0
        % then call gurobi
%         disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!failed over to gurobi');
        model.Q = sparse(Hqp + REG*eye(nparams));
        model.A = [Aeq; Ain];
        model.rhs = [beq; bin];
        model.sense = [obj.eq_array(1:length(beq)); obj.ineq_array(1:length(bin))];
        model.lb = lb;
        model.ub = ub;

        model.obj = fqp;
        if obj.gurobi_options.method==2
          % see drake/algorithms/QuadraticProgram.m solveWGUROBI
          model.Q = .5*model.Q;
        end

        if (any(any(isnan(model.Q))) || any(isnan(model.obj)) || any(any(isnan(model.A))) || any(isnan(model.rhs)) || any(isnan(model.lb)) || any(isnan(model.ub)))
          keyboard;
        end

  %         qp_tic = tic;
        result = gurobi(model,obj.gurobi_options);
  %         qp_toc = toc(qp_tic);
  %         fprintf('QP solve: %2.4f\n',qp_toc);

        alpha = result.x;
      end

      qp_active_set = find(abs(Ain_fqp*alpha - bin_fqp)<1e-6);
      setField(obj.controller_data,'qp_active_set',qp_active_set);

      %----------------------------------------------------------------------
      % Solve for inputs ----------------------------------------------------

      qdd = alpha(1:nq);
      if nc>0
        beta = alpha(nq+(1:nf));
        u = B_act'*(H_act*qdd + C_act - Dbar_act*beta);
      else
        u = B_act'*(H_act*qdd + C_act);
      end
      y = u;
 
      if (obj.use_mex==2)
        des.y = y;
      end
      
      % compute V,Vdot for controller status updates
      if (nc>0)
        %V = x_bar'*S*x_bar + s1'*x_bar + s2;
        %Vdot = (2*x_bar'*S + s1')*(A_ls*x_bar + B_ls*(Jdot*qd + J*qdd)) + x_bar'*Sdot*x_bar + x_bar'*s1dot + s2dot;
        % note for ZMP dynamics, S is constant so Sdot=0
      
        Vdot = (2*x_bar'*S + s1')*(A_ls*x_bar + B_ls*(Jdot*qd + J*qdd)) + x_bar'*s1dot + s2dot;
      end
    end
  
    if (obj.use_mex==1)
      if obj.using_flat_terrain
        height = getTerrainHeight(r,[0;0]); % get height from DRCFlatTerrainMap
      else
        height = 0;
      end
      [y,qdd,info,active_supports,Vdot] = QPControllermex(obj.mex_ptr.data,obj.solver==0,qddot_des,x,...
          varargin{4:end},condof,supp,A_ls,B_ls,Qy,R_ls,C_ls,D_ls,...
          S,s1,s1dot,s2dot,x0,u0,y0,mu,height);
    end

    if ~isempty(active_supports)
      setVdot(obj.controller_data,Vdot);
    else
      setVdot(obj.controller_data,0);
    end
    
    if (obj.use_mex==2)
      % note: this only works when using gurobi
      if obj.using_flat_terrain
        height = getTerrainHeight(r,[0;0]); % get height from DRCFlatTerrainMap
      else
        height = 0;
      end
      [y,qdd,info,active_supports_mex,Vdotmex,Q,gobj,A,rhs,sense,lb,ub] = ...
          QPControllermex(obj.mex_ptr.data,obj.solver==0,qddot_des,x,...
          varargin{4:end},condof,supp,A_ls,B_ls,Qy,R_ls,C_ls,D_ls,...
          S,s1,s1dot,s2dot,x0,u0,y0,mu,height);
      if (nc>0)
        valuecheck(active_supports_mex,active_supports);
        % TODO: fix this
        %valuecheck(Vdotmex,Vdot,1e-3);
      end
%       valuecheck(Q'+Q,model.Q'+model.Q,1e-8);
%       valuecheck(gobj,model.obj,1e-8);
      % had to comment out equality constraints because the contact
      % jacobian rows can be permuted between matlab/mex
      %valuecheck(A,model.A,1e-8);
      %valuecheck(rhs,model.rhs,1e-8);
%       valuecheck(sense',model.sense);
%       valuecheck(lb,model.lb,1e-8);
%       valuecheck(ub,model.ub,1e-8);
      valuecheck(y,des.y,0.5);
    end   
      
    if (1)     % simple timekeeping for performance optimization
      % note: also need to uncomment tic at very top of this method
      out_toc=toc(out_tic);
      persistent average_tictoc average_tictoc_n;
      if isempty(average_tictoc)
        average_tictoc = out_toc;
        average_tictoc_n = 1;
      else
        average_tictoc = (average_tictoc_n*average_tictoc + out_toc)/(average_tictoc_n+1);
        average_tictoc_n = average_tictoc_n+1;
      end
      if mod(average_tictoc_n,50)==0
        fprintf('Average control output duration: %2.4f\n',average_tictoc);
      end
    end
    
    if obj.output_qdd
      varargout = {y,qdd};
    else
      varargout = {y};
    end
  end
  end

  properties (SetAccess=private)
    robot; % to be controlled
    numq;
    controller_data; % shared data handle that holds S, h, foot trajectories, etc.
    w; % objective function weight
    w_grf; % scalar ground reaction force weight
    w_slack; % scalar slack var weight
    slack_limit; % maximum absolute magnitude of acceleration slack variable values
    rfoot_idx;
    lfoot_idx;
    rhand_idx;
    lhand_idx;
    pelvis_idx;
    gurobi_options = struct();
    solver=0;
    debug;
    debug_pub;
    use_mex;
    use_hand_ft;
    mex_ptr;
    lc;
    eq_array = repmat('=',100,1); % so we can avoid using repmat in the loop
    ineq_array = repmat('<',100,1); % so we can avoid using repmat in the loop
    use_bullet;
    using_flat_terrain; % true if using DRCFlatTerrain
    jlmin;
    jlmax;
    output_qdd = false;
    body_accel_input_weights; % array of doubles, negative values signal constraints
    n_body_accel_inputs; % scalar
  end
end
