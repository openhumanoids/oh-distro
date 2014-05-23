classdef MomentumControlBlock < MIMODrakeSystem

  methods
  function obj = MomentumControlBlock(r,body_accel_input_frames,controller_data,options)
    % @param r atlas instance
    % @param controller_data shared data handle containing ZMP-LQR solution, etc
    % @param options structure for specifying objective weight (w), slack
    % variable limits (slack_limit)
    typecheck(r,'Biped');
    typecheck(controller_data,'SharedDataHandle');
    
    if nargin>3
      typecheck(options,'struct');
    else
      options = struct();
    end
    
    qddframe = AtlasCoordinates(r); % input frame for desired qddot, qdd constraints 
    
    if ~isfield(options,'input_foot_contacts')
      options.input_foot_contacts = false;
    else
      typecheck(options.input_foot_contacts,'logical');
    end    
    
    if options.input_foot_contacts
      input_frame = MultiCoordinateFrame({r.getStateFrame,qddframe,FootContactState,body_accel_input_frames{:}});
    else
      input_frame = MultiCoordinateFrame({r.getStateFrame,qddframe,body_accel_input_frames{:}});
    end

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
    obj.mass = getMass(r);
    obj.numq = getNumDOF(r);
    obj.controller_data = controller_data;
    obj.input_foot_contacts = options.input_foot_contacts;
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
    
    if isfield(options,'contact_threshold')
      % minimum height above terrain for points to be in contact
      typecheck(options.contact_threshold,'double');
      sizecheck(options.contact_threshold,[1 1]);
      obj.contact_threshold = options.contact_threshold;
    else
      obj.contact_threshold = 0.001;
    end
    
    % weight for the hdot objective term
    if isfield(options,'W_hdot')
      typecheck(options.W_hdot,'double');
      sizecheck(options.W_hdot,[6 6]);
      obj.W_hdot = options.W_hdot;
    else
      obj.W_hdot = diag([0.1;0.1;0.1;1;1;1]);
    end
    
    % weight for the desired qddot objective term
    if isfield(options,'w_qdd')
      typecheck(options.w_qdd,'double');
      sizecheck(options.w_qdd,[obj.numq 1]); % assume diagonal cost
      obj.w_qdd = options.w_qdd;
    else
      obj.w_qdd = 0.1*ones(obj.numq,1);
    end    

    % weight for grf coefficients
    if isfield(options,'w_grf')
      typecheck(options.w_grf,'double');
      sizecheck(options.w_grf,1);
      obj.w_grf = options.w_grf;
    else
      obj.w_grf = 0.001;
    end    

    % weight for slack vars
    if isfield(options,'w_slack')
      typecheck(options.w_slack,'double');
      sizecheck(options.w_slack,1);
      obj.w_slack = options.w_slack;
    else
      obj.w_slack = 0.001;
    end    

    % com-z PD gains
    if isfield(options,'Kp')
      typecheck(options.Kp,'double');
      sizecheck(options.Kp,1);
      obj.Kp = options.Kp;
    else
      obj.Kp = 100;
    end    

    % com-z PD gains
    if isfield(options,'Kd')
      typecheck(options.Kd,'double');
      sizecheck(options.Kd,1);
      obj.Kd = options.Kd;
    else
      obj.Kd = 20;
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
    
    if isfield(options,'lcm_foot_contacts')
      warning('lcm_foot_contacts option no longer supported, use input_foot_contacts instead');
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

    if isfield(options,'use_mex')
      % 0 - no mex
      % 1 - use mex
      % 2 - run mex and non-mex and valuecheck the result
      sizecheck(options.use_mex,1);
      obj.use_mex = uint32(options.use_mex);
      rangecheck(obj.use_mex,0,2);
      if (obj.use_mex && exist('MomentumControllermex','file')~=3)
        error('can''t find MomentumControllermex.  did you build it?');
      end
    else
      obj.use_mex = 1;
    end
    
    % specifies whether or not to solve QP for all DOFs or just the
    % important subset
    if (isfield(options,'full_body_opt'))
      warning('full_body_opt option no longer supported --- controller is always full body.')
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
    
    obj.lc = lcm.lcm.LCM.getSingleton();
    obj.rfoot_idx = findLinkInd(r,'r_foot');
    obj.lfoot_idx = findLinkInd(r,'l_foot');
    obj.rhand_idx = findLinkInd(r,'r_hand');
    obj.lhand_idx = findLinkInd(r,'l_hand');
    obj.pelvis_idx = findLinkInd(r,'pelvis');    

    if isfield(options,'smooth_contacts')
      typecheck(options.smooth_contacts,'logical');
      obj.smooth_contacts = options.smooth_contacts;
    else
      obj.smooth_contacts = false;
    end
    
    if isfield(obj.controller_data.data,'contact_force_bounds')
      % should be a struct with array fields 'force_bound', and 'force_delta'
      typecheck(obj.controller_data.data.contact_force_bounds,'struct');
      assert(isfield(obj.controller_data.data.contact_force_bounds,'force_bound'));
      assert(isfield(obj.controller_data.data.contact_force_bounds,'force_delta'));
    else
      cfb = struct();
      cfb.force_bound = zeros(r.getNumBodies,1);
      cfb.force_bound(obj.lfoot_idx) = obj.fmax;
      cfb.force_bound(obj.rfoot_idx) = obj.fmax;
      cfb.force_delta = zeros(r.getNumBodies,1);
      obj.controller_data.setField('contact_force_bounds',cfb);  
    end
      
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

      obj.mex_ptr = SharedDataHandle(MomentumControllermex(0,obj,obj.robot.getMexModelPtr.ptr,getB(obj.robot),r.umin,r.umax,terrain_map_ptr,multi_robot_ptr));
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
  
  methods
    
  function varargout=mimoOutput(obj,t,~,varargin)
    persistent infocount active_supports_prev
%     out_tic = tic;
    if isempty(infocount)
      infocount = 0;
    end
    
    ctrl_data = obj.controller_data.data;
      
    x = varargin{1};
    qddot_des = varargin{2};
       
    r = obj.robot;
    nq = obj.numq; 
    q = x(1:nq); 
    qd = x(nq+(1:nq)); 

    x0 = ctrl_data.x0 - [ctrl_data.trans_drift(1:2);0;0]; % for x-y plan adjustment
    if (ctrl_data.is_time_varying)
      % extract current supports
      supp_idx = find(ctrl_data.support_times<=t,1,'last');
      supp = ctrl_data.supports(supp_idx);
      
      force_bound = ctrl_data.contact_force_bounds.force_bound;
      force_delta = ctrl_data.contact_force_bounds.force_delta;
      
      if obj.smooth_contacts
        % look ahead to see if we're going to break contact
        t_lookahead = 0.3;
        if ctrl_data.support_times(supp_idx+1)-t < t_lookahead 
          supp_next = ctrl_data.supports(supp_idx+1);
          if length(supp.bodies) > length(supp_next.bodies)
            % we're going to break contact in less than t_lookahead secs
            foot_to_break_contact = setdiff(supp.bodies,supp_next.bodies);
            force_delta(foot_to_break_contact) = -(obj.fmax/(t_lookahead/0.002));
          end
        end
      end
      
      y0 = fasteval(ctrl_data.K.y0,t) - ctrl_data.trans_drift(1:2); % for x-y plan adjustment
      K = fasteval(ctrl_data.K.D,t); % always constant for ZMP dynamics
    else
      supp = ctrl_data.supports;
      y0 = [0;0]; 
      K = ctrl_data.K.D; % always constant for ZMP dynamics
    end

    if isfield(ctrl_data,'comz_traj')
      comz_des = fasteval(ctrl_data.comz_traj,t);
      dcomz_des = fasteval(ctrl_data.dcomz_traj,t);
      ddcomz_des = fasteval(ctrl_data.ddcomz_traj,t);
    else
      comz_des = 1.03;
      dcomz_des = 0;
      ddcomz_des = 0;
    end
    
    condof = ctrl_data.constrained_dofs; % dof indices for which q_ddd_des is a constraint
        
    % contact_sensor = -1 (no info), 0 (info, no contact), 1 (info, yes contact)
    contact_sensor=-1+0*supp.bodies;  % initialize to -1 for all
    if obj.input_foot_contacts
      % note: since changing this to input frame, always have 0 or 1
      fc = varargin{3};
      contact_sensor(supp.bodies==obj.lfoot_idx) = fc(1);
      contact_sensor(supp.bodies==obj.rfoot_idx) = fc(2);
    end
    
    if (obj.use_mex==0 || obj.use_mex==2)
      kinsol = doKinematics(r,q,false,true,qd);

      % get active contacts
      i=1;
      while i<=length(supp.bodies)
        if ctrl_data.ignore_terrain
          % use all desired supports UNLESS we have sensor information saying no contact
          if (contact_sensor(i)==0) 
            supp = removeBody(supp,i); 
            contact_sensor(i)=[];
            i=i-1;
          end
        else
          % check kinematic contact
          phi = contactConstraints(r,kinsol,false,struct('terrain_only',~obj.use_bullet,'body_idx',[1,supp.bodies(i)]));
          contact_state_kin = any(phi<=obj.contact_threshold);

          if (~contact_state_kin && contact_sensor(i)<1) 
            % no contact from kin, no contact (or no info) from sensor
            supp = removeBody(supp,i); 
            contact_sensor(i)=[];
            i=i-1;
          end
        end
        i=i+1;
      end
      active_supports = (supp.bodies)';
      active_surfaces = supp.contact_surfaces;
      active_contact_pts = supp.contact_pts;
      num_active_contacts = supp.num_contact_pts;      
      
      if obj.smooth_contacts
        if length(active_supports_prev) < length(active_supports)
          % we just made contact
          foot_to_make_contact = setdiff(active_supports,active_supports_prev);
          force_delta(foot_to_make_contact) = 5;
        end
      end
      
      active_supports_prev = active_supports;
      force_bound = min(obj.fmax,max(0,force_bound + force_delta));


      %----------------------------------------------------------------------

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
      [A,Adot] = getCMM(r,kinsol,qd);

      com_dot = J*qd;
      z_com_dot = com_dot(3);
      J = J(1:2,:); % only need COM x-y

      if ~isempty(active_supports)
        nc = sum(num_active_contacts);
        fbound = zeros(1,nc*nd);
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

        ustar = K*x_bar + y0; % ustar==u_bar since u_nom=0
      else
        nc = 0;
        ustar = zeros(2,1);
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
      ub = [ 1e3*ones(1,nq) fbound obj.slack_limit*ones(1,neps)]';

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
        beq_{2} = -Jpdot*qd;
      end

      eq_count=3+obj.input_foot_contacts*1;
      for ii=3:length(varargin)
        if obj.body_accel_input_weights(ii-2) < 0
          body_input = varargin{ii};
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

      % compute desired linear momentum
      comddot_des = [ustar; obj.Kp*(comz_des-xcom(3)) + obj.Kd*(dcomz_des-z_com_dot) + ddcomz_des];
      ldot_des = comddot_des * obj.mass;
      h=A*qd;
      k=h(1:3);
      kdot_des = -5.0 *k; 
      hdot_des = [kdot_des; ldot_des];

      %----------------------------------------------------------------------
      % QP cost function ----------------------------------------------------
      %
      %  min: quad(h_dot_des - Adot*qd - A*qdd) + w*quad(qddot_ref - qdd) + 0.001*quad(epsilon)
      if nc > 0
        Hqp = Iqdd'*A'*obj.W_hdot*A*Iqdd;
        Hqp(1:nq,1:nq) = Hqp(1:nq,1:nq) + diag(obj.w_qdd);

        fqp = qd'*Adot'*obj.W_hdot*A*Iqdd;
        fqp = fqp - hdot_des'*obj.W_hdot*A*Iqdd;
        fqp = fqp - (obj.w_qdd.*qddot_des)'*Iqdd;

        Hqp(nq+(1:nf),nq+(1:nf)) = obj.w_grf*eye(nf); 
        Hqp(nparams-neps+1:end,nparams-neps+1:end) = obj.w_slack*eye(neps); 
      else
        Hqp = Iqdd'*Iqdd;
        fqp = -qddot_des'*Iqdd;
      end

      for ii=1:obj.n_body_accel_inputs
        w = obj.body_accel_input_weights(ii);
        if w>0
          body_input = varargin{ii+2};
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

      % call fastQPmex first
      QblkDiag = {Hqp(1:nq,1:nq) + REG*eye(nq), ...
        obj.w_grf*ones(nf,1) + REG*ones(nf,1), ...
        obj.w_slack*ones(neps,1) + REG*ones(neps,1)};
      
      Aeq_fqp = full(Aeq);
      % NOTE: model.obj is 2* f for fastQP!!!
      [alpha,info_fqp] = fastQPmex(QblkDiag,fqp,Ain_fqp,bin_fqp,Aeq_fqp,beq,ctrl_data.qp_active_set);

      if info_fqp<0
        % then call gurobi
        disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!failed over to gurobi');
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
      
      if obj.debug
        % publish debug 
        debug_data.utime = t*1e6;
        debug_data.alpha = alpha;
        debug_data.u = y;
        debug_data.active_supports = active_supports;
        debug_data.info = info_fqp;
        debug_data.qddot_des = qddot_des;
        debug_data.active_constraints = qp_active_set;
        debug_data.h = h;
        debug_data.hdot_des = hdot_des;
        debug_data.r_foot_contact = any(obj.rfoot_idx==active_supports);
        debug_data.l_foot_contact = any(obj.lfoot_idx==active_supports);
        body_accel_input_start=3+obj.input_foot_contacts*1;
        debug_data.body_acc_des = [];%[varargin{body_accel_input_start}; varargin{body_accel_input_start+1}];
        debug_data.lb = lb;
        debug_data.ub = ub;
        obj.debug_pub.publish(debug_data);
      end
      
    end
  
    if (obj.use_mex==1 || obj.use_mex==2)
      if ctrl_data.ignore_terrain
        contact_thresh =-1;       
      else
        contact_thresh = obj.contact_threshold;
      end
      if obj.using_flat_terrain
        height = getTerrainHeight(r,[0;0]); % get height from DRCFlatTerrainMap
      else
        height = 0;
      end
      body_accel_input_start=3+obj.input_foot_contacts*1;
      if obj.use_mex==1
        mu = 0.75;
        if obj.debug
          [y,qdd,info,active_supports,Hqp_mex,fqp_mex,Aeq_mex,beq_mex,Ain_mex,bin_mex,Qf,Qeps,alpha,...
            active_constraints,h,hdot_des,force_bound,force_delta] = MomentumControllermex(obj.mex_ptr.data,...
            obj.solver==0,qddot_des,x,varargin{body_accel_input_start:end},condof,supp,K,x0,y0,comz_des,...
            dcomz_des,ddcomz_des,mu,contact_sensor,contact_thresh,height,ctrl_data.contact_force_bounds.force_bound,force_delta);

          % publish debug 
          debug_data.utime = t*1e6;
          debug_data.alpha = alpha;
          debug_data.u = y;
          debug_data.active_supports = active_supports;
          debug_data.info = info;
          debug_data.qddot_des = qddot_des;
          debug_data.active_constraints = active_constraints;
          debug_data.h = h;
          debug_data.hdot_des = hdot_des;
          debug_data.r_foot_contact = any(obj.rfoot_idx==active_supports);
          debug_data.l_foot_contact = any(obj.lfoot_idx==active_supports);
          debug_data.body_acc_des = [];%[varargin{body_accel_input_start}; varargin{body_accel_input_start+1}; varargin{body_accel_input_start+2}; varargin{body_accel_input_start+3}];
          
          np=length(alpha);
          bounds = bin_mex((end-2*np+1):end);
          debug_data.lb = -bounds(1:np);
          debug_data.ub = bounds(np + (1:np));
          obj.debug_pub.publish(debug_data);

        else
          [y,qdd,info] = MomentumControllermex(obj.mex_ptr.data,obj.solver==0,qddot_des,x,...
            varargin{body_accel_input_start:end},condof,supp,K,x0,y0,comz_des,dcomz_des,ddcomz_des,mu,...
            contact_sensor,contact_thresh,height,ctrl_data.contact_force_bounds.force_bound,force_delta);
        end
        
        if info < 0
          infocount = infocount +1;
        else
          infocount = 0;
        end
        if infocount > 4
          % kill atlas
          disp('freezing atlas!');
          behavior_pub = AtlasBehaviorModePublisher('ATLAS_BEHAVIOR_COMMAND');
          d.utime = 0;
          d.command = 'freeze';
          behavior_pub.publish(d);
        end      
        
      else
        mu = 1.0;
        [y_mex,mex_qdd,~,active_supports_mex,Hqp_mex,fqp_mex,Aeq_mex,beq_mex,Ain_mex,bin_mex,Qf,Qeps,alpha_mex,...
          active_constraints,h,hdot_des,force_bound,force_delta] = MomentumControllermex(obj.mex_ptr.data,obj.solver==0,qddot_des,x,...
          varargin{body_accel_input_start:end},condof,supp,K,x0,y0,comz_des,dcomz_des,ddcomz_des,mu,...
          contact_sensor,contact_thresh,height,ctrl_data.contact_force_bounds.force_bound,force_delta);
        if (nc>0)
          valuecheck(active_supports_mex,active_supports);
        end
        if size(Hqp_mex,2)==1
          Hqp_mex=diag(Hqp_mex);
        end
        if info_mex < 0
          Hqp_mex=Hqp_mex*2;
          Qf=Qf*2;
          Qeps=Qeps*2;
        end
        try
          valuecheck(Hqp,blkdiag(Hqp_mex,diag(Qf),diag(Qeps)),1e-6);
        catch err
          keyboard
        end
        if (nc>0)
          valuecheck(active_supports_mex,active_supports);
				end
        valuecheck(fqp',fqp_mex,1e-6);
        % had to comment out equality constraints because the contact
        % jacobian rows can be permuted between matlab/mex
%         valuecheck(Aeq,Aeq_mex(1:length(beq),:),1e-6); 
%         valuecheck(beq,beq_mex(1:length(beq)),1e-6); 
        valuecheck(Ain,Ain_mex(1:length(bin),:),1e-6);
        valuecheck(bin,bin_mex(1:length(bin)),1e-6); 
				valuecheck([-lb;ub],bin_mex(length(bin)+1:end),1e-6);
        valuecheck(y,y_mex,1e-2); 
				valuecheck(qdd,mex_qdd,1e-2); 
      end
    end

    if (0)     % simple timekeeping for performance optimization
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
		
    contact_force_bounds.force_bound = force_bound;
    contact_force_bounds.force_delta = force_delta;
    obj.controller_data.setField('contact_force_bounds',contact_force_bounds);
  end
  end

  properties (SetAccess=private)
    robot; % to be controlled
    numq;
    controller_data; % shared data handle that holds S, h, foot trajectories, etc.
    W_hdot; % angular momentum cost term weight matrix
    w_qdd; % qdd objective function weight vector
    w_grf; % scalar ground reaction force weight
    w_slack; % scalar slack var weight
    Kp; % com-z P gain
    Kd; % com-z D gain
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
    input_foot_contacts;  
    eq_array = repmat('=',100,1); % so we can avoid using repmat in the loop
    ineq_array = repmat('<',100,1); % so we can avoid using repmat in the loop
    use_bullet;
    using_flat_terrain; % true if using DRCFlatTerrain
    jlmin;
    jlmax;
    fmax = 1000; % N, global contact force maximum
    smooth_contacts;
    contact_threshold; % min height above terrain to be considered in contact
    output_qdd = false;
    mass; % total robot mass
    body_accel_input_weights; % array of doubles, negative values signal constraints
    n_body_accel_inputs; % scalar
  end
end
