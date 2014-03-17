classdef MomentumControlBlock < MIMODrakeSystem

  methods
  function obj = MomentumControlBlock(r,body_motion_input_frames,controller_data,options)
    % @param r atlas instance
    % @param controller_data shared data handle containing ZMP-LQR solution, etc
    % @param options structure for specifying objective weight (w), slack
    % variable limits (slack_limit)
    typecheck(r,'Atlas');
    typecheck(controller_data,'SharedDataHandle');
    
    if nargin>3
      typecheck(options,'struct');
    else
      options = struct();
    end
    
    qddframe = AtlasCoordinates(r); % input frame for desired qddot, qdd constraints 
    input_frame = MultiCoordinateFrame({r.getStateFrame,qddframe,body_motion_input_frames{:}});

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
    obj.controller_data = controller_data;

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
    
    % weight for desired qddot objective term
    if isfield(options,'W')
      typecheck(options.W,'double');
      sizecheck(options.W,6);
      obj.W = options.W;
    else
      obj.W = diag([0.1;0.1;0.1;1.0;1.0;1.0]);
    end
   
    % weight for desired qddot objective term
    if isfield(options,'w')
      typecheck(options.w,'double');
      sizecheck(options.w,1);
      obj.w = options.w;
    else
      obj.w = 0.1;
    end

    % hard bound on slack variable values
    if isfield(options,'slack_limit')
      typecheck(options.slack_limit,'double');
      sizecheck(options.slack_limit,1);
      obj.slack_limit = options.slack_limit;
    else
      obj.slack_limit = 10;
    end
    
    if ~isfield(options,'lcm_foot_contacts')
      obj.lcm_foot_contacts=true; % listen for foot contacts over LCM
    else
      typecheck(options.lcm_foot_contacts,'logical');
      obj.lcm_foot_contacts=options.lcm_foot_contacts;
    end
    
    if isfield(options,'debug')
      typecheck(options.debug,'logical');
      sizecheck(options.debug,1);
      obj.debug = options.debug;
    else
      obj.debug = false;
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

    obj.lc = lcm.lcm.LCM.getSingleton();
    obj.rfoot_idx = findLinkInd(r,'r_foot');
    obj.lfoot_idx = findLinkInd(r,'l_foot');
    obj.rhand_idx = findLinkInd(r,'r_hand');
    obj.lhand_idx = findLinkInd(r,'l_hand');
    obj.pelvis_idx = findLinkInd(r,'pelvis');
    obj.numq = getNumDOF(r);
    
    if obj.lcm_foot_contacts
      obj.contact_est_monitor = drake.util.MessageMonitor(drc.foot_contact_estimate_t,'utime');
      obj.lc.subscribe('FOOT_CONTACT_ESTIMATE',obj.contact_est_monitor);
    end % else estimate contact via kinematics
    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% NOTE: these parameters need to be set in QPControllermex.cpp, too %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
    obj.solver_options.outputflag = 0; % not verbose
    obj.solver_options.method = 2; % -1=automatic, 0=primal simplex, 1=dual simplex, 2=barrier
    obj.solver_options.presolve = 0;
    % obj.solver_options.prepasses = 1;

    if obj.solver_options.method == 2
      obj.solver_options.bariterlimit = 20; % iteration limit
      obj.solver_options.barhomogeneous = 0; % 0 off, 1 on
      obj.solver_options.barconvtol = 5e-4;
    end
    
    if (obj.use_mex>0)
      terrain = getTerrain(r);
      if isa(terrain,'DRCTerrainMap') 
        terrain_map_ptr = terrain.map_handle.getPointerForMex();
      else
        terrain_map_ptr = 0;
      end
      if isa(obj.multi_robot,'TimeSteppingRigidBodyManipulator')
        multi_robot_ptr = obj.multi_robot.getMexModelPtr.ptr;
      else
        multi_robot_ptr = 0;
      end
      obj.mex_ptr = SharedDataHandle(MomentumControllermex(0,obj,obj.robot.getMexModelPtr.ptr,getB(obj.robot),length(body_motion_input_frames),r.umin,r.umax,terrain_map_ptr,multi_robot_ptr));
    end

    obj.num_body_contacts=zeros(getNumBodies(r),1);
    for i=1:getNumBodies(r)
      obj.num_body_contacts(i) = length(getBodyContacts(r,i));
    end
    
    
    if isa(getTerrain(r),'DRCFlatTerrainMap')
      obj.using_flat_terrain = true;      
    else
      obj.using_flat_terrain = false;
    end
    
    obj.lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'momentum-control-block-debug');

    [obj.jlmin, obj.jlmax] = getJointLimits(r);
        
		obj.output_qdd = options.output_qdd;
  end

  end
  
  methods
    
  function varargout=mimoOutput(obj,t,~,varargin)
    out_tic = tic;
    ctrl_data = obj.controller_data.data;
      
    x = varargin{1};
    q_ddot_des = varargin{2};
       
    r = obj.robot;
    nq = obj.numq; 
    q = x(1:nq); 
    qd = x(nq+(1:nq)); 

    x0 = ctrl_data.x0;
    if (ctrl_data.is_time_varying)
      % extract current supports
      supp_idx = find(ctrl_data.support_times<=t,1,'last');
      supp = ctrl_data.supports(supp_idx);
    else
      supp = ctrl_data.supports;
    end
    y0 = ctrl_data.K.y0.eval(t); 
    K = ctrl_data.K.D.eval(t); % always constant for ZMP dynamics
    
    % contact_sensor = -1 (no info), 0 (info, no contact), 1 (info, yes contact)
    contact_sensor=-1+0*supp.bodies;  % initialize to -1 for all
    if obj.lcm_foot_contacts
      % get foot contact state over LCM
      contact_data = obj.contact_est_monitor.getMessage();
      if ~isempty(contact_data)
        msg = drc.foot_contact_estimate_t(contact_data);
        contact_sensor(supp.bodies==obj.lfoot_idx) = msg.left_contact;
        contact_sensor(supp.bodies==obj.rfoot_idx) = msg.right_contact;
      end
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
          if supp.contact_surfaces(i) == 0
            phi = contactConstraints(r,kinsol,supp.bodies(i),supp.contact_pts{i});
          else
            % use bullet collision between bodies
            phi = pairwiseContactConstraints(obj.multi_robot,kinsol_multi,supp.bodies(i),supp.contact_surfaces(i),supp.contact_pts{i});
          end
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
        c_pre = 0;
        Dbar = [];
        for j=1:length(active_supports)
          if active_surfaces(j) == 0
            [~,~,JB] = contactConstraintsBV(r,kinsol,active_supports(j),active_contact_pts{j});
          else
            % use bullet collision between bodies
            [~,~,JB] = pairwiseContactConstraintsBV(obj.multi_robot,kinsol_multi,active_supports(j),active_surfaces(j),active_contact_pts{j});
          end
          Dbar = [Dbar, [JB{:}]];
          c_pre = c_pre + length(active_contact_pts{j});

        end

        Dbar_float = Dbar(float_idx,:);
        Dbar_act = Dbar(act_idx,:);

        [~,Jp,Jpdot] = contactPositionsJdot(r,kinsol,active_supports,active_contact_pts);
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
      ub = [ 1e3*ones(1,nq) 500*ones(1,nf) obj.slack_limit*ones(1,neps)]';

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

      eq_count=3;
      for ii=3:length(varargin)
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
% 
%       if ~isempty(ctrl_data.constrained_dofs)
%         % add joint acceleration constraints
%         condof = ctrl_data.constrained_dofs;
%         conmap = zeros(length(condof),nq);
%         conmap(:,condof) = eye(length(condof));
%         Aeq_{eq_count} = conmap*Iqdd;
%         beq_{eq_count} = q_ddot_des(condof);
%       end

      % linear equality constraints: Aeq*alpha = beq
      Aeq = sparse(vertcat(Aeq_{:}));
      beq = vertcat(beq_{:});

      % linear inequality constraints: Ain*alpha <= bin
      Ain = sparse(vertcat(Ain_{:}));
      bin = vertcat(bin_{:});

      % compute desired linear momentum
  %     comz_t = fasteval(ctrl_data.comztraj,t);
  %     dcomz_t = fasteval(ctrl_data.dcomztraj,t);
      comddot_des = [ustar; 150*(1.04-xcom(3)) + 10*(0-z_com_dot)];
  %     comddot_des = [ustar; 10*(comz_t-xcom(3)) + 0.5*(dcomz_t-z_com_dot)];
      ldot_des = comddot_des * 161;
      k = A(1:3,:)*qd;
  %     kdot_des = 10.0 * (ctrl_data.ktraj.eval(t) - k); 
      kdot_des = -5.0 *k; 
      hdot_des = [kdot_des; ldot_des];

      %----------------------------------------------------------------------
      % QP cost function ----------------------------------------------------
      %
      %  min: quad(h_dot_des - Adot*qd - A*qdd) + w*quad(qddot_ref - qdd) + 0.001*quad(epsilon)
      if nc > 0
        Hqp = Iqdd'*A'*obj.W*A*Iqdd;
        Hqp(1:nq,1:nq) = Hqp(1:nq,1:nq) + obj.w*eye(nq);

        fqp = qd'*Adot'*obj.W*A*Iqdd;
        fqp = fqp - hdot_des'*obj.W*A*Iqdd;
        fqp = fqp - obj.w*q_ddot_des'*Iqdd;

        % quadratic slack var cost 
        Hqp(nparams-neps+1:end,nparams-neps+1:end) = 0.001*eye(neps); 
      else
        Hqp = Iqdd'*Iqdd;
        fqp = -q_ddot_des'*Iqdd;
      end

      %----------------------------------------------------------------------
      % Solve QP ------------------------------------------------------------

      REG = 1e-8;

      IR = eye(nparams);  
      lbind = lb>-999;  ubind = ub<999;  % 1e3 was used like inf above... right?
      Ain_fqp = full([Ain; -IR(lbind,:); IR(ubind,:)]);
      bin_fqp = [bin; -lb(lbind); ub(ubind)];

      % call fastQPmex first
      QblkDiag = {Hqp(1:nq,1:nq) + REG*eye(nq),zeros(nf,1)+ REG*ones(nf,1),0.001*ones(neps,1)+ REG*ones(neps,1)};
      Aeq_fqp = full(Aeq);
      % NOTE: model.obj is 2* f for fastQP!!!
      [alpha,info_fqp] = fastQPmex(QblkDiag,fqp,Ain_fqp,bin_fqp,Aeq_fqp,beq,ctrl_data.qp_active_set);

      if info_fqp<0
        % then call gurobi
        disp('failed over to gurobi');
        model.Q = sparse(Hqp + REG*eye(nparams));
        model.A = [Aeq; Ain];
        model.rhs = [beq; bin];
        model.sense = [obj.eq_array(1:length(beq)); obj.ineq_array(1:length(bin))];
        model.lb = lb;
        model.ub = ub;

        model.obj = fqp;
        if obj.solver_options.method==2
          % see drake/algorithms/QuadraticProgram.m solveWGUROBI
          model.Q = .5*model.Q;
        end

        if (any(any(isnan(model.Q))) || any(isnan(model.obj)) || any(any(isnan(model.A))) || any(isnan(model.rhs)) || any(isnan(model.lb)) || any(isnan(model.ub)))
          keyboard;
        end

  %         qp_tic = tic;
        result = gurobi(model,obj.solver_options);
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
      mu = 1.0;
      if (obj.use_mex==1)
        [y,~,qdd] = MomentumControllermex(obj.mex_ptr.data,1,q_ddot_des,x,varargin{3:end}, ...
          supp,K,x0,y0,mu,contact_sensor,contact_thresh,height);
      else
        [y_mex,active_supports_mex,qdd,Hqp_mex,fqp_mex,Aeq_mex,beq_mex] = MomentumControllermex(obj.mex_ptr.data,1,q_ddot_des,x,varargin{3:end}, ...
          supp,K,x0,y0,mu,contact_sensor,contact_thresh,height);
        if (nc>0)
          valuecheck(active_supports_mex,active_supports);
        end
        valuecheck(y,y_mex,1e-3); 
        %valuecheck(Hqp(1:nq,1:nq),Hqp_mex,1e-6)
        %valuecheck(fqp',fqp_mex,1e-6);
        %valuecheck(Aeq,Aeq_mex(1:length(beq),:),1e-6)
        %valuecheck(beq,beq_mex(1:length(beq)),1e-6); 
      end
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
    W; % angular momentum cost term weight matrix
    w; % qdd objective function weight
    slack_limit; % maximum absolute magnitude of acceleration slack variable values
    rfoot_idx;
    lfoot_idx;
    rhand_idx;
    lhand_idx;
    pelvis_idx;
		solver_options = struct();
    debug;
    use_mex;
    use_hand_ft;
    mex_ptr;
    lc;
    contact_est_monitor;
    lcm_foot_contacts;  
    eq_array = repmat('=',100,1); % so we can avoid using repmat in the loop
    ineq_array = repmat('<',100,1); % so we can avoid using repmat in the loop
    num_body_contacts; % vector of num contacts for each body
    multi_robot;
    using_flat_terrain; % true if using DRCFlatTerrain
    lcmgl;
    jlmin;
    jlmax;
    contact_threshold; % min height above terrain to be considered in contact
		output_qdd = false;
    end
end
