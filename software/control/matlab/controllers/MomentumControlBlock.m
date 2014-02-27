classdef MomentumControlBlock < MIMODrakeSystem

  methods
  function obj = MomentumControlBlock(r,controller_data,options)
    % @param r atlas instance
    % @param controller_data shared data handle containing ZMP-LQR solution, etc
    % @param options structure for specifying objective weight (w), slack
    % variable limits (slack_limit)
    typecheck(r,'Atlas');
    typecheck(controller_data,'SharedDataHandle');
    
    if nargin>2
      typecheck(options,'struct');
    else
      options = struct();
    end
    
    qddframe = AtlasCoordinates(r); % input frame for desired qddot 
    ft_frame = AtlasForceTorque();

    input_frame = MultiCoordinateFrame({qddframe,ft_frame,r.getStateFrame});
    
    if isfield(options,'dt')
      % controller update rate
      typecheck(options.dt,'double');
      sizecheck(options.dt,[1 1]);
      dt = options.dt;
    else
      dt = 0.001;
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
    obj = setSampleTime(obj,[dt;0]); % sets controller update rate
    obj = setInputFrame(obj,input_frame);
    obj = setOutputFrame(obj,output_frame);

    obj.robot = r;
    obj.controller_data = controller_data;
    
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

    if isfield(options,'use_hand_ft')
      obj.use_hand_ft = options.use_hand_ft;
    else
      obj.use_hand_ft = false;
    end

    obj.lc = lcm.lcm.LCM.getSingleton();
    obj.rfoot_idx = findLinkInd(r,'r_foot');
    obj.lfoot_idx = findLinkInd(r,'l_foot');
    obj.rhand_idx = findLinkInd(r,'r_hand');
    obj.lhand_idx = findLinkInd(r,'l_hand');
    obj.nq = getNumDOF(r);
    
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
%    out_tic = tic;
    ctrl_data = obj.controller_data.data;
      
    q_ddot_des = varargin{1};
    ft = varargin{2};
    hand_ft = ft(6+(1:12));
    x = varargin{3};
       
    r = obj.robot;
    nq = obj.nq; 
    q = x(1:nq); 
    qd = x(nq+(1:nq)); 

    x0 = ctrl_data.x0 - [ctrl_data.trans_drift(1:2);0;0]; % for x-y plan adjustment
    if (ctrl_data.is_time_varying)
      % extract current supports
      supp_idx = find(ctrl_data.support_times<=t,1,'last');
      supp = ctrl_data.supports(supp_idx);
    else
      supp = ctrl_data.supports;
    end
    K = ctrl_data.K;
    mu = ctrl_data.mu;

    % contact_sensor = -1 (no info), 0 (info, no contact), 1 (info, yes contact)
    contact_sensor=-1+0*supp.bodies;  % initialize to -1 for all
    if obj.lcm_foot_contacts
      % get foot contact state over LCM
      contact_data = obj.contact_est_monitor.getMessage();
      if ~isempty(contact_data)
        msg = drc.foot_contact_estimate_t(contact_data);
        contact_sensor(find(supp.bodies==obj.lfoot_idx)) = msg.left_contact;
        contact_sensor(find(supp.bodies==obj.rfoot_idx)) = msg.right_contact;
      end
    end
    
    % Change in logic here due to recent tests with heightmap noise
    % for now, we will do a logical OR of the force-based sensor and the
    % kinematic criterion for foot contacts
    %
    % another option would be to limit forces on the feet when kinematics
    % says 'contact', but force sensors do not. when both agree, allow full
    % forces on the feet
    kinsol = doKinematics(r,q,false,false);

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
    % Disable hand force/torque contribution to dynamics as necessary
    if (~obj.use_hand_ft)
      hand_ft=0*hand_ft;
    else
      if any(active_supports==obj.lhand_idx)
        hand_ft(1:6)=0;
      end
      if any(active_supports==obj.rhand_idx)
        hand_ft(7:12)=0;
      end
    end

    %----------------------------------------------------------------------

    nu = getNumInputs(r);
    nq = getNumDOF(r);
    dim = 3; % 3D
    nd = 4; % for friction cone approx, hard coded for now
    float_idx = 1:6; % indices for floating base dofs
    act_idx = 7:nq; % indices for actuated dofs

    kinsol = doKinematics(r,q,false,false,qd);

    [H,C,B] = manipulatorDynamics(r,q,qd);

    [~,Jlhand] = forwardKin(r,kinsol,obj.lhand_idx,zeros(3,1),1);
    [~,Jrhand] = forwardKin(r,kinsol,obj.rhand_idx,zeros(3,1),1);
    C = C + Jlhand'*hand_ft(1:6) + Jrhand'*hand_ft(7:12);

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

      [cpos,Jp,Jpdot] = contactPositionsJdot(r,kinsol,active_supports,active_contact_pts);
      Jp = sparse(Jp);
      Jpdot = sparse(Jpdot);

      xlimp = [xcom(1:2); J*qd]; % state of LIP model
      x_bar = xlimp - x0;

      ustar = output(K,t,[],x_bar); % ustar==u_bar since u_nom=0
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

    % if at joint limit, disallow accelerations in that direction
    lb(q<=obj.jlmin+1e-4) = 0;
    ub(q>=obj.jlmax-1e-4) = 0;

    Aeq_ = cell(1,4);
    beq_ = cell(1,4);
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
      beq_{2} = -Jpdot*qd - 0*Jp*qd;
    end
    
    % do PD on feet to compute body accelerations
    Kp_body = 20;
    Kd_body = 0.2;
    if ~any(active_supports==ctrl_data.link_constraints(1).link_ndx)
      [p1,J1] = forwardKin(r,kinsol,ctrl_data.link_constraints(1).link_ndx,...
        ctrl_data.link_constraints(1).pt,1);
      J1dot = forwardJacDot(r,kinsol,ctrl_data.link_constraints(1).link_ndx,...
        ctrl_data.link_constraints(1).pt,1);
      
      body1_t = fasteval(ctrl_data.link_constraints(1).traj,t);
      cidx = ~isnan(body1_t);
      body1dd = Kp_body*(body1_t - p1) - Kd_body*J1*qd;
      Aeq_{3} = J1(cidx,:)*Iqdd;
      beq_{3} = -J1dot(cidx,:)*qd + body1dd(cidx);
    end
    
    if ~any(active_supports==ctrl_data.link_constraints(2).link_ndx)
      [p2,J2] = forwardKin(r,kinsol,ctrl_data.link_constraints(2).link_ndx,...
        ctrl_data.link_constraints(2).pt,1);
      J2dot = forwardJacDot(r,kinsol,ctrl_data.link_constraints(2).link_ndx,...
        ctrl_data.link_constraints(2).pt,1);

      body2_t = fasteval(ctrl_data.link_constraints(2).traj,t);
      cidx = ~isnan(body2_t);
      body2dd = Kp_body*(body2_t - p2) - Kd_body*J2*qd;
      Aeq_{4} = J2(cidx,:)*Iqdd;
      beq_{4} = -J2dot(cidx,:)*qd + body2dd(cidx);
    end
    
    % linear equality constraints: Aeq*alpha = beq
    Aeq = sparse(vertcat(Aeq_{:}));
    beq = vertcat(beq_{:});

    % linear inequality constraints: Ain*alpha <= bin
    Ain = sparse(vertcat(Ain_{:}));
    bin = vertcat(bin_{:});

    % compute desired linear momentum
%     comz_t = fasteval(ctrl_data.comztraj,t);
%     dcomz_t = fasteval(ctrl_data.dcomztraj,t);
    comddot_des = [ustar; 10*(1.04-xcom(3)) + 0.5*(0-z_com_dot)];
%     comddot_des = [ustar; 10*(comz_t-xcom(3)) + 0.5*(dcomz_t-z_com_dot)];
    ldot_des = comddot_des * 155;
    k = A(1:3,:)*qd;
%     kdot_des = 10.0 * (ctrl_data.ktraj.eval(t) - k); 
    kdot_des = -10.0 *k; 
    hdot_des = [kdot_des; ldot_des];

    W = diag([.1 .1 .1 1 1 1]);

    %----------------------------------------------------------------------
    % QP cost function ----------------------------------------------------
    %
    %  min: quad(h_dot_des - Adot*qd - A*qdd) + w*quad(qddot_ref - qdd) + 0.001*quad(epsilon)
    if nc > 0
      Hqp = Iqdd'*A'*W*A*Iqdd;
      Hqp(1:nq,1:nq) = Hqp(1:nq,1:nq) + obj.w*eye(nq);

      fqp = qd'*Adot'*W*A*Iqdd;
      fqp = fqp - hdot_des'*W*A*Iqdd;
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
    
    if obj.debug && nc > 0

%       hdot_des
%       h = A*qdd + Adot*qd
      
      Jdot = forwardJacDot(r,kinsol,0);
      Jdot = Jdot(1:2,:);

      xcomdd = Jdot * qd + J * qdd;
      zmppos = xcom(1:2) + D_ls * xcomdd;
      zmp = [zmppos', mean(cpos(3,:))];
      convh = convhull(cpos(1,:), cpos(2,:));
      zmp_ok = inpolygon(zmppos(1), zmppos(2), cpos(1,convh), cpos(2,convh));
      if zmp_ok
        color = [0 1 0];
      else
        color = [1 0 0];
      end
      obj.lcmgl.glColor3f(color(1), color(2), color(3));
      obj.lcmgl.sphere(zmp, 0.015, 20, 20);

      obj.lcmgl.glColor3f(0, 0, 0);
      obj.lcmgl.sphere([xcom(1:2)', mean(cpos(3,:))], 0.015, 20, 20);

%       % plot Vdot indicator
%       headpos = forwardKin(r,kinsol,findLinkInd(r,'head'),[0;0;0]);
%       obj.lcmgl.glLineWidth(3);
%       obj.lcmgl.glPushMatrix();
%       obj.lcmgl.glTranslated(headpos(1),headpos(2),headpos(3)+0.3);
%       obj.lcmgl.glRotated(90, 0, 1, 0);
%       if Vdot < 0
%         obj.lcmgl.glColor3f(0, 1, 0);
%         obj.lcmgl.glBegin(obj.lcmgl.LCMGL_LINES);
%         obj.lcmgl.glVertex3f(0.025, -0.055, 0.01);
%         obj.lcmgl.glVertex3f(0.045, -0.035, 0.01);
%         obj.lcmgl.glVertex3f(0.045, -0.035, 0.01);
%         obj.lcmgl.glVertex3f(0.045, 0.035, 0.01);
%         obj.lcmgl.glVertex3f(0.045, 0.035, 0.01);
%         obj.lcmgl.glVertex3f(0.025, 0.055, 0.01);
%         obj.lcmgl.glEnd();
%       elseif Vdot < 0.1
%         obj.lcmgl.glColor3f(1, 1, 0);
%         obj.lcmgl.glBegin(obj.lcmgl.LCMGL_LINES);
%         obj.lcmgl.glVertex3f(0.045, -0.055, 0.01);
%         obj.lcmgl.glVertex3f(0.045, 0.055, 0.01);
%         obj.lcmgl.glEnd();
%       elseif Vdot < 0.25
%         obj.lcmgl.glColor3f(1, 0.5, 0);
%         obj.lcmgl.glBegin(obj.lcmgl.LCMGL_LINES);
%         obj.lcmgl.glVertex3f(0.045, -0.055, 0.01);
%         obj.lcmgl.glVertex3f(0.045, 0.055, 0.01);
%         obj.lcmgl.glEnd();
%       else
%         obj.lcmgl.glColor3f(1, 0, 0);
%         obj.lcmgl.glBegin(obj.lcmgl.LCMGL_LINES);
%         obj.lcmgl.glVertex3f(0.065, -0.055, 0.01);
%         obj.lcmgl.glVertex3f(0.045, -0.035, 0.01);
%         obj.lcmgl.glVertex3f(0.045, -0.035, 0.01);
%         obj.lcmgl.glVertex3f(0.045, 0.035, 0.01);
%         obj.lcmgl.glVertex3f(0.045, 0.035, 0.01);
%         obj.lcmgl.glVertex3f(0.065, 0.055, 0.01);
%         obj.lcmgl.glEnd();
%       end
%       obj.lcmgl.circle(0,0,0, 0.1);
%       obj.lcmgl.circle(-0.03,-0.035,0.005, 0.01);
%       obj.lcmgl.circle(-0.03,0.035,0.005, 0.01);
%       obj.lcmgl.glPopMatrix();
      
      [~,B] = contactConstraintsBV(r,kinsol,active_supports,active_contact_pts);
      beta = Ibeta*alpha;
      nd=size(B{1},2); 
      for j=1:nc
        beta_j = beta((j-1)*nd+(1:nd),:);
        b=0.1*B{j}; % scale for drawing
        obj.lcmgl.glLineWidth(2);
        obj.lcmgl.glPushMatrix();
        obj.lcmgl.glTranslated(cpos(1,j),cpos(2,j),cpos(3,j));
        obj.lcmgl.glColor3f(0, 0, 1);
        fvec = zeros(3,1);
        for k=1:nd
          obj.lcmgl.glBegin(obj.lcmgl.LCMGL_LINES);
          obj.lcmgl.glVertex3f(0, 0, 0);
          obj.lcmgl.glVertex3f(b(1,k), b(2,k), b(3,k)); 
          obj.lcmgl.glEnd();
          fvec = fvec + beta_j(k)*b(:,k);
        end
        obj.lcmgl.glLineWidth(3);
        obj.lcmgl.glColor3f(1, 0, 0);
        obj.lcmgl.glBegin(obj.lcmgl.LCMGL_LINES);
        obj.lcmgl.glVertex3f(0, 0, 0);
        obj.lcmgl.glVertex3f(0.025*fvec(1), 0.025*fvec(2), 0.025*fvec(3)); 
        obj.lcmgl.glEnd();
        obj.lcmgl.glPopMatrix();
      end
      
      if 0 % plot individual body COMs
        for jj=1:getNumBodies(r)
          b = getBody(r,jj);
          if ~isempty(b.com)
            bpos = forwardKin(r,kinsol,jj,[0;0;0],1);
            obj.lcmgl.glColor3f(1, 0, 0);
            obj.lcmgl.glPushMatrix();
            obj.lcmgl.glTranslated(bpos(1),bpos(2),bpos(3));
            ax = rpy2axis(bpos(4:6));
            obj.lcmgl.glRotated(ax(4)*180/pi, ax(1), ax(2), ax(3));
            obj.lcmgl.sphere(b.com, b.mass*0.005, 20, 20);
            obj.lcmgl.glPopMatrix();
          end
       end
        
      end
      
      if obj.include_angular_momentum
        % plot momentum vectors
        h = Ag*qd;
        obj.lcmgl.glLineWidth(3);

        obj.lcmgl.glPushMatrix();
        obj.lcmgl.glTranslated(xcom(1),xcom(2),xcom(3));
        obj.lcmgl.glColor3f(0.5, 0.25, 0);
        obj.lcmgl.glBegin(obj.lcmgl.LCMGL_LINES);
        obj.lcmgl.glVertex3f(0, 0, 0);
        obj.lcmgl.glVertex3f(0.05*h(4), 0.05*h(5), 0.05*h(6)); % scale for drawing
        obj.lcmgl.glEnd();

        aa_h = rpy2axis(h(1:3));
        obj.lcmgl.glColor3f(0, 0.25, 0.5);
        obj.lcmgl.glBegin(obj.lcmgl.LCMGL_LINES);
        obj.lcmgl.glVertex3f(0, 0, 0);
        obj.lcmgl.glVertex3f(0.2*aa_h(1), 0.2*aa_h(2), 0.2*aa_h(3)); % scale for drawing
        obj.lcmgl.glEnd();
        obj.lcmgl.glPopMatrix();
      end
      
      obj.lcmgl.glLineWidth(1);
      obj.lcmgl.switchBuffers();
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
  end
  end

  properties (SetAccess=private)
    robot; % to be controlled
    nq;
    controller_data; % shared data handle that holds S, h, foot trajectories, etc.
    w; % objective function weight
    slack_limit; % maximum absolute magnitude of acceleration slack variable values
    rfoot_idx;
    lfoot_idx;
    rhand_idx;
    lhand_idx;
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
    num_state_frames; % if there's a multi robot defined this is 1+ the number of other state frames
    using_flat_terrain; % true if using DRCFlatTerrain
    ignore_states; % array if state indices we want to ignore (and substitute with planned values)
    lcmgl;
    include_angular_momentum; % tmp flag for testing out angular momentum control
    jlmin;
    jlmax;
    contact_threshold; % min height above terrain to be considered in contact
		output_qdd = false;
    end
end
