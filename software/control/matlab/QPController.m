classdef QPController < MIMODrakeSystem

  methods
  function obj = QPController(r,controller_data,options)
    % @param r atlas instance
    % @param options structure for specifying objective weight (w), slack
    % variable limits (slack_limit), and input cost (R)
    typecheck(r,'Atlas');
    typecheck(controller_data,'SharedDataHandle');

    ctrl_data = getData(controller_data);
    if ~isfield(ctrl_data,'B') || ~isfield(ctrl_data,'Qy') || ...
      ~isfield(ctrl_data,'C') || ~isfield(ctrl_data,'D') || ...
      ~isfield(ctrl_data,'S')
      error('QPController: Missing fields in controller_data');
    end
    
    if nargin>2
      typecheck(options,'struct');
    else
      options = struct();
    end
    
    qddframe = AtlasCoordinates(r); % input frame for desired qddot 

    input_frame = MultiCoordinateFrame({qddframe,r.getStateFrame});
    output_frame = r.getInputFrame();
    obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
    obj = setSampleTime(obj,[.005;0]); % sets controller update rate
    obj = setInputFrame(obj,input_frame);
    obj = setOutputFrame(obj,output_frame);

    obj.robot = r;
    obj.controller_data = controller_data;

    % weight for desired qddot objective term
    if isfield(options,'w')
      typecheck(options.w,'double');
      sizecheck(options.w,1);
      obj.w = options.w;
    else
      obj.w = 0.5;
    end
    
    % hard bound on slack variable values
    if isfield(options,'slack_limit')
      typecheck(options.slack_limit,'double');
      sizecheck(options.slack_limit,1);
      obj.slack_limit = options.slack_limit;
    else
      obj.slack_limit = 10;
    end
    
    nu = getNumInputs(r);
    % input cost term: u'Ru
    if ~isfield(options,'R')
      obj.R = 1e-6*eye(nu);
    else
      typecheck(options.R,'double');
      sizecheck(options.R,[nu,nu]);
      obj.R = options.R;
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
      if (obj.use_mex && exist('QPControllermex')~=3)
        error('can''t find QPControllermex.  did you build it?');
      end
    else
      obj.use_mex = 0;
    end

    % specifies whether or not to solve QP for all DOFs or just the
    % important subset
    if (isfield(options,'full_body_opt'))
      typecheck(options.full_body_opt,'logical');
    else
      options.full_body_opt = true;
    end
    
    if ~options.full_body_opt
      % perform unconstrained minimization to compute accelerations for a 
      % subset of atlas DOF, then solve for inputs (then threshold).
      % generally these should be the joints for which the columns of the 
      % contact jacobian are zero. The remaining dofs are indexed in con_dof.
      state_names = r.getStateFrame.coordinates(1:getNumDOF(r));
      obj.free_dof = find(~cellfun(@isempty,strfind(state_names,'arm')) + ...
                    ~cellfun(@isempty,strfind(state_names,'neck')));
      obj.con_dof = setdiff(1:getNumDOF(r),obj.free_dof)';
      
      input_names = r.getInputFrame.coordinates;
      obj.free_inputs = find(~cellfun(@isempty,strfind(input_names,'arm')) | ~cellfun(@isempty,strfind(input_names,'neck')));
      obj.con_inputs = setdiff(1:getNumInputs(r),obj.free_inputs)';
    else
      obj.free_dof = [];
      obj.con_dof = 1:getNumDOF(r);
      obj.free_inputs = [];
      obj.con_inputs = 1:getNumInputs(r);
    end
    
    obj.lc = lcm.lcm.LCM.getSingleton();
    obj.rfoot_idx = findLinkInd(r,'r_foot');
    obj.lfoot_idx = findLinkInd(r,'l_foot');

    if obj.lcm_foot_contacts
      obj.contact_est_monitor = drake.util.MessageMonitor(drc.foot_contact_estimate_t,'utime');
      obj.lc.subscribe('FOOT_CONTACT_ESTIMATE',obj.contact_est_monitor);
    end % else estimate contact via kinematics
    
    if obj.solver==1 % use cplex
      obj.solver_options = cplexoptimset('cplex');
      obj.solver_options.diagnostics = 'on';
      obj.solver_options.maxtime = 0.01;
      % QP method: 
      %   0 	Automatic (default)
      %   1 	Primal Simplex
      %   2 	Dual Simplex
      %   3 	Network Simplex
      %   4 	Barrier
      %   5 	Sifting
      %   6 	Concurrent
      obj.solver_options.qpmethod = 4; 
      
    else % use gurobi

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %% NOTE: these parameters need to be set in QPControllermex.cpp, too %%%
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      obj.solver_options.outputflag = 0; % not verbose
      obj.solver_options.method = 2; % -1=automatic, 0=primal simplex, 1=dual simplex, 2=barrier
      obj.solver_options.presolve = 0;
%       obj.solver_options.prepasses = 1;

      if obj.solver_options.method == 2
        obj.solver_options.bariterlimit = 20; % iteration limit
        obj.solver_options.barhomogeneous = 0; % 0 off, 1 on
        obj.solver_options.barconvtol = 5e-4;
      end
    end  
    
    if (obj.use_mex>0)
      obj.mex_ptr = SharedDataHandle(QPControllermex(0,obj,obj.robot.getMexModelPtr.getData(),getB(obj.robot),r.umin,r.umax));
    end
  end
    
  function y=mimoOutput(obj,t,~,varargin)
%    out_tic = tic;
    q_ddot_des = varargin{1};
    x = varargin{2};
    ctrl_data = getData(obj.controller_data);
    
    r = obj.robot;
    %   debugging for the rpy velocities
    nq = getNumDOF(obj.robot); qd = x(nq+(1:nq)); 
    
    % get foot contact state
    if obj.lcm_foot_contacts
      contact_data = obj.contact_est_monitor.getMessage();
      if isempty(contact_data)
        lfoot_contact_state = 0;
        rfoot_contact_state = 0;
      else
        msg = drc.foot_contact_estimate_t(contact_data);
        lfoot_contact_state = msg.left_contact;
        rfoot_contact_state = msg.right_contact;
      end
    else
      nq = getNumDOF(r);
      contact_threshold = 0.002; % m
      q = x(1:nq); 
      kinsol = doKinematics(r,q,false,true);
    
      % get active contacts
      phi = contactConstraints(r,kinsol,[obj.lfoot_idx,obj.rfoot_idx]);

      % if any foot point is in contact, all contact points are active
      if any(phi(1:4)<contact_threshold)
        lfoot_contact_state = 1;
      else
        lfoot_contact_state = 0;
      end

      if any(phi(5:8)<contact_threshold)
        rfoot_contact_state = 1;
      else
        rfoot_contact_state = 0;
      end
    end
    
    % use support trajectory to get desired foot contact state
    if typecheck(ctrl_data.supptraj,'double')
      supp = ctrl_data.supptraj;
    else
      supp = ctrl_data.supptraj.eval(t);
    end
    desired_supports = find(supp);
    
    active_supports = [];
    if any(desired_supports==obj.lfoot_idx) && lfoot_contact_state > 0.5
      active_supports = [active_supports; obj.lfoot_idx];
    end
    if any(desired_supports==obj.rfoot_idx) && rfoot_contact_state > 0.5
      active_supports = [active_supports; obj.rfoot_idx];
    end
    
    %----------------------------------------------------------------------
    % Linear system stuff for zmp/com control -----------------------------
    if ~isempty(active_supports)
      A_ls = ctrl_data.A; % always TI
      B_ls = ctrl_data.B; % always TI
      Qy = ctrl_data.Qy;
      if isfield(ctrl_data,'R')
        R_ls = ctrl_data.R;
      else
        R_ls = zeros(2);
      end
      if typecheck(ctrl_data.C,'double')
        C_ls = ctrl_data.C;
      else
        C_ls = ctrl_data.C.eval(t);
      end
      if ~isempty(ctrl_data.D) && typecheck(ctrl_data.D,'double')
        D_ls = ctrl_data.D;
      else
        % assumed  ZMP system
        hddot = 0; % could use estimated comddot here
        D_ls = -0.89/(hddot+9.81)*eye(2); % TMP hard coding height here. Could be replaced with htraj from planner
        % or current height above height map
      end
      if typecheck(ctrl_data.S,'double')
        S = ctrl_data.S;
      else
        S = ctrl_data.S.eval(t);
      end
      if typecheck(ctrl_data.s1,'double')
        s1= zeros(4,1); % ctrl_data.s1;
      else
        s1= ctrl_data.s1.eval(t);
      end
      if typecheck(ctrl_data.x0,'double')
        x0 = ctrl_data.x0;
      else
        x0 = ctrl_data.x0.eval(t); 
      end
      if typecheck(ctrl_data.u0,'double')
        u0 = ctrl_data.u0;
      else
        u0 = ctrl_data.u0.eval(t); 
      end
      if typecheck(ctrl_data.y0,'double')
        y0 = ctrl_data.y0;
      else
        y0 = ctrl_data.y0.eval(t); 
      end
    else
      % allocate these for passing into mex
      B_ls=zeros(4,2);Qy=zeros(2);R_ls=zeros(2);C_ls=zeros(2,4);D_ls=zeros(2);
      S=zeros(4);s1=zeros(4,1);x0=zeros(4,1);u0=zeros(2,1);y0=zeros(2,1);
    end
    
    R_DQyD_ls = R_ls + D_ls'*Qy*D_ls;


    if (obj.use_mex==0 || obj.use_mex==2)
      r = obj.robot;
      nu = getNumInputs(r);
      nq = getNumDOF(r);
      dim = 3; % 3D
      nd = 4; % for friction cone approx, hard coded for now
      nq_free = length(obj.free_dof);
      nq_con = length(obj.con_dof);
      nu_con = length(obj.con_inputs);
      
      q = x(1:nq);
      qd = x(nq+(1:nq));

      kinsol = doKinematics(r,q,false,true,qd);
      
      [H,C,B] = manipulatorDynamics(r,q,qd);
      
      H_con = H(obj.con_dof,:);
      C_con = C(obj.con_dof);
      B_con = B(obj.con_dof,obj.con_inputs);
      
      if nq_free > 0
        H_free = H(obj.free_dof,:);
        C_free = C(obj.free_dof);
        B_free = B(obj.free_dof,obj.free_inputs);
      end
      
      [xcom,J] = getCOM(r,kinsol);
      Jdot = forwardJacDot(r,kinsol,0);
      J = J(1:2,:); % only need COM x-y
      Jdot = Jdot(1:2,:);
    
      if ~isempty(active_supports)
        [phi,Jz,D_] = contactConstraints(r,kinsol,active_supports);
        nc = length(phi);
      else
        nc = 0;
      end
      neps = nc*dim;
      %     neps = length(active_supports)*2*dim;
      
      if nc > 0
        [cpos,Jp,Jpdot] = contactPositionsJdot(r,kinsol,active_supports);
        %       Jp=zeros(neps,nq);
        %       Jpdot=zeros(neps,nq);
        %       for k=1:length(active_supports)
        %         [~,Jp((k-1)*2*dim+(1:2*dim),:)] = forwardKin(r,kinsol,active_supports(k),[[1;0;0],[0;1;0]],0);
        %         Jpdot((k-1)*2*dim+(1:2*dim),:) = forwardJacDot(r,kinsol,active_supports(k),[[1;0;0],[0;1;0]]);
        %       end
        Jp = sparse(Jp(:,obj.con_dof));
        Jpdot = sparse(Jpdot(:,obj.con_dof));
        
        Jz = sparse(Jz(:,obj.con_dof)); % only care about active contacts
        
        % D_ is the parameterization of the polyhedral approximation of the
        %    friction cone, in joint coordinates (figure 1 from Stewart96)
        %    D{k}(i,:) is the kth direction vector for the ith contact (of nC)
        % Create Dbar such that Dbar(:,(k-1)*nd+i) is ith direction vector for
        % the kth contact point
        %
        % OPT---this isn't necessary
        D = cell(1,nc);
        for k=1:nc
          for i=1:nd
            D{k}(:,i) = D_{i}(k,obj.con_dof)';
          end
        end
        Dbar = sparse([D{:}]);
      end      
        
      if (nc>0)
        xlimp = [xcom(1:2); J*qd]; % state of LIP model
        x_bar = xlimp - x0;      
      end
      
      
      %----------------------------------------------------------------------
      % Free DOF cost function ----------------------------------------------

      if nq_free > 0
        if nc > 0
          % approximate quadratic cost for free dofs with the appropriate matrix block
          Hqp = J(:,obj.free_dof)'*R_DQyD_ls*J(:,obj.free_dof);
          Hqp = Hqp + obj.w*eye(nq_free);

          fqp = xlimp'*C_ls'*Qy*D_ls*J(:,obj.free_dof);
          fqp = fqp + qd(obj.free_dof)'*Jdot(:,obj.free_dof)'*R_DQyD_ls*J(:,obj.free_dof);
          fqp = fqp + (x_bar'*S + 0.5*s1')*B_ls*J(:,obj.free_dof);
          fqp = fqp - u0'*R_ls*J(:,obj.free_dof);
          fqp = fqp - y0'*Qy*D_ls*J(:,obj.free_dof);
          fqp = fqp - obj.w*q_ddot_des(obj.free_dof)';
        else
          Hqp = eye(nq_free);
          fqp = -q_ddot_des(obj.free_dof)';
        end

        % solve for qdd_free unconstrained
        qdd_free = -inv(Hqp)*fqp';
      end      
        
    
      %----------------------------------------------------------------------
      % Build handy index matrices ------------------------------------------
      
      nf = nc+nc*nd; % number of contact force variables
      nparams = nq_con+nu_con+nf+neps;
      Iqdd = zeros(nq_con,nparams); Iqdd(:,1:nq_con) = eye(nq_con);
      Iu = zeros(nu_con,nparams); Iu(:,nq_con+(1:nu_con)) = eye(nu_con);
      Iz = zeros(nc,nparams); Iz(:,nq_con+nu_con+(1:nc)) = eye(nc);
      Ibeta = zeros(nc*nd,nparams); Ibeta(:,nq_con+nu_con+nc+(1:nc*nd)) = eye(nc*nd);
      Ieps = zeros(neps,nparams);
      Ieps(:,nq_con+nu_con+nc+nc*nd+(1:neps)) = eye(neps);
      
      
      %----------------------------------------------------------------------
      % Set up problem constraints ------------------------------------------
      
      lb = [-1e3*ones(1,nq_con) r.umin(obj.con_inputs)' zeros(1,nf)   -obj.slack_limit*ones(1,neps)]'; % qddot/input/contact forces/slack vars
      ub = [ 1e3*ones(1,nq_con) r.umax(obj.con_inputs)' 500*ones(1,nf) obj.slack_limit*ones(1,neps)]';
      
      Aeq_ = cell(1,2);
      beq_ = cell(1,2);
      Ain_ = cell(1,nc);
      bin_ = cell(1,nc);
      
      % constrained dynamics
      if nc>0
        Aeq_{1} = H_con(:,obj.con_dof)*Iqdd - B_con*Iu - Jz'*Iz - Dbar*Ibeta;
      else
        Aeq_{1} = H_con(:,obj.con_dof)*Iqdd - B_con*Iu;
      end
      if nq_free > 0
        beq_{1} = -C_con - H_con(:,obj.free_dof)*qdd_free;
      else
        beq_{1} = -C_con;
      end
      
      if nc > 0
        % relative acceleration constraint
        Aeq_{2} = Jp*Iqdd + Ieps;
        beq_{2} = -Jpdot*qd(obj.con_dof) - 1.0*Jp*qd(obj.con_dof);
        
        % linear friction constraints
        % TEMP: hard code mu
        mu = 0.5*ones(nc,1);
        for i=1:nc
          Ain_{i} = -mu(i)*Iz(i,:) + sum(Ibeta((i-1)*nd+(1:nd),:));
          bin_{i} = 0;
        end
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
      %  min: quad(Jdot*qd + J*qdd,R_ls)+quad(C*x_bar+D*(Jdot*qd + J*qdd),Qy) + (2*x_bar'*S + s1')*(A*x_bar + B*(Jdot*qd + J*qdd-u0)) + w*quad(qddot_ref - qdd) + quad(u,R) + quad(epsilon)
      
      if nc > 0
        Hqp = Iqdd'*J(:,obj.con_dof)'*R_DQyD_ls*J(:,obj.con_dof)*Iqdd;
        Hqp(1:nq_con,1:nq_con) = Hqp(1:nq_con,1:nq_con) + obj.w*eye(nq_con);

        fqp = xlimp'*C_ls'*Qy*D_ls*J(:,obj.con_dof)*Iqdd;
        fqp = fqp + qd(obj.con_dof)'*Jdot(:,obj.con_dof)'*R_DQyD_ls*J(:,obj.con_dof)*Iqdd;
        fqp = fqp + (x_bar'*S + 0.5*s1')*B_ls*J(:,obj.con_dof)*Iqdd;
        fqp = fqp - u0'*R_ls*J(:,obj.con_dof)*Iqdd;
        fqp = fqp - y0'*Qy*D_ls*J(:,obj.con_dof)*Iqdd;
        fqp = fqp - obj.w*q_ddot_des(obj.con_dof)'*Iqdd;

        % quadratic slack var cost 
        Hqp(nparams-neps+1:end,nparams-neps+1:end) = 0.001*eye(neps); 
      else
        Hqp = Iqdd'*Iqdd;
        fqp = -q_ddot_des(obj.con_dof)'*Iqdd;
      end
      
      % quadratic input cost
      Hqp(nq_con+(1:nu_con),nq_con+(1:nu_con)) = obj.R(obj.con_inputs,obj.con_inputs);
      

      %----------------------------------------------------------------------
      % Solve QP ------------------------------------------------------------
      
      if obj.solver==1
        % CURRENTLY CRASHES MATLAB ON MY MACHINE -sk
        alpha = cplexqp(Hqp,fqp,Ain,bin,Aeq,beq,lb,ub,[],obj.solver_options);
        
      else
        model.Q = sparse(Hqp);
        model.obj = 2*fqp;
        model.A = [Aeq; Ain];
        model.rhs = [beq; bin];
        model.sense = [obj.eq_array(1:length(beq)); obj.ineq_array(1:length(bin))];
        model.lb = lb;
        model.ub = ub;
        
        if (any(any(isnan(model.Q))) || any(isnan(model.obj)) || any(any(isnan(model.A))) || any(isnan(model.rhs)) || any(isnan(model.lb)) || any(isnan(model.ub)))
          keyboard;
        end

%       Q=full(Hqp);
%       c=2*fqp;
%       Aeq = full(Aeq);
%       Ain = full(Ain);
%       save(sprintf('data/model_t_%2.3f.mat',t),'Q','c','Aeq','beq','Ain','bin','lb','ub');
%       qp_tic = tic;
        result = gurobi(model,obj.solver_options);
%       qp_toc = toc(qp_tic);
%       fprintf('QP solve: %2.4f\n',qp_toc);
        alpha = result.x;
        
      end

      %----------------------------------------------------------------------
      % Solve for free inputs -----------------------------------------------
      if nq_free > 0
        qdd = zeros(nq,1);
        qdd(obj.free_dof) = qdd_free;
        qdd(obj.con_dof) = alpha(1:nq_con);
        
        u_free = B_free\(H_free*qdd + C_free);
        u = zeros(nu,1);
        u(obj.free_inputs) = u_free;
        u(obj.con_inputs) = alpha(nq_con+(1:nu_con));
        
        % saturate inputs
        y = max(r.umin,min(r.umax,u));
      else
        y = alpha(nq+(1:nu));
      end
      
      if (obj.use_mex==2)
        des.y = y;
      end
      
      % compute V,Vdot for controller status updates
      if (nc>0)
        V = x_bar'*S*x_bar + s1'*x_bar;  % missing affine term here...
        qdd = zeros(nq,1);
        qdd(obj.free_dof) = qdd_free;
        qdd(obj.con_dof) = alpha(1:nq_con);
        
        Vdot = (2*x_bar'*S + s1')*(A_ls*x_bar + B_ls*(Jdot*qd + J*qdd));
        setField(obj.controller_data,'V',V);
        setField(obj.controller_data,'Vdot',Vdot);
      
  %     scope('Atlas','V',t,V,struct('linespec','b','scope_id',1));
  %     scope('Atlas','Vdot',t,Vdot,struct('linespec','g','scope_id',1));
      else
        setField(obj.controller_data,'V',0);
        setField(obj.controller_data,'Vdot',0);
      end
      
    end
  
    if (obj.use_mex==1)
       y = QPControllermex(obj.mex_ptr.getData(),q_ddot_des,x,active_supports,B_ls,Qy,R_ls,C_ls,D_ls,S,s1,x0,u0,y0);
    end
    
    if (obj.use_mex==2)
      [y,Q,gobj,A,rhs,sense,lb,ub] = QPControllermex(obj.mex_ptr.getData(),q_ddot_des,x,active_supports,B_ls,Qy,R_ls,C_ls,D_ls,S,s1,x0,u0,y0);
      valuecheck(Q'+Q,model.Q'+model.Q);
      valuecheck(gobj,model.obj);
      valuecheck(A,model.A);
      valuecheck(rhs,model.rhs);
      valuecheck(sense',model.sense);
      valuecheck(lb,model.lb);
      valuecheck(ub,model.ub);
%       valuecheck(y,des.y,1e-4);  % they are close, but not *quite* the
%       same. ---I don't like this, I'm seeing differences up to 5Nm in
%       some dimensions.
    end
    
   
    if obj.debug && (obj.use_mex==0 || obj.use_mex==2) && nc > 0
      if nq_free > 0
        xcomdd = Jdot * qd + J * qdd;
      else
        xcomdd = Jdot * qd + J * alpha(1:nq);
      end
      zmppos = xcom(1:2) + D_ls * xcomdd;
      convh = convhull(cpos(1,:), cpos(2,:));
      zmp_ok = inpolygon(zmppos(1), zmppos(2), cpos(1,convh), cpos(2,convh));
      if zmp_ok
        color = [0 1 0];
      else
        color = [1 0 0];
      end
      plot_lcm_points([zmppos', mean(cpos(3,:))], color, 660, 'Commanded ZMP', 1, true);

      m = drc.controller_zmp_status_t();
      m.utime = t * 1e9;
      m.zmp_ok = zmp_ok;
      obj.lc.publish('CONTROLLER_ZMP_STATUS', m);
      
      [~,normals] = getTerrainHeight(r,cpos);
      d = RigidBodyManipulator.surfaceTangents(normals);

      lambda = Iz*alpha;
      beta_full = Ibeta*alpha;
      for kk=1:8
        if kk<=nc
          plot_lcm_points([cpos(:,kk) cpos(:,kk)+0.25*normals(:,kk)]', [0 0 1; 0 0 1], 23489083+kk, sprintf('Foot Contact Normal %d',kk), 2, true);
          beta = beta_full((kk-1)*nd+(1:nd),:);
          fvec = lambda(kk)*normals(:,kk) + d{1}(:,kk)*beta(1) + d{2}(:,kk)*beta(2) - d{1}(:,kk)*beta(3) - d{2}(:,kk)*beta(4);
          plot_lcm_points([cpos(:,kk) cpos(:,kk)+0.0025*fvec]', [1 0 0; 1 0 0], 6643+kk, sprintf('Foot Contact Force %d',kk), 2, true);
        else
          plot_lcm_points(zeros(2,3), [0 0 1;0 0 1], 23489083+kk, sprintf('Foot Contact Normal %d',kk), 2, true);
          plot_lcm_points(zeros(2,3), [1 0 0;1 0 0], 6643+kk, sprintf('Foot Contact Force %d',kk), 2, true);
        end
      end

%       % plot body coordinate frames
%       m=vs.obj_collection_t();
%       m.objs = javaArray('vs.obj_t', size(1, 1));
%       m.id=13300;
%       m.type=5; % rgb triad
%       m.name='Drake Body Coords';
%       m.reset=true;
%       m.nobjs=5; 
%       
%       pelvis = findLinkInd(r,'pelvis');
%       xzyrpy = forwardKin(r,kinsol,pelvis,[0;0;0],1);
%       msg=vs.obj_t();
%       msg.id=1;
%       msg.x=xzyrpy(1); msg.y=xzyrpy(2); msg.z=xzyrpy(3);
%       msg.roll=xzyrpy(4); msg.pitch=xzyrpy(5); msg.yaw=xzyrpy(6);
%       m.objs(msg.id) = msg;
% 
%       head = findLinkInd(r,'head');
%       xzyrpy = forwardKin(r,kinsol,head,[0;0;0],1);
%       msg=vs.obj_t();
%       msg.id=2;
%       msg.x=xzyrpy(1); msg.y=xzyrpy(2); msg.z=xzyrpy(3);
%       msg.roll=xzyrpy(4); msg.pitch=xzyrpy(5); msg.yaw=xzyrpy(6);
%       m.objs(msg.id) = msg;
% 
%       xzyrpy = forwardKin(r,kinsol,obj.rfoot_idx,[0;0;0],1);
%       msg=vs.obj_t();
%       msg.id=3;
%       msg.x=xzyrpy(1); msg.y=xzyrpy(2); msg.z=xzyrpy(3);
%       msg.roll=xzyrpy(4); msg.pitch=xzyrpy(5); msg.yaw=xzyrpy(6);
%       m.objs(msg.id) = msg;
% 
%       xzyrpy = forwardKin(r,kinsol,obj.lfoot_idx,[0;0;0],1);
%       msg=vs.obj_t();
%       msg.id=4;
%       msg.x=xzyrpy(1); msg.y=xzyrpy(2); msg.z=xzyrpy(3);
%       msg.roll=xzyrpy(4); msg.pitch=xzyrpy(5); msg.yaw=xzyrpy(6);
%       m.objs(msg.id) = msg;
% 
%       xzyrpy = x(1:6); 
%       msg=vs.obj_t();
%       msg.id=5;
%       msg.x=xzyrpy(1); msg.y=xzyrpy(2); msg.z=xzyrpy(3);
%       msg.roll=xzyrpy(4); msg.pitch=xzyrpy(5); msg.yaw=xzyrpy(6);
%       m.objs(msg.id) = msg;
%       
%       obj.lc.publish('OBJ_COLLECTION', m);
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
  end
  end

  properties (SetAccess=private)
    robot; % to be controlled
    controller_data; % shared data handle that holds S, h, foot trajectories, etc.
    w; % objective function weight
    slack_limit; % maximum absolute magnitude of acceleration slack variable values
    free_dof % dofs for which we perform unconstrained minimization
    con_dof 
    free_inputs
    con_inputs
    rfoot_idx;
    lfoot_idx;
    R; % quadratic input cost matrix
    solver = 0; % 0: gurobi, 1:cplex
    solver_options = struct();
    debug;
    use_mex;
    mex_ptr;
    lc;
    contact_est_monitor;
    lcm_foot_contacts;  
    eq_array = repmat('=',100,1); % so we can avoid using repmat in the loop
    ineq_array = repmat('<',100,1); % so we can avoid using repmat in the loop
  end
end
