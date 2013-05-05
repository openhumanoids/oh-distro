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
    
    % specifies whether or not to solve QP for all DOFs or just the
    % important subset
    if (isfield(options,'full_body_opt'))
      typecheck(options.full_body_opt,'logical');
    else
      options.full_body_opt = true;
    end
    
    if ~options.full_body_opt
      % free_dof we perform unconstrained minimization to compute 
      % accelerations and solve for inputs (then threshold).
      % minimally these should be the joints for which the columns of the 
      % contact jacobian are zero. The remaining dofs are indexed in cnstr_dof.
      jn = getJointNames(r);
      torso = ~cellfun(@isempty,strfind(jn(2:end),'arm')) + ...
                    ~cellfun(@isempty,strfind(jn(2:end),'neck'));
%       torso = ~cellfun(@isempty,strfind(jn(2:end),'ely')) + ...
%                     ~cellfun(@isempty,strfind(jn(2:end),'neck')) + ...
%                     ~cellfun(@isempty,strfind(jn(2:end),'mwx'));
      B = getB(r);
      obj.free_dof = find(torso);
      obj.con_dof = setdiff(1:getNumDOF(r),obj.free_dof)';
      obj.free_inputs = find(B'*torso);
      obj.con_inputs = find(B'*torso==0);
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
      obj.solver_options.outputflag = 0; % not verbose
      obj.solver_options.method = 2; % -1=automatic, 0=primal simplex, 1=dual simplex, 2=barrier
      obj.solver_options.presolve = 0;
%       obj.solver_options.prepasses = 1;

      if obj.solver_options.method == 2
        obj.solver_options.bariterlimit = 20; % iteration limit
        obj.solver_options.barhomogeneous = 0; % 0 off, 1 on
        obj.solver_options.barconvtol = 1e-3;
      end
    end  
    
  end
    
  function y=mimoOutput(obj,t,~,varargin)
    out_tic = tic;

    q_ddot_des = varargin{1};
    x = varargin{2};
    r = obj.robot;
    ctrl_data = getData(obj.controller_data);

    nd = 4; % for friction cone approx, hard coded for now
    dim = 3; % 3D
    nu = getNumInputs(r);
    nq = getNumDOF(r);
    nq_free = length(obj.free_dof); 
    nq_con = length(obj.con_dof); 
    nu_con = length(obj.con_inputs);  
    
    % get foot contact state
    if obj.lcm_foot_contacts
      contact_data = obj.contact_est_monitor.getNextMessage(1);
      if ~isempty(contact_data)
        msg = drc.foot_contact_estimate_t(contact_data);
        obj.lfoot_contact_state = msg.left_contact;
        obj.rfoot_contact_state = msg.right_contact;
      end
    else
      contact_threshold = 0.002; % m
      q = x(1:nq); 
      kinsol = doKinematics(r,q,false,true);
    
      % get active contacts
      phi = contactConstraints(r,kinsol,[obj.lfoot_idx,obj.rfoot_idx]);

      % if any foot point is in contact, all contact points are active
      if any(phi(1:4)<contact_threshold)
        obj.lfoot_contact_state = 1;
      else
        obj.lfoot_contact_state = 0;
      end

      if any(phi(5:8)<contact_threshold)
        obj.rfoot_contact_state = 1;
      else
        obj.rfoot_contact_state = 0;
      end
    end
    
    % get pelvis height above height map --- should probably use height
    % above support foot here
    terrain_height = getTerrainHeight(r,x(1:2));
    x(3) = x(3)-terrain_height;
      
    % use support trajectory to get desired foot contact state
    if typecheck(ctrl_data.supptraj,'double')
      supp = ctrl_data.supptraj;
    else
      supp = ctrl_data.supptraj.eval(t);
    end
    active_supports = find(supp);
    
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
    J = J(1:2,:); % only need COM x-y
    Jdot = forwardJacDot(r,kinsol,0);
    Jdot = Jdot(1:2,:);
    
    % get active contacts --- note, calling this with the z-adjusted state,
    % so phi returned isn't useful 
    [phi_shifted,Jz,D_] = contactConstraints(r,kinsol,active_supports);
    active_contacts = zeros(length(phi_shifted),1);

    % if any foot point is in contact, all contact points are active
    if any(active_supports==obj.rfoot_idx) && obj.rfoot_contact_state > 0.5
      active_contacts((find(active_supports==obj.rfoot_idx)-1)*4+(1:4)) = 1;
    end
    if any(active_supports==obj.lfoot_idx) && obj.lfoot_contact_state > 0.5
      active_contacts((find(active_supports==obj.lfoot_idx)-1)*4+(1:4)) = 1;
    end
    
    nc = sum(active_contacts);
    [cpos,Jp,Jpdot] = contactPositionsJdot(r,kinsol,active_supports);
    
    active_contacts = find(active_contacts);
        
    if nc > 0
      Jz = sparse(Jz(active_contacts,obj.con_dof)); % only care about active contacts

      active_idx = zeros(dim*length(active_contacts),1);
      for i=1:length(active_contacts)
        active_idx((i-1)*dim+1:i*dim) = (active_contacts(i)-1)*dim + (1:dim)';
      end
      Jp = sparse(Jp(active_idx,obj.con_dof)); 
      Jpdot = sparse(Jpdot(active_idx,obj.con_dof));
      
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
          D{k}(:,i) = D_{i}(active_contacts(k),obj.con_dof)'; 
        end
      end
      Dbar = sparse([D{:}]);
    end
    
    %----------------------------------------------------------------------
    % Linear system stuff for zmp/com control -----------------------------
    if nc > 0
%       A_ls = ctrl_data.A; % always TI
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
        D_ls = -xcom(3)/(hddot+9.81)*eye(2);  
      end
      if typecheck(ctrl_data.S,'double')
        % ti-lqr case
        S = ctrl_data.S;
        s1= zeros(4,1); % ctrl_data.s1; 
        xlimp0 = ctrl_data.xlimp0;
      else
        S = ctrl_data.S.eval(t);
        s1= ctrl_data.s1.eval(t);
        xlimp0 = zeros(4,1); % not needed in TV case, capture by s1 term
      end
      xlimp = [xcom(1:2); J*qd]; % state of LIP model
      x_bar = xlimp - xlimp0;
    end
    
    %----------------------------------------------------------------------
    % Free DOF cost function ----------------------------------------------

    if nq_free > 0
      if nc > 0
        % approximate quadratic cost for free dofs with the appropriate matrix block
        Hqp = J(:,obj.free_dof)'*R_ls*J(:,obj.free_dof);
        Hqp = Hqp + J(:,obj.free_dof)'*D_ls'*Qy*D_ls*J(:,obj.free_dof);
        Hqp = Hqp + obj.w*eye(nq_free);

        fqp = x_bar'*C_ls'*Qy*D_ls*J(:,obj.free_dof);
        fqp = fqp + qd(obj.free_dof)'*Jdot(:,obj.free_dof)'*D_ls'*Qy*D_ls*J(:,obj.free_dof);
        fqp = fqp + x_bar'*S*B_ls*J(:,obj.free_dof);
        fqp = fqp + 0.5*s1'*B_ls*J(:,obj.free_dof);
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
    nparams = nq_con+nu_con+nf+nc*dim;
    Iqdd = zeros(nq_con,nparams); Iqdd(:,1:nq_con) = eye(nq_con);
    Iu = zeros(nu_con,nparams); Iu(:,nq_con+(1:nu_con)) = eye(nu_con);
    Iz = zeros(nc,nparams); Iz(:,nq_con+nu_con+(1:nc)) = eye(nc);
    Ibeta = zeros(nc*nd,nparams); Ibeta(:,nq_con+nu_con+nc+(1:nc*nd)) = eye(nc*nd);
    Ieps = zeros(nc*dim,nparams); 
    Ieps(:,nq_con+nu_con+nc+nc*nd+(1:nc*dim)) = eye(nc*dim);
    
    
    %----------------------------------------------------------------------
    % Set up problem constraints ------------------------------------------

    lb = [-1e3*ones(1,nq_con) r.umin(obj.con_inputs)' zeros(1,nf)   -obj.slack_limit*ones(1,nc*dim)]'; % qddot/input/contact forces/slack vars
    ub = [ 1e3*ones(1,nq_con) r.umax(obj.con_inputs)' 500*ones(1,nf) obj.slack_limit*ones(1,nc*dim)]';

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
    %  min: quad(Jdot*qd + J*qdd,R_ls)+quad(C*x+D*(Jdot*qd + J*qdd),Qy) + (2*x'*S + s1')*(A*x + B*(Jdot*qd + J*qdd)) + w*quad(qddot_ref - qdd) + quad(u,R) + quad(epsilon)
    
    if nc > 0
      Hqp = Iqdd'*J(:,obj.con_dof)'*R_ls*J(:,obj.con_dof)*Iqdd;
      Hqp = Hqp + Iqdd'*J(:,obj.con_dof)'*D_ls'*Qy*D_ls*J(:,obj.con_dof)*Iqdd;
      Hqp(1:nq_con,1:nq_con) = Hqp(1:nq_con,1:nq_con) + obj.w*eye(nq_con);

      fqp = x_bar'*C_ls'*Qy*D_ls*J(:,obj.con_dof)*Iqdd;
      fqp = fqp + qd(obj.con_dof)'*Jdot(:,obj.con_dof)'*D_ls'*Qy*D_ls*J(:,obj.con_dof)*Iqdd;
      fqp = fqp + x_bar'*S*B_ls*J(:,obj.con_dof)*Iqdd;
      fqp = fqp + 0.5*s1'*B_ls*J(:,obj.con_dof)*Iqdd;
      fqp = fqp - obj.w*q_ddot_des(obj.con_dof)'*Iqdd;

      % quadratic slack var cost 
      Hqp(nparams-nc*dim+1:end,nparams-nc*dim+1:end) = eye(nc*dim); 
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

%       Q=full(Hqp);
%       c=2*fqp;
%       Aeq = full(Aeq);
%       Ain = full(Ain);
%       save(sprintf('data/model_t_%2.3f.mat',t),'Q','c','Aeq','beq','Ain','bin','lb','ub');
      qp_tic = tic;
      result = gurobi(model,obj.solver_options);
      alpha = result.x;
      qp_toc = toc(qp_tic);
      fprintf('QP solve: %2.4f\n',qp_toc);
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
    
    if obj.debug && nc > 0
      xcomdd = Jdot * qd + J * alpha(1:nq);
      zmppos = xcom(1:2) + D_ls * xcomdd;
      % Set zmp z-pos to 1m for DRC Quals 1
      plot_lcm_points([zmppos', terrain_height], [1, 0, 0], 660, 'Commanded ZMP', 1, true);
      
      [cheight,normals] = getTerrainHeight(r,cpos);
      d = RigidBodyManipulator.surfaceTangents(normals);

      lambda = Iz*alpha;
      beta_full = Ibeta*alpha;
      cpos(3,:) = cpos(3,:) + cheight;
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
      
      % plot body coordinate frames
      m=vs.obj_collection_t();
      m.objs = javaArray('vs.obj_t', size(1, 1));
      m.id=13300;
      m.type=5; % rgb triad
      m.name='Drake Body Coords';
      m.reset=true;
      m.nobjs=5; 
      
      pelvis = findLinkInd(r,'pelvis');
      xzyrpy = forwardKin(r,kinsol,pelvis,[0;0;0],1);
      msg=vs.obj_t();
      msg.id=1;
      msg.x=xzyrpy(1); msg.y=xzyrpy(2); msg.z=xzyrpy(3)+terrain_height;
      msg.roll=xzyrpy(4); msg.pitch=xzyrpy(5); msg.yaw=xzyrpy(6);
      m.objs(msg.id) = msg;

      head = findLinkInd(r,'head');
      xzyrpy = forwardKin(r,kinsol,head,[0;0;0],1);
      msg=vs.obj_t();
      msg.id=2;
      msg.x=xzyrpy(1); msg.y=xzyrpy(2); msg.z=xzyrpy(3)+terrain_height;
      msg.roll=xzyrpy(4); msg.pitch=xzyrpy(5); msg.yaw=xzyrpy(6);
      m.objs(msg.id) = msg;

      xzyrpy = forwardKin(r,kinsol,obj.rfoot_idx,[0;0;0],1);
      msg=vs.obj_t();
      msg.id=3;
      msg.x=xzyrpy(1); msg.y=xzyrpy(2); msg.z=xzyrpy(3)+terrain_height;
      msg.roll=xzyrpy(4); msg.pitch=xzyrpy(5); msg.yaw=xzyrpy(6);
      m.objs(msg.id) = msg;

      xzyrpy = forwardKin(r,kinsol,obj.lfoot_idx,[0;0;0],1);
      msg=vs.obj_t();
      msg.id=4;
      msg.x=xzyrpy(1); msg.y=xzyrpy(2); msg.z=xzyrpy(3)+terrain_height;
      msg.roll=xzyrpy(4); msg.pitch=xzyrpy(5); msg.yaw=xzyrpy(6);
      m.objs(msg.id) = msg;

      xzyrpy = x(1:6); 
      msg=vs.obj_t();
      msg.id=5;
      msg.x=xzyrpy(1); msg.y=xzyrpy(2); msg.z=xzyrpy(3)+terrain_height;
      msg.roll=xzyrpy(4); msg.pitch=xzyrpy(5); msg.yaw=xzyrpy(6);
      m.objs(msg.id) = msg;
      
      obj.lc.publish('OBJ_COLLECTION', m);
    end

%     max(Iz*alpha)
    out_toc=toc(out_tic);
    fprintf('Output loop: %2.4f\n',out_toc);
   
  end
  end

  properties
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
    debug = false;
    lc;
    contact_est_monitor;
    lfoot_contact_state = 0; 
    rfoot_contact_state = 0; 
    lcm_foot_contacts;  
    eq_array = repmat('=',100,1); % so we can avoid using repmat in the loop
    ineq_array = repmat('<',100,1); % so we can avoid using repmat in the loop
  end
end
