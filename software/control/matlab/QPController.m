classdef QPController < MIMODrakeSystem

  % implementation assumes 3D atlas model
  methods
  function obj = QPController(r,zmpdata,options)
    % @param r atlas instance
    % @param options structure for specifying objective weight (w), slack
    % variable limits (slack_limit), and action cost (R)
    typecheck(r,'Atlas');
    typecheck(zmpdata,'SharedDataHandle');
    if nargin>2
      typecheck(options,'struct');
    else
      options = struct();
    end
    
    qddframe = AtlasCoordinates(r);

    input_frame = MultiCoordinateFrame({qddframe,r.getStateFrame});
    output_frame = r.getInputFrame();
    obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
    obj = setSampleTime(obj,[.005;0]); % sets controller update rate
    obj = setInputFrame(obj,input_frame);
    obj = setOutputFrame(obj,output_frame);

    obj.robot = r;
    obj.zmpdata = zmpdata;

    if (isfield(options,'w'))
      typecheck(options.w,'double');
      sizecheck(options.w,1);
      obj.w = options.w;
    end
    
    if (isfield(options,'slack_limit'))
      typecheck(options.slack_limit,'double');
      sizecheck(options.slack_limit,1);
      obj.slack_limit = options.slack_limit;
    end
    
    nu = getNumInputs(r);
    if (~isfield(options,'R'))
      obj.R = 1e-6*eye(nu);
    else
      typecheck(options.R,'double');
      sizecheck(options.R,[nu,nu]);
      obj.R = options.R;
    end
    
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
%       obj.solver_options.presolve = 0;

      if obj.solver_options.method == 2
        obj.solver_options.bariterlimit = 20; % iteration limit
        obj.solver_options.barhomogeneous = 0; % 0 off, 1 on
        obj.solver_options.barconvtol = 1e-4;
      end
    end  
    
    obj.rfoot_idx = findLinkInd(r,'r_foot');
    obj.lfoot_idx = findLinkInd(r,'l_foot');
    
    obj.contact_est_monitor = drake.util.MessageMonitor(drc.foot_contact_estimate_t,'utime');
    obj.lc = lcm.lcm.LCM.getSingleton();
    obj.lc.subscribe('FOOT_CONTACT_ESTIMATE',obj.contact_est_monitor);
    
  end
    
  function y=mimoOutput(obj,t,~,varargin)
%     tic;

    q_ddot_des = varargin{1};
    x = varargin{2};
    r = obj.robot;
    zmpd = getData(obj.zmpdata);

    % get pelvis height above height map
    x(3) = x(3)-getTerrainHeight(r,x(1:2));
      
    % use support trajectory
    if typecheck(zmpd.supptraj,'double') %zmpd.ti_flag
      supp = zmpd.supptraj;
    else
      supp = zmpd.supptraj.eval(t);
    end
    active_supports = find(supp);
    
    nd = 4; % for friction cone approx, hard coded for now
    dim = 3; % 3D
    nu = getNumInputs(r);
    nq = getNumDOF(r);
    
    q = x(1:nq); 
    qd = x(nq+(1:nq));
    kinsol = doKinematics(r,q,false,true,qd);
    
    [H,C,B] = manipulatorDynamics(r,q,qd);
    
    [xcom,J] = getCOM(r,kinsol);
    J = J(1:2,:); % only need COM x-y
    Jdot = forwardJacDot(r,kinsol,0);
    Jdot = Jdot(1:2,:);
    
    % get active contacts
    [phi,Jz,D_] = contactConstraints(r,kinsol,active_supports);
    if obj.debug
      phi
%       x(1:6)
    end
    active_contacts = zeros(length(phi),1);

    contact_data = obj.contact_est_monitor.getNextMessage(1);
    if ~isempty(contact_data)
      msg = drc.foot_contact_estimate_t(contact_data);
      obj.lfoot_contact_state = msg.left_contact;
      obj.rfoot_contact_state = msg.right_contact;
    end
        
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
      Jz = Jz(active_contacts,:); % only care about active contacts

      active_idx = zeros(dim*length(active_contacts),1);
      for i=1:length(active_contacts);
        active_idx((i-1)*dim+1:i*dim) = (active_contacts(i)-1)*dim + (1:dim)';
      end
      Jp = Jp(active_idx,:); 
      Jpdot = Jpdot(active_idx,:);
      
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
          D{k}(:,i) = D_{i}(active_contacts(k),:)'; 
        end
      end
      Dbar = [D{:}];
    end
    
    %----------------------------------------------------------------------
    % Linear inverted pendulum stuff --------------------------------------
    if nc > 0    
      if typecheck(zmpd.h,'double')
        h = zmpd.h; 
        hddot = 0;
      else
        h = zmpd.h.eval(t); 
        hddot = zmpd.hddot.eval(t);
      end
      if typecheck(zmpd.S,'double')
        S = zmpd.S;
        s1= zeros(4,1); % zmpd.s1; 
        xlimp0 = zmpd.xlimp0;
      else
        S = zmpd.S.eval(t);
        s1 = zmpd.s1.eval(t);
        xlimp0 = zeros(4,1);
      end
      G = -h/(hddot+9.81)*eye(2); % zmp-input transfer matrix
      xlimp = [xcom(1:2); J*qd]; % state of LIP model
      x_bar = xlimp - xlimp0;
    end
    
    %----------------------------------------------------------------------
    % Build handy index matrices ------------------------------------------
    
    nf = nc+nc*nd; % number of contact force variables
    nparams = nq+nu+nf+nc*dim;
    Iqdd = zeros(nq,nparams); Iqdd(:,1:nq) = eye(nq);
    Iu = zeros(nu,nparams); Iu(:,nq+(1:nu)) = eye(nu);
    Iz = zeros(nc,nparams); Iz(:,nq+nu+(1:nc)) = eye(nc);
    Ibeta = zeros(nc*nd,nparams); Ibeta(:,nq+nu+nc+(1:nc*nd)) = eye(nc*nd);
    Ieps = zeros(nc*dim,nparams); 
    Ieps(:,nq+nu+nc+nc*nd+(1:nc*dim)) = eye(nc*dim);
    
    
    %----------------------------------------------------------------------
    % Set up problem constraints ------------------------------------------

    lb = [-1e3*ones(1,nq) r.umin' zeros(1,nf)   -obj.slack_limit*ones(1,nc*dim)]'; % qddot/input/contact forces/slack vars
    ub = [ 1e3*ones(1,nq) r.umax' 500*ones(1,nf) obj.slack_limit*ones(1,nc*dim)]';

    Aeq_ = cell(1,2);
    beq_ = cell(1,2);
    Ain_ = cell(1,nc);
    bin_ = cell(1,nc);

    % constrained dynamics
    if nc > 0
      Aeq_{1} = H*Iqdd - B*Iu - Jz'*Iz - Dbar*Ibeta;
    else
      Aeq_{1} = H*Iqdd - B*Iu;
    end
    beq_{1} = -C;
    
    if nc > 0
      % relative acceleration constraint
      Aeq_{2} = Jp*Iqdd + Ieps;
      beq_{2} = -Jpdot*qd - 1.0*Jp*qd;

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
    %  min: quad(F*x+G*(Jdot*qd + J*qdd),Q) + (2*x'*S + s1')*(A*x + E*(Jdot*qd + J*qdd)) + w*quad(qddot_ref - qdd) + quad(u,R) + quad(epsilon)
    
    if nc > 0
      Hqp = Iqdd'*J'*G'*obj.Qy*G*J*Iqdd;
      Hqp(1:nq,1:nq) = Hqp(1:nq,1:nq) + obj.w*eye(nq);

      fqp = x_bar'*obj.F'*obj.Qy*G*J*Iqdd;
      fqp = fqp + qd'*Jdot'*G'*obj.Qy*G*J*Iqdd;
      fqp = fqp + x_bar'*S*obj.E*J*Iqdd;
      fqp = fqp + 0.5*s1'*obj.E*J*Iqdd;
      fqp = fqp - obj.w*q_ddot_des'*Iqdd;

      % quadratic slack var cost 
      Hqp(nparams-nc*dim+1:end,nparams-nc*dim+1:end) = eye(nc*dim); 
    else
      Hqp = Iqdd'*Iqdd;
      fqp = -q_ddot_des'*Iqdd;
    end
    
    % quadratic input cost
    Hqp(nq+(1:nu),nq+(1:nu)) = obj.R;
 

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
      model.sense = [repmat('=',length(beq),1); repmat('<',length(bin),1)];
      model.lb = lb;
      model.ub = ub;

%       tic;
      result = gurobi(model,obj.solver_options);
      alpha = result.x;
%       toc
    end
    
    y = alpha(nq+(1:nu));
    
    if obj.debug && nc > 0
      xcomdd = Jdot * qd + J * alpha(1:nq);
      zmppos = xcom(1:2) + G * xcomdd;
      % Set zmp z-pos to 1m for DRC Quals 1
      plot_lcm_points([zmppos', getTerrainHeight(r,zmppos)], [1, 0, 0], 660, 'Commanded ZMP', 1, true);
      
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
      msg.x=xzyrpy(1); msg.y=xzyrpy(2); msg.z=xzyrpy(3);
      msg.roll=xzyrpy(4); msg.pitch=xzyrpy(5); msg.yaw=xzyrpy(6);
      m.objs(msg.id) = msg;

      head = findLinkInd(r,'head');
      xzyrpy = forwardKin(r,kinsol,head,[0;0;0],1);
      msg=vs.obj_t();
      msg.id=2;
      msg.x=xzyrpy(1); msg.y=xzyrpy(2); msg.z=xzyrpy(3);
      msg.roll=xzyrpy(4); msg.pitch=xzyrpy(5); msg.yaw=xzyrpy(6);
      m.objs(msg.id) = msg;

      xzyrpy = forwardKin(r,kinsol,obj.rfoot_idx,[0;0;0],1);
      msg=vs.obj_t();
      msg.id=3;
      msg.x=xzyrpy(1); msg.y=xzyrpy(2); msg.z=xzyrpy(3);
      msg.roll=xzyrpy(4); msg.pitch=xzyrpy(5); msg.yaw=xzyrpy(6);
      m.objs(msg.id) = msg;

      xzyrpy = forwardKin(r,kinsol,obj.lfoot_idx,[0;0;0],1);
      msg=vs.obj_t();
      msg.id=4;
      msg.x=xzyrpy(1); msg.y=xzyrpy(2); msg.z=xzyrpy(3);
      msg.roll=xzyrpy(4); msg.pitch=xzyrpy(5); msg.yaw=xzyrpy(6);
      m.objs(msg.id) = msg;

      xzyrpy = x(1:6); 
      msg=vs.obj_t();
      msg.id=5;
      msg.x=xzyrpy(1); msg.y=xzyrpy(2); msg.z=xzyrpy(3);
      msg.roll=xzyrpy(4); msg.pitch=xzyrpy(5); msg.yaw=xzyrpy(6);
      m.objs(msg.id) = msg;
      
      obj.lc.publish('OBJ_COLLECTION', m);
    end

%     max(Iz*alpha)
%     toc
   
  end
  end

  properties
    robot; % to be controlled
    zmpdata;
    w = 1.0; % objective function weight
    slack_limit = 1.0; % maximum absolute magnitude of acceleration slack variable values
    rfoot_idx;
    lfoot_idx;
    R; % quadratic input cost matrix
    % LIP stuff
    A = [zeros(2),eye(2); zeros(2,4)]; % state transfer matrix
    E = [zeros(2); eye(2)]; % input transfer matrix
    F = [eye(2),zeros(2)]; % zmp-state transfer matrix
    Qy = eye(2); % output cost matrix--must match ZMP LQR cost 
    solver = 0; % 0: gurobi, 1:cplex
    solver_options = struct();
    debug = false;
    lc;
    contact_est_monitor;
    lfoot_contact_state = 0; 
    rfoot_contact_state = 0; 
  end
end
