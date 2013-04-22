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
    
    obj.rfoot_idx = find(strcmp('r_foot',getLinkNames(r)));
    obj.lfoot_idx = find(strcmp('l_foot',getLinkNames(r)));
    
    obj.lc = lcm.lcm.LCM.getSingleton();
    obj.contact_est = drc.foot_contact_estimate_t();
    obj.contact_est.detection_method = 0;
    
  end
    
  function y=mimoOutput(obj,t,~,varargin)
%     tic;

    q_ddot_des = varargin{1};
    x = varargin{2};
    r = obj.robot;
    zmpd = getData(obj.zmpdata);

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
    
    contact_threshold = 0.0001; % m
    
    % get active contacts
    [phi,Jz,D_] = contactConstraints(r,kinsol,active_supports);
    phi
    active_contacts = phi<contact_threshold;

    % hack to overload support trajectory to allow specification of ground
    % height for that foot step. e.g., supp(idx) = 1 means z=0, supp(idx) =
    % 1.2 means z=1.2
%     if any(phi(1:4)<(supp(active_supports(1))-1+contact_threshold))
%       active_contacts(1:4) = 1;
%     end
%     if length(phi)>4 && any(phi(5:8)<(supp(active_supports(2))-1+contact_threshold))
%       active_contacts(5:8) = 1;
%     end
    
    obj.contact_est.utime = t*1000000;
    
    %%%%% Testing: if any foot point is in contact, all contact points are active %%%%%
    obj.contact_est.right_contact = 0;
    obj.contact_est.left_contact = 0;
    if any(active_contacts(1:4))
      if active_supports(1)==obj.rfoot_idx
        obj.contact_est.right_contact = 1;
      elseif active_supports(1)==obj.lfoot_idx
        obj.contact_est.left_contact = 1;
      end
      active_contacts(1:4) = 1;
    end
    if length(phi)>4 && any(active_contacts(5:8))
      if active_supports(2)==obj.rfoot_idx
        obj.contact_est.right_contact = 1;
      elseif active_supports(2)==obj.lfoot_idx
        obj.contact_est.left_contact = 1;
      end
      active_contacts(5:8) = 1;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    obj.lc.publish('KINEMATIC_FOOT_CONTACT_EST', obj.contact_est);
    
    nc = sum(active_contacts);

    if nc==0
      % ignore supporting body spec, use any foot in contact
      active_supports = [obj.rfoot_idx, obj.lfoot_idx];

      [~,Jp,Jpdot] = contactPositionsJdot(r,kinsol);
      [phi,Jz,D_] = contactConstraints(r,kinsol);
      active_contacts = phi<contact_threshold;

      %%%%% Testing: if any foot point is in contact, all contact points are active %%%%%
      if any(active_contacts(1:4))
        if active_supports(1)==obj.rfoot_idx
          obj.contact_est.right_contact = 1;
        elseif active_supports(1)==obj.lfoot_idx
          obj.contact_est.left_contact = 1;
        end
        active_contacts(1:4) = 1;
      end
      if length(phi)>4 && any(active_contacts(5:8))
        if active_supports(2)==obj.rfoot_idx
          obj.contact_est.right_contact = 1;
        elseif active_supports(2)==obj.lfoot_idx
          obj.contact_est.left_contact = 1;
        end
        active_contacts(5:8) = 1;
      end
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      nc = sum(active_contacts);
    else
      % get support contact J, dJ for no-slip constraint
      [~,Jp,Jpdot] = contactPositionsJdot(r,kinsol,active_supports);
    end
    
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
%     try
      result = gurobi(model,obj.solver_options);
      alpha = result.x;
%     catch err
%       stophere=1;
%     end
%       toc
    end
    
    y = alpha(nq+(1:nu));
    
    if obj.debug && nc > 0
      xcomdd = Jdot * qd + J * alpha(1:nq);
      zmppos = xcom(1:2) + G * xcomdd;
      % Set zmp z-pos to 1m for DRC Quals 1
      plot_lcm_points([zmppos', 1], [1, 0, 0], 60, 'Current ZMP', 1, true);
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
    contact_est;
  end
end
