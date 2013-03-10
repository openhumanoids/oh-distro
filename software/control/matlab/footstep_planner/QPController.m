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
    supportframe = AtlasBody(r);

    input_frame = MultiCoordinateFrame({qddframe,supportframe,r.getStateFrame});
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
      obj.R = 0.00001*eye(nu);
    else
      typecheck(options.R,'double');
      sizecheck(options.R,[nu,nu]);
      obj.R = options.R;
    end
      
end
    
  function y=mimoOutput(obj,t,~,varargin)
    % alpha = [qdd; \bar{u}; z_1; ...; z_nc; beta_1; ...; beta_nc; ...; 
    %           beta_{nc*nd}]
    % where nc is the number of contact points, and nd is the (even) 
    % number of direction vectors in the polyhedral friction cone
    % approx. 

    tic;
    zmpd = getData(obj.zmpdata);
    h=zmpd.h; 
    % linear inverted pendulum stuff
    if isTI(zmpd.V)
      S = zmpd.V.S;
    else
      S = zmpd.V.S.eval(t);
    end
    G = -h/9.81*eye(2); % zmp-input transfer matrix
    % NOTE: should we include hddot in G as well?
   
    r = obj.robot;
    nd = 4; % for friction cone approx, hard coded for now
    dim = 3;
    nu = getNumInputs(r);
    nq = getNumDOF(r);
    
    q_ddot_des = varargin{1};
    supports = varargin{2};
    x = varargin{3};
 
    q = x(1:nq);
    qd = x(nq+(1:nq));
    kinsol = doKinematics(r,q,true);
    
    [H,C,B] = manipulatorDynamics(r,q,qd);
    [xcom,J,dJ] = getCOM(r,kinsol);
       
    xlimp = [xcom(1:2); J(1:2,:)*qd];

    active_supports = find(supports~=0);
    if (isempty(active_supports))
      warning('QPController::No supporting bodies...');
    end
    partial_supports = intersect(find(supports>0),find(supports<1));
   
    % get active contacts
    [phi,Jz,D_] = contactConstraints(r,kinsol,active_supports);
    active_contacts = find(abs(phi)<0.005);
    nc = length(active_contacts);

    if nc==0
      % ignore supporting body spec, use any body in contact
      [~,Jp,dJp] = contactPositions(r,kinsol);
      [phi,Jz,D_] = contactConstraints(r,kinsol);
      active_contacts = find(abs(phi)<0.005);
      nc = length(active_contacts);
      partial_contacts = [];
      partial_idx = [];
    else
      % get support contact J, dJ for no-slip constraint
      n_support_contacts=0;
      %contact_pos = zeros(3,size(getBodyContacts(r,active_supports),2));
      Jp = zeros(size(getBodyContacts(r,active_supports),2)*3,getNumDOF(r));
      partial_contacts = [];
      for i=1:length(active_supports)
        nC = size(getBodyContacts(r,active_supports(i)),2);
        if nC>0
          %[contact_pos(:,n_support_contacts+(1:nC)),Jp(3*n_support_contacts+(1:3*nC),:),dJp(3*n_support_contacts+(1:3*nC),:)] = forwardKin(r,kinsol,active_supports(i),getBodyContacts(r,active_supports(i)));
          [~,Jp(3*n_support_contacts+(1:3*nC),:),dJp(3*n_support_contacts+(1:3*nC),:)] = forwardKin(r,kinsol,active_supports(i),getBodyContacts(r,active_supports(i)));
          if any(partial_supports==i)
            partial_contacts = [partial_contacts; (num_support_contacts+1:n_support_contacts + nC)];
          end
          n_support_contacts = n_support_contacts + nC;
        end
      end

      % get subset of active_contacts that are partial supports
      partial_contacts = intersect(active_contacts,partial_contacts);
      partial_idx = zeros(dim*length(partial_contacts),1);
      for i=1:length(partial_contacts);
        partial_idx((i-1)*dim+1:i*dim) = (partial_contacts(i)-1)*dim + (1:dim)';
      end
    end
    
    nf = nc+nc*nd;
    nparams = nq+nu+nf+nc*dim;
    
    % handy index matrices
    Iqdd = zeros(nq,nparams); Iqdd(:,1:nq) = eye(nq);
    Iu = zeros(nu,nparams); Iu(:,nq+(1:nu)) = eye(nu);
    Iz = zeros(nc,nparams); Iz(:,nq+nu+(1:nc)) = eye(nc);
    Ibeta = zeros(nc*nd,nparams); Ibeta(:,nq+nu+nc+(1:nc*nd)) = eye(nc*nd);
    Ieps = zeros(nc*dim,nparams); 
    Ieps(:,nq+nu+nc+nc*dim+(1:nc*dim)) = eye(nc*dim);
    
    lb = [-1e3*ones(1,nq) r.umin' zeros(1,nf)   -obj.slack_limit*ones(1,nc*dim)]'; % qddot/input/contact forces/slack vars
    ub = [ 1e3*ones(1,nq) r.umax' 1e4*ones(1,nf) obj.slack_limit*ones(1,nc*dim)]';
    
    % D_ is the parameterization of the polyhedral approximation of the 
    %    friction cone, in joint coordinates (figure 1 from Stewart96)
    %    D{k}(i,:) is the kth direction vector for the ith contact (of nC)
    % Create Dbar such that Dbar(:,(k-1)*nd+i) is ith direction vector for the kth
    % contact point
    D = cell(1,nc);
    for k=1:nc
      for i=1:nd
        D{k}(:,i) = D_{i}(active_contacts(k),:)'; 
      end
    end
    Dbar = [D{:}];

    % compute linear constraints
    Aeq_ = cell(1,2);
    beq_ = cell(1,2);
    Ain_ = cell(1,nc);
    bin_ = cell(1,nc);

    % TODO: handle the case with no contacts
    % CT dynamics 
    Jz = Jz(active_contacts,:);
    Aeq_{1} = H*Iqdd - B*Iu - Jz'*Iz - Dbar*Ibeta;
    beq_{1} = -C;

    % no-slip constraint
    active_idx = zeros(dim*length(active_contacts),1);
    for i=1:length(active_contacts);
      active_idx((i-1)*dim+1:i*dim) = (active_contacts(i)-1)*dim + (1:dim)';
    end
    Jp = Jp(active_idx,:);
    Jpdot = zeros(nc*dim,nq);
    for i=1:nq
      Jpdot(:,i) = dJp(active_idx,(i-1)*nq+(1:nq))*qd;
    end
    Aeq_{2} = Jp*Iqdd + Ieps;
    beq_{2} = -Jpdot*qd - 1.0*Jp*qd;
    
    % TEMP: hard code mu
    mu = 1.0*ones(nc,1);
    for i=1:nc
      Ain_{i} = -mu(i)*Iz(i,:) + ones(1,nd)*Ibeta((i-1)*nd+(1:nd),:);
      bin_{i} = 0;
    end

    % linear equality constraints: Aeq*alpha = beq
    Aeq = sparse(blkdiag(Aeq_{:}) * repmat(eye(nparams),2,1));
    beq = vertcat(beq_{:});
      
    % linear inequality constraints: Ain*alpha <= bin
    Ain = sparse(blkdiag(Ain_{:}) * repmat(eye(nparams),nc,1));
    bin = vertcat(bin_{:});

    Jdot = zeros(dim,nq);
    for i=1:nq
      Jdot(:,i) = dJ(:,(i-1)*nq+(1:nq))*qd;
    end
    
    % minimize: 
    %  quad(F*x+G*(Jdot*qd + J*qdd),Qy) + 2*x'*S*(A*x + E*(Jdot*qd + J*qdd)) + w*quad(qddot_ref - qdd) + quad(u,R) + quad(epsilon)
    % 
    % the below is vectorized with constant terms dropped
    
    J2 = J(1:2,:);
    J2dot = Jdot(1:2,:);
    
    Hqp = repmat(eye(nparams),2,1)'*blkdiag(Iqdd'*J2'*G'*obj.Qy*G*J2*Iqdd, obj.w*Iqdd'*Iqdd)*repmat(eye(nparams),2,1);
    % quadratic input cost
    Hqp(nq+(1:nu),nq+(1:nu)) = obj.R;

    % add cost term for transitional contacts
    qz = zeros(nc,1);
    qz(partial_contacts) = 1e-6*ones(length(partial_contacts),1);
    qbeta = zeros(nc*nd,1);
    qbeta(partial_idx) = 1e-6*ones(length(partial_idx),1);
    Hqp(nq+nu+(1:nc),nq+nu+(1:nc)) = diag(qz);
    Hqp(nq+nu+nc+(1:nc*nd),nq+nu+nc+(1:nc*nd)) = diag(qbeta);
    
    % quadratic slack var cost 
    Hqp(nparams-nc*dim+1:end,nparams-nc*dim+1:end) = eye(nc*dim); 

    fqp = horzcat(xlimp'*obj.F'*obj.Qy*G*J2*Iqdd, ...
          qd'*J2dot'*G'*obj.Qy*G*J2*Iqdd, ...
          xlimp'*S*obj.E*J2*Iqdd, ...
          -obj.w*q_ddot_des'*Iqdd)*repmat(eye(nparams),4,1);
        
    if 0
      % try selecting interior point method (QP_method=4?)
      alpha = cplexqp(Hqp,fqp,Ain,bin,Aeq,beq,lb,ub,[],obj.options);
    
    else
      model.Q = sparse(Hqp);
      model.obj = 2*fqp;
      model.A = [Aeq; Ain];
      model.rhs = [beq; bin];
      model.sense = [repmat('=',length(beq),1); repmat('<',length(bin),1)];
      model.lb = lb;
      model.ub = ub;
  
      params.outputflag = 0; % not verbose
      params.method = -1; % -1 auto, 2 barrier

      result = gurobi(model,params);
      alpha = result.x;
    end
    
    y=alpha(nq+(1:nu));
    toc
   
  end
  end

  properties
    robot % to be controlled
    zmpdata
    w = 1.0; % objective function weight
    slack_limit = 1.0; % maximum absolute magnitude of acceleration slack variable values
    R  % quadratic input cost matrix
    % LIP stuff
    A = [zeros(2),eye(2); zeros(2,4)]; % state transfer matrix
    E = [zeros(2); eye(2)]; % input transfer matrix
    F = [eye(2),zeros(2)]; % zmp-state transfer matrix
    Qy = eye(2); % output cost matrix--must match ZMP LQR cost 
    options = cplexoptimset('Diagnostics','on');
    %options = cplexoptimset('MaxTime',0.01);
  end
end
