classdef QPController < MIMODrakeSystem
  % implementation assumes 3D atlas model
  methods
  function obj = QPController(r,options)
    % @param r atlas instance
    % @param options structure for specifying objective weights (w) and
    % method for handling support (i.e., foot) acceleration conditions (support_accel)
    typecheck(r,'Atlas');
    typecheck(options,'struct');
    
    comframe = AtlasCOM(r);
    qddframe = AtlasCoordinates(r);
    supportframe = AtlasBody(r);

    input_frame = MultiCoordinateFrame({comframe,qddframe,supportframe,r.getStateFrame});
    output_frame = r.getInputFrame();
    obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
    obj = setSampleTime(obj,[.005;0]); % sets controller update rate
    obj = setInputFrame(obj,input_frame);
    obj = setOutputFrame(obj,output_frame);
    obj.robot = r;
    
    if (~isfield(options,'w'))
      options.w = [1;1];
    else
      typecheck(options.w,'double');
      sizecheck(options.w,2);
    end
    obj.w = options.w;

    if (~isfield(options,'support_accel'))
      options.support_accel = 'constraint';
    else
      typecheck(options.support_accel,'char');
      if ~any(strcmp(options.support_accel,{'constraint','objective','none'}))
        error('QPController: unrecognized value given for support_accel.')
      end
    end
    
    if strcmp(obj.support_accel,'objective')
      warning('QPController: foot acceleration objective term option not implemented yet.');
      options.support_accel = 'none';
    end
    obj.support_accel = options.support_accel;
  end
    
  function y=mimoOutput(obj,t,~,varargin)
    % alpha = [qdd; \bar{u}; z_1; ...; z_nc; beta_1; ...; beta_nc; ...; 
    %           beta_{nc*nd}]
    % where nc is the number of contact points, and nd is the (even) 
    % number of direction vectors in the polyhedral friction cone
    % approx. 

    tic;
    r = obj.robot;
    
    nd = 4; % for friction cone approx, hard coded for now
    dim = 3;
    nu = getNumInputs(r);
    nq = getNumDOF(r);
    
    com_ddot_des = varargin{1};
    q_ddot_des = varargin{2};
    supports = varargin{3};
    x = varargin{4};
 
    q = x(1:nq);
    qd = x(nq+(1:nq));
    kinsol = doKinematics(r,q,true);
    
    [H,C,B] = manipulatorDynamics(r,q,qd);
    [~,J,dJ] = getCOM(r,kinsol);
    
    active_supports = find(supports~=0);
    if (isempty(active_supports))
      warning('QPController::No supporting bodies...');
    end
    
    if any(strcmp(obj.support_accel,{'constraint','objective'}))
      % get support contact J, dJ for no-slip constraint
      n_support_contacts=0;
      contact_pos = zeros(3,size(getBodyContacts(r,active_supports),2));
      Jp = zeros(size(getBodyContacts(r,active_supports),2)*3,getNumDOF(r));
      for i=1:length(active_supports)
        nC = size(getBodyContacts(r,active_supports(i)),2);
        if nC>0
          [contact_pos(:,n_support_contacts+(1:nC)),Jp(3*n_support_contacts+(1:3*nC),:),dJp(3*n_support_contacts+(1:3*nC),:)] = forwardKin(r,kinsol,active_supports(i),getBodyContacts(r,active_supports(i)));
          n_support_contacts = n_support_contacts + nC;
        end
      end
    end
    % for now only allow forces on support body contacts
    [phi,Jz,D_] = contactConstraints(r,kinsol,active_supports);
    
    % get active contacts
    active_contacts = find(abs(phi)<0.004);
%     k = convhull(contact_pos(1:2,active_contacts)'); % use only points on convex hull
%     active_contacts = active_contacts(k(1:end-1));
%     num_active_contacts = length(active_contacts)
    
    nc = length(active_contacts);
    
    nf = nc+nc*nd;
    if any(strcmp(obj.support_accel,{'constraint','objective'}))
      nparams = nq+nu+nf+nc*dim;
    else
      nparams = nq+nu+nf;
    end
    % TODO: handle the case with no contacts
    
    % handy index matrices
    Iqdd = zeros(nq,nparams); Iqdd(:,1:nq) = eye(nq);
    Iu = zeros(nu,nparams); Iu(:,nq+(1:nu)) = eye(nu);
    Iz = zeros(nc,nparams); Iz(:,nq+nu+(1:nc)) = eye(nc);
    Ibeta = zeros(nc*nd,nparams); Ibeta(:,nq+nu+nc+(1:nc*nd)) = eye(nc*nd);
    if any(strcmp(obj.support_accel,{'constraint','objective'}))
      Ieps = zeros(nc*dim,nparams); 
      Ieps(:,nq+nu+nc+nc*dim+(1:nc*dim)) = eye(nc*dim);
      lb = [-1e3*ones(1,nq) r.umin' zeros(1,nf)   -.5*ones(1,nc*dim)]'; % qddot/input/contact forces/slack vars
      ub = [ 1e3*ones(1,nq) r.umax' 1e4*ones(1,nf) .5*ones(1,nc*dim)]';
%       lb = [-1e3*ones(1,nq) r.umin' zeros(1,nf)   -100.0*ones(1,nc*dim)]'; % qddot/input/contact forces/slack vars
%       ub = [ 1e3*ones(1,nq) r.umax' 1e4*ones(1,nf) 100.0*ones(1,nc*dim)]';
    else % no constraints on support contact accelerations
      lb = [-1e3*ones(1,nq) r.umin' zeros(1,nf)]'; % qddot/input/contact forces
      ub = [ 1e3*ones(1,nq) r.umax' 1e4*ones(1,nf)]';
    end
    
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

    % CT dynamics 
    Jz = Jz(active_contacts,:);
    Aeq_{1} = H*Iqdd - B*Iu - Jz'*Iz - Dbar*Ibeta;
    beq_{1} = -C;

    if strcmp(obj.support_accel,'constraint')
      % no-slip constraint
      active_idx = zeros(dim*length(active_contacts),1);
      for i=1:length(active_contacts);
        active_idx((i-1)*dim+1:i*dim) = (active_contacts(i)-1)*dim + (1:dim)';
      end
      if t>0.5
         
      end
      
      Jp = Jp(active_idx,:);
      Jpdot = zeros(nc*dim,nq);
      for i=1:nq
        Jpdot(:,i) = dJp(active_idx,(i-1)*nq+(1:nq))*qd;
      end
      Aeq_{2} = Jp*Iqdd + Ieps;
      beq_{2} = -Jpdot*qd - 1.0*Jp*qd;
    end
    
    % TEMP: hard code mu
    mu = 0.9*ones(nc,1);
    for i=1:nc
      Ain_{i} = -mu(i)*Iz(i,:) + ones(1,nd)*Ibeta((i-1)*nd+(1:nd),:);
      bin_{i} = 0;
    end

    % linear equality constraints: Aeq*alpha = beq
    if any(strcmp(obj.support_accel,{'none','objective'}))
    	Aeq = Aeq_{1};
    	beq = beq_{1};
    else
      Aeq = sparse(blkdiag(Aeq_{:}) * repmat(eye(nparams),2,1));
      beq = vertcat(beq_{:});
    end
      
    % linear inequality constraints: Ain*alpha <= bin
    Ain = sparse(blkdiag(Ain_{:}) * repmat(eye(nparams),nc,1));
    bin = vertcat(bin_{:});

    Jdot = zeros(dim,nq);
    for i=1:nq
      Jdot(:,i) = dJ(:,(i-1)*nq+(1:nq))*qd;
    end

    Hqp = repmat(eye(nparams),2,1)'*blkdiag(obj.w(1)*Iqdd'*(J'*J + 0.0001*eye(nq))*Iqdd, obj.w(2)*Iqdd'*Iqdd)*repmat(eye(nparams),2,1);
    Hqp(nq+(1:nu),nq+(1:nu)) = 0.00001*eye(nu);
    Hqp(nparams-nc*dim+1:end,nparams-nc*dim+1:end) = eye(nc*dim); % drive slack vars to 0
    fqp = horzcat(obj.w(1)*(Jdot*qd - com_ddot_des)'*J*Iqdd, -obj.w(2)*q_ddot_des'*Iqdd)*repmat(eye(nparams),2,1);

    if 1
      % try selecting interior point method (QP_method=4?)
      alpha = cplexqp(Hqp,fqp,Ain,bin,Aeq,beq,lb,ub); %[],obj.options);
    
    else
      model.Q = sparse(Hqp);
      model.obj = fqp;
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
    w % objective function weights
    robot % to be controlled
    support_accel % string property designating how support contact accelerations are handled 
    %options = cplexoptimset('Diagnostics','on');
    %options = cplexoptimset('MaxTime',0.01);
  end
end
