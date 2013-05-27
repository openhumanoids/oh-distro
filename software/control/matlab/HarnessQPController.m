classdef HarnessQPController < MIMODrakeSystem

  methods
  function obj = HarnessQPController(r,options)
    % @param r atlas instance
    typecheck(r,'Atlas');

    qddframe = AtlasCoordinates(r);

    input_frame = MultiCoordinateFrame({qddframe,r.getStateFrame});
    output_frame = r.getInputFrame();
    obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
    obj = setSampleTime(obj,[.005;0]); % sets controller update rate
    obj = setInputFrame(obj,input_frame);
    obj = setOutputFrame(obj,output_frame);

    obj.robot = r;
    
    obj.nu = getNumInputs(r);
    obj.nq = getNumDOF(r);
    if (~isfield(options,'R'))
      obj.R = 1e-6*eye(obj.nu);
    else
      typecheck(options.R,'double');
      sizecheck(options.R,[obj.nu,obj.nu]);
      obj.R = options.R;
    end
    
    if obj.solver==1 % use cplex
      obj.solver_options = cplexoptimset('cplex');
      obj.solver_options.diagnostics = 'on';
      obj.solver_options.maxtime = 0.001;
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
        obj.solver_options.bariterlimit = 30; % iteration limit
        obj.solver_options.barhomogeneous = 0; % 0 off, 1 on
        obj.solver_options.barconvtol = 1e-4;
      end

    end    
  end
    
  function y=mimoOutput(obj,t,~,varargin)
%     tic;
    q_ddot_des = varargin{1};
    x = varargin{2};
    r = obj.robot;
    q = x(1:obj.nq); 
    qd = x(obj.nq+(1:obj.nq));
   
    % TODO: this whole thing could be easily mex'd
    
    [H,C,B] = manipulatorDynamics(r,q,qd);
    
    nparams = obj.nq+obj.nu;
    Iqdd = zeros(obj.nq,nparams); Iqdd(:,1:obj.nq) = eye(obj.nq);
    Iu = zeros(obj.nu,nparams); Iu(:,obj.nq+(1:obj.nu)) = eye(obj.nu);
    
    lb = [-1e3*ones(obj.nq,1); r.umin]; 
    ub = [ 1e3*ones(obj.nq,1); r.umax];

    Aeq = H*Iqdd - B*Iu; 
    beq = -C;

    Hqp = Iqdd'*Iqdd;
    fqp = -q_ddot_des'*Iqdd;

    % quadratic input cost
    Hqp(obj.nq+(1:obj.nu),obj.nq+(1:obj.nu)) = obj.R;

    if obj.solver==1
      alpha = cplexqp(Hqp,fqp,Ain,bin,[],[],lb,ub,[],obj.solver_options);
    
    else
      model.Q = sparse(Hqp);
      model.obj = 2*fqp;
      model.A = sparse(Aeq);
      model.rhs = beq;
      model.sense = repmat('=',length(beq),1);
      model.lb = lb;
      model.ub = ub;

      result = gurobi(model,obj.solver_options);
      alpha = result.x;
    end
    
    y = alpha(obj.nq+(1:obj.nu));
%     toc
   
  end
  end

  properties
    robot % to be controlled
    nq
    nu
    R  % quadratic input cost matrix
    solver = 0; % 0: gurobi, 1:cplex
    solver_options = struct();
  end
end
