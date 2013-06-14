classdef Atlas < Biped
  
  methods
    
    function obj=Atlas(urdf,options)
      typecheck(urdf,'char');

      if nargin < 2
        options = struct();
      end
      if ~isfield(options,'dt')
        options.dt = 0.001;
      end
      if ~isfield(options,'floating')
        options.floating = true;
      end
      
      S = warning('off','Drake:RigidBodyManipulator:SingularH');
  
%       obj = obj@TimeSteppingRigidBodyManipulator(urdf,options.dt,options);
      obj = obj@Biped(urdf,options.dt,options);

      
      if options.floating
        % could also do fixed point search here
        obj = obj.setInitialState(double(obj.manip.resolveConstraints(zeros(obj.getNumStates(),1))));
      else
        % TEMP HACK to get by resolveConstraints
        for i=1:length(obj.manip.body), obj.manip.body(i).contact_pts=[]; end
        obj.manip = compile(obj.manip);
        obj = obj.setInitialState(zeros(obj.getNumStates(),1));
      end
      warning(S);
    end
    
    function obj = compile(obj)
      S = warning('off','Drake:RigidBodyManipulator:SingularH');
      obj = compile@TimeSteppingRigidBodyManipulator(obj);
      warning(S);
      
      state_frame = AtlasState(obj);
      obj = obj.setStateFrame(state_frame);
      obj = obj.setOutputFrame(state_frame);
    
      input_frame = AtlasInput(obj);
      obj = obj.setInputFrame(input_frame);
    end

    function obj = setInitialState(obj,x0)
      if isa(x0,'Point')
        obj.x0 = double(x0); %.inFrame(obj.getStateFrame));
      else
        typecheck(x0,'double');
        sizecheck(x0,obj.getNumStates());
        obj.x0 = x0;
      end
    end
    
    function x0 = getInitialState(obj)
      x0 = obj.x0;
    end    
    
    function [xstar,ustar,zstar] = getFixedPoint(obj,options)
      if nargin < 2 || ~isfield(options,'visualize')
        options.visualize = false;
      end
      
      x0 = Point(obj.getStateFrame());
      x0 = resolveConstraints(obj,x0);
      u0 = zeros(obj.getNumInputs(),1);

      nq = obj.getNumDOF();
      nu = obj.getNumInputs();
      nz = obj.getNumContacts()*3;
      z0 = zeros(nz,1);
      q0 = x0(1:nq);
    
      problem.x0 = [q0;u0;z0];
      problem.objective = @(quz) 0; % feasibility problem
      problem.nonlcon = @(quz) mycon(quz);
      problem.solver = 'fmincon';

      if options.visualize
        v = obj.constructVisualizer;
        %problem.options=optimset('DerivativeCheck','on','GradConstr','on','Algorithm','interior-point','Display','iter','OutputFcn',@drawme,'TolX',1e-14,'MaxFunEvals',5000);
        problem.options=optimset('GradConstr','on','Algorithm','interior-point','Display','iter','OutputFcn',@drawme,'TolX',1e-14,'MaxFunEvals',5000);
      else
        problem.options=optimset('GradConstr','on','Algorithm','interior-point','TolX',1e-14,'MaxFunEvals',5000);
      end
      
      lb_z = -1e6*ones(nz,1);
      lb_z(3:3:end) = 0; % normal forces must be >=0
      ub_z = 1e6*ones(nz,1);
    
      [jl_min,jl_max] = obj.getJointLimits();
      % force search to be close to starting position
      problem.lb = [max(q0-0.05,jl_min+0.01); obj.umin; lb_z];
      problem.ub = [min(q0+0.05,jl_max-0.01); obj.umax; ub_z];
      %problem.lb(2) = 0.0; % body z

      [quz_sol,~,exitflag] = fmincon(problem);
      success=(exitflag==1);
      xstar = [quz_sol(1:nq); zeros(nq,1)];
      ustar = quz_sol(nq+(1:nu));
      zstar = quz_sol(nq+nu+(1:nz));
      if (~success)
        error('failed to find fixed point');
      end

      function stop=drawme(quz,optimValues,state)
        stop=false;
        v.draw(0,[quz(1:nq); zeros(nq,1)]);
      end

      function [c,ceq,GC,GCeq] = mycon(quz)
        q=quz(1:nq);
        u=quz(nq+(1:nu));
        z=quz(nq+nu+(1:nz));

        [~,C,B,~,dC,~] = obj.manip.manipulatorDynamics(q,zeros(nq,1));
        [phiC,JC] = obj.contactConstraints(q);
        [~,J,dJ] = obj.contactPositions(q);
        
        % ignore friction constraints for now
        c = 0;
        GC = zeros(nq+nu+nz,1); 
        
        dJz = zeros(nq,nq);
        for i=1:nq
            dJz(:,i) = dJ(:,(i-1)*nq+1:i*nq)'*z;
        end
        
        ceq = [C-B*u-J'*z; phiC];
        GCeq = [[dC(1:nq,1:nq)-dJz,-B,-J']',[JC'; zeros(nu+nz,length(phiC))]]; 
      end
    end
    
    function [u,obj] = inverseDynamics(obj,q,qdot,qddot_des,active_supports)
      if isempty(obj.inverse_dyn_qp_controller)
        ctrl_data = SharedDataHandle(struct(...
          'A',zeros(4),...
          'B',zeros(4,2),...
          'C',zeros(2,4),...
          'Qy',zeros(2),...
          'R',zeros(2),...
          'is_time_varying',false,...
          'S',zeros(4),...
          's1',zeros(4,1),...
          's2',0,...
          'x0',zeros(4,1),...
          'u0',zeros(2,1),...
          'trans_drift',zeros(3,1),...
          'support_times',0,...
          'supports',active_supports,...
          'mu',1,...
          'ignore_terrain',true,...
          'y0',zeros(2,1)));
        
        % instantiate QP controller
        options.slack_limit = 30.0;
        options.w = 0.01;
        options.R = 1e-12*eye(getNumInputs(obj));
        options.lcm_foot_contacts = false;
        options.full_body_opt = true;
        options.debug = false;
        options.use_mex = 2;
      
        obj.inverse_dyn_qp_controller = QPController(obj,ctrl_data,options);
      else
        ctrl_data = obj.inverse_dyn_qp_controller.controller_data;
        setField(ctrl_data,'supports',active_supports);
      end
      
      u = obj.inverse_dyn_qp_controller.mimoOutput(0,[],qddot_des,zeros(12,1),[q;qdot]);
    end
      
  end
  
  properties (SetAccess = protected, GetAccess = public)
    x0
    inverse_dyn_qp_controller;
  end
end
