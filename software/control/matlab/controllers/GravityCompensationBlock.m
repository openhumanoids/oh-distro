classdef GravityCompensationBlock < DrakeSystem

  methods
  function obj = GravityCompensationBlock(r,options)
    % @param r atlas instance
    typecheck(r,'Atlas');

    if nargin > 1
      assert(isa(options,'struct'));
    else
      options = struct();
    end

    if isfield(options,'dt')
      % controller update rate
      typecheck(options.dt,'double');
      sizecheck(options.dt,[1 1]);
      dt = options.dt;
    else
      dt = 0.003;
    end
    
    input_frame = getStateFrame(r);
    output_frame = getInputFrame(r);
    obj = obj@DrakeSystem(0,0,input_frame.dim,output_frame.dim,true,true);
    obj = setSampleTime(obj,[dt;0]); % sets controller update rate
    obj = setInputFrame(obj,input_frame);
    obj = setOutputFrame(obj,output_frame);

    obj.robot = r;
    
    obj.nu = getNumInputs(r);
    obj.nq = getNumDOF(r);
  end
    
  function y=output(obj,t,~,x)
    r = obj.robot;
    
    persistent P x_est tlast;

    if isempty(P)
      P = eye(2*obj.nq);
      x_est=zeros(2*obj.nq,1);
      tlast=t-0.003;
    end
    
    H = [eye(obj.nq) zeros(obj.nq)];
    R = 5e-4*eye(obj.nq);

    dt = t-tlast;
    F = [eye(obj.nq) dt*eye(obj.nq); zeros(obj.nq) eye(obj.nq)];
    Q = 0.3*[dt*eye(obj.nq) zeros(obj.nq); zeros(obj.nq) eye(obj.nq)];
    
    % compute filtered velocity
    jprior = F*x_est;
    Pprior = F*P*F' + Q;
    meas_resid = x(1:obj.nq) - H*jprior;
    S = H*Pprior*H' + R;
    K = (P*H')/S;
    x_est = jprior + K*meas_resid;
    P = (eye(2*obj.nq) - K*H)*Pprior;
    tlast=t;
    
    q = x_est(1:obj.nq);
    qd = x_est(obj.nq+(1:obj.nq));
    
    [~,C,B] = manipulatorDynamics(r,q,qd);

    y=B\C;
  end
  end

  properties
    robot % to be controlled
    nq
    nu
  end
end
