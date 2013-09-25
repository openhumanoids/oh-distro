classdef InverseDynamicsBlock < MIMODrakeSystem

  methods
  function obj = InverseDynamicsBlock(r,options)
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
    
    qddframe = AtlasCoordinates(r);
    input_frame = MultiCoordinateFrame({qddframe,getStateFrame(r)});
    output_frame = getInputFrame(r);
    
    obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
    obj = setSampleTime(obj,[dt;0]); % sets controller update rate
    obj = setInputFrame(obj,input_frame);
    obj = setOutputFrame(obj,output_frame);

    obj.robot = r;
    
    obj.nu = getNumInputs(r);
    obj.nq = getNumDOF(r);
  end
    
  function y=mimoOutput(obj,t,~,varargin)
    q_ddot_des = varargin{1};
    x = varargin{2};
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
    
    [H,C,B] = manipulatorDynamics(r,q,qd);

    % add friction compensating force using desired accelerations
    f_friction = computeFrictionForce(r,qd + 0.1*q_ddot_des) - computeFrictionForce(r,qd);

    y=B\(H*q_ddot_des + C + f_friction);
  end
  end

  properties
    robot % to be controlled
    nq
    nu
  end
end
