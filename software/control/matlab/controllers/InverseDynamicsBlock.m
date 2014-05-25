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
    
    if ~isfield(options,'output_qdd')
      options.output_qdd = false;
    else
      typecheck(options.output_qdd,'logical');
    end
    
    qddframe = AtlasCoordinates(r);
    input_frame = MultiCoordinateFrame({qddframe,getStateFrame(r)});

    if options.output_qdd
      output_frame = MultiCoordinateFrame({r.getInputFrame(),qddframe});
    else
      output_frame = r.getInputFrame();
    end
    
    obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
    obj = setSampleTime(obj,[dt;0]); % sets controller update rate
    obj = setInputFrame(obj,input_frame);
    obj = setOutputFrame(obj,output_frame);

    obj.robot = r;
    obj.output_qdd = options.output_qdd;
    obj.nu = getNumInputs(r);
    obj.nq = getNumDOF(r);
  end
    
  function varargout=mimoOutput(obj,t,~,varargin)
    persistent qd_int t_prev
    
    if isempty(qd_int)
      qd_int = zeros(obj.nq,1);
    end
    if isempty(t_prev)
      t_prev = 0;
    end
    dt = t-t_prev;
    t_prev = t;
    
    q_ddot_des = varargin{1};
    x = varargin{2};
    r = obj.robot;
    
    q = x(1:obj.nq);
    qd = x(obj.nq+(1:obj.nq));
    
    [H,C,B] = manipulatorDynamics(r,q,qd);

    % add friction compensating force using desired accelerations
    eta = 0.01;
    qd_int = ((1-eta)*qd_int + eta*qd) + q_ddot_des*dt; 
    f_friction = computeFrictionForce(r,qd_int) - computeFrictionForce(r,qd);
%     f_friction = computeFrictionForce(r,qd + 0.3*q_ddot_des) - computeFrictionForce(r,qd);

    y=B\(H*q_ddot_des + C + f_friction);
    
    if obj.output_qdd
      varargout = {y,qdd};
    else
      varargout = {y};
    end
  end
  end

  properties
    robot % to be controlled
    nq
    nu
    output_qdd
  end
end
