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
    output_frame = AtlasPosTorqueRef(r);
    
    obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
    obj = setSampleTime(obj,[dt;0]); % sets controller update rate
    obj = setInputFrame(obj,input_frame);
    obj = setOutputFrame(obj,output_frame);

    obj.robot = r;
    
    obj.nu = getNumInputs(r);
    obj.nq = getNumDOF(r);
  end
    
  function y=mimoOutput(obj,~,~,varargin)
    q_ddot_des = varargin{1};
    x = varargin{2};
    r = obj.robot;
    q = x(1:obj.nq); 
    qd = x(obj.nq+(1:obj.nq));

    [H,C,B] = manipulatorDynamics(r,q,qd);
    y=B\(H*q_ddot_des + C);
  end
  end

  properties
    robot % to be controlled
    nq
    nu
  end
end
