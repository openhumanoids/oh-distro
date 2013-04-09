classdef HarnessController < DRCController
  
  properties (SetAccess=protected,GetAccess=protected)
    robot;
  end  
    
  methods
    function obj = HarnessController(name,r)
      typecheck(r,'Atlas');

      ctrl_data = SharedDataHandle(struct('qtraj',[],'ti_flag',true));
      
      % instantiate QP controller
      options = struct();
      options.R = 1e-12*eye(getNumInputs(r));
      qp = HarnessQPController(r,options);

      % cascade PD controller 
      options.Kp=diag([zeros(6,1);100*ones(getNumDOF(r)-6,1)]);
      options.Kd=0.2*options.Kp;
      pd = SimplePDController(r,ctrl_data,options);
      ins(1).system = 1;
      ins(1).input = 1;
      ins(2).system = 2;
      ins(2).input = 2;
      outs(1).system = 2;
      outs(1).output = 1;
      sys = mimoCascade(pd,qp,[],ins,outs);

      obj = obj@DRCController(name,sys);

      obj.robot = r;
      obj.controller_data = ctrl_data;

      % controller timeout must match the harness time set in VRCPlugin.cpp
      obj = setTimedTransition(obj,5,'standing',true); 
    end
    
    function obj = initialize(obj,msg_data)
      r = obj.robot;
      nq = getNumDOF(r);
      
      % use saved nominal pose --- could make this more general
      d = load('data/atlas_fp.mat');
      xstar = d.xstar;
      
      q0 = xstar(1:nq);
      obj.controller_data.setField('qtraj',q0);
  
    end
  end  
end
