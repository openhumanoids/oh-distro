classdef HarnessController < DRCController
  
  properties (SetAccess=protected,GetAccess=protected)
    robot;
  end  
    
  methods
    function obj = HarnessController(name,r,timeout)
      typecheck(r,'Atlas');

      ctrl_data = SharedDataHandle(struct('qtraj',[],'ti_flag',true));
      
      % instantiate QP controller
      options = struct();
      options.R = 1e-12*eye(getNumInputs(r));
      qp = HarnessQPController(r,options);

      % cascade PD controller 
      options.Kp=diag([zeros(6,1);100*ones(getNumDOF(r)-6,1)]);
      options.Kd=0.1*options.Kp;
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

      if nargin < 3
        % controller timeout must match the harness time set in VRCPlugin.cpp
        obj = setTimedTransition(obj,5,'standing',true);
      else
        obj = setTimedTransition(obj,timeout,'standing',true);
      end
      
      obj = addLCMTransition(obj,'COMMITTED_ROBOT_PLAN',drc.robot_plan_t(),name);

    end
    
    function obj = initialize(obj,data)
      
      if isfield(data,'COMMITTED_ROBOT_PLAN')
        % pinned reaching plan
        msg = getfield(data,'COMMITTED_ROBOT_PLAN');
        [xtraj,ts] = RobotPlanListener.decodeRobotPlan(msg,true);        
        qtraj = PPTrajectory(spline(ts,xtraj(1:getNumDOF(obj.robot),:)));

        obj.controller_data.setField('ti_flag',false);
        obj = setDuration(obj,inf,false); % set the controller timeout
      else
        % use saved nominal pose
        d = load('data/atlas_fp.mat');
        qtraj = d.xstar(1:getNumDOF(obj.robot));
        obj.controller_data.setField('ti_flag',true);
      end
      
      obj.controller_data.setField('qtraj',qtraj);
  
    end
  end  
end
