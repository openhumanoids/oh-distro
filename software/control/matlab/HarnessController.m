classdef HarnessController < DRCController
  
  properties (SetAccess=protected,GetAccess=protected)
    robot;
    floating;
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
      if getNumDOF(r)==34 % floating model
        options.Kp=diag([zeros(6,1); 200*ones(getNumDOF(r)-6,1)]);
        float = true;
      else
        options.Kp=diag(200*ones(getNumDOF(r),1));
        float = false;
      end
      
      options.Kd=0.12*options.Kp;
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
      obj.floating = float;
      
      if nargin < 3
        % controller timeout must match the harness time set in VRCPlugin.cpp
        obj = setTimedTransition(obj,5,'standing',true);
      else
        obj = setTimedTransition(obj,timeout,'standing',true);
      end
      
      obj = addLCMTransition(obj,'COMMITTED_ROBOT_PLAN',drc.robot_plan_t(),name);
      obj = addPrecomputeResponseHandler(obj,'STANDING_PREC_RESPONSE','standing');
      
    end
    
    function obj = initialize(obj,data)

      if isfield(data,'COMMITTED_ROBOT_PLAN')
        % pinned reaching plan
        msg = getfield(data,'COMMITTED_ROBOT_PLAN');
        [xtraj,ts] = RobotPlanListener.decodeRobotPlan(msg,true);        
        if obj.floating
          qtraj = PPTrajectory(spline(ts,xtraj(1:getNumDOF(obj.robot),:)));
        else
          qtraj = PPTrajectory(spline(ts,xtraj(6+(1:getNumDOF(obj.robot)),:)));
        end
        obj.controller_data.setField('ti_flag',false);
        obj = setDuration(obj,inf,false); % set the controller timeout
      else
        % use saved nominal pose
        d = load('data/atlas_fp.mat');
        if ~obj.floating
          q_nom = d.xstar(6+(1:getNumDOF(obj.robot)));
        else
          q_nom = d.xstar(1:getNumDOF(obj.robot));
        end
        q0 = zeros(getNumDOF(obj.robot),1);
        qtraj = PPTrajectory(spline([0 2],[q0 q_nom]));
        
        if ~isinf(getDuration(obj))
          q_nom = d.xstar(1:getNumDOF(obj.robot));
          disp('publishing precompute request');
          % send standing precomputation request
          req_msg = drc.precompute_request_t();
          req_msg.utime = 0;
          req_msg.robot_name = 'atlas';
          req_msg.response_channel = 'STANDING_PREC_RESPONSE';
          req_msg.precompute_type = 0;

          save('prec_r.mat','q_nom');
          fid = fopen('prec_r.mat','r');
          req_msg.matdata = fread(fid,inf,'*uint8');
          fclose(fid);
          req_msg.n_bytes = length(req_msg.matdata);
          lc = lcm.lcm.LCM.getSingleton();
          lc.publish('STANDING_PREC_REQUEST', req_msg);
        end
        
        obj.controller_data.setField('ti_flag',true);
      end
      
      obj.controller_data.setField('qtraj',qtraj);
  
    end
  end  
end
