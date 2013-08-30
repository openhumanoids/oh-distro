classdef AtlasManipController < DRCController
  
  properties (SetAccess=protected,GetAccess=protected)
    robot;
  end
  
  methods
  
    function obj = AtlasManipController(name,r,options)
      typecheck(r,'Atlas');

      ctrl_data = SharedDataHandle(struct('qtraj',zeros(getNumDOF(r),1)));
      
      % instantiate position ref publisher
      qref = PositionRefFeedthroughBlock(r);

      % instantiate qtraj eval block
      qt = NeckControlBlock(r,ctrl_data);
      
      ins(1).system = 1;
      ins(1).input = 1;
      ins(2).system = 1;
      ins(2).input = 2;
      outs(1).system = 2;
      outs(1).output = 1;
      sys = mimoCascade(qt,qref,[],ins,outs);
 
      obj = obj@DRCController(name,sys,AtlasState(r));
 
      obj.robot = r;
      obj.controller_data = ctrl_data;
      
      % use saved nominal pose 
      d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
      q0 = d.xstar(1:getNumDOF(obj.robot));
      obj.controller_data.setField('qtraj',q0);
      
      obj = addLCMTransition(obj,'COMMITTED_ROBOT_PLAN',drc.robot_plan_t(),name); % for standing/reaching tasks
    end
    
    function msg = status_message(obj,t_sim,t_ctrl)
        msg = drc.controller_status_t();
        msg.utime = t_sim * 1000000;
        msg.state = msg.STANDING;
        msg.controller_utime = t_ctrl * 1000000;
        msg.V = 0;
        msg.Vdot = 0;
    end
    
    function obj = initialize(obj,data)
      
      if isfield(data,'COMMITTED_ROBOT_PLAN')
        % standing and reaching plan
        try
          msg = data.COMMITTED_ROBOT_PLAN;
          joint_names = obj.robot.getStateFrame.coordinates(1:getNumDOF(obj.robot));
          [xtraj,ts] = RobotPlanListener.decodeRobotPlan(msg,true,joint_names); 
          qtraj = PPTrajectory(spline(ts,xtraj(1:getNumDOF(obj.robot),:)));
          obj.controller_data.setField('qtraj',qtraj);
        catch err
          r = obj.robot;
          x0 = data.AtlasState; % should have an atlas state
          q0 = x0(1:getNumDOF(r));
          obj.controller_data.setField('qtraj',q0);
        end
      end
      obj = setDuration(obj,inf,false); % set the controller timeout
    end
  end  
end
