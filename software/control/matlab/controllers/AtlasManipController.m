classdef AtlasManipController < DRCController
  
  properties (SetAccess=protected,GetAccess=protected)
    robot;
  end
  
  methods
  
    function obj = AtlasManipController(name,r,options)
      typecheck(r,'Atlas');

      ctrl_data = SharedDataHandle(struct('qtraj',zeros(getNumDOF(r),1),...
                            'qddtraj',zeros(getNumDOF(r),1)));

      qt = NeckControlBlock(r,ctrl_data);
      
      if 0 % use PD control
        
        % instantiate position ref publisher
        qref = PositionRefFeedthroughBlock(r);
        
        ins(1).system = 1;
        ins(1).input = 1;
        ins(2).system = 1;
        ins(2).input = 2;
        outs(1).system = 2;
        outs(1).output = 1;
        sys = mimoCascade(qt,qref,[],ins,outs);
     
      else % use inverse dynamics

        % cascade eval block with signal duplicator
        dupl = SignalDuplicator(AtlasCoordinates(r),2);
        ins(1).system = 1;
        ins(1).input = 1;
        ins(2).system = 1;
        ins(2).input = 2;
        outs(1).system = 2;
        outs(1).output = 1;
        outs(2).system = 2;
        outs(2).output = 2;
        outs(3).system = 1;
        outs(3).output = 2;
        sys = mimoCascade(qt,dupl,[],ins,outs);
        clear ins outs;
        
        % cascade PD block
        options.Kp = 20.0*eye(getNumDOF(r));
        options.Kd =  5.0*eye(getNumDOF(r));
        options.use_qddtraj = true;
        pd = SimplePDBlock(r,ctrl_data,options);
        ins(1).system = 1;
        ins(1).input = 1;
        ins(2).system = 1;
        ins(2).input = 2;      
        outs(1).system = 1;
        outs(1).output = 1;
        outs(2).system = 2;
        outs(2).output = 1;
        conn(1).from_output = 2;
        conn(1).to_input = 1;
        conn(2).from_output = 3;
        conn(2).to_input = 2;
        sys = mimoCascade(sys,pd,conn,ins,outs);
        clear ins outs conn;
        
        % cascade inverse dynamics block
        invdyn = InverseDynamicsBlock(r);
        ins(1).system = 1;
        ins(1).input = 1;
        ins(2).system = 1;
        ins(2).input = 2;      
        ins(3).system = 2;
        ins(3).input = 2;      
        outs(1).system = 1;
        outs(1).output = 1;
        outs(2).system = 2;
        outs(2).output = 1;
        conn(1).from_output = 2;
        conn(1).to_input = 1;        
        sys = mimoCascade(sys,invdyn,conn,ins,outs);
        clear ins outs conn;
        
        % cascade pos/torque ref feedthrough block
        q_tau_ref = PosTorqueRefFeedthroughBlock(r);
        ins(1).system = 1;
        ins(1).input = 1;
        ins(2).system = 1;
        ins(2).input = 2;      
        ins(3).system = 1;
        ins(3).input = 3;      
        outs(1).system = 2;
        outs(1).output = 1;
        sys = mimoCascade(sys,q_tau_ref,[],ins,outs);
        clear ins outs;
        
      end
      
      obj = obj@DRCController(name,sys,AtlasState(r));
 
      obj.robot = r;
      obj.controller_data = ctrl_data;
      
      % use saved nominal pose 
      d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
      q0 = d.xstar(1:getNumDOF(obj.robot));
      obj.controller_data.setField('qtraj',q0(7:end));
      obj.controller_data.setField('qddtraj',ConstantTrajectory(zeros(getNumDOF(r),1)));
      
      obj = addLCMTransition(obj,'COMMITTED_ROBOT_PLAN',drc.robot_plan_t(),name); % for standing/reaching tasks
      obj = addLCMTransition(obj,'ATLAS_BEHAVIOR_COMMAND',drc.atlas_behavior_command_t(),'init'); % for standing/reaching tasks
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
          [xtraj,ts] = RobotPlanListener.decodeRobotPlan(msg,false,joint_names); 
          qtraj = PPTrajectory(spline(ts,xtraj(1:getNumDOF(obj.robot),:)));
          obj.controller_data.setField('qtraj',qtraj);
          obj.controller_data.setField('qddtraj',fnder(qtraj,2));
        catch err
          r = obj.robot;
          x0 = data.AtlasState; % should have an atlas state
          q0 = x0(1:getNumDOF(r));
          obj.controller_data.setField('qtraj',q0(7:end));
          obj.controller_data.setField('qddtraj',ConstantTrajectory(zeros(getNumDOF(r),1)));
        end
      end
      obj = setDuration(obj,inf,false); % set the controller timeout
    end
  end  
end
