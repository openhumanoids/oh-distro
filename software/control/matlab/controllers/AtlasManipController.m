classdef AtlasManipController < DRCController
  
  properties (SetAccess=protected,GetAccess=protected)
    robot;
    arm_joints;
    back_joints;
  end
  
  methods
  
    function obj = AtlasManipController(name,r,options)
      typecheck(r,'Atlas');

      if nargin < 3
        options = struct();
      end
      
      if ~isfield(options,'controller_type')
        options.controller_type = 1;
      end
      
      arm_joints = ~cellfun(@isempty,strfind(r.getStateFrame.coordinates(1:getNumDOF(r)),'arm'));
      back_joints = ~cellfun(@isempty,strfind(r.getStateFrame.coordinates(1:getNumDOF(r)),'back'));
  
      integral_gains = zeros(getNumDOF(r),1);
      if options.controller_type == 1 % use PID control
        integral_gains(arm_joints) = 0.4;
        integral_gains(back_joints) = 0.0;
      end
      
      ctrl_data = SharedDataHandle(struct('qtraj',zeros(getNumDOF(r),1),...
                        'qddtraj',zeros(getNumDOF(r),1),...
                        'integral',zeros(getNumDOF(r),1),...
                        'integral_gains',integral_gains)); 

      options.use_error_integrator = true;
      qt = QTrajEvalBlock(r,ctrl_data,options);

      if options.controller_type == 1 % PID control
        
        % instantiate position ref publisher
        qref = PositionRefFeedthroughBlock(r);
        
        ins(1).system = 1;
        ins(1).input = 1;
        outs(1).system = 2;
        outs(1).output = 1;
        sys = mimoCascade(qt,qref,[],ins,outs);

      elseif options.controller_type == 2 % PID control w/pelvis adjustment
        
        % instantiate position ref publisher
        qref = PositionRefFeedthroughBlock(r);
        manip_cmd = BDIManipCommandBlock(r,ctrl_data);
        
        ins(1).system = 1;
        ins(1).input = 1;
        outs(1).system = 2;
        outs(1).output = 1;
        sys = mimoCascade(qt,manip_cmd,[],ins,outs);
        sys = mimoCascade(sys,qref,[],ins,outs);

      elseif options.controller_type == 3 % use PD + gravity compensation

        % cascade gravity compensation block
        gc = GravityCompensationBlock(r);
        ins(1).system = 1;
        ins(1).input = 1;
        outs(1).system = 1;
        outs(1).output = 1;
        outs(2).system = 2;
        outs(2).output = 1;
        conn(1).from_output = 2;
        conn(1).to_input = 1;        
        sys = mimoCascade(qt,gc,conn,ins,outs);
        clear ins outs conn;
        
        % cascade pos/torque ref feedthrough block
        options.open_loop_torque = false;
        q_tau_ref = PosTorqueRefFeedthroughBlock(r,options);
        ins(1).system = 1;
        ins(1).input = 1;
        outs(1).system = 2;
        outs(1).output = 1;
        sys = mimoCascade(sys,q_tau_ref,[],ins,outs);
        clear ins outs;
        
      elseif options.controller_type == 4 % use inverse dynamics

        % cascade eval block with signal duplicator
        dupl = SignalDuplicator(AtlasCoordinates(r),2);
        ins(1).system = 1;
        ins(1).input = 1;
        outs(1).system = 2;
        outs(1).output = 1;
        outs(2).system = 2;
        outs(2).output = 2;
        outs(3).system = 1;
        outs(3).output = 2;
        sys = mimoCascade(qt,dupl,[],ins,outs);
        clear ins outs;
        
        % cascade PD block
        options.Kp = 19.0*eye(getNumDOF(r));
        options.Kd =  5.0*eye(getNumDOF(r));
        options.use_qddtraj = true;
        options.use_kalman_est = true;
        pd = SimplePDBlock(r,ctrl_data,options);
        ins(1).system = 1;
        ins(1).input = 1;
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
        ins(2).system = 2;
        ins(2).input = 2;      
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
        outs(1).system = 2;
        outs(1).output = 1;
        sys = mimoCascade(sys,q_tau_ref,[],ins,outs);
        clear ins outs;
        
      end
      
      obj = obj@DRCController(name,sys,AtlasState(r));
 
      obj.robot = r;
      obj.controller_data = ctrl_data;
      obj.arm_joints = arm_joints;
      obj.back_joints = back_joints;
      
      % use saved nominal pose 
      d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
      q0 = d.xstar(1:getNumDOF(obj.robot));
      obj.controller_data.setField('qtraj',q0(7:end));
      obj.controller_data.setField('qddtraj',ConstantTrajectory(zeros(getNumDOF(r),1)));
      
      obj = addLCMTransition(obj,'COMMITTED_ROBOT_PLAN',drc.robot_plan_t(),name); % for standing/reaching tasks
      obj = addLCMTransition(obj,'COMMITTED_PLAN_PAUSE',drc.plan_control_t(),'init'); % stop plan execution
      obj = addLCMTransition(obj,'ATLAS_BEHAVIOR_COMMAND',drc.atlas_behavior_command_t(),'init'); 
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
          [xtraj,ts] = RobotPlanListener.decodeRobotPlan(msg,obj.robot.floating,joint_names); 
          % TODO: REMOVE THIS ********************************************
          % try using the desired position of the last plan as the first
          % point in the plan as using the current position causes a "jump"
          % in robot position due to steady state error of the controller.
          % this really shouldn't happen here, the planner should be
          % sending exactly what we want to be executed.
          qtraj_prev = obj.controller_data.data.qtraj;
          q0=xtraj(1:getNumDOF(obj.robot),1);
          
          torso = (obj.arm_joints | obj.back_joints);
          if isa(qtraj_prev,'PPTrajectory') 
            % smooth transition from end of previous trajectory
            qprev_end = fasteval(qtraj_prev,qtraj_prev.tspan(end));
            if obj.robot.floating
              % always use current desired body pose
              qprev_end(1:6) = q0(1:6);
            end
            if max(abs(q0(torso)-qprev_end(torso))) < 0.15
              qtraj = PPTrajectory(spline(ts,[qprev_end xtraj(1:getNumDOF(obj.robot),2:end)]));
            else
              qtraj = PPTrajectory(spline(ts,xtraj(1:getNumDOF(obj.robot),:)));
            end
          else
            % first plan
            qtraj = PPTrajectory(spline(ts,xtraj(1:getNumDOF(obj.robot),:)));
          end          
          obj.controller_data.setField('qtraj',qtraj);
          obj.controller_data.setField('qddtraj',fnder(qtraj,2));
          %obj.controller_data.setField('integral',zeros(getNumDOF(obj.robot),1));
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
