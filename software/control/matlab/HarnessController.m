classdef HarnessController < DRCController
    
  properties (SetAccess=protected,GetAccess=protected)
    robot;
    floating;
  end
    
  methods
    function obj = HarnessController(name,r,timeout)
      typecheck(r,'Atlas');
            
      ctrl_data = SharedDataHandle(struct('qtraj',zeros(getNumDOF(r),1)));
            
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
      % cascade PD qtraj controller 
      pd = SimplePDBlock(r,ctrl_data,options);
      ins(1).system = 1;
      ins(1).input = 1;
      ins(2).system = 1;
      ins(2).input = 2;
      ins(3).system = 2;
      ins(3).input = 2;
      outs(1).system = 2;
      outs(1).output = 1;
      sys = mimoCascade(pd,qp,[],ins,outs);
      clear ins outs;

      if strcmp(name,'seated_driving')
        % cascade qtraj eval block
        qt = QTrajEvalBlock(r,ctrl_data);
        ins(1).system = 1;
        ins(1).input = 1;
        ins(2).system = 2;
        ins(2).input = 3;
        outs(1).system = 2;
        outs(1).output = 1;
        connection(1).from_output = 1;
        connection(1).to_input = 1;
        connection(2).from_output = 2;
        connection(2).to_input = 2;
        sys = mimoCascade(qt,sys,connection,ins,outs);
      else
        % cascade neck pitch control block
        neck = NeckControlBlock(r,ctrl_data);
        ins(1).system = 1;
        ins(1).input = 1;
        ins(2).system = 1;
        ins(2).input = 2;
        ins(3).system = 2;
        ins(3).input = 3;
        outs(1).system = 2;
        outs(1).output = 1;
        connection(1).from_output = 1;
        connection(1).to_input = 1;
        connection(2).from_output = 2;
        connection(2).to_input = 2;
        sys = mimoCascade(neck,sys,connection,ins,outs);
      end

      obj = obj@DRCController(name,sys);

      obj.robot = r;
      obj.controller_data = ctrl_data;
      obj.floating = float;

      if nargin < 3
        if((strcmp(name,'harnessed_notwalking'))||(strcmp(name,'seated_driving')))
          % controller timeout must match the harness time set in mit_not_walking.launch
          obj = setTimedTransition(obj,inf,'standing',true);
        else
          % controller timeout must match the harness time set in mit.launch
          obj = setTimedTransition(obj,10,'standing',true);
        end
      else
        obj = setTimedTransition(obj,timeout,'standing',true);
      end

      obj = addLCMTransition(obj,'COMMITTED_ROBOT_PLAN',drc.robot_plan_t(),name);

    end

    function send_status(obj,t_sim,t_ctrl)
      msg = drc.controller_status_t();
      msg.utime = t_sim * 1000000;
      msg.state = msg.HARNESSED;
      msg.controller_utime = t_ctrl * 1000000;
      msg.V = 0;
      msg.Vdot = 0;
      obj.lc.publish('CONTROLLER_STATUS',msg);
    end        
        
    function obj = initialize(obj,data)

      if isfield(data,'COMMITTED_ROBOT_PLAN')
        % pinned reaching plan
        msg = data.COMMITTED_ROBOT_PLAN;
        joint_names = obj.robot.getStateFrame.coordinates(1:getNumDOF(obj.robot));
        [xtraj,ts] = RobotPlanListener.decodeRobotPlan(msg,true,joint_names);
        if obj.floating
          qtraj = PPTrajectory(spline(ts,xtraj(1:getNumDOF(obj.robot),:)));
        else
          qtraj = PPTrajectory(spline(ts,xtraj(6+(1:getNumDOF(obj.robot)),:)));
        end
        obj = setDuration(obj,inf,false); % set the controller timeout
      else
        % use saved nominal pose
        if(strcmp(obj.name,'seated_driving'))
          d =load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_seated_pose.mat'));%hands down
          %d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/aa_atlas_seated.mat'));%hands up
        else
          d = load('data/atlas_fp.mat');
        end

        if ~obj.floating
          q_nom = d.xstar(6+(1:getNumDOF(obj.robot)));
        else
          q_nom = d.xstar(1:getNumDOF(obj.robot));
        end
        q0 = zeros(getNumDOF(obj.robot),1);
        if((strcmp(obj.name,'harnessed_notwalking'))||(strcmp(obj.name,'seated_driving')))
          qtraj = PPTrajectory(spline([0 4],[q0 q_nom]));
        else
          qtraj = PPTrajectory(spline([5 9],[q0 q_nom]));
        end
      end

      obj.controller_data.setField('qtraj',qtraj);

    end
end
    
end
