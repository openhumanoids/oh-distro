classdef StandingController < DRCController
  
  properties (SetAccess=protected,GetAccess=protected)
    robot;
    foot_idx;
    contact_est_monitor;
  end
  
  methods
  
    function obj = StandingController(name,r,options)
      typecheck(r,'Atlas');

      ctrl_data = SharedDataHandle(struct(...
        'A',[zeros(2),eye(2); zeros(2,4)],...
        'B',[zeros(2); eye(2)],...
        'C',[eye(2),zeros(2)],...
        'R',zeros(2),...
        'Qy',eye(2),...
        'is_time_varying',false,...
        'S',zeros(4),... 
        's1',zeros(4,1),...
        's2',0,...
        'x0',zeros(4,1),...
        'u0',zeros(2,1),...
        'y0',zeros(2,1),...
        'support_times',0,...
        'supports',[],...
        'mu',1.0,...
        'qtraj',zeros(getNumDOF(r),1),...
        'V',0,... % cost to go used in controller status message
        'Vdot',0)); % time derivative of cost to go used in controller status message
      
      % instantiate QP controller
      options.slack_limit = 30.0;
      options.w = 0.01;
      options.lcm_foot_contacts = true;
      options.full_body_opt = false; % if false, doesn't include arms/neck in QP solve (faster)
      nu=getNumInputs(r);
      options.R = 1e-12*eye(nu);
      input_names = r.getInputFrame.coordinates;
      ankle_idx = ~cellfun(@isempty,strfind(input_names,'lax')) | ~cellfun(@isempty,strfind(input_names,'uay'));
      ankle_idx = find(ankle_idx);
      options.R(ankle_idx,ankle_idx) = 10*options.R(ankle_idx,ankle_idx); % soft ankles
      if(~isfield(options,'use_mex')) options.use_mex = false; end
      if(~isfield(options,'debug')) options.debug = false; end
      
      qp = QPController(r,ctrl_data,options);

      % cascade PD qtraj controller 
      options.soft_ankles = true;
      pd = SimplePDBlock(r,ctrl_data);
      ins(1).system = 1;
      ins(1).input = 1;
      ins(2).system = 1;
      ins(2).input = 2;
      ins(3).system = 2;
      ins(3).input = 3;
      outs(1).system = 2;
      outs(1).output = 1;
      sys = mimoCascade(pd,qp,[],ins,outs);
      clear connection ins outs;
      
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
      
      obj = obj@DRCController(name,sys);
 
      obj.robot = r;
      obj.controller_data = ctrl_data;
      
      obj.contact_est_monitor = drake.util.MessageMonitor(drc.foot_contact_estimate_t,'utime');
      obj.lc.subscribe('FOOT_CONTACT_ESTIMATE',obj.contact_est_monitor);
      
      % use saved nominal pose 
      d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
      q0 = d.xstar(1:getNumDOF(obj.robot));
      kinsol = doKinematics(obj.robot,q0);
      com = getCOM(obj.robot,kinsol);

      % build TI-ZMP controller 
      foot_pos = contactPositions(obj.robot,kinsol); 
      ch = convhull(foot_pos(1:2,:)'); % assumes foot-only contact model
      comgoal = mean(foot_pos(1:2,ch(1:end-1)),2);
      limp = LinearInvertedPendulum(com(3));
      [~,V] = lqr(limp,comgoal);

      obj.foot_idx = [r.findLinkInd('r_foot'),r.findLinkInd('l_foot')];
      supports = SupportState(r,obj.foot_idx);
      
      obj.controller_data.setField('S',V.S);
      obj.controller_data.setField('D',-com(3)/9.81*eye(2));
      obj.controller_data.setField('qtraj',q0);
      obj.controller_data.setField('x0',[comgoal;0;0]);
      obj.controller_data.setField('y0',comgoal);
      obj.controller_data.setField('supports',supports);
      
      obj = addLCMTransition(obj,'WALKING_PLAN',drc.walking_plan_t(),'walking');
      obj = addLCMTransition(obj,'BRACE_FOR_FALL',drc.utime_t(),'bracing');

      % should make this a more specific channel name
      obj = addLCMTransition(obj,'COMMITTED_ROBOT_PLAN',drc.robot_plan_t(),name); % for standing/reaching tasks
      obj = addLCMTransition(obj,'QUASISTATIC_ROBOT_PLAN',drc.walking_plan_t(),'qs_motion'); % for standing/reaching tasks
    end
    
    function msg = status_message(obj,t_sim,t_ctrl)
        msg = drc.controller_status_t();
        msg.utime = t_sim * 1000000;
        msg.state = msg.STANDING;
        msg.controller_utime = t_ctrl * 1000000;
        msg.V = obj.controller_data.getField('V');
        msg.Vdot = obj.controller_data.getField('Vdot');
    end
    
    function obj = initialize(obj,data)

      
      if isfield(data,'STOP_WALKING')
        % transition from walking:
        % take in new nominal pose and compute standing controller

        % get foot contact state over LCM
        contact_data = obj.contact_est_monitor.getMessage();
        if isempty(contact_data)
          lfoot_contact_state = 1;
          rfoot_contact_state = 1;
        else
          msg = drc.foot_contact_estimate_t(contact_data);
          lfoot_contact_state = msg.left_contact > 0.5;
          rfoot_contact_state = msg.right_contact > 0.5;
        end
        
        r = obj.robot;
        q0 = data.AtlasState(1:getNumDOF(r));
        kinsol = doKinematics(r,q0);
%         com = getCOM(r,kinsol);

        foot_pos = contactPositions(r,kinsol,obj.foot_idx([rfoot_contact_state lfoot_contact_state])); 
        ch = convhull(foot_pos(1:2,:)');
        comgoal = mean(foot_pos(1:2,ch(1:end-1)),2);
%         zmap = getTerrainHeight(r,com(1:2));
%         robot_z = com(3)-zmap;
  
%         obj.controller_data.setField('D',-robot_z/9.81*eye(2));
        obj.controller_data.setField('qtraj',q0);
        obj.controller_data.setField('x0',[comgoal;0;0]);
        obj.controller_data.setField('y0',comgoal);
        
      elseif isfield(data,'COMMITTED_ROBOT_PLAN')
        % standing and reaching plan
        sprintf('standing controller on\n');
        msg = data.COMMITTED_ROBOT_PLAN;
        joint_names = obj.robot.getStateFrame.coordinates(1:getNumDOF(obj.robot));
        [xtraj,ts] = RobotPlanListener.decodeRobotPlan(msg,true,joint_names); 
        qtraj = PPTrajectory(spline(ts,xtraj(1:getNumDOF(obj.robot),:)));

        obj.controller_data.setField('qtraj',qtraj);
        obj = setDuration(obj,inf,false); % set the controller timeout
        
      elseif isfield(data,'AtlasState')
        % transition from walking:
        % take in new nominal pose and compute standing controller
        r = obj.robot;

        x0 = data.AtlasState;
        q0 = x0(1:getNumDOF(r));
        kinsol = doKinematics(r,q0);
%         com = getCOM(r,kinsol);

        foot_pos = contactPositions(r,kinsol,obj.foot_idx); 
        ch = convhull(foot_pos(1:2,:)');
        comgoal = mean(foot_pos(1:2,ch(1:end-1)),2);
%         zmap = getTerrainHeight(r,com(1:2));
%         robot_z = com(3)-zmap;
  
%         obj.controller_data.setField('D',-robot_z/9.81*eye(2));
        obj.controller_data.setField('qtraj',q0);
        obj.controller_data.setField('x0',[comgoal;0;0]);
        obj.controller_data.setField('y0',comgoal);
     
      
      else
        % first initialization should come here... wait for state
        state_frame = getStateFrame(obj.robot);
        state_frame.subscribe(state_frame.channel);
        while true
          [x,~] = getNextMessage(state_frame,10);
          if (~isempty(x))
            data = struct();
            data.AtlasState = x;
            break;
          end
        end
        obj = initialize(obj,data);
      end
     
      QPController.check_ctrl_data(obj.controller_data);  
      obj = setDuration(obj,inf,false); % set the controller timeout
    end
  end  
end
