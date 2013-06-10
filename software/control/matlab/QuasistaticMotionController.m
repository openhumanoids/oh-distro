classdef QuasistaticMotionController < DRCController
  
  properties (SetAccess=protected,GetAccess=protected)
    robot;
    Q; % LQR cost
    R; 
    foot_idx;
    ltisys;
  end
  
  methods
  
    function obj = QuasistaticMotionController(name,r,options)
      typecheck(r,'Atlas');
      
      A = [zeros(2),eye(2); zeros(2,4)];
      B = [zeros(2); eye(2)];
      Q = eye(4);
      R = 0.001*eye(2);

      ctrl_data = SharedDataHandle(struct(...
        'A',A,...
        'B',B,...
        'C',zeros(4),...
        'D',zeros(4,2),...
        'Qy',zeros(4),...
        'R',R,...
        'is_time_varying',false,...
        'S',zeros(4),...
        's1',zeros(4,1),...
        's2',0,...
        'x0',zeros(4,1),...
        'u0',zeros(2,1),...
        'y0',zeros(4,1),...
        'support_times',0,...
        'supports',[],...
        'mu',1.0,...
        'qtraj',zeros(getNumDOF(r),1),...
        'V',0,... % cost to go used in controller status message
        'Vdot',0)); % time derivative of cost to go used in controller status message
      
      % instantiate QP controller
      options.slack_limit = 30.0;
      options.w = 0.01;
      options.R = 1e-50*eye(getNumInputs(r));
      if(~isfield(options,'use_mex')) options.use_mex = false; end
      if(~isfield(options,'debug')) options.debug = false; end

      options.lcm_foot_contacts = true;
      qp = QPController(r,ctrl_data,options);

      % cascade PD qtraj controller 
      pd = SimplePDBlock(r,ctrl_data);
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
      clear ins outs;
      
      obj = obj@DRCController(name,sys,AtlasState(r));
 
      obj.robot = r;
      obj.controller_data = ctrl_data;
      obj.Q=Q;
      obj.R=R;
      
      % use saved nominal standing pose 
      d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
      q0 = d.xstar(1:getNumDOF(obj.robot));
      kinsol = doKinematics(obj.robot,q0);

      % build TI-LQR controller 
      foot_pos = contactPositions(obj.robot,kinsol); 
      ch = convhull(foot_pos(1:2,:)'); % assumes foot-only contact model
      comgoal = mean(foot_pos(1:2,ch(1:end-1)),2);
      obj.ltisys = LinearSystem(A,B,[],[],[],[]);
      [~,V] = tilqr(obj.ltisys,Point(getStateFrame(obj.ltisys),[comgoal;0*comgoal]),Point(getInputFrame(obj.ltisys)),obj.Q,obj.R);

      obj.foot_idx=find(~cellfun(@isempty,strfind(obj.robot.getLinkNames(),'foot')));
      supports = SupportState(r,obj.foot_idx);
      
      obj.controller_data.setField('S',V.S);
      obj.controller_data.setField('qtraj',q0);
      obj.controller_data.setField('x0',[comgoal;0;0]);
      obj.controller_data.setField('supports',supports);
      
      obj = addLCMTransition(obj,'WALKING_PLAN',drc.walking_plan_t(),'walking');
      obj = addLCMTransition(obj,'BRACE_FOR_FALL',drc.utime_t(),'bracing');
      % hijack the walking plan type for now
      obj = addLCMTransition(obj,'QUASISTATIC_ROBOT_PLAN',drc.walking_plan_t(),name); % for standing/reaching tasks
  
    end

    function msg = status_message(obj,t_sim,t_ctrl)
      msg = drc.controller_status_t();
      msg.utime = t_sim * 1000000;
      msg.state = msg.QUASISTATIC;
      msg.controller_utime = t_ctrl * 1000000;
      msg.V = obj.controller_data.getField('V');
      msg.Vdot = obj.controller_data.getField('Vdot');
    end
    
    function obj = initialize(obj,data)

      if isfield(data,'QUASISTATIC_ROBOT_PLAN')
        % execute quasistatic motion
        msg = data.QUASISTATIC_ROBOT_PLAN;
        cdata = WalkingPlanListener.decode(msg);
        
        obj.controller_data.setField('S',cdata.S);
        obj.controller_data.setField('s1',cdata.s1);
        obj.controller_data.setField('s2',0);
        obj.controller_data.setField('qtraj',cdata.qtraj);
        obj.controller_data.setField('comtraj',cdata.comtraj);
        obj.controller_data.setField('supports',cdata.supports);
        obj.controller_data.setField('support_times',cdata.support_times);
      elseif isfield(data,'AtlasState')
        % take in new nominal pose and compute quasistatic standing
        % controller
        r = obj.robot;

        x0 = data.AtlasState;
        q0 = x0(1:getNumDOF(r));
        kinsol = doKinematics(r,q0);
        
        % build TI-LQR controller (this could be done analytically) 
        foot_pos = contactPositions(r,kinsol,obj.foot_idx); 
        ch = convhull(foot_pos(1:2,:)'); 
        comgoal = mean(foot_pos(1:2,ch(1:end-1)),2);
        [~,V] = tilqr(obj.ltisys,Point(getStateFrame(obj.ltisys),[comgoal;0*comgoal]),Point(getInputFrame(obj.ltisys)),obj.Q,obj.R);

        supports = SupportState(r,obj.foot_idx);
        
        obj.controller_data.setField('S',V.S);
        obj.controller_data.setField('s1',zeros(4,1));
        obj.controller_data.setField('s2',0);
        obj.controller_data.setField('qtraj',q0);
        obj.controller_data.setField('x0',[comgoal;0;0]);
        obj.controller_data.setField('supports',supports);
        obj.controller_data.setField('support_times',0);
        
      else
        % ...wait for state (hopefully we don't enter here)
        warning('QuasistaticMotionController:initialize: waiting for state');
        state_frame = getStateFrame(obj.robot);
        state_frame.subscribe('EST_ROBOT_STATE');
        while true
          [x,~] = getNextMessage(state_frame,5);
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
