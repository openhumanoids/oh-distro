classdef QuasistaticMotionController < DRCController
  
  properties (SetAccess=protected,GetAccess=protected)
    robot;
    Q; % LQR cost
    R; % LQR cost
  end
  
  methods
  
    function obj = QuasistaticMotionController(name,r,options)
      typecheck(r,'Atlas');

      Q = eye(4);
      R = 0.001*eye(2);
      
      ctrl_data = SharedDataHandle(struct(...
        'A',[zeros(2),eye(2); zeros(2,4)],...
        'B',[zeros(2); eye(2)],...
        'C',[zeros(2),eye(2); zeros(2,4)],...
        'D',[zeros(2); eye(2)],...
        'R',R,...
        'Qy',zeros(4),...
        'S',[],...
        's1',zeros(4,1),...
        'x0',zeros(4,1),...
        'u0',zeros(2,1),...
        'y0',[],...
        'supptraj',[],...
        'qtraj',zeros(getNumDOF(r),1),...
        'V',0,... % cost to go used in controller status message
        'Vdot',0)); % time derivative of cost to go used in controller status message
      
      % instantiate QP controller
      options.slack_limit = 30.0;
      options.w = 1e-1;
      options.R = 1e-50*eye(getNumInputs(r));
      %options.debug = true;
      %options.lcm_foot_contacts = false;

%       input_names = r.getInputFrame.coordinates;
%       ankle_idx = ~cellfun(@isempty,strfind(input_names,'lax')) | ~cellfun(@isempty,strfind(input_names,'uay'));
%       ankle_idx = find(ankle_idx);
%       options.R(ankle_idx,ankle_idx) = 10*options.R(ankle_idx,ankle_idx);
      
      qp = QPController(r,ctrl_data,options);

      % cascade PD qtraj controller 
      pd = SimplePDBlock(r,ctrl_data);
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
      obj.Q=Q;
      obj.R=R;
      
      %obj = addLCMTransition(obj,'WALKING_PLAN',drc.walking_plan_t(),'walking');

      % hijack the walking plan type for now
      obj = addLCMTransition(obj,'QUASISTATIC_ROBOT_PLAN',drc.walking_plan_t(),name); % for standing/reaching tasks

      obj = initialize(obj,struct());
  
    end

        
    function send_status(obj,t_sim,t_ctrl)
      msg = drc.controller_status_t();
      msg.utime = t_sim * 1000000;
      msg.state = msg.QUASISTATIC;
      msg.controller_utime = t_ctrl * 1000000;
      msg.V = obj.controller_data.getField('V');
      msg.Vdot = obj.controller_data.getField('Vdot');
      obj.lc.publish('CONTROLLER_STATUS',msg);
    end
    
    function obj = initialize(obj,data)

      if isfield(data,'AtlasState')
        % transition from walking:
        % take in new nominal pose and compute standing controller
        r = obj.robot;

        x0 = data.AtlasState;
        q0 = x0(1:getNumDOF(r));

        % build TI-ZMP controller 
        foot_pos = contactPositions(r,q0); 
        ch = convhull(foot_pos(1:2,:)'); % assumes foot-only contact model
        comgoal = mean(foot_pos(1:2,ch),2);
        ltisys = LinearSystem([zeros(2),eye(2); zeros(2,4)],[zeros(2); eye(2)],[],[],[],[]);
        [~,V] = tilqr(ltisys,Point(getStateFrame(ltisys),[comgoal;0*comgoal]),Point(getInputFrame(ltisys)),obj.Q,obj.R);

        foot_support=1.0*~cellfun(@isempty,strfind(r.getLinkNames(),'foot'));
        
        obj.controller_data.setField('S',V.S);
        obj.controller_data.setField('s1',zeros(4,1));
        obj.controller_data.setField('qtraj',q0);
        obj.controller_data.setField('x0',[comgoal;0;0]);
        obj.controller_data.setField('supptraj',foot_support);

      elseif isfield(data,'QUASISTATIC_ROBOT_PLAN')
        % execute quasistatic motion
        msg = data.QUASISTATIC_ROBOT_PLAN;
        cdata = WalkingPlanListener.decode(msg);
        
%         % compute TILQR
%         comtraj = cdata.comtraj;
%         comgoal = comtraj.eval(comtraj.tspan(2));
%         ltisys = LinearSystem([zeros(2),eye(2); zeros(2,4)],[zeros(2); eye(2)],[],[],[],[]);
%         [~,V] = tilqr(ltisys,Point(getStateFrame(ltisys),[comgoal;0*comgoal]),Point(getInputFrame(ltisys)),obj.Q,obj.R);
%         
%         % compute TVLQR
%         options.tspan = linspace(comtraj.tspan(1),comtraj.tspan(2),10);
%         options.sqrtmethod = false;
%         x0traj = setOutputFrame([comtraj;0;0],ltisys.getStateFrame);
%         u0traj = setOutputFrame(ConstantTrajectory([0;0]),ltisys.getInputFrame);
%         S = warning('off','Drake:TVLQR:NegativeS');  % i expect to have some zero eigenvalues, which numerically fluctuate below 0
%         warning(S);
%         [~,V] = tvlqr(ltisys,x0traj,u0traj,obj.Q,obj.R,V,options);
        
        obj.controller_data.setField('S',cdata.S);
        obj.controller_data.setField('s1',cdata.s1);
        obj.controller_data.setField('qtraj',cdata.qtraj);
        obj.controller_data.setField('comtraj',cdata.comtraj);
        obj.controller_data.setField('x0',[cdata.comtraj;0;0]); % ??
        obj.controller_data.setField('supptraj',cdata.supptraj);
      else
        % use saved nominal pose 
        d = load('data/atlas_fp.mat');
        q0 = d.xstar(1:getNumDOF(obj.robot));
        r = obj.robot;

        % build TI-ZMP controller 
        foot_pos = contactPositions(r,q0); 
        ch = convhull(foot_pos(1:2,:)'); % assumes foot-only contact model
        comgoal = mean(foot_pos(1:2,ch),2);
        ltisys = LinearSystem([zeros(2),eye(2); zeros(2,4)],[zeros(2); eye(2)],[],[],[],[]);
        [~,V] = tilqr(ltisys,Point(getStateFrame(ltisys),[comgoal;0*comgoal]),Point(getInputFrame(ltisys)),obj.Q,obj.R);

        foot_support=1.0*~cellfun(@isempty,strfind(obj.robot.getLinkNames(),'foot'));

        obj.controller_data.setField('S',V.S);
        obj.controller_data.setField('s1',zeros(4,1));
        obj.controller_data.setField('qtraj',q0);
        obj.controller_data.setField('x0',[comgoal;0;0]);
        obj.controller_data.setField('supptraj',foot_support);
      end
     
      obj = setDuration(obj,inf,false); % set the controller timeout
    end
  end  
end
