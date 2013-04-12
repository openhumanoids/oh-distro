classdef StandingController < DRCController
  
  properties (SetAccess=protected,GetAccess=protected)
    robot;
  end
  
  methods
  
    function obj = StandingController(name,r)
      typecheck(r,'Atlas');

      ctrl_data = SharedDataHandle(struct('S',[],'h',[],'hddot',[],'qtraj',[],'supptraj',[],'ti_flag',true));
      
      % instantiate QP controller
      options.slack_limit = 40.0;
      options.w = 1.0;
      options.R = 1e-12*eye(getNumInputs(r));
      qp = QPController(r,ctrl_data,options);

      % cascade PD qtraj controller 
      pd = SimplePDController(r,ctrl_data);
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
      
      obj = addLCMTransition(obj,'COMMITTED_WALKING_PLAN',drc.walking_plan_t(),'walking');
      obj = addLCMTransition(obj,'COMMITTED_ROBOT_PLAN',drc.robot_plan_t(),name); % for standing/reaching tasks

      obj = initialize(obj,struct());
  
    end
    
    function obj = initialize(obj,data)

      if isfield(data,'AtlasState')
        % transition from walking:
        % take in new nominal pose and compute standing controller
        r = obj.robot;

        x0 = data.AtlasState;
        q0 = x0(1:getNumDOF(r));
        kinsol = doKinematics(r,q0);
        com = getCOM(r,kinsol);

        % build TI-ZMP controller 
        foot_pos = contactPositions(r,q0); 
        ch = convhull(foot_pos(1:2,:)'); % assumes foot-only contact model
        comgoal = [mean(foot_pos(1:2,ch),2);com(3)];
        limp = LinearInvertedPendulum(com(3));
        options.Qy = diag([0 0 0 0 1 1]);
        [~,V] = tilqr(limp,Point(limp.getStateFrame,[comgoal(1:2);0;0]), ...
                      Point(limp.getInputFrame),zeros(4),zeros(2),options);

        foot_support=1.0*~cellfun(@isempty,strfind(r.getLinkNames(),'foot'));

        obj.controller_data.setField('S',PPTrV.S);
        obj.controller_data.setField('h',com(3));
        obj.controller_data.setField('hddot',0);
        obj.controller_data.setField('qtraj',q0);
        obj.controller_data.setField('supptraj',foot_support);
        obj.controller_data.setField('ti_flag',true);

      elseif isfield(data,'COMMITTED_ROBOT_PLAN')
        % standing and reaching plan
        msg = data.COMMITTED_ROBOT_PLAN;
        [xtraj,ts] = RobotPlanListener.decodeRobotPlan(msg,true); 
        ts
        %%% TMP HACK %%%
        xtraj(3,:) = xtraj(3,:) - 1; 
        %%% TMP HACK %%%
        qtraj = PPTrajectory(spline(ts,xtraj(1:getNumDOF(obj.robot),:)));

        obj.controller_data.setField('qtraj',qtraj);
        obj.controller_data.setField('ti_flag',false);
        obj = setDuration(obj,inf,false); % set the controller timeout

      else
        % use saved nominal pose 
        d = load('data/atlas_fp.mat');
        q0 = d.xstar(1:getNumDOF(obj.robot));
        kinsol = doKinematics(obj.robot,q0);
        com = getCOM(obj.robot,kinsol);

        % build TI-ZMP controller 
        foot_pos = contactPositions(obj.robot,q0); 
        ch = convhull(foot_pos(1:2,:)'); % assumes foot-only contact model
        comgoal = [mean(foot_pos(1:2,ch),2);com(3)];
        limp = LinearInvertedPendulum(com(3));
        options.Qy = diag([0 0 0 0 1 1]);
        [~,V] = tilqr(limp,Point(limp.getStateFrame,[comgoal(1:2);0;0]), ...
                      Point(limp.getInputFrame),zeros(4),zeros(2),options);

        foot_support=1.0*~cellfun(@isempty,strfind(obj.robot.getLinkNames(),'foot'));

        obj.controller_data.setField('S',V.S);
        obj.controller_data.setField('h',com(3));
        obj.controller_data.setField('hddot',0);
        obj.controller_data.setField('qtraj',q0);
        obj.controller_data.setField('supptraj',foot_support);
        obj.controller_data.setField('ti_flag',true);
      end
     
      obj = setDuration(obj,inf,false); % set the controller timeout
    end
  end  
end
