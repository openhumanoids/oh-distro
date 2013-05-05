classdef StandingController < DRCController
  
  properties (SetAccess=protected,GetAccess=protected)
    robot;
  end
  
  methods
  
    function obj = StandingController(name,r)
      typecheck(r,'Atlas');

      ctrl_data = SharedDataHandle(struct('A',[zeros(2),eye(2); zeros(2,4)],...
        'B',[zeros(2); eye(2)],'C',[eye(2),zeros(2)],'D',[],...
        'R',zeros(2),'Qy',eye(2),'S',[],'s1',zeros(4,1),'xlimp0',[],...
        'qtraj',[],'supptraj',[]));
      
      % instantiate QP controller
      options.slack_limit = 30.0;
      options.w = 0.1;
      options.lcm_foot_contacts = true;
      options.full_body_opt = false; % if false, doesn't include arms/neck in QP solve (faster)
      nu=getNumInputs(r);
      options.R = 1e-12*eye(nu);
      input_names = r.getInputFrame.coordinates;
      ankle_idx = ~cellfun(@isempty,strfind(input_names,'lax')) | ~cellfun(@isempty,strfind(input_names,'uay'));
      ankle_idx = find(ankle_idx);
      options.R(ankle_idx,ankle_idx) = 10*options.R(ankle_idx,ankle_idx); % soft ankles
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

      % should make this a more specific channel name
      %obj = addLCMTransition(obj,'COMMITTED_ROBOT_PLAN',drc.robot_plan_t(),name); % for standing/reaching tasks
      obj = addLCMTransition(obj,'QUASISTATIC_ROBOT_PLAN',drc.walking_plan_t(),'qs_motion'); % for standing/reaching tasks

      obj = initialize(obj,struct());
  
    end
    
    function obj = initialize(obj,data)

      if isfield(data,'precomp')
        disp('standing controller: using precompute data');
        cdata = data.precomp.resp_data;
        
        obj.controller_data.setField('S',cdata.S);
        obj.controller_data.setField('D',-cdata.h/9.81*eye(2));
        obj.controller_data.setField('qtraj',cdata.q_nom);
        obj.controller_data.setField('xlimp0',cdata.xlimp0);
        obj.controller_data.setField('supptraj',cdata.support);
        
      elseif isfield(data,'AtlasState')
        % transition from walking:
        % take in new nominal pose and compute standing controller
        r = obj.robot;

        x0 = data.AtlasState;
				% get pelvis height above height map
				x0(3) = x0(3)-getTerrainHeight(r,x0(1:2));
        q0 = x0(1:getNumDOF(r));
        kinsol = doKinematics(r,q0);
        com = getCOM(r,kinsol);

        % build TI-ZMP controller 
        foot_pos = contactPositions(r,q0); 
        ch = convhull(foot_pos(1:2,:)'); % assumes foot-only contact model
        comgoal = mean(foot_pos(1:2,ch),2);
        limp = LinearInvertedPendulum(com(3));
        [~,V] = lqr(limp,comgoal);

        foot_support=1.0*~cellfun(@isempty,strfind(r.getLinkNames(),'foot'));
        
        obj.controller_data.setField('S',V.S);
        obj.controller_data.setField('D',-com(3)/9.81*eye(2));
        obj.controller_data.setField('qtraj',q0);
        obj.controller_data.setField('xlimp0',[comgoal;0;0]);
        obj.controller_data.setField('supptraj',foot_support);

      elseif isfield(data,'COMMITTED_ROBOT_PLAN')
        % standing and reaching plan
        msg = data.COMMITTED_ROBOT_PLAN;
        [xtraj,ts] = RobotPlanListener.decodeRobotPlan(msg,true); 
        qtraj = PPTrajectory(spline(ts,xtraj(1:getNumDOF(obj.robot),:)));

        obj.controller_data.setField('qtraj',qtraj);
        obj = setDuration(obj,inf,false); % set the controller timeout

      else
        % use saved nominal pose 
        d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
        q0 = d.xstar(1:getNumDOF(obj.robot));
        kinsol = doKinematics(obj.robot,q0);
        com = getCOM(obj.robot,kinsol);

        % build TI-ZMP controller 
        foot_pos = contactPositions(obj.robot,q0); 
        ch = convhull(foot_pos(1:2,:)'); % assumes foot-only contact model
        comgoal = mean(foot_pos(1:2,ch),2);
        limp = LinearInvertedPendulum(com(3));
        [~,V] = lqr(limp,comgoal);

        foot_support=1.0*~cellfun(@isempty,strfind(obj.robot.getLinkNames(),'foot'));

        obj.controller_data.setField('S',V.S);
        obj.controller_data.setField('D',-com(3)/9.81*eye(2));
        obj.controller_data.setField('qtraj',q0);
        obj.controller_data.setField('xlimp0',[comgoal;0;0]);
        obj.controller_data.setField('supptraj',foot_support);
      end
     
      obj = setDuration(obj,inf,false); % set the controller timeout
    end
  end  
end
