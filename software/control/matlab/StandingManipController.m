classdef StandingManipController < DRCController
  
  properties (SetAccess=protected,GetAccess=protected)
    robot;
  end
  
  methods
  
    function obj = StandingManipController(name,r,options)
      typecheck(r,'Atlas');
      
      ctrl_data = SharedDataHandle(struct(...
        'A',[zeros(2),eye(2); zeros(2,4)],...
        'B',[zeros(2); eye(2)],...
        'C',[eye(2),zeros(2)],...
        'D',[],...
        'R',zeros(2),...
        'Qy',eye(2),...
        'S',[],...
        's1',zeros(4,1),...
        'x0',zeros(4,1),...
        'u0',zeros(2,1),...
        'y0',zeros(2,1),...
        'supptraj',[],...
        'qtraj',zeros(getNumDOF(r),1),...
        'V',0,... % cost to go used in controller status message
        'Vdot',0,...
        'ee_link_ind',[]));
      
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

      qp = QPController(r,ctrl_data,options);

      % cascade PD qtraj controller 
      if(~isfield(options,'control'))
        options.control = 'SimplePD';
      end
      if(strcmp(options.control,'SimplePD'))
        pd = SimplePDBlock(r,ctrl_data);
      elseif(strcmp(options.control,'NaivePD'))
        pd = NaivePDController(r,ctrl_data);
      elseif(strcmp(options.control,'Impedance'))
        if(isfield(options,'impedance'))
          pd = ImpedanceController(r,ctrl_data,options.impedance);
        else
          pd = ImpedanceController(r,ctrl_data);
        end
      end
      ins(1).system = 1;
      ins(1).input = 1;
      ins(2).system = 1;
      ins(2).input = 2;
      ins(3).system = 2;
      ins(3).input = 2;
      outs(1).system = 2;
      outs(1).output = 1;
      sys = mimoCascade(pd,qp,[],ins,outs);
      
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
      lhand_ind = obj.robot.findLinkInd('l_hand+l_hand_point_mass');
      rhand_ind = obj.robot.findLinkInd('r_hand+r_hand_point_mass');
      ctrl_data.setField('ee_link_ind',[lhand_ind rhand_ind])
      obj.controller_data = ctrl_data;
      
      obj = addLCMTransition(obj,'WALKING_PLAN',drc.walking_plan_t(),'walking');

      % should make this a more specific channel name
      obj = addLCMTransition(obj,'COMMITTED_ROBOT_PLAN',drc.robot_plan_t(),name); % for standing/reaching tasks
%       obj = addLCMTransition(obj,'QUASISTATIC_ROBOT_PLAN',drc.walking_plan_t(),'qs_motion'); % for standing/reaching tasks

      obj = initialize(obj,struct());
  
    end
    
    function send_status(obj,t_sim,t_ctrl)
        msg = drc.controller_status_t();
        msg.utime = t_sim * 1000000;
        msg.state = msg.STANDING;
        msg.controller_utime = t_ctrl * 1000000;
        msg.V = obj.controller_data.getField('V');
        msg.Vdot = obj.controller_data.getField('Vdot');
        obj.lc.publish('CONTROLLER_STATUS',msg);
    end
    
    function obj = initialize(obj,data)

      if isfield(data,'precomp')
        disp('standing controller: using precompute data');
        cdata = data.precomp.resp_data;
        
        obj.controller_data.setField('S',cdata.S);
        obj.controller_data.setField('D',-cdata.h/9.81*eye(2));
        obj.controller_data.setField('qtraj',cdata.q_nom);
        obj.controller_data.setField('x0',cdata.x0);
        obj.controller_data.setField('y0',cdata.y0);
        obj.controller_data.setField('supptraj',cdata.support);
        
      elseif isfield(data,'AtlasState')
        % transition from walking:
        % take in new nominal pose and compute standing controller
        r = obj.robot;

        x0 = data.AtlasState;
        q0 = x0(1:getNumDOF(r));
        kinsol = doKinematics(r,q0);
        com = getCOM(r,kinsol);

        % build TI-ZMP controller 
        foot_pos = contactPositions(r,kinsol); 
        ch = convhull(foot_pos(1:2,:)'); % assumes foot-only contact model
        comgoal = mean(foot_pos(1:2,ch),2);
        zmap = getTerrainHeight(r,com(1:2));
        robot_z = com(3)-zmap;
        limp = LinearInvertedPendulum(robot_z);
        [~,V] = lqr(limp,comgoal);

        foot_support=1.0*~cellfun(@isempty,strfind(r.getLinkNames(),'foot'));
        
        obj.controller_data.setField('S',V.S);
        obj.controller_data.setField('D',-robot_z/9.81*eye(2));
        obj.controller_data.setField('qtraj',q0);
        obj.controller_data.setField('x0',[comgoal;0;0]);
        obj.controller_data.setField('y0',comgoal);
        obj.controller_data.setField('supptraj',foot_support);

      elseif isfield(data,'COMMITTED_ROBOT_PLAN')
        % standing and reaching plan
        sprintf('standing controller on\n');
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
        foot_pos = contactPositions(obj.robot,kinsol); 
        ch = convhull(foot_pos(1:2,:)'); % assumes foot-only contact model
        comgoal = mean(foot_pos(1:2,ch),2);
        limp = LinearInvertedPendulum(com(3));
        [~,V] = lqr(limp,comgoal);

        foot_support=1.0*~cellfun(@isempty,strfind(obj.robot.getLinkNames(),'foot'));

        obj.controller_data.setField('S',V.S);
        obj.controller_data.setField('D',-com(3)/9.81*eye(2));
        obj.controller_data.setField('qtraj',q0);
        obj.controller_data.setField('x0',[comgoal;0;0]);
        obj.controller_data.setField('y0',comgoal);
        obj.controller_data.setField('supptraj',foot_support);
%         obj.controller_data.setField('qnom',q0);
      end
     
      obj = setDuration(obj,inf,false); % set the controller timeout
    end
  end  
end
