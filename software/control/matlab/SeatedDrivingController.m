classdef SeatedDrivingController < DRCController
    
  properties (SetAccess=protected,GetAccess=protected)
    robot;
    foot_idx;
    pelvis_idx;
  end
  
  methods
    function obj = SeatedDrivingController(name,r,options)
      typecheck(r,'Atlas');
      
      ctrl_data = SharedDataHandle(struct(...
        'A',zeros(4),...
        'B',zeros(4,2),...
        'C',zeros(4),...
        'D',zeros(4,2),...
        'Qy',zeros(4),...
        'R',zeros(2),...
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
      options.w = 1;
      options.R = 1e-12*eye(getNumInputs(r));
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
      
%       % cascade neck pitch control block
%       neck = NeckControlBlock(r,ctrl_data);
%       ins(1).system = 1;
%       ins(1).input = 1;
%       ins(2).system = 1;
%       ins(2).input = 2;
%       ins(3).system = 2;
%       ins(3).input = 3;
%       outs(1).system = 2;
%       outs(1).output = 1;
%       connection(1).from_output = 1;
%       connection(1).to_input = 1;
%       connection(2).from_output = 2;
%       connection(2).to_input = 2;
%       sys = mimoCascade(neck,sys,connection,ins,outs);
%       clear ins outs;
      
      obj = obj@DRCController(name,sys);
 
      obj.robot = r;
      obj.controller_data = ctrl_data;
      
      % set up butt and foot supports
      obj.foot_idx=obj.robot.findLinkInd('l_foot');
      obj.pelvis_idx=obj.robot.findLinkInd('pelvis');
      pelvis=obj.robot.findLink('pelvis');
      butt_grp_idx = find(strcmp('butt',pelvis.collision_group_name));
      butt_grp_pts = pelvis.collision_group{butt_grp_idx};
      supports = SupportState(r,[obj.foot_idx obj.pelvis_idx],{[1 2 3 4],butt_grp_pts},[1 1]);
      obj.controller_data.setField('supports',supports);

      % hijack the walking plan type for now
      obj = addLCMTransition(obj,'QUASISTATIC_ROBOT_PLAN',drc.walking_plan_t(),'qs_motion'); % for standing/reaching tasks
      obj = addLCMTransition(obj,'COMMITTED_ROBOT_PLAN',drc.robot_plan_t(),name);
      
      obj = initialize(obj,struct());

    end

    function msg = status_message(obj,t_sim,t_ctrl)
      msg = drc.controller_status_t();
      msg.utime = t_sim * 1000000;
      msg.state = msg.QUASISTATIC; % TODO: UPDATE ME
      msg.controller_utime = t_ctrl * 1000000;
      msg.V = 0;
      msg.Vdot = 0;
    end
    
    function obj = initialize(obj,data)
      if isfield(data,'COMMITTED_ROBOT_PLAN')
        % pinned reaching plan
        msg = data.COMMITTED_ROBOT_PLAN;
        joint_names = obj.robot.getStateFrame.coordinates(1:getNumDOF(obj.robot));
        [xtraj,ts] = RobotPlanListener.decodeRobotPlan(msg,true,joint_names);
        qtraj = PPTrajectory(spline(ts,xtraj(1:getNumDOF(obj.robot),:)));
        obj.controller_data.setField('qtraj',qtraj);
  
      elseif isfield(data,'AtlasState')
        % use last state as nominal pose 
        r = obj.robot;
        x0 = data.AtlasState;
        q0 = x0(1:getNumDOF(r));
        obj.controller_data.setField('qtraj',q0);
     
      else
        % use saved nominal standing pose 
        d =load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_seated_pose.mat'));
        q0 = d.xstar(1:getNumDOF(obj.robot));
        obj.controller_data.setField('qtraj',q0);
      end
     
      QPController.check_ctrl_data(obj.controller_data);  
      obj = setDuration(obj,inf,false); % set the controller timeout
    end    
  end
end  
