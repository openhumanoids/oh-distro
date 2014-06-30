classdef WalkingController < DRCController

  properties (SetAccess=protected,GetAccess=protected)
    robot;
    controller_state_dim;
    controller_data;
  end

  methods
    function obj = WalkingController(name,r,options)
      typecheck(r,'Atlas');
      
      if nargin < 3
        options = struct();
      end

      % initialize with junk, populate when recieving first plan
      ctrl_data = AtlasQPControllerData(false,struct(...
        'acceleration_input_frame',AtlasCoordinates(r),...
        'D',-com(3)/9.81*eye(2),...
        'Qy',eye(2),...
        'S',zeros(4),...
        's1',zeros(4,1),...
        's2',0,...
        'x0',zeros(4,1),...
        'u0',zeros(2,1),...
        'y0',zeros(2,1),...
        'comtraj',[],...
        'qtraj',zeros(getNumDOF(r),1),...
        'mu',1.0,...
        'ignore_terrain',false,...
        'link_constraints',link_constraints,...
        'force_controlled_joints',force_controlled_joints,...
        'position_controlled_joints',position_controlled_joints,...
        'integral',zeros(getNumDOF(r),1),...
        'integral_gains',integral_gains,...
        'integral_clamps',integral_clamps,...
        'firstplan',true,...
        'constrained_dofs',[findJointIndices(r,'arm');findJointIndices(r,'back');findJointIndices(r,'neck')]));

      if ~isfield(options,'use_mex') options.use_mex = true; end
      if ~isfield(options,'debug') options.debug = false; end
      if ~isfield(options,'lcm_foot_contacts') options.lcm_foot_contacts = true; end

      sys = AtlasWalkingWrapper(r,ctrl_data,options);
      obj = obj@DRCController(name,sys,AtlasState(r));

      obj.controller_state_dim = sys.velocity_int_block.getStateFrame.dim;
      obj.robot = r;
      obj.controller_data = ctrl_data;
      
      obj = addLCMTransition(obj,'START_MIT_STAND',drc.utime_t(),'stand');  
      obj = setTimedTransition(obj,inf,'standing',false); % default, updated in initialize() using plan duration
    end

    function msg = status_message(obj,t_sim,t_ctrl)
        msg = drc.controller_status_t();
        msg.utime = t_sim * 1000000;
        msg.state = msg.WALKING;
        msg.controller_utime = t_ctrl * 1000000;
        msg.V = 0;
        msg.Vdot = 0;
    end

    function obj = initialize(obj,data)
      msg_data = data.WALKING_PLAN;
      walk_ctrl_data = WalkingControllerData.from_walking_plan_t(msg_data);
      obj.controller_data.S = walk_ctrl_data.S;
      obj.controller_data.s1 = walk_ctrl_data.s1;
      obj.controller_data.is_time_varying = true;
      obj.controller_data.s1dot = walk_ctrl_data.s1dot;
      obj.controller_data.s2 = walk_ctrl_data.s2;
      obj.controller_data.s2dot = walk_ctrl_data.s2dot;
      support_times = msg_data.support_times;
      obj.controller_data.support_times = support_times;
      obj.controller_data.supports = walk_ctrl_data.supports;
      obj.controller_data.comtraj = walk_ctrl_data.comtraj;
      T = walk_ctrl_data.zmptraj.tspan(end);
      obj.controller_data.x0 = [fasteval(walk_ctrl_data.zmptraj,T);0;0];
      obj.controller_data.y0 = fasteval(walk_ctrl_data.zmptraj,T);
      obj.controller_data.link_constraints = walk_ctrl_data.link_constraints;
      obj.controller_data.qtraj = walk_ctrl_data.qtraj;
      obj.controller_data.mu = msg_data.mu;
      obj.controller_data.plan_shift = [0;0;0];
      obj = setDuration(obj,T,false); % set the controller timeout
    end
  end
end
