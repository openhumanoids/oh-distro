classdef AtlasWalkingController < DRCController

  properties (SetAccess=protected,GetAccess=protected)
    robot;
    controller_state_dim;
  end

  methods
    function obj = AtlasWalkingController(name,r,options)
      typecheck(r,'Atlas');
      
      if nargin < 3
        options = struct();
      end
      
      
      if (~options.run_in_simul_mode)
        force_control_joint_str = {'leg','back_bkx'};% <---- cell array of (sub)strings
      else
        force_control_joint_str = {'leg', 'arm', 'back', 'neck'};
      end
      force_controlled_joints = [];
      for i=1:length(force_control_joint_str)
        force_controlled_joints = union(force_controlled_joints,find(~cellfun(@isempty,strfind(r.getInputFrame.coordinates,force_control_joint_str{i}))));
      end
      
      act_ind = (1:r.getNumInputs)';
      position_controlled_joints = setdiff(act_ind,force_controlled_joints);
      
      if (~options.run_in_simul_mode)
        % integral gains for position controlled joints
        integral_gains = zeros(getNumPositions(r),1);
        integral_clamps = zeros(getNumPositions(r),1);
        arm_ind = findJointIndices(r,'arm');
        back_ind = findJointIndices(r,'back');
        back_y_ind = findJointIndices(r,'back_bky');
        integral_gains(arm_ind) = 1.0; % TODO: generalize this
        integral_gains(back_ind) = 0.2;
        integral_clamps(arm_ind) = 0.3;
        integral_clamps(back_ind) = 0.2;
        integral_clamps(back_y_ind) = 0.1;
      end
      
      % initialize with junk, populate when recieving first plan
      if (isfield(options, 'run_in_simul_mode') && ...
          ~options.run_in_simul_mode)
        ctrl_data = AtlasQPControllerData(true,struct(...
          'acceleration_input_frame',AtlasCoordinates(r),...
          'D',-getAtlasNominalCOMHeight()/9.81*eye(2),...
          'Qy',eye(2),...
          'S',zeros(4),...
          's1',ConstantTrajectory(zeros(4,1)),...
          's2',ConstantTrajectory(0),...
          'x0',ConstantTrajectory(zeros(4,1)),...
          'u0',ConstantTrajectory(zeros(2,1)),...
          'y0',ConstantTrajectory(zeros(2,1)),...        
          'comtraj',ConstantTrajectory(zeros(4,1)),...
          'qtraj',zeros(getNumPositions(r),1),...
          'mu',0.5,...
          'support_times',0,...
          'ignore_terrain',false,...
          'force_controlled_joints',force_controlled_joints,...
          'position_controlled_joints',position_controlled_joints,...
          'integral',zeros(getNumPositions(r),1),...
          'integral_gains',integral_gains,...
          'integral_clamps',integral_clamps,...
          'constrained_dofs',[findJointIndices(r,'arm');findJointIndices(r,'neck');findJointIndices(r,'back_bkz');findJointIndices(r,'back_bky')]));
      else
        ctrl_data = AtlasQPControllerData(true,struct(...
          'acceleration_input_frame',AtlasCoordinates(r),...
          'D',-getAtlasNominalCOMHeight()/9.81*eye(2),...
          'Qy',eye(2),...
          'S',zeros(4),...
          's1',ConstantTrajectory(zeros(4,1)),...
          's2',ConstantTrajectory(0),...
          'x0',ConstantTrajectory(zeros(4,1)),...
          'u0',ConstantTrajectory(zeros(2,1)),...
          'y0',ConstantTrajectory(zeros(2,1)),...        
          'comtraj',ConstantTrajectory(zeros(4,1)),...
          'qtraj',zeros(getNumPositions(r),1),...
          'mu',0.5,...
          'support_times',0,...
          'ignore_terrain',false,...
          'force_controlled_joints',force_controlled_joints,...
          'position_controlled_joints',position_controlled_joints,...
          'constrained_dofs',[findJointIndices(r,'arm');findJointIndices(r,'neck');findJointIndices(r,'back_bkz');findJointIndices(r,'back_bky')]));
      end
      
      if ~isfield(options,'use_mex') options.use_mex = true; end
      if ~isfield(options,'debug') options.debug = false; end
      if ~isfield(options,'lcm_foot_contacts') options.lcm_foot_contacts = true; end

      sys = AtlasWalkingWrapper(r,ctrl_data,options);
      obj = obj@DRCController(name,sys,AtlasState(r));

      obj.controller_state_dim = sys.velocity_int_block.getStateFrame.dim;
      obj.robot = r;
      obj.controller_data = ctrl_data;
      
      obj = addLCMTransition(obj,'START_MIT_STAND',drc.utime_t(),'stand');  
      obj = setTimedTransition(obj,inf,'stand',false); % default, updated in initialize() using plan duration
    end

    function msg = status_message(~,t_sim,t_ctrl)
        msg = drc.controller_status_t();
        msg.utime = t_sim * 1000000;
        msg.state = msg.WALKING;
        msg.controller_utime = t_ctrl * 1000000;
        msg.V = 0;
        msg.Vdot = 0;
    end

    function obj = initialize(obj,data)
      msg_data = data.WALKING_CONTROLLER_PLAN_RESPONSE;
      walk_ctrl_data = WalkingControllerData.from_walking_plan_t(msg_data);
      obj.controller_data.S = walk_ctrl_data.S;
      obj.controller_data.s1 = walk_ctrl_data.s1;
      obj.controller_data.lqr_is_time_varying = true;
      %obj.controller_data.s1dot = walk_ctrl_data.s1dot;
      obj.controller_data.s2 = walk_ctrl_data.s2;
      %obj.controller_data.s2dot = walk_ctrl_data.s2dot;
      support_times = msg_data.support_times;
      obj.controller_data.support_times = support_times;
      obj.controller_data.supports = walk_ctrl_data.supports;
      obj.controller_data.comtraj = walk_ctrl_data.comtraj;
      T = walk_ctrl_data.zmptraj.tspan(end);
      obj.controller_data.x0 = ConstantTrajectory([fasteval(walk_ctrl_data.zmptraj,T);0;0]);
      obj.controller_data.y0 = walk_ctrl_data.zmptraj;
      obj.controller_data.link_constraints = walk_ctrl_data.link_constraints;
      obj.controller_data.qtraj = walk_ctrl_data.qtraj;
%       obj.controller_data.mu = msg_data.mu;
      obj.controller_data.plan_shift = [0;0;0];
      obj = setDuration(obj,T,false); % set the controller timeout
      controller_state = obj.controller_data.qd_int_state;
      controller_state(3) = 0; % reset time
      controller_state(4) = 0; % reset eta
      obj.controller_data.qd_int_state = controller_state;
      obj.controller_data.integral = 0*obj.controller_data.integral;
    end
  end
end
