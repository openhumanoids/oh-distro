classdef Atlas < TimeSteppingRigidBodyManipulator & Biped
  methods

    function obj=Atlas(urdf,options)

      if nargin < 2
        options = struct();
      end
      if ~isfield(options,'dt')
        options.dt = 0.001;
      end
      if ~isfield(options,'floating')
        options.floating = true;
      end
      if ~isfield(options,'terrain')
        options.terrain = RigidBodyFlatTerrain;
      end
      if ~isfield(options,'atlas_version') 
        options.atlas_version = 4; 
      end

      if ~any(options.atlas_version == [3,4,5])
        error('Atlas:badVersion','Invalid Atlas version. Valid values are 3, 4, and 5')
      end

      if nargin < 1 || isempty(urdf)
        switch options.atlas_version
          case 3
            urdf = strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf');
          case 4
            urdf = strcat(getenv('DRC_PATH'),'/models/atlas_v4/model_minimal_contact.urdf');
          case 5
            urdf = strcat(getenv('DRC_PATH'),'/models/atlas_v5/model_minimal_contact.urdf');
        end
      else
        typecheck(urdf,'char');
      end
      
      if ~isfield(options,'control_rate')
        options.control_rate = 250;
      end

      obj = obj@TimeSteppingRigidBodyManipulator(urdf,options.dt,options);
      obj = obj@Biped('r_foot_sole', 'l_foot_sole');
      obj.atlas_version = options.atlas_version;
      obj = compile(obj);
      
      % add hands
      if ~isfield(options,'hands')
        options.hands = 'none';
      end
      if (~strcmp(options.hands, 'none'))
        if (strcmp(options.hands, 'robotiq'))
          options_hand.weld_to_link = findLinkId(obj,'r_hand');
          obj.hands = 1;
          obj = obj.addRobotFromURDF(getFullPathFromRelativePath('urdf/robotiq.urdf'), [0; -0.195; -0.01], [0; -3.1415/2; 3.1415], options_hand);
        elseif (strcmp(options.hands, 'robotiq_weight_only'))
          % Adds a box with weight roughly approximating the hands, so that
          % the controllers know what's up
          options_hand.weld_to_link = findLinkId(obj,'r_hand');
          obj = obj.addRobotFromURDF(getFullPathFromRelativePath('urdf/robotiq_box.urdf'), [0; -0.195; -0.01], [0; -3.1415/2; 3.1415], options_hand);
        else
          error('unsupported hand type');
        end
      end
      
      % set up sensor system      
      feedback = FullStateFeedbackSensor();
      obj = addSensor(obj, feedback);
      if (~isfield(options, 'hokuyo'))
        options.hokuyo = false;
      end
      if (options.hokuyo)
        if (isfield(options, 'hokuyo_spin_rate'))
          obj.hokuyo_spin_rate = options.hokuyo_spin_rate;
        end
        % Add lidar -- hokuyo / spindle frames are pulled from
        % config/config_components/multisense_sim.cfg
        % trying new value that lines up more accurately with
        % head_to_left_eye, left_eye_to_spindle transforms
        % from multisense_sim.cfg
        obj = addFrame(obj,RigidBodyFrame(findLinkId(obj,'head'),[-0.0446, -0.0087, 0.0880].',[0, 0, 0].','hokuyo_frame'));
        hokuyo = RigidBodyLidarSpinningStateless('hokuyo',findFrameId(obj,'hokuyo_frame'), ...
          -obj.hokuyo_yaw_width/2.0, obj.hokuyo_yaw_width/2.0, obj.hokuyo_num_pts, obj.hokuyo_max_range, obj.hokuyo_spin_rate, ...
          obj.hokuyo_mirror_offset);
        if (~isfield(options, 'visualize') || options.visualize)
          hokuyo = enableLCMGL(hokuyo);
        end
        obj = addSensor(obj,hokuyo);
      end
      
      % And foot force sensors?
      obj.foot_indices = [obj.findLinkId('l_foot'), obj.findLinkId('r_foot')];
      if (~isfield(options, 'foot_force_sensors'))
        options.foot_force_sensors = false;
      end
      obj.foot_force_sensors = options.foot_force_sensors;
      if (options.foot_force_sensors)
        l_foot_body = findLinkId(obj,'l_foot');
        l_foot_frame = RigidBodyFrame(l_foot_body,zeros(3,1),zeros(3,1),'l_foot');
        l_foot_force_sensor = ContactForceTorqueSensor(obj, l_foot_frame);
        obj = addSensor(obj, l_foot_force_sensor);
        r_foot_body = findLinkId(obj,'r_foot');
        r_foot_frame = RigidBodyFrame(r_foot_body,zeros(3,1),zeros(3,1),'r_foot');
        r_foot_force_sensor = ContactForceTorqueSensor(obj, r_foot_frame);
        obj = addSensor(obj, r_foot_force_sensor);
      end
      
      obj = compile(obj);
      
      % Add obstacles if we want 
      % (here is just a box in front of the robot to look at)
      if (isfield(options,'obstacles') && options.obstacles)
        height = 0.1;
        shape = RigidBodyBox([1.0 1.0 height], [2; 0; height/2], [0; 0; 0;]);
        shape.c = rand(3, 1);
        obj = addShapeToBody(obj, 'world', shape);
        obj = addContactShapeToBody(obj, 'world', shape);
        obj = compile(obj);
      end
      
      S = warning('off','Drake:RigidBodyManipulator:SingularH');
      warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');

      obj.control_rate = options.control_rate;
      if (isa(obj.getStateFrame(), 'MultiCoordinateFrame'))
        obj.getStateFrame().getFrameByName('AtlasState').setMaxRate(obj.control_rate);
      else
        obj.getStateFrame().setMaxRate(obj.control_rate);
      end
      
      obj.floating = options.floating;

      obj.stateToBDIInd = 6*obj.floating+[1 2 3 28 9 10 11 12 13 14 21 22 23 24 25 26 4 5 6 7 8 15 16 17 18 19 20 27]';
      obj.BDIToStateInd = 6*obj.floating+[1 2 3 17 18 19 20 21 5 6 7 8 9 10 22 23 24 25 26 27 11 12 13 14 15 16 28 4]';

      if options.floating
        % could also do fixed point search here
        ts_init = zeros(obj.getNumStates(), 1);
        manip_init = double(obj.manip.resolveConstraints(ts_init));
        % Pad back up
        ts_init(1:length(manip_init)) = manip_init;
        obj = obj.setInitialState(ts_init);

        switch obj.atlas_version
          case 3
            obj.fixed_point_file = fullfile(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat');
          case 4
            obj.fixed_point_file = fullfile(getenv('DRC_PATH'),'/control/matlab/data/atlas_v4_fp.mat');
          case 5
            obj.fixed_point_file = fullfile(getenv('DRC_PATH'),'/control/matlab/data/atlas_v5_fp.mat');
        end
      else
        % TEMP HACK to get by resolveConstraints
        %for i=1:length(obj.manip.body), obj.manip.body(i).contact_pts=[]; end
        %obj.manip = compile(obj.manip);
        %obj = obj.setInitialState(zeros(obj.getNumStates(),1));
      end
      warning(S);
      
      obj.left_full_support = RigidBodySupportState(obj,obj.foot_body_id.left);
      obj.left_toe_support = RigidBodySupportState(obj,obj.foot_body_id.left,{{'toe'}});
      obj.right_full_support = RigidBodySupportState(obj,obj.foot_body_id.right);
      obj.right_toe_support = RigidBodySupportState(obj,obj.foot_body_id.right,{{'toe'}});
      obj.left_full_right_full_support = RigidBodySupportState(obj,[obj.foot_body_id.left,obj.foot_body_id.right]);
      obj.left_toe_right_full_support = RigidBodySupportState(obj,[obj.foot_body_id.left,obj.foot_body_id.right],{{'toe'},{'heel','toe'}});
      obj.left_full_right_toe_support = RigidBodySupportState(obj,[obj.foot_body_id.left,obj.foot_body_id.right],{{'heel','toe'},{'toe'}});
    end

    function obj = compile(obj)
      if isempty(obj.atlas_version)
        return;
      end
      obj = compile@TimeSteppingRigidBodyManipulator(obj);

      % Sanity check if we have hands.
      if (~isa(obj.manip.getStateFrame().getFrameByNum(1), 'MultiCoordinateFrame'))
        obj.hands = 0;
      end
      % Construct state vector itself
      if (obj.hands == 0 && obj.foot_force_sensors == 0)
        atlas_state_frame = AtlasState(obj);
      else
        atlas_state_frame = getStateFrame(obj);
        atlas_state_frame = replaceFrameNum(atlas_state_frame,1,AtlasState(obj));
      end
      if (obj.hands > 0)
        % Sub in handstates for the hand (curently assuming just 1)
        % TODO: by name?
        for i=2:2
          atlas_state_frame = replaceFrameNum(atlas_state_frame,i,HandState(obj,i,'HandState'));
        end
      end
      if (obj.foot_force_sensors)
        startind = 1;
        if (obj.hands > 0)
          startind = startind + 2;
        end
        for i=startind:startind+1
          atlas_state_frame = replaceFrameNum(atlas_state_frame, i, ForceTorque());
        end
      end
      tsmanip_state_frame = obj.getStateFrame();
      if tsmanip_state_frame.dim>atlas_state_frame.dim
        id = findSubFrameEquivalentModuloTransforms(tsmanip_state_frame,atlas_state_frame);
        tsmanip_state_frame.frame{id} = atlas_state_frame;
        state_frame = tsmanip_state_frame;
      else
        state_frame = atlas_state_frame;
      end
      obj.manip = obj.manip.setStateFrame(atlas_state_frame);
      obj = obj.setStateFrame(state_frame);
      
      % Same bit of complexity for input frame to get hand inputs
      if (obj.hands > 0)
        input_frame = getInputFrame(obj);
        input_frame  = replaceFrameNum(input_frame,1,AtlasInput(obj));
        % Sub in handstates for each hand
        % TODO: by name?
        for i=2:2
          input_frame = replaceFrameNum(input_frame,i,HandInput(obj,i,'HandInput'));
        end
      else
        input_frame = AtlasInput(obj);
      end
      obj = obj.setInputFrame(input_frame);
      obj.manip = obj.manip.setInputFrame(input_frame);
      
      % Construct output frame, which comes from state plus sensor
      % info
      atlas_output_frame = atlas_state_frame;
      if (~isempty(obj.manip.sensor))
        for i=1:length(obj.manip.sensor)
          % If it's not a full state feedback sensor (we have already
          % got the state for that above in the state frame
          if (~isa(obj.manip.sensor{i}, 'FullStateFeedbackSensor'))
            if (isa(atlas_output_frame, 'MultiCoordinateFrame'))
              atlas_output_frame = atlas_output_frame.appendFrame(obj.manip.sensor{i}.constructFrame(obj.manip));
            else
              atlas_output_frame = MultiCoordinateFrame({atlas_output_frame, obj.manip.sensor{i}.constructFrame(obj.manip)});
            end
          end
        end
      end
      output_frame = atlas_output_frame;
      % Continuing frame from above...
      if (~isempty(obj.sensor))
        for i=1:length(obj.sensor)
          if (~isa(obj.sensor{i}, 'FullStateFeedbackSensor'))
            if (isa(output_frame, 'MultiCoordinateFrame'))
              output_frame = output_frame.appendFrame(obj.sensor{i}.constructFrame(obj));
            else
              output_frame = MultiCoordinateFrame({output_frame, obj.sensor{i}.constructFrame(obj)});
            end
          end
        end
      end
%       atlas_output_frame = cell(0);
%       if (~isempty(obj.manip.sensor))
%         for i=1:length(obj.manip.sensor)
%           atlas_output_frame{i} = obj.manip.sensor{i}.constructFrame(obj.manip);
%         end
%       end
%       output_frame = atlas_output_frame;
%       % Continuing frame from above...
%       if (~isempty(obj.sensor))
%         for i=1:length(obj.sensor)
%           output_frame{length(atlas_output_frame)+i} = obj.sensor{i}.constructFrame(obj);
%         end
%       end
%       atlas_output_frame = MultiCoordinateFrame.constructFrame(atlas_output_frame);
%       output_frame = MultiCoordinateFrame.constructFrame(output_frame);
      
      if ~isequal_modulo_transforms(atlas_output_frame,getOutputFrame(obj.manip))
        obj.manip = obj.manip.setNumOutputs(atlas_output_frame.dim);
        obj.manip = obj.manip.setOutputFrame(atlas_output_frame);
      end
      
      if ~isequal_modulo_transforms(output_frame,getOutputFrame(obj))
        obj = obj.setNumOutputs(output_frame.dim);
        obj = obj.setOutputFrame(output_frame);
      end
    end

    function z = getPelvisHeightAboveFeet(obj,q)
      kinsol = doKinematics(obj,q);
      foot_z = getFootHeight(obj,q);
      pelvis = forwardKin(obj,kinsol,findLinkId(obj,'pelvis'),[0;0;0]);
      z = pelvis(3) - foot_z;
    end
    
    function bool = isDoubleSupport(obj,rigid_body_support_state)
      bool = any(rigid_body_support_state.bodies==obj.robot.foot_body_id.left) && any(rigid_body_support_state.bodies==obj.robot.foot_body_id.right);
    end

    function bool = isLeftSupport(obj,rigid_body_support_state)
      bool = any(rigid_body_support_state.bodies==obj.robot.foot_body_id.left) && ~any(rigid_body_support_state.bodies==obj.robot.foot_body_id.right);
    end

    function bool = isRightSupport(obj,rigid_body_support_state)
      bool = ~any(rigid_body_support_state.bodies==obj.robot.foot_body_id.left) && any(rigid_body_support_state.bodies==obj.robot.foot_body_id.right);
    end
    
    function foot_z = getFootHeight(obj,q)
      kinsol = doKinematics(obj,q);
      rfoot_cpos = terrainContactPositions(obj,kinsol,obj.foot_body_id.right);
      lfoot_cpos = terrainContactPositions(obj,kinsol,obj.foot_body_id.left);
      foot_z = min(mean(rfoot_cpos(3,:)),mean(lfoot_cpos(3,:)));
    end
    
    function fc = getFootContacts(obj, q)
      [phiC,~,~,~,idxA,idxB] = obj.collisionDetect(q,false);
      within_thresh = phiC < 0.002;
      contact_pairs = [idxA(within_thresh); idxB(within_thresh)];
      
      % The following would be faster but would require us to have
      % hightmaps in Bullet
      %[~,~,idxA,idxB] = obj.r_control.allCollisions(x(1:obj.nq_control));
      %contact_pairs = [idxA; idxB];
      foot_indices = [obj.foot_body_id.right, obj.foot_body_id.left];
      fc = any(bsxfun(@eq, contact_pairs(:), foot_indices),1)';
    end
    
    function [zmin,zmax] = getPelvisHeightLimits(obj,q) % for BDI manip mode
      z_above_feet = getPelvisHeightAboveFeet(obj,q);
      zmin = q(3) - (z_above_feet-obj.pelvis_min_height);
      zmax = q(3) + (obj.pelvis_max_height-z_above_feet);
    end

    function obj = setInitialState(obj,x0)
      if isa(x0,'Point')
        obj.x0 = double(x0); %.inFrame(obj.getStateFrame));
      else
        typecheck(x0,'double');
        sizecheck(x0,obj.getNumStates());
        obj.x0 = x0;
      end
    end

    function weights = getFootstepOptimizationWeights(obj)
      % Return a reasonable set of default weights for the footstep planner
      % optimization. The weights describe the following quantities:
      % 'relative': the contribution to the cost function of the
      %             displacement from one step to the next
      % 'relative_final': the cost contribution of the displacement of the
      %                   displacement of the very last step (this can be
      %                   larger than the normal 'relative' cost in
      %                   order to encourage the feet to be close together
      %                   at the end of a plan)
      % 'goal': the cost contribution on the distances from the last two
      %         footsteps to their respective goal poses.
      % Each weight is a 6 element vector, describing the weights on
      % [x, y, z, roll, pitch, yaw]

      weights = struct('relative', [1;1;1;0;0;0.5],...
                       'relative_final', [10;10;10;0;0;2],...
                       'goal', [100;100;0;0;0;10]);
    end

    function x0 = getInitialState(obj)
      x0 = obj.x0;
    end

    function [qp,lfoot_control_block,rfoot_control_block,pelvis_control_block,pd,options] = constructQPWalkingController(obj,controller_data,options)
      if nargin < 3
        options = struct();
      end
      options = ifNotIsFieldThenVal(options,'w_qdd',zeros(obj.getNumVelocities(),1));
      options = ifNotIsFieldThenVal(options,'input_foot_contacts',true);
      options = ifNotIsFieldThenVal(options,'Kp_pelvis',[0; 0; 20; 20; 20; 20]);
      options = ifNotIsFieldThenVal(options,'use_walking_pelvis_block',true);
      options = ifNotIsFieldThenVal(options,'pelvis_damping_ratio',0.5);
      options = ifNotIsFieldThenVal(options,'Kp_accel',0.0);
      options = ifNotIsFieldThenVal(options,'body_accel_input_weights',[0.15 0.15 0.075]);
      options = ifNotIsFieldThenVal(options,'use_walking_pelvis_block',true);
      options = ifNotIsFieldThenVal(options,'use_foot_motion_block',true);
      options = ifNotIsFieldThenVal(options,'Kp_foot',[12; 12; 12; 12; 12; 12]);
      options = ifNotIsFieldThenVal(options,'foot_damping_ratio',0.7);
      options = ifNotIsFieldThenVal(options,'min_knee_angle',0.7);
      options = ifNotIsFieldThenVal(options,'Kp_q',0.0*ones(obj.getNumPositions(),1));
      options = ifNotIsFieldThenVal(options,'q_damping_ratio',0.5);

      options.w_qdd(findPositionIndices(obj,'back_bkx')) = 0.01;
      options.Kp_q(findPositionIndices(obj,'back_bkx')) = 50;

      acc_limit = [100;100;100;50;50;50];
      body_accel_bounds(1).body_idx = obj.foot_body_id.right;
      body_accel_bounds(1).min_acceleration = -acc_limit;
      body_accel_bounds(1).max_acceleration = acc_limit;
      body_accel_bounds(2).body_idx = obj.foot_body_id.left;
      body_accel_bounds(2).min_acceleration = -acc_limit;
      body_accel_bounds(2).max_acceleration = acc_limit;
      options = ifNotIsFieldThenVal(options,'body_accel_bounds',body_accel_bounds);
      
      [qp,lfoot_control_block,rfoot_control_block,pelvis_control_block,pd,options] = ...
        constructQPBalancingController(obj,controller_data,options);
    end

    function [qp,lfoot_control_block,rfoot_control_block,pelvis_control_block,pd,options] = constructQPBalancingController(obj,controller_data,options)
      if nargin < 3
        options = struct();
      end
      options = ifNotIsFieldThenVal(options,'slack_limit',30);
      options = ifNotIsFieldThenVal(options,'w_qdd',0.0*ones(obj.getNumVelocities(),1));
      options = ifNotIsFieldThenVal(options,'W_kdot',0.0*eye(3));
      options = ifNotIsFieldThenVal(options,'w_grf',0.0);
      options = ifNotIsFieldThenVal(options,'w_slack',0.05);
      options = ifNotIsFieldThenVal(options,'Kp_accel',1.0);
      options = ifNotIsFieldThenVal(options,'debug',false);
      options = ifNotIsFieldThenVal(options,'use_mex',true);
      options = ifNotIsFieldThenVal(options,'contact_threshold',0.001);
      options = ifNotIsFieldThenVal(options,'output_qdd',true);
      options = ifNotIsFieldThenVal(options,'solver',0);  % 0 fastqp, 1 gurobi
      options = ifNotIsFieldThenVal(options,'Kp_pelvis',20*[1; 1; 1; 0.6; 0.6; 0.6]);
      options = ifNotIsFieldThenVal(options,'pelvis_damping_ratio',0.7);
      options = ifNotIsFieldThenVal(options,'body_accel_input_weights',0.01);
      options = ifNotIsFieldThenVal(options,'use_ik',false);
      options = ifNotIsFieldThenVal(options,'Kp_q',0.0*ones(obj.getNumPositions(),1));
      options = ifNotIsFieldThenVal(options,'q_damping_ratio',0.0);

      options.w_qdd(findPositionIndices(obj,'back_bkx')) = 0.1;
      options.Kp_q(findPositionIndices(obj,'back_bkx')) = 50;

      options.Kp = options.Kp_pelvis;
      options.Kd = getDampingGain(options.Kp,options.pelvis_damping_ratio);
      if isfield(options,'use_walking_pelvis_block') && options.use_walking_pelvis_block
        pelvis_control_block = PelvisMotionControlBlock(obj,'pelvis',controller_data,options);
      else
        pelvis_control_block = BodyMotionControlBlock(obj,'pelvis',controller_data,options);
      end

      if isfield(options,'use_foot_motion_block') && options.use_foot_motion_block
        options.Kp = options.Kp_foot;
        options.Kd = getDampingGain(options.Kp,options.foot_damping_ratio);
        lfoot_control_block = FootMotionControlBlock(obj,'l_foot',controller_data,options);
        rfoot_control_block = FootMotionControlBlock(obj,'r_foot',controller_data,options);
        motion_frames = {lfoot_control_block.getOutputFrame,rfoot_control_block.getOutputFrame,...
          pelvis_control_block.getOutputFrame};
      else
        lfoot_control_block = [];
        rfoot_control_block = [];
        motion_frames = {pelvis_control_block.getOutputFrame};
      end
      qp = AtlasQPController(obj,motion_frames,controller_data,options);

      options.Kp = options.Kp_q;
      options.Kd = getDampingGain(options.Kp,options.q_damping_ratio);
      pd = IKPDBlock(obj,controller_data,options);
    end
    
  end
  properties
    fixed_point_file; 
  end
  properties (SetAccess = protected, GetAccess = public)
    x0
    floating
    control_rate
    inverse_dyn_qp_controller;
    pelvis_min_height = 0.65; % [m] above feet, for hardware
    pelvis_max_height = 0.92; % [m] above feet, for hardware
    stateToBDIInd;
    BDIToStateInd;
    default_footstep_params = struct('nom_forward_step', 0.15,... %m
                                      'max_forward_step', 0.3,...%m
                                      'max_step_width', 0.40,...%m
                                      'min_step_width', 0.21,...%m
                                      'nom_step_width', 0.26,...%m
                                      'max_outward_angle', pi/8,... % rad
                                      'max_inward_angle', 0.01,... % rad
                                      'nom_upward_step', 0.2,... % m
                                      'nom_downward_step', 0.2,...% m
                                      'max_num_steps', 10,...
                                      'min_num_steps', 1,...
                                      'leading_foot', 1); % 0: left, 1: right
    default_walking_params = struct('step_speed', 0.5,... % speed of the swing foot (m/s)
                                    'step_height', 0.05,... % approximate clearance over terrain (m)
                                    'hold_frac', 0.4,... % fraction of the swing time spent in double support
                                    'drake_min_hold_time', 1.0,... % minimum time in double support (s)
                                    'drake_instep_shift', 0.0275,... % Distance to shift ZMP trajectory inward toward the instep from the center of the foot (m)
                                    'mu', 1.0,... % friction coefficient
                                    'constrain_full_foot_pose', true); % whether to constrain the swing foot roll and pitch

    hokuyo_yaw_width = 2.0; % total -- i.e., whole FoV, not from center of vision
    hokuyo_num_pts = 250;   
    hokuyo_max_range = 6; % meters?
    hokuyo_spin_rate = 16; % rad/sec
    hokuyo_mirror_offset = [0.015; 0.0; -0.03]; % from multisense_sim.urdf
                                               % with a rotation
                                               % (due to rotation of 
                                               % frame)

    foot_force_sensors = false;
    hands = 0; % 0, none; 1, Robotiq
    
    % preconstructing these for efficiency
    left_full_support
    left_toe_support
    right_full_support
    right_toe_support
    left_full_right_full_support
    left_toe_right_full_support
    left_full_right_toe_support
    atlas_version=[] % model version 3, 4, 5
    
  end
end
