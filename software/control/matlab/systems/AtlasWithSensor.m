classdef AtlasWithSensor < TimeSteppingRigidBodyManipulator & Biped
  methods

    function obj=AtlasWithSensor(urdf,options)

      if nargin < 1 || isempty(urdf)
        urdf = strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf');
      else
        typecheck(urdf,'char');
      end

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

      if ~isfield(options,'control_rate')
        options.control_rate = 250;
      end

      S = warning('off','Drake:RigidBodyManipulator:SingularH');
      warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
      
      obj = obj@TimeSteppingRigidBodyManipulator(urdf,options.dt,options);
      obj = obj@Biped('r_foot_sole', 'l_foot_sole');
      
      if isfield(options,'obstacles')
        for i=1:options.obstacles
          xy = randn(2,1);
          while(norm(xy)<1), xy = randn(2,1); end
          height = .05;
          shape = RigidBodyBox([.2+.8*rand(1,2) height],[xy;height/2],[0;0;randn]);
          shape.c = rand(3,1);
          obj = addShapeToBody(obj,'world',shape);
          obj = addContactShapeToBody(obj,'world',shape);
        end
      end
      
      % Add full state feedback sensor
      feedback = FullStateFeedbackSensor();
      obj = addSensor(obj, feedback);
      
      % Add lidar
      obj = addFrame(obj,RigidBodyFrame(findLinkInd(obj,'head'),[-0.0446; 0.0; 0.0880],zeros(3,1),'hokuyo_frame'));
      hokuyo = RigidBodyLidarSpinningStateless('hokuyo',findFrameId(obj,'hokuyo_frame'), ...
        -obj.hokuyo_yaw_width/2.0, obj.hokuyo_yaw_width/2.0, obj.hokuyo_num_pts, obj.hokuyo_max_range, obj.hokuyo_spin_rate);
      hokuyo = enableLCMGL(hokuyo);
      obj = addSensor(obj,hokuyo);
       obj = compile(obj);
      
      obj.control_rate = options.control_rate;
      obj.getStateFrame().setMaxRate(obj.control_rate);

      obj.floating = options.floating;

      obj.stateToBDIInd = 6*obj.floating+[1 2 3 28 9 10 11 12 13 14 21 22 23 24 25 26 4 5 6 7 8 15 16 17 18 19 20 27]';
      obj.BDIToStateInd = 6*obj.floating+[1 2 3 17 18 19 20 21 5 6 7 8 9 10 22 23 24 25 26 27 11 12 13 14 15 16 28 4]';
      
      if options.floating
        % could also do fixed point search here
        obj = obj.setInitialState(double(obj.manip.resolveConstraints(zeros(obj.getNumStates(),1))));
      else
        % TEMP HACK to get by resolveConstraints
        %for i=1:length(obj.manip.body), obj.manip.body(i).contact_pts=[]; end
        %obj.manip = compile(obj.manip);
        %obj = obj.setInitialState(zeros(obj.getNumStates(),1));
      end
      warning(S);
    end

    function obj = compile(obj)
      S = warning('off','Drake:RigidBodyManipulator:SingularH');
      obj = compile@TimeSteppingRigidBodyManipulator(obj);
      warning(S);

      state_frame = AtlasState(obj);
      
      obj.manip = obj.manip.setStateFrame(state_frame);
      obj = obj.setStateFrame(state_frame);
      
      %atlas_output_frame{1} = atlas_state_frame;
      atlas_output_frame = cell(0);
      if (~isempty(obj.manip.sensor))
        for i=1:length(obj.manip.sensor)
          atlas_output_frame{i} = obj.manip.sensor{i}.constructFrame(obj.manip);
        end
      end
      output_frame = atlas_output_frame;
      % Continuing frame from above...
      if (~isempty(obj.sensor))
        for i=1:length(obj.sensor)
          output_frame{length(atlas_output_frame)+i} = obj.sensor{i}.constructFrame(obj);
        end
      end
      atlas_output_frame = MultiCoordinateFrame.constructFrame(atlas_output_frame);
      output_frame = MultiCoordinateFrame.constructFrame(output_frame);
      
      if ~isequal_modulo_transforms(atlas_output_frame,getOutputFrame(obj.manip))
        obj.manip = obj.manip.setNumOutputs(atlas_output_frame.dim);
        obj.manip = obj.manip.setOutputFrame(atlas_output_frame);
      end
      
      if ~isequal_modulo_transforms(output_frame,getOutputFrame(obj))
        obj = obj.setNumOutputs(output_frame.dim);
        obj = obj.setOutputFrame(output_frame);
      end
      
      input_frame = AtlasInput(obj);
      obj = obj.setInputFrame(input_frame);
    end

    function z = getPelvisHeightAboveFeet(obj,q)
      kinsol = doKinematics(obj,q);
      foot_z = getFootHeight(obj,q);
      pelvis = forwardKin(obj,kinsol,findLinkInd(obj,'pelvis'),[0;0;0]);
      z = pelvis(3) - foot_z;
    end

    function foot_z = getFootHeight(obj,q)
      kinsol = doKinematics(obj,q);
      rfoot_cpos = terrainContactPositions(obj,kinsol,findLinkInd(obj,'r_foot'));
      lfoot_cpos = terrainContactPositions(obj,kinsol,findLinkInd(obj,'l_foot'));
      foot_z = min(mean(rfoot_cpos(3,:)),mean(lfoot_cpos(3,:)));
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
                       'relative_final', [10;10;10;0;0;1],...
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
      options = ifNotIsFieldThenVal(options,'pelvis_damping_ratio',0.6);
      options = ifNotIsFieldThenVal(options,'Kp_accel',2.0);
      options = ifNotIsFieldThenVal(options,'body_accel_input_weights',[0.3 0.3 0.1]);
      options = ifNotIsFieldThenVal(options,'use_walking_pelvis_block',true);
      options = ifNotIsFieldThenVal(options,'use_foot_motion_block',true);
      options = ifNotIsFieldThenVal(options,'Kp_accel',2.0);
      options = ifNotIsFieldThenVal(options,'Kp_foot',[20; 20; 20; 20; 20; 20]);
      options = ifNotIsFieldThenVal(options,'foot_damping_ratio',0.7);
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
      options = ifNotIsFieldThenVal(options,'contact_threshold',0.01);
      options = ifNotIsFieldThenVal(options,'output_qdd',true);
      options = ifNotIsFieldThenVal(options,'solver',0);  % 0 fastqp, 1 gurobi
      options = ifNotIsFieldThenVal(options,'Kp_pelvis',20*[1; 1; 1; 0.6; 0.6; 0.6]);
      options = ifNotIsFieldThenVal(options,'pelvis_damping_ratio',0.7);
      options = ifNotIsFieldThenVal(options,'body_accel_input_weights',0.01);
      options = ifNotIsFieldThenVal(options,'use_ik',false);
      options = ifNotIsFieldThenVal(options,'Kp_q',0.0*ones(obj.getNumPositions(),1));
      options = ifNotIsFieldThenVal(options,'q_damping_ratio',0.0);
      
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
      qp = QPController(obj,motion_frames,controller_data,options);

      options.Kp = options.Kp_q;
      options.Kd = getDampingGain(options.Kp,options.q_damping_ratio);
      pd = IKPDBlock(obj,controller_data,options);
    end
  end
  properties
    fixed_point_file = fullfile(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat');
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
                                      'leading_foot', 0); % 0: left, 1: right
    default_walking_params = struct('step_speed', 0.5,... % speed of the swing foot (m/s)
                                    'step_height', 0.05,... % approximate clearance over terrain (m)
                                    'hold_frac', 0.4,... % fraction of the swing time spent in double support
                                    'drake_min_hold_time', 1.0,... % minimum time in double support (s)
                                    'drake_instep_shift', 0.0275,... % Distance to shift ZMP trajectory inward toward the instep from the center of the foot (m)
                                    'mu', 1.0,... % friction coefficient
                                    'constrain_full_foot_pose', true); % whether to constrain the swing foot roll and pitch
    hokuyo_yaw_width = 1.6; % total -- i.e., whole FoV, not from center of vision
    hokuyo_num_pts = 30;   
    hokuyo_max_range = 6; % meters?
    hokuyo_spin_rate = 10; % rad/sec
  end
end
