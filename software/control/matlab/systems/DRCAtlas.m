classdef DRCAtlas < Atlas
  methods

    function obj=DRCAtlas(urdf,options)

      if nargin < 2
        options = struct();
      end
      options = applyDefaults(options,...
                              struct('atlas_version', 5,...
                                     'use_new_kinsol', true));

      if ~any(options.atlas_version == [3,4,5])
        error('Atlas:badVersion','Invalid Atlas version. Valid values are 3, 4, and 5')
      end

      if nargin < 1 || isempty(urdf)
        switch options.atlas_version
          case 3
            urdf = strcat(getenv('DRC_PATH'),'/models/atlas_v3/model_minimal_contact_point_hands.urdf');
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

      obj = obj@Atlas(urdf, options);

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
        obj = addFrame(obj,RigidBodyFrame(findLinkId(obj,'head'),[-0.0446, 0, 0.0880].',[0, 0, 0].','hokuyo_frame'));
        hokuyo = RigidBodyLidarSpinningStateless('hokuyo',findFrameId(obj,'hokuyo_frame'), ...
          -obj.hokuyo_yaw_width/2.0, obj.hokuyo_yaw_width/2.0, obj.hokuyo_num_pts, obj.hokuyo_max_range, obj.hokuyo_spin_rate, ...
          obj.hokuyo_mirror_offset);
        if (~isfield(options, 'visualize') || options.visualize)
          hokuyo = enableLCMGL(hokuyo);
        end
        obj = addSensor(obj,hokuyo);
      end
      
      % And foot force sensors?
      if (~isfield(options, 'foot_force_sensors'))
        options.foot_force_sensors = false;
      end
      obj.foot_force_sensors = options.foot_force_sensors;
      if (options.foot_force_sensors)
        l_foot_body = findLinkId(obj,'l_foot');
        l_foot_frame = RigidBodyFrame(l_foot_body,zeros(3,1),zeros(3,1),'l_foot_ft');
        obj = obj.addFrame(l_foot_frame);
        l_foot_force_sensor = ContactForceTorqueSensor(obj, l_foot_frame);
        obj = addSensor(obj, l_foot_force_sensor);
        r_foot_body = findLinkId(obj,'r_foot');
        r_foot_frame = RigidBodyFrame(r_foot_body,zeros(3,1),zeros(3,1),'r_foot_ft');
        obj = obj.addFrame(r_foot_frame);
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
        obj.getStateFrame().getFrameByNameRecursive('drcFrames.AtlasState').setMaxRate(obj.control_rate);
      else
        obj.getStateFrame().setMaxRate(obj.control_rate);
      end

  	  switch obj.atlas_version
        case 3
          obj.fixed_point_file = fullfile(getenv('DRC_PATH'),'/control/matlab/data/atlas_v3/atlas_v3_fp.mat');
          obj.stateToBDIInd = 6+[1 2 3 28 9 10 11 12 13 14 21 22 23 24 25 26 4 5 6 7 8 15 16 17 18 19 20 27]';
          obj.BDIToStateInd = 6+[1 2 3 17 18 19 20 21 5 6 7 8 9 10 22 23 24 25 26 27 11 12 13 14 15 16 28 4]';
        case 4
          obj.fixed_point_file = fullfile(getenv('DRC_PATH'),'/control/matlab/data/atlas_v4/atlas_v4_fp.mat');
          obj.stateToBDIInd = 6+[1 2 3 28 9 10 11 12 13 14 21 22 23 24 25 26 4 5 6 7 8 15 16 17 18 19 20 27]';
          obj.BDIToStateInd = 6+[1 2 3 17 18 19 20 21 5 6 7 8 9 10 22 23 24 25 26 27 11 12 13 14 15 16 28 4]';
        case 5
          obj.fixed_point_file = fullfile(getenv('DRC_PATH'),'/control/matlab/data/atlas_v5/atlas_v5_fp.mat');
          obj.bracing_config_file = fullfile(getenv('DRC_PATH'),'/control/matlab/data/atlas_v5/atlas_bracing_v5.mat');
          obj.stateToBDIInd = 6+[1 2 3 30 5 6 7 8 9 10 18 19 20 21 22 23 4 11 12 13 14 15 16 17 24 25 26 27 28 29]';
      end
      warning(S);
    end

    function obj = compile(obj)
      obj = compile@TimeSteppingRigidBodyManipulator(obj);
      if isempty(obj.atlas_version)
        return;
      end

      % Sanity check if we don't have hands.
      if (strcmp(obj.manip.getStateFrame().getFrameByNum(1).name, 'atlasPosition'))
        obj.hand_right = 0;
        obj.hand_left = 0;
      end
      % Construct state vector itself -- start by replacing the
      % atlasPosition and atlasVelocity frames with a single
      % larger state frame
      if (obj.hand_right == 0 && obj.hand_left == 0 && strcmp(obj.manip.getStateFrame().getFrameByNum(1).name, 'atlasPosition'))
        atlas_state_frame = drcFrames.AtlasState(obj);
      else
        atlas_state_frame = obj.manip.getStateFrame();
        atlas_state_frame = replaceFrameNum(atlas_state_frame,1,drcFrames.AtlasState(obj));
      end
      % Sub in handstates for the hands
      % If we sub in the order that they are added
      % we should get this in the right order
      if (obj.hand_right > 0)
        id = atlas_state_frame.getFrameNumByName('s-model_articulatedPosition+s-model_articulatedVelocity');
        if (length(id) > 1)
          id = id(1);
        end
        atlas_state_frame = replaceFrameNum(atlas_state_frame,id,atlasFrames.HandState(obj,id,'right_atlasFrames.HandState'));
      end
      if (obj.hand_left > 0)
        id = atlas_state_frame.getFrameNumByName('s-model_articulatedPosition+s-model_articulatedVelocity');
        if (length(id) > 1)
          id = id(1);
        end
        atlas_state_frame = replaceFrameNum(atlas_state_frame,id,atlasFrames.HandState(obj,id,'left_atlasFrames.HandState'));
      end
      
      tsmanip_state_frame = obj.getStateFrame();
      if tsmanip_state_frame.dim>atlas_state_frame.dim
        tsmanip_state_frame.frame{1} = atlas_state_frame;
        state_frame = tsmanip_state_frame;
      else
        state_frame = atlas_state_frame;
      end
      obj.manip = obj.manip.setStateFrame(atlas_state_frame);
      obj = obj.setStateFrame(state_frame);
      
      % Same bit of complexity for input frame to get hand inputs
      if (obj.hand_right > 0 || obj.hand_left > 0 || obj.external_force > 0)
        input_frame = getInputFrame(obj);
        input_frame  = replaceFrameNum(input_frame,1,drcFrames.AtlasInput(obj));
      else
        input_frame = drcFrames.AtlasInput(obj);
      end
      if (obj.hand_right > 0)
        id = input_frame.getFrameNumByName('s-model_articulatedInput');
        if (length(id) > 1)
          id = id(1);
        end
        input_frame = replaceFrameNum(input_frame,id,atlasFrames.HandInput(obj,id,'right_atlasFrames.HandInput'));
      end
      if (obj.hand_left > 0)
        id = input_frame.getFrameNumByName('s-model_articulatedInput');
        if (length(id) > 1)
          id = id(1);
        end 
        input_frame = replaceFrameNum(input_frame,id,atlasFrames.HandInput(obj,id,'left_atlasFrames.HandInput'));
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
      % The output function of a TSRBM appends the TS sensors to the
      % output of the RBM. So get ready for that:
      output_frame = atlas_output_frame;
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
      
      if ~isequal_modulo_transforms(atlas_output_frame,getOutputFrame(obj.manip))
        obj.manip = obj.manip.setNumOutputs(atlas_output_frame.dim);
        obj.manip = obj.manip.setOutputFrame(atlas_output_frame);
      end
      
      if ~isequal_modulo_transforms(output_frame,getOutputFrame(obj))
        obj = obj.setNumOutputs(output_frame.dim);
        obj = obj.setOutputFrame(output_frame);
      end
    end

    function [zmin,zmax] = getPelvisHeightLimits(obj,q) % for BDI manip mode
      kinsol = doKinematics(obj,q);
      foot_z = getFootHeight(obj,q);
      pelvis = forwardKin(obj,kinsol,findLinkInd(obj,'pelvis'),[0;0;0]);
      z_above_feet = pelvis(3) - foot_z;
      zmin = q(3) - (z_above_feet-obj.pelvis_min_height);
      zmax = q(3) + (obj.pelvis_max_height-z_above_feet);
    end

  end

  properties (SetAccess = protected, GetAccess = public)
    control_rate = 250;
    inverse_dyn_qp_controller;
    pelvis_min_height = 0.65; % [m] above feet, for hardware
    pelvis_max_height = 0.92; % [m] above feet, for hardware
    stateToBDIInd;
    BDIToStateInd;
    hokuyo_yaw_width = 2.0; % total -- i.e., whole FoV, not from center of vision
    hokuyo_num_pts = 250;   
    hokuyo_max_range = 6; % meters?
    hokuyo_spin_rate = 16; % rad/sec
    hokuyo_mirror_offset = [0.03; 0.0; -0.015]; % from multisense_sim.urdf
                                               % with a rotation
                                               % (due to rotation of 
                                               % frame)
    bracing_config_file;
    foot_force_sensors = false;
    floating = true; % for backwards compatibility (floating is always true now)
  end
end
