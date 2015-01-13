classdef DRCAtlas < Atlas
  methods

    function obj=DRCAtlas(urdf,options)

      if nargin < 2
        options = struct();
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
        obj.getStateFrame().getFrameByName('drcFrames.AtlasState').setMaxRate(obj.control_rate);
      else
        obj.getStateFrame().setMaxRate(obj.control_rate);
      end
      

      obj.stateToBDIInd = 6+[1 2 3 28 9 10 11 12 13 14 21 22 23 24 25 26 4 5 6 7 8 15 16 17 18 19 20 27]';
      obj.BDIToStateInd = 6+[1 2 3 17 18 19 20 21 5 6 7 8 9 10 22 23 24 25 26 27 11 12 13 14 15 16 28 4]';

  	  switch obj.atlas_version
        case 3
          obj.fixed_point_file = fullfile(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat');
        case 4
          obj.fixed_point_file = fullfile(getenv('DRC_PATH'),'/control/matlab/data/atlas_v4_fp.mat');
        case 5
          obj.fixed_point_file = fullfile(getenv('DRC_PATH'),'/control/matlab/data/atlas_v5_fp.mat');
      end
      warning(S);
    end

    function obj = compile(obj)
      obj = compile@TimeSteppingRigidBodyManipulator(obj);
      if isempty(obj.atlas_version)
        return;
      end

      % Sanity check if we have hands.
      if (~isa(obj.manip.getStateFrame().getFrameByNum(1), 'MultiCoordinateFrame'))
        obj.hands = 0;
      end
      % Construct state vector itself
      if (obj.hands == 0 && obj.foot_force_sensors == 0)
        atlas_state_frame = drcFrames.AtlasState(obj);
      else
        atlas_state_frame = getStateFrame(obj);
        atlas_state_frame = replaceFrameNum(atlas_state_frame,1,drcFrames.AtlasState(obj));
      end
      if (obj.hands > 0)
        % Sub in handstates for the hand (curently assuming just 1)
        % TODO: by name?
        for i=2:2
          atlas_state_frame = replaceFrameNum(atlas_state_frame,i,drcFrames.HandState(obj,i,'drcFrames.HandState'));
        end
      end
      if (obj.foot_force_sensors)
        startind = 1;
        if (obj.hands > 0)
          startind = startind + 2;
        end
        for i=startind:startind+1
          atlas_state_frame = replaceFrameNum(atlas_state_frame, i, drcFrames.ForceTorque());
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
        input_frame  = replaceFrameNum(input_frame,1,drcFrames.AtlasInput(obj));
        % Sub in handstates for each hand
        % TODO: by name?
        for i=2:2
          input_frame = replaceFrameNum(input_frame,i,drcFrames.HandInput(obj,i,'drcFrames.HandInput'));
        end
      else
        input_frame = drcFrames.AtlasInput(obj);
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
    hokuyo_mirror_offset = [0.015; 0.0; -0.015]; % from multisense_sim.urdf
                                               % with a rotation
                                               % (due to rotation of 
                                               % frame)

    foot_force_sensors = false;
  end
end
