classdef Valkyrie < TimeSteppingRigidBodyManipulator & Biped
  methods
    function obj = Valkyrie(urdf, options)

      if nargin < 2
        options = struct();
      end
      options = applyDefaults(options,...
                              struct('valkyrie_version', 1,...
                                     'use_new_kinsol', true));

      if ~any(options.valkyrie_version == [1,2])
        error('Valkyrie:badVersion','Invalid Valkyrie version. Valid values are 1 and 2')
      end

      if nargin < 1 || isempty(urdf)
        switch options.valkyrie_version
          case 1
            %urdf = strcat(getenv('DRC_PATH'),'/models/valkyrie/V1_sim_mit_drake.urdf');
            urdf = strcat(getenv('DRC_PATH'),'/models/valkyrie/V1_sim_shells_reduced_polygon_count_mit.urdf');
          case 2
            urdf = strcat(getenv('DRC_PATH'),'/models/val_description/urdf/valkyrie_A_sim_drake.urdf');
        end
      else
        typecheck(urdf,'char');
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
      if ~isfield(options,'hands')
        options.hands = 'none';
      end

      S = warning('off','Drake:RigidBodyManipulator:SingularH');
      warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');

      obj = obj@TimeSteppingRigidBodyManipulator(urdf,options.dt,options);
      obj = obj@Biped('r_foot_sole', 'l_foot_sole');

      if options.floating
        % could also do fixed point search here
        %obj = obj.setInitialState(double(obj.manip.resolveConstraints(zeros(obj.getNumStates(),1))));
      end
      warning(S);

      obj.left_full_support = RigidBodySupportState(obj,obj.foot_body_id.left);
      obj.left_toe_support = RigidBodySupportState(obj,obj.foot_body_id.left,struct('contact_groups',{{'toe'}}));
      obj.right_full_support = RigidBodySupportState(obj,obj.foot_body_id.right);
      obj.right_toe_support = RigidBodySupportState(obj,obj.foot_body_id.right,struct('contact_groups',{{'toe'}}));
      obj.left_full_right_full_support = RigidBodySupportState(obj,[obj.foot_body_id.left,obj.foot_body_id.right]);
      obj.left_toe_right_full_support = RigidBodySupportState(obj,[obj.foot_body_id.left,obj.foot_body_id.right],struct('contact_groups',{{{'toe'},{'heel','toe'}}}));
      obj.left_full_right_toe_support = RigidBodySupportState(obj,[obj.foot_body_id.left,obj.foot_body_id.right],struct('contact_groups',{{{'heel','toe'},{'toe'}}}));
    end

    function obj = compile(obj)
      S = warning('off','Drake:RigidBodyManipulator:SingularH');
      obj = compile@TimeSteppingRigidBodyManipulator(obj);
      warning(S);

      state_frame = drcFrames.ValkyrieState(obj);
      obj = obj.setStateFrame(state_frame);
      obj = obj.setOutputFrame(state_frame);

      % input_frame = ValkyrieInput(obj);
      % obj = obj.setInputFrame(input_frame);
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

    function x0 = getInitialState(obj)
      x0 = obj.x0;
    end
  end

  properties (SetAccess = protected, GetAccess = public)
    x0
    % preconstructing these for efficiency
    left_full_support
    left_toe_support
    right_full_support
    right_toe_support
    left_full_right_full_support
    left_toe_right_full_support
    left_full_right_toe_support
  end

  properties
    fixed_point_file = fullfile(getenv('DRC_PATH'),'/control/matlab/data/valkyrie_fp.mat');
    default_footstep_params = struct('nom_forward_step', 0.15,... %m
                                  'max_forward_step', 0.3,...%m
                                  'max_backward_step', 0.2,...%m
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
                                    'drake_instep_shift', 0.0,... % Distance to shift ZMP trajectory inward toward the instep from the center of the foot (m)
                                    'mu', 1.0,... % friction coefficient
                                    'constrain_full_foot_pose', true,... % whether to constrain the swing foot roll and pitch
                                    'pelvis_height_above_foot_sole', 1.0,... % default pelvis height when walking. MFALLON ADDED THESE VALUES, NOT OPTIMIZED... only barely works
                                    'nominal_LIP_COM_height', 1.0); % nominal height used to construct D_ls for our linear inverted pendulum model

  end
end

