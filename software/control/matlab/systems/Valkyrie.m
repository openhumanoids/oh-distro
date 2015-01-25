classdef Valkyrie < TimeSteppingRigidBodyManipulator & Biped
  methods
    function obj = Valkyrie(urdf, options)
      if nargin < 1 || isempty(urdf)
        urdf = strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/V1/models/V1/urdf/V1_sim_mit_drake.urdf');
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

      S = warning('off','Drake:RigidBodyManipulator:SingularH');
      warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');

      obj = obj@TimeSteppingRigidBodyManipulator(urdf,options.dt,options);
      obj = obj@Biped('r_foot_sole', 'l_foot_sole');

      if options.floating
        % could also do fixed point search here
        %obj = obj.setInitialState(double(obj.manip.resolveConstraints(zeros(obj.getNumStates(),1))));
      end
      warning(S);
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

  properties
    fixed_point_file = fullfile(getenv('DRC_PATH'),'/control/matlab/data/valkyrie_fp.mat');
  end

  properties (SetAccess = protected, GetAccess = public)
    x0
    default_footstep_params = struct('nom_forward_step', 0.15,... %m
                                  'max_forward_step', 0.3,...%m
                                  'max_step_width', 0.40,...%m
                                  'min_step_width', 0.21,...%m
                                  'nom_step_width', 0.26,...%m
                                  'max_outward_angle', pi/8,... % rad
                                  'max_inward_angle', 0.01,... % rad
                                  'max_upward_step', 0.2,... % m
                                  'max_downward_step', 0.2); % m
    default_walking_params = struct('step_speed', 0.5,... % speed of the swing foot (m/s)
                                    'step_height', 0.05,... % approximate clearance over terrain (m)
                                    'hold_frac', 0.4,... % fraction of the swing time spent in double support
                                    'drake_min_hold_time', 1.0,... % minimum time in double support (s)
                                    'mu', 1.0,... % friction coefficient
                                    'constrain_full_foot_pose', true); % whether to constrain the swing foot roll and pitch

  end
end

