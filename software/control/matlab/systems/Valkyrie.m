classdef Valkyrie < Biped
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

      obj = obj@Biped(urdf,options.dt,options);

      obj.floating =options.floating;

      if options.floating
        % could also do fixed point search here
        obj = obj.setInitialState(double(obj.manip.resolveConstraints(zeros(obj.getNumStates(),1))));
      end
      warning(S);
    end

    function obj = compile(obj)
      S = warning('off','Drake:RigidBodyManipulator:SingularH');
      obj = compile@TimeSteppingRigidBodyManipulator(obj);
      warning(S);

      state_frame = ValkyrieState(obj);
      obj = obj.setStateFrame(state_frame);
      obj = obj.setOutputFrame(state_frame);

      % input_frame = ValkyrieInput(obj);
      % obj = obj.setInputFrame(input_frame);
    end

    function xstar = loadFixedPoint(obj)
      d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/valkyrie_fp.mat'));
      xstar = d.xstar;
    end
  end

  properties
    default_footstep_params = struct('nom_forward_step', 0.15,... %m
                                  'max_forward_step', 0.3,...%m
                                  'max_step_width', 0.40,...%m
                                  'min_step_width', 0.21,...%m
                                  'nom_step_width', 0.26,...%m
                                  'max_outward_angle', pi/8,... % rad
                                  'max_inward_angle', 0.01,... % rad
                                  'max_upward_step', 0.2,... % m
                                  'max_downward_step', 0.2); % m
  end
end

