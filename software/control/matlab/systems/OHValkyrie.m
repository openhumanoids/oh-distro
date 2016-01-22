classdef OHValkyrie < Valkyrie
  methods
    function obj = OHValkyrie(urdf, options)

      if nargin < 2
        options = struct();
      end
      options = applyDefaults(options,...
                              struct('valkyrie_version', 1,...
                                     'use_new_kinsol', true));

      if ~any(options.valkyrie_version == [1,2])
        error('OHValkyrie:badVersion','Invalid OHValkyrie version. Valid values are 1 and 2')
      end

      if nargin < 1 || isempty(urdf)
        switch options.valkyrie_version
          case 1
            %urdf = strcat(getenv('DRC_PATH'),'/models/valkyrie/V1_sim_mit_drake.urdf');
            urdf = strcat(getenv('DRC_PATH'),'/models/valkyrie/V1_sim_shells_reduced_polygon_count_mit.urdf');
          case 2
            urdf = strcat(getenv('DRC_PATH'),'/models/val_description/urdf/valkyrie_sim_drake.urdf');
        end
      else
        typecheck(urdf,'char');
      end

      if ~isfield(options,'hands')
        options.hands = 'none';
      end

      S = warning('off','Drake:RigidBodyManipulator:SingularH');
      warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');

      obj = obj@Valkyrie(urdf, options);

      warning(S);
    end

    function obj = compile(obj)
      S = warning('off','Drake:RigidBodyManipulator:SingularH');
      obj = compile@TimeSteppingRigidBodyManipulator(obj);
      warning(S);

      state_frame = drcFrames.ValkyrieState(obj);
      obj = obj.setStateFrame(state_frame);
      obj = obj.setOutputFrame(state_frame);

      % input_frame = OHValkyrieInput(obj);
      % obj = obj.setInputFrame(input_frame);
    end
  end
end

