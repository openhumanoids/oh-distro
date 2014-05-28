classdef Atlas < Biped
  methods

    function obj=Atlas(urdf,options)

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
      
      S = warning('off','Drake:RigidBodyManipulator:SingularH');
      warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');

%       obj = obj@TimeSteppingRigidBodyManipulator(urdf,options.dt,options);
      obj = obj@Biped(urdf,options.dt,options);

      obj.floating =options.floating;

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
      obj = obj.setStateFrame(state_frame);
      obj = obj.setOutputFrame(state_frame);

      input_frame = AtlasInput(obj);
      obj = obj.setInputFrame(input_frame);
    end

    function [u,obj] = inverseDynamics(obj,q,qdot,qddot_des,active_supports)
      if isempty(obj.inverse_dyn_qp_controller)
        ctrl_data = SharedDataHandle(struct(...
          'A',zeros(4),...
          'B',zeros(4,2),...
          'C',zeros(2,4),...
          'D',zeros(2),...
          'Qy',zeros(2),...
          'R',zeros(2),...
          'is_time_varying',false,...
          'S',zeros(4),...
          's1',zeros(4,1),...
          's2',0,...
          'x0',zeros(4,1),...
          'u0',zeros(2,1),...
          'trans_drift',zeros(3,1),...
          'support_times',0,...
          'supports',active_supports,...
          'mu',1,...
          'ignore_terrain',true,...
          'y0',zeros(2,1)));

        % instantiate QP controller
        options.slack_limit = 30.0;
        options.w = 0.01;
        options.R = 1e-12*eye(getNumInputs(obj));
        options.lcm_foot_contacts = false;
        options.debug = false;
        options.use_mex = 1;

        obj.inverse_dyn_qp_controller = QPControlBlock(obj,ctrl_data,options);
      else
        ctrl_data = obj.inverse_dyn_qp_controller.controller_data;
        setField(ctrl_data,'supports',active_supports);
      end

      u = obj.inverse_dyn_qp_controller.mimoOutput(0,[],qddot_des,zeros(12,1),[q;qdot]);
    end

    function z = getPelvisHeightAboveFeet(obj,q)
      kinsol = doKinematics(obj,q);
      foot_z = getFootHeight(obj,q);
      pelvis = forwardKin(obj,kinsol,findLinkInd(obj,'pelvis'),[0;0;0]);
      z = pelvis(3) - foot_z;
    end

    function foot_z = getFootHeight(obj,q)
      kinsol = doKinematics(obj,q);
      rfoot_cpos = terrainContactPointsInWorld(obj,kinsol,obj.foot_bodies_idx.right);
      lfoot_cpos = terrainContactPointsInWorld(obj,kinsol,obj.foot_bodies_idx.left);
>>>>>>> Re-update files that called contactPositions
      foot_z = min(mean(rfoot_cpos(3,:)),mean(lfoot_cpos(3,:)));
    end

    function [zmin,zmax] = getPelvisHeightLimits(obj,q) % for BDI manip mode
      z_above_feet = getPelvisHeightAboveFeet(obj,q);
      zmin = q(3) - (z_above_feet-obj.pelvis_min_height);
      zmax = q(3) + (obj.pelvis_max_height-z_above_feet);
    end

    function xstar = loadFixedPoint(obj)
      d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
      xstar = d.xstar;
    end
  end
  properties (SetAccess = protected, GetAccess = public)
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
                                      'max_upward_step', 0.2,... % m
                                      'max_downward_step', 0.2); % m
  end
end
