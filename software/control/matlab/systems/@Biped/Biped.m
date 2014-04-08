classdef Biped < TimeSteppingRigidBodyManipulator
  properties
    max_forward_step
    nom_forward_step
    max_step_dz
    max_step_width
    min_step_width
    nom_step_width
    nom_step_clearance
    max_step_rot
    foot_contact_offsets
    terrain_step_threshold
    r_foot_name
    l_foot_name
    foot_bodies
    foot_bodies_idx
    next_step_id
    lc
  end

  methods
    function obj = Biped(urdf,dt,options)
      if nargin < 3
        options = struct();
        options.floating = true;
      end
      if nargin < 2
        dt = 0.002;
      end
      obj = obj@TimeSteppingRigidBodyManipulator(urdf,dt,options);
      defaults = struct('nom_forward_step', 0.15,... %m
        'max_forward_step', 0.3,...%m
        'max_step_width', 0.40,...%m
        'max_step_dz', 0.4,...%m
        'min_step_width', 0.21,...%m
        'nom_step_width', 0.28,...%m (nominal step width)
        'nom_step_clearance', 0.04,...%m
        'max_step_rot', pi/8,... % rad
        'r_foot_name', 'r_foot',...
        'terrain_step_threshold', 0.03,...%m
        'l_foot_name', 'l_foot');
      fields = fieldnames(defaults);
      for i = 1:length(fields)
        if ~isfield(options, fields{i})
          obj.(fields{i}) = defaults.(fields{i});
        else
          obj.(fields{i}) = options.(fields{i});
        end
      end
      obj.next_step_id = 0;
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.foot_bodies = struct('right', getBody(obj, findLinkInd(obj, obj.r_foot_name)),...
                            'left', getBody(obj, findLinkInd(obj, obj.l_foot_name)));
      obj.foot_contact_offsets = obj.findContactOffsets();
      obj.foot_bodies_idx = struct('right', findLinkInd(obj, obj.r_foot_name), 'left', findLinkInd(obj, obj.l_foot_name));
    end

    function X = planFootsteps(obj, x0, navgoal, options)
      planner = FootstepPlanner(obj);
      X = planner.plan(navgoal, struct('x0', x0, 'plan_con', [], 'plan_commit', [], 'plan_reject', [], 'utime', 0));
    end

    % function [pos, width] = feetPosition(obj, q0)
    function foot_orig = feetPosition(obj, q0)
      typecheck(q0,'numeric');
      sizecheck(q0,[obj.getNumDOF,1]);

      kinsol = doKinematics(obj,q0);

      rfoot0 = forwardKin(obj,kinsol,obj.foot_bodies_idx.right,[0;0;0],true);
      lfoot0 = forwardKin(obj,kinsol,obj.foot_bodies_idx.left,[0;0;0],true);

      foot_orig = struct('right', rfoot0, 'left', lfoot0);
    end

    function apex_pos = findApexPos(obj, last_pos, next_pos, apex_height)
      next_pos = last_pos + angleDiff(last_pos, next_pos);
      apex_pos = mean([last_pos, next_pos], 2);
%       apex_pos(4:6) = mean(unwrap([last_pos(4:6), next_pos(4:6)], [], 2), 2);
      if nargin < 4
        apex_height = obj.nom_step_clearance;
      end
      apex_pos(3) = max([last_pos(3), next_pos(3)]) + apex_height;
    end

    function id = getNextStepID(obj, reset)
      persistent next_id
      if (nargin < 2)
        reset = false;
      end
      if (isempty(next_id) || reset)
        next_id = 0;
      end
      id = next_id;
      next_id = next_id + 1;
    end

    function publish_footstep_plan(obj, X, t, isnew, options)
      if nargin < 4
        isnew = true;
      end
      if nargin < 3
        t = now() * 24 * 60 * 60;
      end
      if ~isfield(options, 'channel')
        options.channel = 'CANDIDATE_FOOTSTEP_PLAN';
      end
      if length(X) > 0
        msg = FootstepPlanPublisher.encodeFootstepPlan(X, t, isnew, options);
        obj.lc.publish(options.channel, msg);
      end
    end

    function options = getReachabilityOptions(obj, options)
      if ~isfield(options, 'forward_step')
        options.forward_step = obj.max_forward_step;
      end
      if ~isfield(options, 'nom_step_width')
        options.nom_step_width = obj.nom_step_width;
      end
      if ~isfield(options, 'max_step_width')
        options.max_step_width = max([obj.max_step_width, options.nom_step_width + 0.05]);
      end
      if ~isfield(options, 'min_step_width')
        options.min_step_width = min([obj.min_step_width, options.nom_step_width - 0.05]);
      end
      if ~isfield(options, 'backward_step')
        options.backward_step = options.forward_step;
      end
      if ~isfield(options, 'max_step_rot')
        options.max_outward_step_rot = obj.max_step_rot;
      end
      if ~isfield(options, 'max_inward_step_rot')
        options.max_inward_step_rot = 0.00; % BDI walker doesn't allow toe to rotate inward at all
      end
      if ~isfield(options, 'max_step_dz')
        options.max_step_dz = obj.max_step_dz;
      end
    end

    function [A, b] = getFootstepDiamondCons(obj, p0_is_right_foot, options)
      % Alternative (experimental) formulation of footstep linear constraints, in which the reachable region is defined by a diamond which extends out to the max forward and backward distances only at the nominal step width, rather than a rectangle defined by the max forward/back step and the max/min step width.

      if nargin < 3
        options = struct();
      end

      options = getReachabilityOptions(obj, options);
      options.forward_step = max([options.forward_step, 0.005]);
      options.backward_step = max([options.backward_step, 0.005]);
      options.max_step_width = max([options.max_step_width, options.nom_step_width + 0.005]);
      options.min_step_width = min([options.min_step_width, options.nom_step_width - 0.005]);

      [Axy, bxy] = poly2lincon([0, options.forward_step, 0, -options.backward_step], [options.min_step_width, options.nom_step_width, options.max_step_width, options.nom_step_width]);
      A = [Axy, zeros(4, 4);
           0 0 1 0 0 0;
           0 0 -1 0 0 0;
           0 0 0 0 0 -1;
           0 0 0 0 0 1;
           0, 1/(options.max_step_width-options.nom_step_width), 0, 0, 0, 1/options.max_outward_step_rot;
           0, 1/(options.min_step_width-options.nom_step_width), 0, 0, 0, 1/options.max_outward_step_rot];
      if ~p0_is_right_foot
        A(:,2) = -A(:,2);
        A(:,6) = -A(:,6);
      end
      b = [bxy;
           options.max_step_dz;
           options.max_step_dz;
           options.max_inward_step_rot;
           options.max_outward_step_rot;
           1 + options.nom_step_width/(options.max_step_width-options.nom_step_width);
           1 + options.nom_step_width/(options.min_step_width-options.nom_step_width)
           ];
   end

    function [A, b] = getFootstepLinearCons(obj, p0_is_right_foot, options)
      % Get the linear inequality constraints for Ax - b <= 0, where x is a column of relative step positions, as given by Biped.relativeSteps(). Automatically flips the y direction for left steps to make them equivalent to right steps.

      if nargin < 3
        options = struct();
      end
      options = getReachabilityOptions(obj, options);
      options.max_inward_step_rot = 0.1;

      A = [1 0 0 0 0 0;
           -1 0 0 0 0 0;
           0 1 0 0 0 0;
           0 -1 0 0 0 0;
           0 0 1 0 0 0;
           0 0 -1 0 0 0;
           0 0 0 0 0 -1;
           0 0 0 0 0 1];
      if ~p0_is_right_foot
        A(:,2) = -A(:,2);
        A(:,6) = -A(:,6);
      end
      b = [options.forward_step;
           options.backward_step;
           options.max_step_width;
           -options.min_step_width;
           options.max_step_dz;
           options.max_step_dz;
           options.max_inward_step_rot;
           options.max_outward_step_rot];
    end

    function [A, b] = getXYFootstepLinearCons(obj, p0_is_right_foot, options)
      % Like getFootstepLinearCons, but only considers x and y

      if nargin < 3
        options = struct();
      end
      A = [1 0;
           -1 0;
           0 1;
           0 -1];
      if ~p0_is_right_foot
        A(:,2) = -A(:,2);
      end
      if ~isfield(options, 'forward_step')
        options.forward_step = obj.max_forward_step;
      end
      if ~isfield(options, 'nom_step_width')
        options.nom_step_width = obj.nom_step_width;
      end
      if ~isfield(options, 'max_step_width')
        options.max_step_width = max([obj.max_step_width, options.nom_step_width + 0.01]);
      end
      if ~isfield(options, 'min_step_width')
        options.min_step_width = min([obj.min_step_width, options.nom_step_width - 0.01]);
      end
      if ~isfield(options, 'backward_step')
        options.backward_step = options.forward_step;
      end
      b = [options.forward_step;
           options.backward_step;
           options.max_step_width;
           -options.min_step_width];
    end
  end

  methods (Static)
    function u = relativeSteps(p0, pf)
      % For each final step in pf, compute its offset from the foot position given by p0.

      sizecheck(p0, [6, 1]);
      sizecheck(pf, [6, nan]);
      sn = sin(-p0(6));
      cs = cos(-p0(6));
      M = [cs,-sn,0; sn,cs,0; 0,0,1];
      u = M * bsxfun(@minus, pf(1:3,:), p0(1:3,:));
      u = [u; zeros(2, size(u,2)); bsxfun(@angleDiff, pf(6,:), p0(6))];
    end

    function Xo = stepCenter2FootCenter(Xc, is_right_foot, nom_step_width)
      Xo = Biped.footCenter2StepCenter(Xc, is_right_foot, -nom_step_width);
    end

    function Xc = footCenter2StepCenter( Xo, is_right_foot, nom_step_width)
      % Convert a position of the center of one of the biped's feet to the
      % corresponding point half the step width toward the bot's center.
      % nom_step_width should be scalar or vector of size(1, size(Xo,2))
      if length(nom_step_width) == 1
        nom_step_width = repmat(nom_step_width, 1, size(Xo, 2));
      end
      if is_right_foot
        offs = [zeros(1,length(nom_step_width)); -nom_step_width/2; zeros(1,length(nom_step_width))];
      else
        offs = [zeros(1,length(nom_step_width)); nom_step_width/2; zeros(1,length(nom_step_width))];
      end
      for j = 1:length(Xo(1,:))
        M = rpy2rotmat(Xo(4:6,j));
        d = M * offs(:,j);
        Xc(:,j) = [Xo(1:3,j) - d(1:3); Xo(4:end,j)];
      end
    end
  end
end
