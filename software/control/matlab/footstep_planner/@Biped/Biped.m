classdef Biped < TimeSteppingRigidBodyManipulator
  properties
    max_forward_step
    nom_forward_step
    max_backward_step
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
      defaults = struct('nom_forward_step', 0.25,... %m
        'max_forward_step', 0.5,...%m
        'max_backward_step', 0.20,...%m
        'max_step_width', 0.35,...%m
        'max_step_dz', 0.3,...%m
        'min_step_width', 0.2,...%m
        'nom_step_width', 0.26,...%m (nominal step width)
        'nom_step_clearance', 0.05,...%m
        'max_step_rot', pi/6,... % rad
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
      obj.foot_contact_offsets = obj.findContactOffsets();
      obj.foot_bodies = struct('right', findLink(obj, obj.r_foot_name),...
                            'left', findLink(obj, obj.l_foot_name));
      obj.foot_bodies_idx = [findLinkInd(obj, obj.r_foot_name),findLinkInd(obj, obj.l_foot_name)];
    end
    
    function X = planFootsteps(obj, x0, navgoal, options)
      planner = FootstepPlanner(obj);
      X = planner.plan(navgoal, struct('x0', x0, 'plan_con', [], 'plan_commit', [], 'plan_reject', [], 'utime', 0));
    end

    function Xo = stepCenter2FootCenter(obj, Xc, is_right_foot)
      if is_right_foot
        offs = [0; -obj.nom_step_width/2; 0];
      else
        offs = [0; obj.nom_step_width/2; 0];
      end
      for j = 1:length(Xc(1,:))
        M = makehgtform('xrotate', Xc(4, j), 'yrotate', Xc(5, j), 'zrotate', Xc(6, j));
        d = M * [offs; 1];
        Xo(:,j) = [Xc(1:3,j) + d(1:3); Xc(4:end, j)];
      end
    end

    function Xc = footCenter2StepCenter(obj, Xo, is_right_foot)
      if is_right_foot
        offs = [0; -obj.nom_step_width/2; 0];
      else
        offs = [0; obj.nom_step_width/2; 0];
      end
      for j = 1:length(Xo(1,:))
        M = makehgtform('xrotate', Xo(4, j), 'yrotate', Xo(5, j), 'zrotate', Xo(6, j));
        d = M * [offs; 1];
        Xc(:,j) = [Xo(1:3,j) - d(1:3); Xo(4:end, j)];
      end
    end

    % function [pos, width] = feetPosition(obj, q0)
    function foot_orig = feetPosition(obj, q0)
      typecheck(q0,'numeric');
      sizecheck(q0,[obj.getNumDOF,1]);

      kinsol = doKinematics(obj,q0);

      rfoot0 = forwardKin(obj,kinsol,obj.foot_bodies.right,[0;0;0],true);
      lfoot0 = forwardKin(obj,kinsol,obj.foot_bodies.left,[0;0;0],true);

      foot_orig = struct('right', rfoot0, 'left', lfoot0);
    end

    function ndx = getStepNdx(obj, total_steps)
      % Lead with the right foot, for no particular reason
      ndx = struct('right', int32([1, 2, 4:2:(total_steps-1), total_steps]),...
                   'left', int32([1:2:(total_steps-1), total_steps]));
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

      msg = FootstepPlanPublisher.encodeFootstepPlan(X, t, isnew, options);
      obj.lc.publish('CANDIDATE_FOOTSTEP_PLAN', msg);
    end
  end
end

      
      
