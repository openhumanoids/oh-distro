classdef Biped < TimeSteppingRigidBodyManipulator
  properties
    step_time
    max_step_length
    max_step_rot
    min_foot_proximity
    foot_contact_offsets
    r_foot_name
    l_foot_name
    step_width
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
      defaults = struct('step_time', 1.1,... % s
        'max_step_length', .60,... % m
        'max_step_rot', pi/4,... % rad
        'min_foot_proximity', 0.15,... % m
        'r_foot_name', 'r_foot',...
        'step_width', 0.22,...
        'l_foot_name', 'l_foot');
      fields = fieldnames(defaults);
      for i = 1:length(fields)
        if ~isfield(options, fields{i})
          obj.(fields{i}) = defaults.(fields{i});
        else
          obj.(fields{i}) = options.(fields{i});
        end
      end
      

      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.foot_contact_offsets = obj.findContactOffsets();
    end
    
    function X = planFootsteps(obj, x0, navgoal, options)
      planner = FootstepPlanner(obj);
      X = planner.plan(navgoal, struct('x0', x0, 'plan_con', [], 'plan_commit', [], 'plan_reject', [], 'utime', 0));
    end
%     function [xtraj, ts] = walkingPlanFromSteps(obj, x0, X, options)
%       Xpos = [X.pos];
%       Xright = Xpos(:, [X.is_right_foot] == 1);
%       Xleft = Xpos(:, [X.is_right_foot] == 0);
%       q0 = x0(1:end/2);
%       [zmptraj, foottraj] = planZMPandHeelToeTrajectory(obj, q0, Xright, Xleft, obj.step_time, options);
%       ts = zmptraj.tspan(1):0.05:zmptraj.tspan(end);
%       xtraj = computeHeelToeZMPPlan(obj, x0, zmptraj, foottraj, ts);
%     end
    function [xtraj, qtraj, htraj, V, ts] = walkingPlan(obj, x0, qstar, navgoal, options)
      if nargin < 5
        options = struct();
      end
      X = obj.planFootsteps(x0, navgoal, options);
      [xtraj, qtraj, htraj, V, ts] = obj.walkingPlanFromSteps(x0, qstar, X);
    end

%     function [xtraj, ts] = walkingPlan(obj, x0, poses, options)
%       if nargin < 4
%         options = struct();
%       end
%       defaults = struct('flat_foot', true, 'interactive', true, 'plotting', true);
%       fields = fieldnames(defaults);
%       for i = 1:length(fields)
%         if ~isfield(options, fields{i})
%           options.(fields{i}) = defaults.(fields{i});
%         end
%       end
%       if ~options.flat_foot
%         obj.max_step_length = 0.6;
%       end
%       X = planFootsteps(obj, x0, poses, options);
%       [xtraj, ts] = walkingPlanFromSteps(obj, x0, X, options);
%     end

    function Xo = stepCenter2FootCenter(obj, Xc, is_right_foot)
      if is_right_foot
        offs = [0; -obj.step_width/2; 0];
      else
        offs = [0; obj.step_width/2; 0];
      end
      for j = 1:length(Xc(1,:))
        M = makehgtform('xrotate', Xc(4, j), 'yrotate', Xc(5, j), 'zrotate', Xc(6, j));
        d = M * [offs; 1];
        Xo(:,j) = [Xc(1:3,j) + d(1:3); Xc(4:end, j)];
      end
    end

    function Xc = footCenter2StepCenter(obj, Xo, is_right_foot)
      if is_right_foot
        offs = [0; -obj.step_width/2; 0];
      else
        offs = [0; obj.step_width/2; 0];
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
      rfoot_body = findLink(obj,obj.r_foot_name);
      lfoot_body = findLink(obj,obj.l_foot_name);

      rfoot0 = forwardKin(obj,kinsol,rfoot_body,[0;0;0],true);
      lfoot0 = forwardKin(obj,kinsol,lfoot_body,[0;0;0],true);

      foot_orig = struct('right', rfoot0, 'left', lfoot0);
    end

    function ndx = getStepNdx(obj, total_steps)
      % Lead with the right foot, for no particular reason
      ndx = struct('right', int32([1, 2, 4:2:(total_steps-1), total_steps]),...
                   'left', int32([1:2:(total_steps-1), total_steps]));
    end

    function t = getStepTimes(obj, X)
      % Assume the columns of X are already in order by time
      nsteps = length(X(1,:));
      t = zeros(1, nsteps);
      t(3:end) = (1:nsteps-2) * obj.step_time;
    end

    function id = getNextStepID(obj)
      persistent counter
      if isempty(counter)
        counter = 0;
      end
      counter = counter + 1;
      id = counter;
    end

    function publish_footstep_plan(obj, X, htfun, t, isnew)
      if nargin < 4
        isnew = true;
      end
      if nargin < 3
        t = now() * 24 * 60 * 60;
      end
      % ndx = obj.getStepNdx(length(X(1,:)));
      % [Xright, Xleft] = obj.stepGoals(X, ndx.right, ndx.left);
      % Xright = obj.stepCenter2FootCenter(X(:, ndx.right), 1);
      % Xleft = obj.stepCenter2FootCenter(X(:, ndx.left), 0);
      % Xright(end+1, :) = 1;
      % Xleft(end+1, :) = 0;

      % Xright(3,:) = htfun(Xright(1:2,:));
      % Xleft(3,:) = htfun(Xleft(1:2,:));


      msg = FootstepPlanPublisher.encodeFootstepPlan(X, t, isnew);
      obj.lc.publish('CANDIDATE_FOOTSTEP_PLAN', msg);
    end
  end
end

      
      