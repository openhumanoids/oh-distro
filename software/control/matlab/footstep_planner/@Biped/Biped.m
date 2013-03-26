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
      obj = obj@TimeSteppingRigidBodyManipulator(urdf,dt,options);
      if nargin < 2
        dt = 0.002;
      end
      defaults = struct('step_time', 1.0,... % s
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
    
    function [Xright, Xleft] = planFootsteps(obj, x0, poses, options)
      % [Xright, Xleft] = obj.optimizeFreeFootsteps([start_pos, poses], options.interactive);

      if ~options.interactive
        [Xright, Xleft] = obj.optimizeFootstepPlan(x0, poses);
        return;
      end 

      planner = FootstepPlanner(obj)
      [Xright, Xleft] = planner.plan(poses, struct('x0', x0, 'plan_con', [], 'plan_commit', [], 'plan_reject', [], 'utime', 0));
      if options.plotting
        figure(22)
        plotFootstepPlan([], Xright, Xleft);
        drawnow
      end
    end
    function [xtraj, ts] = walkingPlanFromSteps(obj, x0, Xright, Xleft, options)
      % Xright and Xleft should be expressed as locations of the foot centers, not the foot origins
      Xright = obj.footContact2Orig(Xright, 'center', 1);
      Xleft = obj.footContact2Orig(Xleft, 'center', 0);
      q0 = x0(1:end/2);
      [zmptraj, foottraj] = planZMPandHeelToeTrajectory(obj, q0, Xright, Xleft, obj.step_time, options);
      ts = zmptraj.tspan(1):0.05:zmptraj.tspan(end);
      xtraj = computeHeelToeZMPPlan(obj, x0, zmptraj, foottraj, ts);
    end
    
    function [xtraj, ts] = walkingPlan(obj, x0, poses, options)
      if nargin < 4
        options = struct();
      end
      defaults = struct('flat_foot', true, 'interactive', true, 'plotting', true);
      fields = fieldnames(defaults);
      for i = 1:length(fields)
        if ~isfield(options, fields{i})
          options.(fields{i}) = defaults.(fields{i});
        end
      end
      if ~options.flat_foot
        obj.max_step_length = 0.6;
      end
      [Xright, Xleft] = planFootsteps(obj, x0, poses, options);
      [xtraj, ts] = walkingPlanFromSteps(obj, x0, Xright, Xleft, options);
    end

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

    function [pos, width] = feetPosition(obj, q0)
      typecheck(q0,'numeric');
      sizecheck(q0,[obj.getNumDOF,1]);

      kinsol = doKinematics(obj,q0);
      rfoot_body = findLink(obj,obj.r_foot_name);
      lfoot_body = findLink(obj,obj.l_foot_name);

      rfoot0 = forwardKin(obj,kinsol,rfoot_body,[0;0;0],true);
      lfoot0 = forwardKin(obj,kinsol,lfoot_body,[0;0;0],true);

      foot_centers = struct('right', obj.footOrig2Contact(rfoot0, 'center', 1),...
                            'left', obj.footOrig2Contact(lfoot0, 'center', 0));
      p0 = mean([foot_centers.right(1:3), foot_centers.left(1:3)], 2);
      yaw = atan2(foot_centers.left(2) - foot_centers.right(2),...
                  foot_centers.left(1) - foot_centers.right(1)) - pi/2;
      pos = [p0; 0; 0; yaw];
      width = sqrt(sum((foot_centers.right(1:2) - foot_centers.left(1:2)) .^ 2));

    end

    function ndx = getStepNdx(obj, total_steps)
      % Lead with the right foot, for no particular reason
      ndx = struct('right', int32([1, 2, 4:2:(total_steps-1), total_steps]),...
                   'left', int32([1:2:(total_steps-1), total_steps]));
    end

    function publish_footstep_plan(obj, X, htfun, t, isnew)
      if nargin < 4
        isnew = true;
      end
      if nargin < 3
        t = now() * 24 * 60 * 60;
      end
      ndx = obj.getStepNdx(length(X(1,:)));
      % [Xright, Xleft] = obj.stepGoals(X, ndx.right, ndx.left);
      Xright = obj.stepCenter2FootCenter(X(:, ndx.right), 1);
      Xleft = obj.stepCenter2FootCenter(X(:, ndx.left), 0);
      Xright(end+1, :) = 1;
      Xleft(end+1, :) = 0;

      Xright(3,:) = htfun(Xright(1:2,:));
      Xleft(3,:) = htfun(Xleft(1:2,:));

      % Xright and Xleft are expressed as position of the foot contact center, so we need to transform them into the position of the foot origin
      for j = 1:length(Xright(1,:))
        Xright(:,j) = obj.footContact2Orig(Xright(:,j), 'center', 1);
      end
      for j = 1:length(Xleft(1,:))
        Xleft(:,j) = obj.footContact2Orig(Xleft(:,j), 'center', 0);
      end

      msg = FootstepPlanPublisher.encodeFootstepPlan([Xright, Xleft], t, isnew);
      obj.lc.publish('CANDIDATE_FOOTSTEP_PLAN', msg);
    end
  end
end

      
      