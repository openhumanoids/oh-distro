classdef Biped < TimeSteppingRigidBodyManipulator
  properties
    step_time
    max_step_length
    max_step_rot
    r_foot_name
    l_foot_name
    foot_angles
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

      obj.lc = lcm.lcm.LCM.getSingleton();
      
      defaults = struct('step_time', 1.0,... % s
        'max_step_length', .55,... % m
        'max_step_rot', pi/4,... % rad
        'r_foot_name', 'r_foot',...
        'l_foot_name', 'l_foot',...
        'foot_angles', [-pi/2, pi/2]);
      fields = fieldnames(defaults);
      for i = 1:length(fields)
        if ~isfield(options, fields{i})
          obj.(fields{i}) = defaults.(fields{i});
        else
          obj.(fields{i}) = options.(fields{i});
        end
      end
    end
    
    function [Xright, Xleft] = planFootsteps(obj, x0, poses, options)
      q0 = x0(1:end/2);
      [start_pos, obj.step_width] = obj.feetPosition(q0);
      % [Xright, Xleft] = obj.optimizeFreeFootsteps([start_pos, poses], options.interactive);

      if ~options.interactive
        [Xright, Xleft] = obj.optimizeFootstepPlan([start_pos, poses]);
        return;
      end 

      planner = FootstepPlanner(obj)
      [Xright, Xleft] = planner.plan(poses(:,end), struct('x0', x0, 'plan_con', [], 'plan_commit', [], 'plan_reject', [], 'utime', 0));
      if options.plotting
        figure(22)
        plotFootstepPlan([], Xright, Xleft);
        drawnow
      end
    end
    function [xtraj, ts] = walkingPlanFromSteps(obj, x0, Xright, Xleft, options)
      q0 = x0(1:end/2);
      [zmptraj, foottraj, contact_ref] = planZMPandHeelToeTrajectory(obj, q0, Xright, Xleft, obj.step_time, options);
      ts = zmptraj.tspan(1):0.05:zmptraj.tspan(end);
      xtraj = computeHeelToeZMPPlan(obj, x0, zmptraj, foottraj, contact_ref, ts);
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
    
    function [Xright, Xleft] = stepGoals(obj, X, ndx_r, ndx_l)
      if nargin == 2
        ndx_r = 1:length(X(1,:));
        ndx_l = 1:length(X(1,:));
      end
      yaw = X(6,:);
      foot_angle_r = obj.foot_angles(1) + yaw(ndx_r);
      foot_angle_l = obj.foot_angles(2) + yaw(ndx_l);
      Xright = X(:,ndx_r) + [cos(foot_angle_r); sin(foot_angle_r); zeros(12, length(ndx_r))] .* (obj.step_width / 2);
      Xleft = X(:,ndx_l) + [cos(foot_angle_l); sin(foot_angle_l); zeros(12, length(ndx_l))] .* (obj.step_width / 2);
      Xright(end+1,:) = 1;
      Xleft(end+1,:) = 0;
    end
    
    function [Xright, Xleft] = stepLocations(obj, X, ndx_r, ndx_l)
      % Return left and right foot poses only
      if nargin == 2
        ndx_r = 1:length(X(1,:));
        ndx_l = 1:length(X(1,:));
      end
      [Xright, Xleft] = obj.stepGoals(X, ndx_r, ndx_l);
      Xright = Xright(1:6,:);
      Xleft = Xleft(1:6,:);
    end

    function [X] = stepCenters(obj, Xfoot, is_right_foot)
      % Transform foot position into position of the footstep path center. Reverses biped.stepLocations
      yaw = Xfoot(6,:);
      if is_right_foot
        foot_angle = obj.foot_angles(1) + yaw;
      else
        foot_angle = obj.foot_angles(2) + yaw;
      end
      X = Xfoot - [cos(foot_angle); sin(foot_angle); zeros(4, length(Xfoot(1,:)))] .* (obj.step_width / 2);
    end
    
    function [pos, width] = feetPosition(obj, q0)
      typecheck(q0,'numeric');
      sizecheck(q0,[obj.getNumDOF,1]);

      kinsol = doKinematics(obj,q0);
      rfoot_body = findLink(obj,obj.r_foot_name);
      lfoot_body = findLink(obj,obj.l_foot_name);

      rfoot0 = forwardKin(obj,kinsol,rfoot_body,[0;0;0],true);
      lfoot0 = forwardKin(obj,kinsol,lfoot_body,[0;0;0],true);

      gc = obj.contactPositions(q0);

      % compute desired COM projection
      % assumes minimal contact model for now
      k = convhull(gc(1:2,1:4)');
      lfootcen0 = [mean(gc(1:2,k),2);0];
      k = convhull(gc(1:2,5:8)');
      rfootcen0 = [mean(gc(1:2,4+k),2);0];
      roffset = rfootcen0 - rfoot0(1:3);
      loffset = lfootcen0 - lfoot0(1:3);

      function pos = rfootCenter(rfootpos)
        yaw = rfootpos(6);
        offset = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)] * roffset(1:2);
        pos = rfootpos(1:2)+offset;
      end    

      function pos = lfootCenter(lfootpos)
        yaw = lfootpos(6);
        offset = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)] * loffset(1:2);
        pos = lfootpos(1:2)+offset;
      end

      function pos = feetCenter(rfootpos,lfootpos)
        rcen = rfootCenter(rfootpos);
        lcen = lfootCenter(lfootpos);
        pos = mean([rcen,lcen],2);
      end

      p0 = feetCenter(rfoot0, lfoot0);
      pos = [p0; 0; 0; 0; atan2(lfoot0(2) - rfoot0(2), lfoot0(1) - rfoot0(1)) - pi/2];
      width = sqrt(sum((rfoot0 - lfoot0) .^ 2));
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
      [Xright, Xleft] = obj.stepGoals(X, ndx.right, ndx.left);
      Xright(3,:) = htfun(Xright(1:2,:));
      Xleft(3,:) = htfun(Xleft(1:2,:));

      msg = FootstepPlanPublisher.encodeFootstepPlan([Xright, Xleft], t, isnew);
      % figure(22);
      % plotFootstepPlan([], Xright, Xleft)
      % drawnow
      obj.lc.publish('CANDIDATE_FOOTSTEP_PLAN', msg);
    end
  end
end

      
      