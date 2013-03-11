classdef Biped < TimeSteppingRigidBodyManipulator
  properties
    step_time
    max_step_length
    max_step_rot
    r_foot_name
    l_foot_name
    foot_angles
    step_width
  end
  
  methods
    function obj = Biped(urdf,dt,options)
      obj = obj@TimeSteppingRigidBodyManipulator(urdf,dt,options);
      
      if nargin < 2
        options = struct();
      end
      defaults = struct('step_time', 2,... % s
        'max_step_length', .4,... % m
        'max_step_rot', pi/8,... % rad
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
      [Xright, Xleft] = obj.optimizeFreeFootsteps([start_pos, poses], options.interactive);

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
    
    function [Xright, Xleft] = stepLocations(obj, X, ndx_r, ndx_l)
      if nargin == 2
        ndx_r = 1:length(X(1,:));
        ndx_l = 1:length(X(1,:));
      end
      yaw = X(6,:);
      foot_angle_r = obj.foot_angles(1) + yaw(ndx_r);
      foot_angle_l = obj.foot_angles(2) + yaw(ndx_l);
      Xright = X(:,ndx_r) + [cos(foot_angle_r); sin(foot_angle_r); zeros(4, length(ndx_r))] .* (obj.step_width / 2);
      Xleft = X(:,ndx_l) + [cos(foot_angle_l); sin(foot_angle_l); zeros(4, length(ndx_l))] .* (obj.step_width / 2);
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

      rfootpos = [rfoot0, rfoot0];
      lfootpos = [lfoot0, lfoot0];

      p0 = feetCenter(rfoot0, lfoot0);
      pos = [p0; 0; 0; 0; atan2(lfoot0(2) - rfoot0(2), lfoot0(1) - rfoot0(1)) - pi/2];
      width = sqrt(sum((rfoot0 - lfoot0) .^ 2));
    end
  end
end

      
      