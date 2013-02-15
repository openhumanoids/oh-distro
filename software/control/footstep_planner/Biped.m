classdef Biped
  properties
    manip
    step_time
    max_step_length
    max_step_rot
    r_foot_name
    l_foot_name
  end
  
  methods
    function obj = Biped(manip, options)
      obj.manip = manip;
      if nargin < 2
        options = struct();
      end
      defaults = struct('step_time', 3,... % s
        'max_step_length', .3,... % m
        'max_step_rot', pi/8,... % rad
        'r_foot_name', 'r_foot',...
        'l_foot_name', 'l_foot');
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
      defaults = struct('traj_type', 'turn_and_go',...
        'interactive', false,...
        'plotting', true);
      if nargin < 3
        options = struct();
      end
      fields = fieldnames(defaults);
      for i = 1:length(fields)
        if ~isfield(options, fields{i})
          options.(fields{i}) = defaults.(fields{i});
        end
      end
      q0 = x0(1:end/2);
      v = obj.manip.constructVisualizer();
      [start_pos, step_width] = getFeetPos(obj.manip, q0);
      
      if strcmp(options.traj_type, 'turn_and_go')
        sizecheck(poses(:,1), [6]);
        traj = turnGoTraj([start_pos, poses]);
      else
        sizecheck(poses, [6,1]);
        traj = cubicSplineTraj([start_pos, poses]);
      end
      
      [lambda, ndx_r, ndx_l] = constrainedFootsteps(traj, obj.max_step_length,...
        step_width, obj.max_step_rot);
      if options.plotting
        figure(21)
        Xright = footstepLocations(traj, lambda(ndx_r), -pi/2, step_width);
        Xleft = footstepLocations(traj, lambda(ndx_l), pi/2, step_width);
        plotFootstepPlan(traj, Xright, Xleft);
        drawnow
      end
      
      if options.interactive
        [~, Xright, Xleft] = interactiveFootstepOptimization(traj,lambda,obj.max_step_length,step_width,obj.max_step_rot,ndx_r,ndx_l);
      else
        [~, Xright, Xleft] = optimizeFootsteps(traj, lambda, obj.max_step_length, step_width, obj.max_step_rot, ndx_r, ndx_l);
        if options.plotting
          figure(22)
          plotFootstepPlan(traj, Xright, Xleft);
          drawnow
        end
      end
    end
    
    function xtraj = roughWalkingPlanFromSteps(obj, x0, Xright, Xleft)
      v = obj.manip.constructVisualizer();
      q0 = x0(1:end/2);
      [zmptraj, lfoottraj, rfoottraj, ts] = planZMPandFootTrajectory(obj.manip, q0, Xright, Xleft, obj.step_time);
      xtraj = computeZMPPlan(obj.manip, v, x0, zmptraj, lfoottraj, rfoottraj, ts);
    end
    
    function xtraj = walkingPlanFromSteps(obj, x0, Xright, Xleft)
      q0 = x0(1:end/2);
      [zmptraj, lfoottraj, rfoottraj] = planZMPandFootTrajectory(obj.manip, q0, Xright, Xleft, obj.step_time);
      ts = zmptraj.tspan(1):0.05:zmptraj.tspan(end);
      v = obj.manip.constructVisualizer();
      xtraj = computeZMPPlan(obj.manip, v, x0, zmptraj, lfoottraj, rfoottraj, ts);
    end
    
    function xtraj = walkingPlan(obj, x0, poses, options)
      if nargin < 4
        options = struct();
      end
      [Xright, Xleft] = planFootsteps(obj, x0, poses, options);
      xtraj = walkingPlanFromSteps(obj, x0, Xright, Xleft);
    end
    
    function xtraj = roughWalkingPlan(obj, x0, poses, options)
      if nargin < 4
        options = struct();
      end
      [Xright, Xleft] = planFootsteps(obj, x0, poses, options);
      xtraj = roughWalkingPlanFromSteps(obj, x0, Xright, Xleft);
    end
  end
end

      
      