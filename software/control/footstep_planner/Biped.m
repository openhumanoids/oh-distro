classdef Biped
  properties
    manip
    visualizer
    step_time
    max_step_length
    max_step_rot
    r_foot_name
    l_foot_name
    foot_angles
    step_width
  end
  
  methods
    function obj = Biped(manip, options)
      obj.manip = manip;
      obj.visualizer = obj.manip.constructVisualizer();
      if nargin < 2
        options = struct();
      end
      defaults = struct('step_time', 3,... % s
        'max_step_length', .3,... % m
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
      [start_pos, obj.step_width] = getFeetPos(obj.manip, q0);
      
      if strcmp(options.traj_type, 'turn_and_go')
        sizecheck(poses(:,1), 6);
        traj = turnGoTraj([start_pos, poses]);
      elseif strcmp(options.traj_type, 'cubic_spline')
        sizecheck(poses, [6,1]);
        traj = cubicSplineTraj([start_pos, poses]);
      else
        error('Invalid trajectory type specified: %s', options.traj_type);
      end
      
%       [lambda, ndx_r, ndx_l] = constrainedFootsteps(traj, obj.max_step_length,...
%         obj.step_width, obj.max_step_rot);
%       [Xright, Xleft] = obj.footPositions(traj.eval(lambda), ndx_r, ndx_l);
%       if options.plotting
%         figure(21)
%         plotFootstepPlan(traj, Xright, Xleft);
%         drawnow
%       end
       lambda = [0,0.5,1];
%       plot_lcm_poses(Xright(1:3,:)', Xright([6,5,4],:)', 1, 'Foot Steps (right)', 4, 1, 0, -1);
%       plot_lcm_poses(Xleft(1:3,:)', Xright([6,5,4],:)', 2, 'Foot Steps (left)', 4, 1, 0, -1);

      [Xright, Xleft] = optimizeFreeFootsteps(traj, lambda, [start_pos, poses], obj, options.interactive);
%       if options.interactive
% %         [~, Xright, Xleft] = interactiveFootstepOptimization(traj,lambda,obj.max_step_length,obj.step_width,obj.max_step_rot,ndx_r,ndx_l);
%          [Xright, Xleft] = interactiveFreeFootstepOptimization(traj, lambda, obj, ndx_r, ndx_l);
%       else
% %         [~, Xright, Xleft] = optimizeFootsteps(traj, lambda, obj.max_step_length, step_width, obj.max_step_rot, ndx_r, ndx_l);
%         
%         X = traj.eval(lambda(1:end));
%         [~, Xright, Xleft] = optimizeFreeFootsteps(X, obj, ndx_r, ndx_l);
%       end
      if options.plotting
        figure(22)
        plotFootstepPlan(traj, Xright, Xleft);
        drawnow
      end
    end
    
    function [xtraj, ts] = roughWalkingPlanFromSteps(obj, x0, Xright, Xleft)
      q0 = x0(1:end/2);
      [zmptraj, lfoottraj, rfoottraj, ts] = planZMPandFootTrajectory(obj.manip, q0, Xright, Xleft, obj.step_time);
      xtraj = computeZMPPlan(obj.manip, obj.visualizer, x0, zmptraj, lfoottraj, rfoottraj, ts);
    end
    
    function [xtraj, ts] = walkingPlanFromSteps(obj, x0, Xright, Xleft)
      q0 = x0(1:end/2);
      [zmptraj, lfoottraj, rfoottraj] = planZMPandFootTrajectory(obj.manip, q0, Xright, Xleft, obj.step_time);
      ts = zmptraj.tspan(1):0.05:zmptraj.tspan(end);
      xtraj = computeZMPPlan(obj.manip, obj.visualizer, x0, zmptraj, lfoottraj, rfoottraj, ts);
    end
    
    function [xtraj, ts] = walkingPlan(obj, x0, poses, options)
      if nargin < 4
        options = struct();
      end
      [Xright, Xleft] = planFootsteps(obj, x0, poses, options);
      [xtraj, ts] = walkingPlanFromSteps(obj, x0, Xright, Xleft);
    end
    
    function [xtraj, ts] = roughWalkingPlan(obj, x0, poses, options)
      if nargin < 4
        options = struct();
      end
      [Xright, Xleft] = planFootsteps(obj, x0, poses, options);
      [xtraj, ts] = roughWalkingPlanFromSteps(obj, x0, Xright, Xleft);
    end
    
    function [Xright, Xleft] = footPositions(obj, X, ndx_r, ndx_l)
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

  end
end

      
      