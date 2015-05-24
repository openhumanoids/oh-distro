function [qtraj_rescaled, t_original_rescaled] = rescalePlanTimingGetOriginalTimes(qtraj, qd_max, t_original, varargin)
  % qtraj_rescaled = rescalePlanTiming(qtraj, qd_max, <warpTime args>, [options])
  % See warpTime for a description of <warpTime args>.
  %
  % options is a struct with fields, body_id, pts, max_v and max_theta
  %   * max_v is the maximum cartesian velocity of the body point
  %   * max_theta is the maximum degrees/second in the quaternion arc length metric
  
  % Scale timing to obey joint velocity limits
  % Create initial spline
  n_breaks = numel(qtraj.getBreaks());
  t = linspace(qtraj.tspan(1), qtraj.tspan(2),max(n_breaks, 20));
  t_original = t_original(:)'; % make it a row vector
  t = union(t,t_original);

  % idx_original is the indices of t_original in t
  [~,idx_original,~] = intersect(t,t_original);  

  q_path = eval(qtraj, t); %#ok
  t_mid = mean([t(2:end); t(1:end-1)],1);
  qd_mid = qtraj.fnder().eval(t_mid);
  scale_factor = max(abs(bsxfun(@rdivide, qd_mid, qd_max)), [], 1);

  % check to see whether options were passed in as part of varagin
  if ~isempty(varargin) && isstruct(varargin{end})
    options = varargin{end};
    num_bodies = length(options.body_id);
    if num_bodies > 0
      body_path = zeros(3,num_bodies,length(t));
      body_v_max = options.max_v;
      body_v_max = body_v_max(:);
      body_quat = zeros(4,num_bodies,length(t));
      robot = options.robot;
      for j = 1:length(t)
        kinsol = robot.doKinematics(q_path(:,j));
        for k = 1:num_bodies
          body_pos = robot.forwardKin(kinsol,options.body_id(k),options.pts(:,k),2);
          body_path(:,k,j) = body_pos(1:3);
          body_quat(:,k,j) = body_pos(4:7);
        end
      end
      body_v_mid = zeros(num_bodies,length(t_mid));
      theta_mid = zeros(num_bodies,length(t_mid));
      theta_max = options.max_theta(:);
      for j = 1:numel(t)-1
        for k = 1:num_bodies
          body_v_mid(k,j) = norm(body_path(:,k,j) - body_path(:,k,j+1))/(t(j+1) - t(j));
          theta_mid(k,j) = quatArcDistance(body_quat(:,k,j),body_quat(:,k,j+1))/(t(j+1) - t(j));
        end
      end
      body_v_scale_factor = max(abs(bsxfun(@rdivide, body_v_mid, body_v_max)), [], 1); 
      theta_scale_factor = max(abs(bsxfun(@rdivide, theta_mid, theta_max)), [], 1);
      body_scale_factor = max(body_v_scale_factor,theta_scale_factor);
      scale_factor = max(body_scale_factor,scale_factor);
    end
    varargin = varargin(1:end-1);
  end

  % Adjust durations to keep velocity below max
  t_scaled = [0, cumsum(diff(t).*scale_factor)];
  tf = t_scaled(end);

  % Warp time to give gradual acceleration/deceleration
  t_warped = tf*warpTime(t_scaled/tf, varargin{:});
  t_original_warped = t_warped(idx_original);
  [t_unique, idx_unique] = unique(t_warped,'stable');
  [~,idx_original_unique,~] = intersect(t_unique, t_original_warped);
  t_original_rescaled = t_unique(idx_original_unique);

  qtraj_rescaled = PPTrajectory(pchip(t_unique, q_path(:,idx_unique)));
end

