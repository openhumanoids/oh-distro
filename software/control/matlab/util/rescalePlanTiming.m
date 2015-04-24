function qtraj_rescaled = rescalePlanTiming(qtraj, qd_max, varargin)
  % @param acceleration_param - Scalar parameter greater than or equal to 2 that
  %                             adjusts the acceleration profile. Higher values
  %                             yield more gradual accelerations. @defualt 3

  % check to see whether options were passed in as part of varagin
  
  % Scale timing to obey joint velocity limits
  % Create initial spline
  n_breaks = numel(qtraj.getBreaks());
  t = linspace(qtraj.tspan(1), qtraj.tspan(2),max(n_breaks, 20));
  q_path = eval(qtraj, t); %#ok
  t_mid = mean([t(2:end); t(1:end-1)],1);
  qd_mid = qtraj.fnder().eval(t_mid);
  body_scaling = 0;

  if isstruct(varargin{end})
    body_scaling = 1;
    % options is a struct with fields, body_id, pts, theta_max
    options = varargin{end};
    num_bodies = length(options.body_id);
    body_path = zeros(3*num_bodies,length(t));
    body_xyz_max = options.max(1:3,:);
    body_xyz_max = body_xyz_max(:);
    body_quat = zeros(4,num_bodies,length(t));
    robot = options.robot;
    for j = 1:length(t)
      kinsol = robot.doKinematics(q_path(:,j));
      for k = 1:num_bodies
        idx = 3*(k-1) + 1;
        body_pos = robot.forwardKin(kinsol,options.body_id(k),options.pts(:,k),2);
        body_path(idx:idx+2,j) = body_pos(1:3);
        body_quat(:,k,j) = body_pos(4:7);
      end
    end
    body_traj = PPTrajectory(pchip(t,body_path));
    body_mid = body_traj.fnder().eval(t_mid);
    max_vel = [qd_max;body_xyz_max];
    mid = [qd_mid;body_mid];

    theta_mid = zeros(num_bodies,length(t_mid));
    theta_max = options.max(4,:);
    theta_max = theta_max(:);
    for j = 1:numel(t)-1
      for k = 1:num_bodies
        theta_mid(k,j) = quatRotationDistance(body_quat(:,k,j),body_quat(:,k,j+1))/(t(j+1) - t(j));
      end
    end
    theta_scale_factor = max(abs(bsxfun(@rdivide, theta_mid, theta_max)), [], 1);
    varargin = varargin(1:end-1);
  end

  % scaling for rotation speed, if present
  scale_factor = max(abs(bsxfun(@rdivide, mid, max_vel)), [], 1);
  if body_scaling
    scale_factor = max(scale_factor,theta_scale_factor);
  end

  % Adjust durations to keep velocity below max
  t_scaled = [0, cumsum(diff(t).*scale_factor)];
  tf = t_scaled(end);

  % Warp time to give gradual acceleration/deceleration
  t_warped = tf*warpTime(t_scaled/tf, varargin{:});
  [t_unique, idx_unique] = unique(t_warped,'stable');

  qtraj_rescaled = PPTrajectory(pchip(t_unique, q_path(:,idx_unique)));
end

