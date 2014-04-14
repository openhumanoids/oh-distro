%% Observation likelihood PDF p(y[k] | x[k])
function p = p_yk_given_xk(k, yk, xk, camposek) 
  global bound hbound
  obs_error = deg2rad(5);
  %xk = quat2dcm(camposek(4:7, 1)') \ (xk - camposek(1:3, 1));  % transform to robot hand cam frame
  %xk = quat2dcm(camposek(4:7, 1)') * (xk - camposek(1:3, 1));  % transform to robot hand cam frame
  xk = quatrotate(camposek(4:7, 1)', (xk- camposek(1:3, 1))')' ;  % transform to robot hand cam frame
  if xk(3) < bound && abs(xk(1)) < hbound && abs(xk(2)) < hbound
    p = normpdf(yk(1) - atan((xk(1))/xk(3)), 0, obs_error) * normpdf(yk(2) - atan((xk(2))/xk(3)), 0, obs_error); 
  else
    p = 0;
  end
%p_obs_noise(yk - obs(k, xk, 0));
