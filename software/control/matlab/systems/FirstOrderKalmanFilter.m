classdef FirstOrderKalmanFilter < DrakeSystem
% obj = FirstOrderKalmanFilter(process_noise, obs_noise)
% Create an n-state FirstOrderKalmanFilter
% Models the system as 1st order with position measurements
% Estimates velocity by a kalman filter
% @param process_noise (nx1) vector of the velocity process noise
% @param obs_noise (nx1) vector of the position observation noise
% state order is
% [q;qd;P1(1,1);P2(1,1);...Pn(1,1);P1(1,2);...Pn(1,2);P1(2,2);...Pn(2,2)]
% where Pi is the 2x2 symmetric covariance matrix of joint i
  methods
    function obj = FirstOrderKalmanFilter(process_noise, obs_noise)
      n = length(process_noise);
      sizecheck(process_noise,[n 1]);
      sizecheck(obs_noise,[n 1]);
      % 5n+1 states: 1 for last time, 3n for sym-covariance, 2n for state
      % inputs are n measurements
      % outputs is the state
      obj = obj@DrakeSystem(0,1+5*n,n,2*n,false,true);
      obj.process_noise = process_noise;
      obj.obs_noise = obs_noise;
      obj.n = n;
    end
    
    % Default initial state is t=q=v=zeros() and Pi = eye;
    function x0 = getInitialState(obj)
      x0 = [0;zeros(2*obj.n,1);ones(obj.n,1);zeros(obj.n,1);ones(obj.n,1)];
    end
    
    function xdn = update(obj,t,x,u)
      % This is all an optimized version of running n Kalman filters
      % in parallel.  For a joint space of 30, this is ~4x faster than
      % constructing one 60 dimensional Kalman filter
      % it's also 2x faster than using a for loop
      q = x(2:obj.n+1);
      qd = x(obj.n+2:2*obj.n+1);
      dt = t - x(1);
      
      if dt > 1e-2
        warning('Update step seems large. t=%d and last update was at t=%d',t,x(1));
      end
      
      q_prior = q + qd*dt;
      qd_prior = qd;
      
      meas_resid = u - q_prior;      
      
      P1 = x(2+2*obj.n:1+3*obj.n);
      P2 = x(2+3*obj.n:1+4*obj.n);
      P3 = x(2+4*obj.n:1+5*obj.n);
      
      
      P_p1 = P1 + 2*dt*P2 + dt*obj.process_noise + dt^2*P3;
      P_p2 = P2 + dt*P3;
%       P_p4 = P3 + obj.process_noise;
      
      S = P_p1 + obj.obs_noise;
      
      q_est = q_prior + P_p1./S.*meas_resid;
      qd_est = qd_prior + P_p2./S.*meas_resid;
      
      KH1 = P_p1./S;
      KH2 = P_p2./S;
      
      P_e1 =  -(KH1 - 1).*(P1 + 2*dt*P2 + dt*obj.process_noise + dt^2*P3);
      P_e2 = -(KH1 - 1).*(P2 + dt*P3);
      P_e3 = P3 + obj.process_noise - KH2.*(P2 + dt*P3);
      
      
      xdn = [t;q_est;qd_est;P_e1;P_e2;P_e3];      
    end
    
    function y = output(obj,t,x,u)
      y = x(2:1+2*obj.n);
    end
  end
  
  properties (SetAccess=protected)
    process_noise
    obs_noise
    n
  end
  
end