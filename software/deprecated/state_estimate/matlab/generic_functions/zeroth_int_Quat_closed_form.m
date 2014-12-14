function [aQb_k0] = zeroth_int_Quat_closed_form(w, aQb_k1, dt)
% Propagate orientation quaternion from (k-1) to (k-0) 
% through closed form solution (Taylor Expansion -> trigonometric). Assuming rotation 
% rate vector w (at time (k-0)) is constant over integration interval dt.
% Quaternion convention is scalar first: [w;x;y;z]
% Ensures: | 1 - ||q|| | < 1E-13
% 
% Version 1.0: 12/31/2013 
%
% D Fourie

w_dt = w*dt;
w_norm = norm(w);
w_norm_dt = w_norm*dt;

if (w_norm>1E-6) % 1E-7 is chosen because double numerical LSB is around 1E-18 for nominal values [-pi..pi]
    % and (1E-7)^2 is at 1E-14, but 1E-7 rad/s is 0.02 deg/h
    r = [cos(w_norm*dt/2);...
         w./w_norm*sin(w_norm*dt/2)];
     
    aQb_k0 = qprod(r,aQb_k1);
else
    r = [cos(w_norm_dt/2);...
         w_dt*(0.5-w_norm_dt*w_norm_dt/48)];
    
    aQb_k0 = qprod(r,aQb_k1);
end

if (abs(1-norm(aQb_k0))>1E-13)
    disp 'zeroth_int_Quat_closed_form -- normalizing time propagated quaternion'
    aQb_k0 = aQb_k0./norm(aQb_k0);
end

