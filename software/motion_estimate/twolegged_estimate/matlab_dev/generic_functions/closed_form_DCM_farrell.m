function [R_a__b_k0] = closed_form_DCM_farrell(w_k0,R_a__b_k1,dt)
% Function to propagate DCM differential equation from (k-1) to (k-0) 
% through closed form solution (Taylor Expansion). Assuming rotation 
% rate vector w (at time (k-0)) is constant over integration interval dt.
% 
% Farrell 2008, Aided Navigation GPS with High Rate sensors.
% 
% Version 1.0: 14/06/2012 - Copied from p56 of the book.
%
% d Fourie


v = w_k0*dt; % can this not at least be trapezoidal integration -- devil is in the details
Gam = vec2skew(v);
v_norm = norm(v);

if (v_norm>1E-7) % 1E-7 is chosen because double numerical LSB is around 1E-18 for nominal values [-pi..pi]
    % and (1E-7)^2 is at 1E-14, but 1E-7 rad/s is 0.02 deg/h
    coeff = eye(3) - sin(v_norm)/v_norm*Gam + (1-cos(v_norm))/(v_norm^2)*Gam^2;
else
    coeff = eye(3) - Gam + (Gam^2)/2;
end


R_a__b_k0 = coeff*R_a__b_k1;
