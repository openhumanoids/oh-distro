function [F, G] = step_user_fun(x)

rot_scale = 1;

x = reshape(x, 3, []); % x y yaw

% objective: (x(1,2) - x(1,1))^2 + (x(2,2) - x(2,1))^2 + (rot_scale *
%               (x(3,2) - x(3,1))) ^2
% = x12^2 - 2 * x12 * x11 + x11^2 + x22^2 - 2 * x22 * x21 + x21^2 + r^2 *
%         x32^2 - 2 * r^2 * x32 * x31 + r^2 * x31^2
% for each pair of rows (1,2)
% so, we end up with:
% x11^2 + 2*x12^2 + 2*x13^2 + 2*x14^2 + ... + x1N^2
% + x21^2 + 2*x22^2 + ... + x2N^2
% + r^2*x31^2 + 2*r^2*x32^2 + ... + r^2*x3N^2
% - 2*x11*x12 - 2*x12*x13 - .... - 2*x1(N-1)*x1N
% - 2*x21*x22 - 2*x22*x23 - .... - 2*x2(N-1)*x2N
% - 2*r^2*x31*x32 - 2*r^2*x32*x33 - .... - 2*r^2*x3(N-1)*x3N

% the first constraint func is: x11^2 - 2*x11*x12 + x12^2 
%                               + x21^2 - 2*x21*x22 + x22^2
%                               + r^2*x31^2 - 2*r^2*x31*x32 + r^2*x32^2

% so the first gradient is: [2*x11 - 2*x12,         2*x12 - 2*x11,0,...,0;
%                            2*x21 - 2*x22,         2*x22 - 2*x21,0,...,0;
%                    2*r^2*x31 - 2*r^2*x32, 2*r^2*x32 - 2*r^2*x31,0,...,0]

d_dists = sum((x(1:2,2:end) - x(1:2,1:(end-1))) .^ 2, 1)
r_dists = (x(3,2:end) - x(3,1:(end-1))) .^2 * rot_scale^2

g_d_cons_up = [[(2*x(1:2,1:(end-1)) - 2*x(1:2,2:end))];
               zeros(1, length(x(1,:))-1)]
g_r_cons = [zeros(2, length(x(1,:))-1);
             (2*x(3,1:(end-1)) - 2*x(3,2:end)) * rot_scale^2]

g_d_obj = [[(2*x(1:2,1:(end-1)) - 2*x(1:2,2:end)), [0;0]] + ...
           [[0;0], 2*x(1:2,2:end) - 2*x(1:2,1:(end-1))];
           zeros(1, length(x(1,:)))]

g_r_obj = [zeros(2, length(x(1,:)));
           [(2*x(3,1:(end-1)) - 2*x(3,2:end)) * rot_scale^2, 0] + ...
           [0, (2*x(3,2:end) - 2*x(3,1:(end-1))) * rot_scale^2]]

g_obj = g_d_obj + g_r_obj

J = [