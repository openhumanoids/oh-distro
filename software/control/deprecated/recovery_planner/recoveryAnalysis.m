load('limp_history.mat', 'limp_history');
load('recovery_output.mat', 'result', 'footsteps');
lh = limp_history;
t = lh(1,:);
limp_x = lh(2,:);
limp_y = lh(3,:);
limp_xd = lh(4,:);
limp_yd = lh(5,:);
limp_xdd = [0, diff(limp_xd) ./ diff(t)];
limp_ydd = [0, diff(limp_yd) ./ diff(t)];
zmp_x = lh(6,:);
zmp_y = lh(7,:);
ic_x = lh(8,:);
ic_y = lh(9,:);
omega_0 = lh(10,:);
dzmp = lh(11:12,:);
dzmp_x = dzmp(1,:);
dzmp_y = dzmp(2,:);
dcom_x = lh(13,:);
dcom_y = lh(14,:);

load('zmp_traj', 'zmptraj', 'foottraj');
zmptraj_xy = zmptraj.eval(t);

figure(1)
plot(t, medfilt1(limp_ydd ./ (omega_0 .^2 .* (limp_y - zmp_y))))
xlim([0.05,1])
xlabel('t')
ylabel('$\frac{\ddot{r_y}}{\omega_0^2 (r_y - zmp_y)}$', 'Interpreter', 'LaTeX', 'FontSize', 24)
ylim([0,3])

figure(2)
plot(t, zmp_y, 'r.-', t, dzmp_y, 'k.-', t, limp_y, 'g-', t, dcom_y, 'b-');
legend('measured zmp_y', 'dzmp_y', 'com_y', 'dcom_y')
xlabel('t')
ylabel('y')

figure(3)
plot(t, medfilt1(limp_xdd ./ (omega_0 .^2 .* (limp_x - zmp_x))))
xlim([0.05,1])
xlabel('t')
ylabel('$\frac{\ddot{r_x}}{\omega_0^2 (r_x - zmp_x)}$', 'Interpreter', 'LaTeX', 'FontSize', 24)
ylim([0,3])

figure(4)
plot(t, zmp_x, 'r.-', t, dzmp_x, 'k.-', t, limp_x, 'g-', t, dcom_x, 'b-');
legend('measured zmp_x', 'dzmp_x', 'com_x', 'dcom_x')
xlabel('t')
ylabel('x')