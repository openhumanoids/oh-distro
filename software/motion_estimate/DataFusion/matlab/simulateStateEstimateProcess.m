

clear

iterations = 5000;

RESULTS = MotionSimulator(iterations);


%% plot some stuff


figure(1), clf;
plot3(RESULTS.traj.true.P_l(:,1),RESULTS.traj.true.P_l(:,2),RESULTS.traj.true.P_l(:,3),'Linewidth',2)
hold on
plot3(RESULTS.trueINSPose.P_l(:,1),RESULTS.trueINSPose.P_l(:,2),RESULTS.trueINSPose.P_l(:,3),'g:','Linewidth',2.5)
plot3(RESULTS.cppINSPose.P_l(:,1),RESULTS.cppINSPose.P_l(:,2),RESULTS.cppINSPose.P_l(:,3),'r','Linewidth',2)
grid on
% title(['3D Position from ' num2str(t(1)) ' s to ' num2str(t(end)) ' s'])
axis equal

%%
return
if (false)
    
figure(2),clf;
plot(t,lookatvector(data,start,stop,'true.pose.V_l(1)'),t,lookatvector(data,start,stop,'true.pose.V_l(2)'),t,lookatvector(data,start,stop,'true.pose.V_l(3)'))
grid on
title('Velocity components')
xlabel('Time [s]')
ylabel('[m/s]')
legend({'X';'Y';'Z'})

end


%%

figure(3), clf;

errAx = (lookatvector(data,start,stop,'true.pose.f_l(1)')-lookatvector(data,start,stop,'trueINS.pose.f_l(1)'));
errAy = (lookatvector(data,start,stop,'true.pose.f_l(2)')-lookatvector(data,start,stop,'trueINS.pose.f_l(2)'));
errAz = (lookatvector(data,start,stop,'true.pose.f_l(3)')-lookatvector(data,start,stop,'trueINS.pose.f_l(3)'));

subplot(331)
plot(t, errAx, t, errAy, t, errAz);
title('Local true INS accel residual')
grid on

errVx = (lookatvector(data,start,stop,'true.pose.V_l(1)')-lookatvector(data,start,stop,'trueINS.pose.V_l(1)'));
errVy = (lookatvector(data,start,stop,'true.pose.V_l(2)')-lookatvector(data,start,stop,'trueINS.pose.V_l(2)'));
errVz = (lookatvector(data,start,stop,'true.pose.V_l(3)')-lookatvector(data,start,stop,'trueINS.pose.V_l(3)'));

subplot(332)
plot(t, errVx, t, errVy, t, errVz);
title('Local true INS V residual')
grid on


errPx = (lookatvector(data,start,stop,'true.pose.P_l(1)')-lookatvector(data,start,stop,'trueINS.pose.P_l(1)'));
errPy = (lookatvector(data,start,stop,'true.pose.P_l(2)')-lookatvector(data,start,stop,'trueINS.pose.P_l(2)'));
errPz = (lookatvector(data,start,stop,'true.pose.P_l(3)')-lookatvector(data,start,stop,'trueINS.pose.P_l(3)'));

subplot(333)
plot(t, errPx, t, errPy, t, errPz);
title('Local true INS P residual')
grid on


subplot(3,3,4)
errFx = (lookatvector(data,start,stop,'true.pose.f_l(1)')-lookatvector(data,start,stop,'INS.pose.f_l(1)'));
errFy = (lookatvector(data,start,stop,'true.pose.f_l(2)')-lookatvector(data,start,stop,'INS.pose.f_l(2)'));
errFz = (lookatvector(data,start,stop,'true.pose.f_l(3)')-lookatvector(data,start,stop,'INS.pose.f_l(3)'));

% plot(t, sqrt(errPx.^2 + errPy.^2 + errPz.^2));
plot(t, errFx, t, errFy, t, errFz);
grid on
title('Local frame applied force residual')
xlabel('Time [s]')

subplot(3,3,5)
errVx = (lookatvector(data,start,stop,'true.pose.V_l(1)')-lookatvector(data,start,stop,'INS.pose.V_l(1)'));
errVy = (lookatvector(data,start,stop,'true.pose.V_l(2)')-lookatvector(data,start,stop,'INS.pose.V_l(2)'));
errVz = (lookatvector(data,start,stop,'true.pose.V_l(3)')-lookatvector(data,start,stop,'INS.pose.V_l(3)'));

% plot(t, sqrt(errPx.^2 + errPy.^2 + errPz.^2));
plot(t, errVx, t, errVy, t, errVz);
grid on
title('Velocity residual, truth and outside INS')
xlabel('Time [s]')

subplot(3,3,6)
errPx = (lookatvector(data,start,stop,'true.pose.P_l(1)')-lookatvector(data,start,stop,'INS.pose.P_l(1)'));
errPy = (lookatvector(data,start,stop,'true.pose.P_l(2)')-lookatvector(data,start,stop,'INS.pose.P_l(2)'));
errPz = (lookatvector(data,start,stop,'true.pose.P_l(3)')-lookatvector(data,start,stop,'INS.pose.P_l(3)'));

% plot(t, sqrt(errPx.^2 + errPy.^2 + errPz.^2));
plot(t, errPx, t, errVy, t, errVz);
grid on
title('Position residual')
xlabel('Time [s]')


% still have to ensure that the rotations between the systems are correct


if (false)
subplot(3,3,5)
estPx = (lookatvector(data,start,stop,'df.posterior.x(1)'));
estPy = (lookatvector(data,start,stop,'df.posterior.x(2)'));
estPz = (lookatvector(data,start,stop,'df.posterior.x(3)'));

plot(t, estPx, t, estPy, t, estPz);
title('Est P error')
grid on
xlabel('Time [s]')
end