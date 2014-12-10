function plotPoses(poses, fignumber)

if (nargin <2)
    fignumber = 1;
end
    
figure(fignumber), clf
subplot(511),plot(poses.lQb(:,2:4)),title('Quaternion vector'), grid on
subplot(512),plot(poses.a_l),title('Local frame acceleration'), grid on
subplot(513),plot(poses.f_l),title('Local force'), grid on
subplot(514),plot(poses.V_l),title('Local velocity'), grid on
subplot(515),plot(poses.P_l),title('Local position'), grid on

