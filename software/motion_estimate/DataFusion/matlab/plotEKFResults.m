function plotEKFResults(RESULTS, fignumber)

if (nargin <2)
    fignumber = 1;
end

figH = figure(fignumber);
clf
set(figH,'Name',['EKF poseterior state vector, ' num2str(clock())],'NumberTitle','off')

subplot(511),plot(RESULTS.STATEX(:,1:3)),title('Misalignment Estimate'), grid on
subplot(512),plot(RESULTS.STATEX(:,4:6)),title('Gyro Bias Estimate'), grid on
subplot(513),plot(RESULTS.STATEX(:,7:9)),title('delta Velocity Estimate'), grid on
subplot(514),plot(RESULTS.STATEX(:,10:12)),title('Acc Bias Estimate'), grid on
subplot(515),plot(RESULTS.STATEX(:,13:15)),title('delta Position estimate'), grid on

