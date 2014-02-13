function figH = plotEKFResults(RESULTS, fignumber)

if (nargin <2)
    fignumber = 1;
end

figH = figure(fignumber);
clf
set(figH,'Name',['EKF poseterior state vector, ' num2str(clock())],'NumberTitle','off')

subplot(511),plot(RESULTS.STATEX(:,1:3)),title('State 1:3 -- Misalignment Estimate'), grid on
subplot(512),plot(RESULTS.STATEX(:,4:6)),title('State 4:6 -- Gyro Bias Estimate'), grid on
subplot(513),plot(RESULTS.STATEX(:,7:9)),title('State 7:9 -- delta Velocity Estimate'), grid on
subplot(514),plot(RESULTS.STATEX(:,10:12)),title('State 10:12 -- Acc Bias Estimate'), grid on
subplot(515),plot(RESULTS.STATEX(:,13:15)),title('State 13:15 -- delta Position estimate'), grid on


