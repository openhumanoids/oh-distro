function plotReferenceMeasurement(RESULTS, fignumber)

if (nargin <2)
    fignumber = 1;
end

figH = figure(fignumber);
clf
set(figH,'Name',['Reference measuremens, ' num2str(clock())],'NumberTitle','off')

subplot(511),plot(RESULTS.Reference.V_p),title('Pelvis Velocity'), grid on
subplot(512),plot(RESULTS.Reference.V_l),title('Rotated pelvis to local Velocity'), grid on
subplot(513),plot(RESULTS.Reference.V_innov_l),title('Local frame velocity residuals'), grid on
% subplot(514),plot(RESULTS.STATEX(:,10:12)),title('State 10:12 -- Acc Bias Estimate'), grid on
% subplot(515),plot(RESULTS.STATEX(:,13:15)),title('State 13:15 -- delta Position estimate'), grid on


