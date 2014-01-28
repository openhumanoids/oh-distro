function plotGrayINSPredicted(predicted, fignumber)

if (nargin <2)
    fignumber = 1;
end

figH = figure(fignumber);
clf
set(figH,'Name',['Gray box INS predicted states, ' num2str(clock())],'NumberTitle','off')

subplot(621)
plot(predicted.bg)
title('Predicted gyro biases')
grid on

subplot(622)
plot(predicted.ba)
title('Predicted accelerometer biases')
grid on

subplot(623)
plot(predicted.w_b)
title('Predicted body rates')
grid on

subplot(624)
plot(predicted.a_b)
title('Predicted body accelerations')
grid on

subplot(613),
plot(predicted.lQb(:,2:4))
title('Vector portion predicted lQb')
grid on

subplot(614)
plot(predicted.f_l)
title('Predicted local frame specific force')
grid on

subplot(615)
plot(predicted.V_l)
title('Predicted local frame velocity')
grid on

subplot(616)
plot(predicted.P_l)
title('Predicted local frame positions')
grid on




