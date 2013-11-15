function irobot_hand_joint_angle_estimation()

data = import_irobot_hand_data('../data/joint_angle_estimation_data.csv');

xData = [data.motorHallEncoder0; data.motorHallEncoder1];
yData = [data.proximalJointAngle0; data.proximalJointAngle1];

validIndices = find(xData > 0);
xData = xData(validIndices);
yData = yData(validIndices);

nKnotPoints = 3;
[param0, xKnot1, xKnotFinal] = getParam0(xData, yData, nKnotPoints);

[lb, ub] = getbounds(param0, xKnot1, xKnotFinal, nKnotPoints);

param = lsqcurvefit(@piecewiseLinear, param0, xData, yData, lb, ub);

xKnotSol = [xKnot1; param(1 : nKnotPoints - 2); xKnotFinal];
yKnotSol = param(nKnotPoints - 1 : end);
slopes = diff(yKnotSol) ./ diff(xKnotSol);
intercepts = yKnotSol(2 : end) - slopes .* xKnotSol(2 : end);

fprintf('xKnotSol:\n');
disp(xKnotSol);

fprintf('yKnotSol:\n');
disp(yKnotSol);

fprintf('slopes:\n')
disp(slopes);

fprintf('intercepts:\n')
disp(intercepts);

hold on;
plot(data.motorHallEncoder0, data.proximalJointAngle0, 'b');
plot(data.motorHallEncoder1, data.proximalJointAngle1, 'b');
plot(data.motorHallEncoder0, piecewiseLinear(param, data.motorHallEncoder0), 'r');
plot(data.motorHallEncoder1, piecewiseLinear(param, data.motorHallEncoder1), 'r');

xlabel('motor tendon excursion')
ylabel('proximal joint angle')
% for i = 1 : length(slopes)
%     slope = slopes(i);
%     intercept = intercepts(i);
%     plot(data.motorHallEncoder0, slope * data.motorHallEncoder0 + intercept, 'k');
% end
hold off;

  function y = piecewiseLinear(param, xDatai)
    xKnot = [xKnot1; param(1 : nKnotPoints - 2); xKnotFinal];
    yKnot = param(nKnotPoints - 1 : end);    
    y = interp1(xKnot, yKnot, xDatai, 'linear');
  end
end

function [x0, xKnot1, xKnotFinal] = getParam0(xData, yData, nKnotPoints)
[xKnot1, minIndex] = min(xData);
[xKnotFinal, maxIndex] = max(xData);

yDataKnot1 = yData(minIndex);
yDataKnotFinal = yData(maxIndex);

xKnot = [xKnot1; xKnotFinal];
yDataKnot = [yDataKnot1; yDataKnotFinal];

xs = linspace(xKnot1, xKnotFinal, nKnotPoints)';
ys = interp1(xKnot, yDataKnot, xs, 'linear');

x0 = [xs(2 : end - 1); ys];
end

function [lb, ub] = getbounds(param0, xKnot1, xKnotFinal, nKnotPoints)
xs = [xKnot1; param0(1 : nKnotPoints - 2); xKnotFinal];
ys = param0(nKnotPoints - 1 : end);

xlb = xs(1 : end - 2);
xub = xs(3 : end);

lb = [xlb; -inf * ones(size(ys))];
ub = [xub; inf * ones(size(ys))];

end